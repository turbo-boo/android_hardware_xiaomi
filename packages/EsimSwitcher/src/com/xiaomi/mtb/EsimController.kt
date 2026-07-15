/*
 * SPDX-FileCopyrightText: 2025 Paranoid Android
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

package com.xiaomi.mtb

import android.Manifest
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.Handler
import android.os.Looper
import android.telephony.SubscriptionManager
import android.util.Log
import com.qti.extphone.Client
import com.qti.extphone.ExtPhoneCallbackBase
import com.qti.extphone.ExtTelephonyManager
import com.qti.extphone.QtiSimType
import com.qti.extphone.ServiceCallback
import com.qti.extphone.Status
import com.qti.extphone.Token

class EsimController private constructor(private val context: Context) {

    data class State(
        val available: Boolean = false,
        val esimEnabled: Boolean = false,
        val requestInProgress: Boolean = false,
    )

    enum class Error {
        ACTIVE_ESIM_PROFILE,
        BUSY,
        REQUEST_FAILED,
        REQUEST_TIMEOUT,
        SERVICE_UNAVAILABLE,
        UNSUPPORTED,
    }

    interface Listener {
        fun onStateChanged(state: State)

        fun onError(error: Error) {}
    }

    companion object {
        private const val TAG = "EsimController"
        private val DEBUG = Log.isLoggable(TAG, Log.DEBUG)

        private const val ACTION_SIM_TYPE_UPDATE =
            "com.android.euicc.service.SIM_TYPE_UPDATE_ACTION"
        private const val EXTRA_SIM_TYPE = "com.android.euicc.service.extra_sim_type"
        private const val INVALID_SLOT = -1
        private const val MAX_REFRESH_ATTEMPTS = 15
        private const val REFRESH_DELAY_MS = 2_000L
        private const val REQUEST_TIMEOUT_MS = 30_000L

        @Volatile private var instance: EsimController? = null

        fun getInstance(context: Context): EsimController {
            return instance
                ?: synchronized(this) {
                    instance ?: EsimController(context.applicationContext).also { instance = it }
                }
        }
    }

    private enum class CapabilityStatus {
        NOT_READY,
        READY,
        UNSUPPORTED,
    }

    private val mainHandler = Handler(Looper.getMainLooper())
    private val extTelephonyManager = ExtTelephonyManager.getInstance(context)
    private val listeners = mutableSetOf<Listener>()

    private val esimSlotIndex: Int by lazy {
        val slots =
            context.resources.getIntArray(com.android.internal.R.array.non_removable_euicc_slots)
        slots.singleOrNull()?.takeIf { it >= 0 } ?: INVALID_SLOT
    }

    private var client: Client? = null
    private var connectionRequested = false
    private var currentSimTypes: Array<QtiSimType>? = null
    private var lpaReceiverRegistered = false
    private var queuedTarget: Boolean? = null
    private var requestToken: Int? = null
    private var requestTimeoutRunnable: Runnable? = null
    private var refreshAttempts = 0
    private var refreshScheduled = false
    private var state = State()
    private var supportedSimTypes: Array<QtiSimType>? = null

    private val lpaReceiver =
        object : BroadcastReceiver() {
            override fun onReceive(context: Context, intent: Intent) {
                if (intent.action != ACTION_SIM_TYPE_UPDATE) return

                val simType = intent.getIntExtra(EXTRA_SIM_TYPE, QtiSimType.SIM_TYPE_INVALID)
                when (simType) {
                    QtiSimType.SIM_TYPE_PHYSICAL -> setEsimEnabled(false)
                    QtiSimType.SIM_TYPE_ESIM -> setEsimEnabled(true)
                    else -> Log.w(TAG, "Ignoring invalid LPA SIM type: $simType")
                }
            }
        }

    private val qtiCallback =
        object : ExtPhoneCallbackBase() {
            override fun setSimTypeResponse(token: Token, status: Status) {
                mainHandler.post { handleSetSimTypeResponse(token.get(), status.get()) }
            }

            override fun onSimTypeChanged(simtype: Array<QtiSimType>) {
                mainHandler.post {
                    currentSimTypes = copySimTypes(simtype)
                    updateState()
                    refreshState()
                }
            }
        }

    private val refreshRunnable = Runnable {
        refreshScheduled = false
        refreshState()
    }

    private val serviceCallback =
        object : ServiceCallback {
            override fun onConnected() {
                mainHandler.post { handleServiceConnected() }
            }

            override fun onDisconnected() {
                mainHandler.post { handleServiceDisconnected() }
            }
        }

    fun onBootCompleted() {
        if (DEBUG) Log.d(TAG, "onBootCompleted")
        start()
    }

    fun addListener(listener: Listener) {
        mainHandler.post {
            listeners.add(listener)
            listener.onStateChanged(state)
            refreshAttempts = 0
            startInternal()
            if (client != null) refreshState()
        }
    }

    fun removeListener(listener: Listener) {
        mainHandler.post { listeners.remove(listener) }
    }

    fun getEsimActive(): Boolean {
        return runCatching {
                val subscriptionManager =
                    context.getSystemService(SubscriptionManager::class.java)
                        ?: return@runCatching true
                subscriptionManager.activeSubscriptionInfoList.orEmpty().any { it.isEmbedded }
            }
            .onFailure { Log.e(TAG, "Failed to query active eSIM profiles", it) }
            .getOrDefault(true)
    }

    fun setEsimEnabled(isEnabled: Boolean) {
        mainHandler.post { enqueueRequest(isEnabled) }
    }

    private fun start() {
        mainHandler.post { startInternal() }
    }

    private fun startInternal() {
        if (esimSlotIndex == INVALID_SLOT) {
            updateState()
            return
        }

        registerLpaReceiver()
        ensureServiceConnected()
    }

    private fun registerLpaReceiver() {
        if (lpaReceiverRegistered) return

        try {
            context.registerReceiver(
                lpaReceiver,
                IntentFilter(ACTION_SIM_TYPE_UPDATE),
                Manifest.permission.READ_PRIVILEGED_PHONE_STATE,
                mainHandler,
                Context.RECEIVER_EXPORTED,
            )
            lpaReceiverRegistered = true
        } catch (e: RuntimeException) {
            Log.e(TAG, "Failed to register the LPA SIM type receiver", e)
        }
    }

    private fun ensureServiceConnected() {
        if (connectionRequested) {
            if (extTelephonyManager.isServiceConnected && client == null) {
                handleServiceConnected()
            }
            return
        }

        connectionRequested = true
        val bindStarted =
            runCatching { extTelephonyManager.connectService(serviceCallback) }
                .onFailure { Log.e(TAG, "Failed to bind ExtTelephonyService", it) }
                .getOrDefault(false)
        if (!bindStarted) {
            runCatching { extTelephonyManager.disconnectService(serviceCallback) }
                .onFailure { Log.e(TAG, "Failed to clean up the service connection", it) }
            connectionRequested = false
            failQueuedRequest(Error.SERVICE_UNAVAILABLE)
            updateState()
        }
    }

    private fun handleServiceConnected() {
        if (client != null) {
            refreshState()
            return
        }

        client =
            runCatching { extTelephonyManager.registerCallback(context.packageName, qtiCallback) }
                .onFailure { Log.e(TAG, "Failed to register the QTI telephony callback", it) }
                .getOrNull()
        if (client == null) {
            failQueuedRequest(Error.SERVICE_UNAVAILABLE)
            updateState()
            return
        }

        refreshAttempts = 0
        refreshState()
    }

    private fun handleServiceDisconnected() {
        val hadRequest = queuedTarget != null || requestToken != null
        cancelRefresh()
        cancelRequestTimeout()
        client = null
        currentSimTypes = null
        queuedTarget = null
        requestToken = null
        supportedSimTypes = null
        updateState()
        if (hadRequest) notifyError(Error.SERVICE_UNAVAILABLE)
    }

    private fun refreshState() {
        if (client == null || !extTelephonyManager.isServiceConnected) {
            updateState()
            return
        }

        currentSimTypes =
            runCatching { extTelephonyManager.currentSimType }
                .onFailure { Log.e(TAG, "Failed to query current SIM types", it) }
                .getOrNull()
                ?.let(::copySimTypes)
        supportedSimTypes =
            runCatching { extTelephonyManager.supportedSimTypes }
                .onFailure { Log.e(TAG, "Failed to query supported SIM types", it) }
                .getOrNull()
                ?.let(::copySimTypes)

        updateState()
        when (getCapabilityStatus()) {
            CapabilityStatus.READY -> {
                cancelRefresh()
                refreshAttempts = 0
                dispatchQueuedRequest()
            }
            CapabilityStatus.NOT_READY -> scheduleRefresh()
            CapabilityStatus.UNSUPPORTED -> {
                cancelRefresh()
                failQueuedRequest(Error.UNSUPPORTED)
            }
        }
    }

    private fun scheduleRefresh() {
        if (refreshScheduled) return
        if (refreshAttempts >= MAX_REFRESH_ATTEMPTS) {
            failQueuedRequest(Error.SERVICE_UNAVAILABLE)
            return
        }

        refreshAttempts++
        refreshScheduled = true
        mainHandler.postDelayed(refreshRunnable, REFRESH_DELAY_MS)
    }

    private fun cancelRefresh() {
        if (!refreshScheduled) return
        mainHandler.removeCallbacks(refreshRunnable)
        refreshScheduled = false
    }

    private fun enqueueRequest(isEnabled: Boolean) {
        if (esimSlotIndex == INVALID_SLOT) {
            notifyError(Error.UNSUPPORTED)
            return
        }
        if (queuedTarget != null || requestToken != null) {
            notifyError(Error.BUSY)
            return
        }
        if (!isEnabled && getEsimActive()) {
            notifyError(Error.ACTIVE_ESIM_PROFILE)
            return
        }

        queuedTarget = isEnabled
        updateState()
        startInternal()
        dispatchQueuedRequest()
    }

    private fun dispatchQueuedRequest() {
        val target = queuedTarget ?: return
        val registeredClient = client ?: return

        when (getCapabilityStatus()) {
            CapabilityStatus.NOT_READY -> {
                scheduleRefresh()
                return
            }
            CapabilityStatus.UNSUPPORTED -> {
                failQueuedRequest(Error.UNSUPPORTED)
                return
            }
            CapabilityStatus.READY -> Unit
        }

        if (!target && getEsimActive()) {
            failQueuedRequest(Error.ACTIVE_ESIM_PROFILE)
            return
        }

        val current = currentSimTypes ?: return
        val targetType = if (target) QtiSimType.SIM_TYPE_ESIM else QtiSimType.SIM_TYPE_PHYSICAL
        if (current[esimSlotIndex].get() == targetType) {
            queuedTarget = null
            updateState()
            return
        }

        val requestedTypes = copySimTypes(current)
        requestedTypes[esimSlotIndex] = QtiSimType(targetType)
        Log.i(TAG, "Requesting SIM type $targetType for physical slot $esimSlotIndex")

        val token =
            runCatching { extTelephonyManager.setSimType(registeredClient, requestedTypes) }
                .onFailure { Log.e(TAG, "Failed to request a SIM type change", it) }
                .getOrNull()
        val tokenValue = token?.get()
        if (tokenValue == null || tokenValue == Token.UNSOL) {
            failQueuedRequest(Error.REQUEST_FAILED)
            return
        }

        queuedTarget = null
        requestToken = tokenValue
        updateState()
        scheduleRequestTimeout(tokenValue)
    }

    private fun handleSetSimTypeResponse(token: Int, status: Int) {
        if (requestToken != token) {
            if (DEBUG) Log.d(TAG, "Ignoring response for token $token")
            return
        }

        cancelRequestTimeout()
        requestToken = null
        if (status != Status.SUCCESS) {
            Log.e(TAG, "SIM type request $token failed with status $status")
            updateState()
            notifyError(Error.REQUEST_FAILED)
            refreshState()
            return
        }

        Log.i(TAG, "SIM type request $token completed")
        updateState()
        refreshState()
    }

    private fun scheduleRequestTimeout(token: Int) {
        val timeoutRunnable = Runnable {
            if (requestToken != token) return@Runnable

            Log.e(TAG, "SIM type request $token timed out")
            requestToken = null
            requestTimeoutRunnable = null
            updateState()
            notifyError(Error.REQUEST_TIMEOUT)
            refreshState()
        }
        requestTimeoutRunnable = timeoutRunnable
        mainHandler.postDelayed(timeoutRunnable, REQUEST_TIMEOUT_MS)
    }

    private fun cancelRequestTimeout() {
        requestTimeoutRunnable?.let(mainHandler::removeCallbacks)
        requestTimeoutRunnable = null
    }

    private fun failQueuedRequest(error: Error) {
        if (queuedTarget == null) return
        queuedTarget = null
        updateState()
        notifyError(error)
    }

    private fun getCapabilityStatus(): CapabilityStatus {
        val slotIndex = esimSlotIndex
        if (slotIndex == INVALID_SLOT) return CapabilityStatus.UNSUPPORTED

        val current = currentSimTypes ?: return CapabilityStatus.NOT_READY
        val supported = supportedSimTypes ?: return CapabilityStatus.NOT_READY
        if (slotIndex !in current.indices || slotIndex !in supported.indices) {
            return CapabilityStatus.UNSUPPORTED
        }
        if (
            current.any { it.get() == QtiSimType.SIM_TYPE_INVALID } ||
                supported[slotIndex].get() == QtiSimType.SIM_TYPE_INVALID
        ) {
            return CapabilityStatus.NOT_READY
        }
        if (current.any { !isSettableCurrentType(it.get()) }) {
            return CapabilityStatus.UNSUPPORTED
        }

        val supportedType = supported[slotIndex].get()
        return if (
            supportsMode(supportedType, QtiSimType.SIM_TYPE_PHYSICAL) &&
                supportsMode(supportedType, QtiSimType.SIM_TYPE_ESIM)
        ) {
            CapabilityStatus.READY
        } else {
            CapabilityStatus.UNSUPPORTED
        }
    }

    private fun updateState() {
        val newState =
            State(
                available = getCapabilityStatus() == CapabilityStatus.READY,
                esimEnabled =
                    currentSimTypes?.getOrNull(esimSlotIndex)?.get() == QtiSimType.SIM_TYPE_ESIM,
                requestInProgress = queuedTarget != null || requestToken != null,
            )
        if (newState == state) return

        state = newState
        listeners.forEach { it.onStateChanged(newState) }
    }

    private fun notifyError(error: Error) {
        Log.w(TAG, "SIM type operation failed: $error")
        listeners.forEach { it.onError(error) }
    }

    private fun copySimTypes(simTypes: Array<QtiSimType>): Array<QtiSimType> {
        return Array(simTypes.size) { index -> QtiSimType(simTypes[index].get()) }
    }

    private fun isSettableCurrentType(type: Int): Boolean {
        return type == QtiSimType.SIM_TYPE_PHYSICAL ||
            type == QtiSimType.SIM_TYPE_ESIM ||
            type == QtiSimType.SIM_TYPE_IUICC
    }

    private fun supportsMode(supportedType: Int, mode: Int): Boolean {
        return when (mode) {
            QtiSimType.SIM_TYPE_PHYSICAL ->
                supportedType == QtiSimType.SIM_TYPE_PHYSICAL ||
                    supportedType == QtiSimType.SIM_TYPE_PHYSICAL_ESIM ||
                    supportedType == QtiSimType.SIM_TYPE_PHYSICAL_IUICC ||
                    supportedType == QtiSimType.SIM_TYPE_PHYSICAL_ESIM_IUICC
            QtiSimType.SIM_TYPE_ESIM ->
                supportedType == QtiSimType.SIM_TYPE_ESIM ||
                    supportedType == QtiSimType.SIM_TYPE_PHYSICAL_ESIM ||
                    supportedType == QtiSimType.SIM_TYPE_ESIM_IUICC ||
                    supportedType == QtiSimType.SIM_TYPE_PHYSICAL_ESIM_IUICC
            else -> false
        }
    }
}
