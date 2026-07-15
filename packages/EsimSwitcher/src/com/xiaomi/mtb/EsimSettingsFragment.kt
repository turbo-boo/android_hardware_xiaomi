/*
 * SPDX-FileCopyrightText: 2025 Paranoid Android
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

package com.xiaomi.mtb

import android.os.Bundle
import android.util.Log
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.preference.Preference
import com.android.settingslib.widget.FooterPreference
import com.android.settingslib.widget.MainSwitchPreference
import com.android.settingslib.widget.SettingsBasePreferenceFragment

class EsimSettingsFragment :
    SettingsBasePreferenceFragment(), Preference.OnPreferenceChangeListener {

    companion object {
        private const val TAG = "EsimSettingsFragment"
        private val DEBUG = Log.isLoggable(TAG, Log.DEBUG)
    }

    private val esimController by lazy { EsimController.getInstance(requireContext()) }

    private val switchBar by lazy { findPreference<MainSwitchPreference>("esim_enable")!! }
    private val footerPref by lazy { findPreference<FooterPreference>("esim_footer")!! }

    private val controllerListener =
        object : EsimController.Listener {
            override fun onStateChanged(state: EsimController.State) {
                switchBar.isChecked = state.esimEnabled
                switchBar.isEnabled = state.available && !state.requestInProgress
                switchBar.summary =
                    when {
                        state.requestInProgress -> getString(R.string.esim_switching)
                        !state.available -> getString(R.string.esim_switch_unavailable)
                        else -> null
                    }
            }

            override fun onError(error: EsimController.Error) {
                if (error == EsimController.Error.ACTIVE_ESIM_PROFILE) {
                    showActiveEsimWarning()
                    return
                }

                val message =
                    when (error) {
                        EsimController.Error.BUSY -> R.string.esim_switch_busy
                        EsimController.Error.REQUEST_TIMEOUT -> R.string.esim_switch_timeout
                        EsimController.Error.SERVICE_UNAVAILABLE,
                        EsimController.Error.UNSUPPORTED -> R.string.esim_switch_unavailable
                        EsimController.Error.REQUEST_FAILED -> R.string.esim_switch_failed
                        EsimController.Error.ACTIVE_ESIM_PROFILE -> return
                    }
                Toast.makeText(requireContext(), message, Toast.LENGTH_LONG).show()
            }
        }

    override fun onCreatePreferences(savedInstanceState: Bundle?, rootKey: String?) {
        if (DEBUG) Log.d(TAG, "onCreatePreferences")
        setPreferencesFromResource(R.xml.settings_esim, rootKey)

        switchBar.onPreferenceChangeListener = this
        switchBar.isEnabled = false
        footerPref.title = getString(R.string.esim_footer_note)
    }

    override fun onStart() {
        super.onStart()
        esimController.addListener(controllerListener)
    }

    override fun onStop() {
        esimController.removeListener(controllerListener)
        super.onStop()
    }

    override fun onPreferenceChange(preference: Preference, newValue: Any?): Boolean {
        if (preference == switchBar) {
            val isChecked = newValue as? Boolean ?: return false
            if (DEBUG) Log.d(TAG, "onPreferenceChange: $isChecked")
            if (!isChecked && esimController.getEsimActive()) {
                showActiveEsimWarning()
                return false
            }
            esimController.setEsimEnabled(isChecked)
        }
        return false
    }

    private fun showActiveEsimWarning() {
        AlertDialog.Builder(requireContext())
            .setTitle(R.string.esim_warning_title)
            .setMessage(R.string.esim_warning_message)
            .setPositiveButton(android.R.string.ok, null)
            .setCancelable(false)
            .show()
    }
}
