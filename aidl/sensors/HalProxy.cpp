/*
 * Copyright (C) 2019 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "HalProxy.h"

#include <android/hardware/sensors/2.0/types.h>

#include <android-base/file.h>
#include "hardware_legacy/power.h"

#include <dlfcn.h>

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <deque>
#include <fstream>
#include <functional>
#include <limits>
#include <mutex>
#include <thread>

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace implementation {

using ::android::hardware::sensors::V1_0::Result;
using ::android::hardware::sensors::V2_0::EventQueueFlagBits;
using ::android::hardware::sensors::V2_0::WakeLockQueueFlagBits;
using ::android::hardware::sensors::V2_0::implementation::getTimeNow;
using ::android::hardware::sensors::V2_0::implementation::kWakelockTimeoutNs;

#if defined(XIAOMI_DUAL_ALS_FUSION)
namespace {

constexpr char kBackAlsType[] = "xiaomi.sensor.back_lux";
constexpr char kFusedAlsType[] = "xiaomi.sensor.fused_light";
constexpr char kFusedAlsName[] = "Xiaomi Fused Ambient Light";

// Keep the virtual handle outside the range used by the vendor sub-HALs.
constexpr int32_t kFusedAlsHandle = 0x7f000001;
constexpr int32_t kFusedAlsSensorType =
        static_cast<int32_t>(V2_1::SensorType::DEVICE_PRIVATE_BASE) + 0xdada;

struct BatchParams {
    bool valid = false;
    int64_t samplingPeriodNs = 0;
    int64_t maxReportLatencyNs = 0;
};

struct FlushRequest {
    uint64_t id = 0;
    bool fused = false;
};

struct FusedFlushRequest {
    uint64_t id = 0;
    bool frontComplete = false;
    bool backComplete = false;
};

struct FusedAlsState {
    int32_t fusedHandle = -1;
    int32_t fusedSensorType = -1;
    int32_t frontHandle = -1;
    int32_t backHandle = -1;
    V2_1::SensorType frontSensorType = V2_1::SensorType::LIGHT;
    V2_1::SensorType backSensorType = V2_1::SensorType::DEVICE_PRIVATE_BASE;
    bool configured = false;
    bool active = false;
    bool enabling = false;
    bool frontExternalEnabling = false;
    bool backExternalEnabling = false;
    bool frontExternallyActive = false;
    bool backExternallyActive = false;
    bool frontHardwareActive = false;
    bool backHardwareActive = false;
    bool frontValueValid = false;
    bool backValueValid = false;
    float frontLux = 0.0f;
    float backLux = 0.0f;
    int64_t frontTimestamp = 0;
    int64_t backTimestamp = 0;
    bool lastOutputValid = false;
    float lastOutputLux = 0.0f;
    int64_t lastOutputTimestamp = 0;
    BatchParams fusedBatch;
    BatchParams frontExternalBatch;
    BatchParams backExternalBatch;
    BatchParams frontProgrammedBatch;
    BatchParams backProgrammedBatch;
    std::vector<V2_1::Event> frontPendingActivationEvents;
    std::vector<V2_1::Event> backPendingActivationEvents;
    uint64_t nextFlushId = 1;
    std::deque<FlushRequest> frontFlushRequests;
    std::deque<FlushRequest> backFlushRequests;
    std::deque<FusedFlushRequest> fusedFlushRequests;
};

FusedAlsState gFusedAls;
std::mutex gFusedAlsMutex;
std::recursive_mutex gFusedAlsControlMutex;

bool batchParamsEqual(const BatchParams& left, const BatchParams& right) {
    return left.valid == right.valid &&
            (!left.valid || (left.samplingPeriodNs == right.samplingPeriodNs &&
                             left.maxReportLatencyNs == right.maxReportLatencyNs));
}

BatchParams combineBatchParams(const BatchParams& first, const BatchParams& second) {
    if (!first.valid) return second;
    if (!second.valid) return first;
    return {
            true,
            std::min(first.samplingPeriodNs, second.samplingPeriodNs),
            std::min(first.maxReportLatencyNs, second.maxReportLatencyNs),
    };
}

void restoreFusedEventStateLocked(const FusedAlsState& state) {
    gFusedAls.frontValueValid = state.frontValueValid;
    gFusedAls.backValueValid = state.backValueValid;
    gFusedAls.frontLux = state.frontLux;
    gFusedAls.backLux = state.backLux;
    gFusedAls.frontTimestamp = state.frontTimestamp;
    gFusedAls.backTimestamp = state.backTimestamp;
    gFusedAls.lastOutputValid = state.lastOutputValid;
    gFusedAls.lastOutputLux = state.lastOutputLux;
    gFusedAls.lastOutputTimestamp = state.lastOutputTimestamp;
}

V2_1::Event makeFusedAlsEventLocked(float lux, int64_t timestamp) {
    if (gFusedAls.lastOutputValid && timestamp <= gFusedAls.lastOutputTimestamp) {
        if (gFusedAls.lastOutputTimestamp == std::numeric_limits<int64_t>::max()) {
            return {};
        }
        timestamp = gFusedAls.lastOutputTimestamp + 1;
    }

    V2_1::Event event{};
    event.timestamp = timestamp;
    event.sensorHandle = gFusedAls.fusedHandle;
    event.sensorType = static_cast<V2_1::SensorType>(gFusedAls.fusedSensorType);
    event.u.data[0] = lux;
    gFusedAls.lastOutputLux = lux;
    gFusedAls.lastOutputTimestamp = timestamp;
    gFusedAls.lastOutputValid = true;
    return event;
}

template <typename EventQueue>
bool enqueueEventsLocked(const std::vector<V2_1::Event>& events, EventQueue* eventQueue,
                         EventFlag* eventQueueFlag,
                         const std::map<int32_t, V2_1::SensorInfo>& sensors,
                         std::queue<std::pair<std::vector<V2_1::Event>, size_t>>* pendingEvents,
                         size_t* pendingSize, size_t* mostPending, size_t maxPending,
                         std::condition_variable* eventQueueWriteCV,
                         size_t* droppedWakeupEvents = nullptr) {
    if (droppedWakeupEvents != nullptr) *droppedWakeupEvents = 0;
    if (events.empty()) return true;
    if (eventQueue == nullptr || eventQueueFlag == nullptr) return false;

    auto countWakeupEvents = [&sensors, &events](size_t begin) {
        size_t count = 0;
        for (size_t i = begin; i < events.size(); ++i) {
            auto sensor = sensors.find(events[i].sensorHandle);
            if (sensor != sensors.end() &&
                (sensor->second.flags & static_cast<uint32_t>(V1_0::SensorFlagBits::WAKE_UP))) {
                ++count;
            }
        }
        return count;
    };

    size_t numToWrite = 0;
    if (pendingEvents->empty()) {
        numToWrite = std::min(events.size(), eventQueue->availableToWrite());
        if (numToWrite > 0) {
            if (eventQueue->write(events.data(), numToWrite)) {
                eventQueueFlag->wake(
                        static_cast<uint32_t>(EventQueueFlagBits::READ_AND_PROCESS));
            } else {
                numToWrite = 0;
            }
        }
    }

    const size_t numLeft = events.size() - numToWrite;
    if (numLeft == 0) return true;
    const size_t pendingWakeupEvents = countWakeupEvents(numToWrite);
    if (*pendingSize + numLeft > maxPending) {
        if (droppedWakeupEvents != nullptr) *droppedWakeupEvents = pendingWakeupEvents;
        return false;
    }

    std::vector<V2_1::Event> eventsLeft(events.begin() + numToWrite, events.end());
    pendingEvents->push({std::move(eventsLeft), pendingWakeupEvents});
    *pendingSize += numLeft;
    *mostPending = std::max(*mostPending, *pendingSize);
    eventQueueWriteCV->notify_one();
    return true;
}

}  // namespace
#endif

typedef V2_0::implementation::ISensorsSubHal*(SensorsHalGetSubHalFunc)(uint32_t*);
typedef V2_1::implementation::ISensorsSubHal*(SensorsHalGetSubHalV2_1Func)(uint32_t*);

static constexpr int32_t kBitsAfterSubHalIndex = 24;

/**
 * Set the subhal index as first byte of sensor handle and return this modified version.
 *
 * @param sensorHandle The sensor handle to modify.
 * @param subHalIndex The index in the hal proxy of the sub hal this sensor belongs to.
 *
 * @return The modified sensor handle.
 */
int32_t setSubHalIndex(int32_t sensorHandle, size_t subHalIndex) {
    return sensorHandle | (static_cast<int32_t>(subHalIndex) << kBitsAfterSubHalIndex);
}

/**
 * Extract the subHalIndex from sensorHandle.
 *
 * @param sensorHandle The sensorHandle to extract from.
 *
 * @return The subhal index.
 */
size_t extractSubHalIndex(int32_t sensorHandle) {
    return static_cast<size_t>(sensorHandle >> kBitsAfterSubHalIndex);
}

/**
 * Convert nanoseconds to milliseconds.
 *
 * @param nanos The nanoseconds input.
 *
 * @return The milliseconds count.
 */
int64_t msFromNs(int64_t nanos) {
    constexpr int64_t nanosecondsInAMillsecond = 1000000;
    return nanos / nanosecondsInAMillsecond;
}

bool patchXiaomiPickupSensor(V2_1::SensorInfo& sensor) {
    if (sensor.typeAsString != "xiaomi.sensor.pickup" &&
        sensor.typeAsString != "xiaomi pick up sensor") {
        return true;
    }

    /*
     * Implement only the wake-up version of this sensor.
     */
    if (!(sensor.flags & V1_0::SensorFlagBits::WAKE_UP)) {
        return false;
    }

    sensor.type = V2_1::SensorType::PICK_UP_GESTURE;
    sensor.typeAsString = SENSOR_STRING_TYPE_PICK_UP_GESTURE;
    sensor.maxRange = 1;

    return true;
}

HalProxy::HalProxy() {
    static const std::string kMultiHalConfigFiles[] = {"/vendor/etc/sensors/hals.conf",
                                                       "/odm/etc/sensors/hals.conf"};
    for (const std::string& configFile : kMultiHalConfigFiles) {
        initializeSubHalListFromConfigFile(configFile.c_str());
    }
    init();
}

HalProxy::HalProxy(std::vector<ISensorsSubHalV2_0*>& subHalList) {
    for (ISensorsSubHalV2_0* subHal : subHalList) {
        mSubHalList.push_back(std::make_unique<SubHalWrapperV2_0>(subHal));
    }

    init();
}

HalProxy::HalProxy(std::vector<ISensorsSubHalV2_0*>& subHalList,
                   std::vector<ISensorsSubHalV2_1*>& subHalListV2_1) {
    for (ISensorsSubHalV2_0* subHal : subHalList) {
        mSubHalList.push_back(std::make_unique<SubHalWrapperV2_0>(subHal));
    }

    for (ISensorsSubHalV2_1* subHal : subHalListV2_1) {
        mSubHalList.push_back(std::make_unique<SubHalWrapperV2_1>(subHal));
    }

    init();
}

HalProxy::~HalProxy() {
    stopThreads();
#if defined(XIAOMI_DUAL_ALS_FUSION)
    std::lock_guard<std::mutex> lock(gFusedAlsMutex);
    gFusedAls = {};
#endif
}

Return<void> HalProxy::getSensorsList_2_1(ISensorsV2_1::getSensorsList_2_1_cb _hidl_cb) {
    std::vector<V2_1::SensorInfo> sensors;
    for (const auto& iter : mSensors) {
        sensors.push_back(iter.second);
    }
    _hidl_cb(sensors);
    return Void();
}

Return<void> HalProxy::getSensorsList(ISensorsV2_0::getSensorsList_cb _hidl_cb) {
    std::vector<V1_0::SensorInfo> sensors;
    for (const auto& iter : mSensors) {
        if (iter.second.type != SensorType::HINGE_ANGLE) {
            sensors.push_back(convertToOldSensorInfo(iter.second));
        }
    }
    _hidl_cb(sensors);
    return Void();
}

Return<Result> HalProxy::setOperationMode(OperationMode mode) {
    Result result = Result::OK;
    size_t subHalIndex;
    for (subHalIndex = 0; subHalIndex < mSubHalList.size(); subHalIndex++) {
        result = mSubHalList[subHalIndex]->setOperationMode(mode);
        if (result != Result::OK) {
            ALOGE("setOperationMode failed for SubHal: %s",
                  mSubHalList[subHalIndex]->getName().c_str());
            break;
        }
    }

    if (result != Result::OK) {
        // Reset the subhal operation modes that have been flipped
        for (size_t i = 0; i < subHalIndex; i++) {
            mSubHalList[i]->setOperationMode(mCurrentOperationMode);
        }
    } else {
        mCurrentOperationMode = mode;
    }
    return result;
}

Return<Result> HalProxy::activate(int32_t sensorHandle, bool enabled) {
#if defined(XIAOMI_DUAL_ALS_FUSION)
    bool isFusedAls = false;
    bool isFusedAlsSource = false;
    bool sourceIsFront = false;
    {
        std::lock_guard<std::mutex> lock(gFusedAlsMutex);
        if (gFusedAls.configured && sensorHandle == gFusedAls.fusedHandle) {
            isFusedAls = true;
        } else if (gFusedAls.configured && sensorHandle == gFusedAls.frontHandle) {
            isFusedAlsSource = true;
            sourceIsFront = true;
        } else if (gFusedAls.configured && sensorHandle == gFusedAls.backHandle) {
            isFusedAlsSource = true;
        }
    }

    if (isFusedAls || isFusedAlsSource) {
        std::lock_guard<std::recursive_mutex> controlLock(gFusedAlsControlMutex);
        FusedAlsState oldState;
        {
            std::lock_guard<std::mutex> lock(gFusedAlsMutex);
            oldState = gFusedAls;
        }

        auto activateSource = [this](int32_t sourceHandle, bool sourceEnabled) -> Result {
            return getSubHalForSensorHandle(sourceHandle)->activate(
                    clearSubHalIndex(sourceHandle), sourceEnabled);
        };
        auto programBatch = [this](int32_t sourceHandle, const BatchParams& params) -> Result {
            if (!params.valid) return Result::OK;
            return getSubHalForSensorHandle(sourceHandle)->batch(
                    clearSubHalIndex(sourceHandle), params.samplingPeriodNs,
                    params.maxReportLatencyNs);
        };

        if (isFusedAls) {
            const BatchParams frontEffective = combineBatchParams(
                    oldState.fusedBatch,
                    oldState.frontExternallyActive ? oldState.frontExternalBatch : BatchParams{});
            const BatchParams backEffective = combineBatchParams(
                    oldState.fusedBatch,
                    oldState.backExternallyActive ? oldState.backExternalBatch : BatchParams{});

            if (enabled) {
                if (oldState.active && oldState.frontHardwareActive &&
                    oldState.backHardwareActive) {
                    return Result::OK;
                }

                {
                    std::lock_guard<std::mutex> lock(gFusedAlsMutex);
                    gFusedAls.enabling = true;
                    if (!oldState.active) {
                        gFusedAls.lastOutputValid = false;
                    }
                }

                BatchParams frontProgrammed = oldState.frontProgrammedBatch;
                BatchParams backProgrammed = oldState.backProgrammedBatch;
                if (!batchParamsEqual(frontEffective, frontProgrammed)) {
                    Result result = programBatch(oldState.frontHandle, frontEffective);
                    if (result != Result::OK) {
                        std::lock_guard<std::mutex> lock(gFusedAlsMutex);
                        gFusedAls.enabling = false;
                        return result;
                    }
                    frontProgrammed = frontEffective;
                }
                if (!batchParamsEqual(backEffective, backProgrammed)) {
                    Result result = programBatch(oldState.backHandle, backEffective);
                    if (result != Result::OK) {
                        if (oldState.frontProgrammedBatch.valid &&
                            !batchParamsEqual(frontProgrammed, oldState.frontProgrammedBatch)) {
                            if (programBatch(oldState.frontHandle, oldState.frontProgrammedBatch) ==
                                Result::OK) {
                                frontProgrammed = oldState.frontProgrammedBatch;
                            }
                        }
                        std::lock_guard<std::mutex> lock(gFusedAlsMutex);
                        gFusedAls.enabling = false;
                        gFusedAls.frontProgrammedBatch = frontProgrammed;
                        return result;
                    }
                    backProgrammed = backEffective;
                }

                bool frontHardwareActive = oldState.frontHardwareActive;
                bool backHardwareActive = oldState.backHardwareActive;
                const bool enabledFrontThisCall = !frontHardwareActive;
                if (enabledFrontThisCall) {
                    Result result = activateSource(oldState.frontHandle, true);
                    if (result != Result::OK) {
                        if (oldState.frontProgrammedBatch.valid &&
                            !batchParamsEqual(frontProgrammed, oldState.frontProgrammedBatch) &&
                            programBatch(oldState.frontHandle, oldState.frontProgrammedBatch) ==
                                    Result::OK) {
                            frontProgrammed = oldState.frontProgrammedBatch;
                        }
                        if (oldState.backProgrammedBatch.valid &&
                            !batchParamsEqual(backProgrammed, oldState.backProgrammedBatch) &&
                            programBatch(oldState.backHandle, oldState.backProgrammedBatch) ==
                                    Result::OK) {
                            backProgrammed = oldState.backProgrammedBatch;
                        }
                        std::lock_guard<std::mutex> lock(gFusedAlsMutex);
                        gFusedAls.enabling = false;
                        gFusedAls.frontProgrammedBatch = frontProgrammed;
                        gFusedAls.backProgrammedBatch = backProgrammed;
                        restoreFusedEventStateLocked(oldState);
                        return result;
                    }
                    frontHardwareActive = true;
                }

                if (!backHardwareActive) {
                    Result result = activateSource(oldState.backHandle, true);
                    if (result != Result::OK) {
                        if (enabledFrontThisCall &&
                            activateSource(oldState.frontHandle, false) == Result::OK) {
                            frontHardwareActive = false;
                        }
                        if (oldState.frontProgrammedBatch.valid &&
                            !batchParamsEqual(frontProgrammed, oldState.frontProgrammedBatch) &&
                            programBatch(oldState.frontHandle, oldState.frontProgrammedBatch) ==
                                    Result::OK) {
                            frontProgrammed = oldState.frontProgrammedBatch;
                        }
                        if (oldState.backProgrammedBatch.valid &&
                            !batchParamsEqual(backProgrammed, oldState.backProgrammedBatch) &&
                            programBatch(oldState.backHandle, oldState.backProgrammedBatch) ==
                                    Result::OK) {
                            backProgrammed = oldState.backProgrammedBatch;
                        }
                        std::lock_guard<std::mutex> lock(gFusedAlsMutex);
                        gFusedAls.enabling = false;
                        gFusedAls.frontHardwareActive = frontHardwareActive;
                        gFusedAls.frontProgrammedBatch = frontProgrammed;
                        gFusedAls.backProgrammedBatch = backProgrammed;
                        restoreFusedEventStateLocked(oldState);
                        return result;
                    }
                    backHardwareActive = true;
                }

                std::lock_guard<std::mutex> lock(gFusedAlsMutex);
                gFusedAls.frontHardwareActive = frontHardwareActive;
                gFusedAls.backHardwareActive = backHardwareActive;
                gFusedAls.frontProgrammedBatch = frontProgrammed;
                gFusedAls.backProgrammedBatch = backProgrammed;
                gFusedAls.active = true;
                gFusedAls.enabling = false;

                if ((gFusedAls.frontValueValid || gFusedAls.backValueValid) &&
                    !gFusedAls.lastOutputValid) {
                    const float fusedLux = gFusedAls.frontValueValid && gFusedAls.backValueValid
                            ? std::max(gFusedAls.frontLux, gFusedAls.backLux)
                            : gFusedAls.frontValueValid ? gFusedAls.frontLux : gFusedAls.backLux;
                    const int64_t timestamp = gFusedAls.frontValueValid &&
                                    gFusedAls.backValueValid
                            ? std::max(gFusedAls.frontTimestamp, gFusedAls.backTimestamp)
                            : gFusedAls.frontValueValid ? gFusedAls.frontTimestamp
                                                       : gFusedAls.backTimestamp;
                    const Event fusedEvent = makeFusedAlsEventLocked(fusedLux, timestamp);
                    std::lock_guard<std::mutex> queueLock(mEventQueueWriteMutex);
                    if (!enqueueEventsLocked(
                                {fusedEvent}, mEventQueue.get(), mEventQueueFlag, mSensors,
                                &mPendingWriteEventsQueue, &mSizePendingWriteEventsQueue,
                                &mMostEventsObservedPendingWriteEventsQueue,
                                kMaxSizePendingWriteEventsQueue, &mEventQueueWriteCV)) {
                        gFusedAls.lastOutputValid = false;
                        ALOGE("Failed to queue initial fused ALS event");
                    }
                }
                return Result::OK;
            }

            if (!oldState.active &&
                oldState.frontHardwareActive == oldState.frontExternallyActive &&
                oldState.backHardwareActive == oldState.backExternallyActive) {
                return Result::OK;
            }

            bool frontHardwareActive = oldState.frontHardwareActive;
            bool backHardwareActive = oldState.backHardwareActive;
            bool disabledFrontThisCall = false;
            if (frontHardwareActive && !oldState.frontExternallyActive) {
                Result result = activateSource(oldState.frontHandle, false);
                if (result != Result::OK) return result;
                frontHardwareActive = false;
                disabledFrontThisCall = true;
            }
            if (backHardwareActive && !oldState.backExternallyActive) {
                Result result = activateSource(oldState.backHandle, false);
                if (result != Result::OK) {
                    if (disabledFrontThisCall &&
                        activateSource(oldState.frontHandle, true) == Result::OK) {
                        frontHardwareActive = true;
                    }
                    std::lock_guard<std::mutex> lock(gFusedAlsMutex);
                    gFusedAls.frontHardwareActive = frontHardwareActive;
                    return result;
                }
                backHardwareActive = false;
            }

            BatchParams frontProgrammed = oldState.frontProgrammedBatch;
            BatchParams backProgrammed = oldState.backProgrammedBatch;
            if (oldState.frontExternallyActive && oldState.frontExternalBatch.valid &&
                !batchParamsEqual(frontProgrammed, oldState.frontExternalBatch) &&
                programBatch(oldState.frontHandle, oldState.frontExternalBatch) == Result::OK) {
                frontProgrammed = oldState.frontExternalBatch;
            }
            if (oldState.backExternallyActive && oldState.backExternalBatch.valid &&
                !batchParamsEqual(backProgrammed, oldState.backExternalBatch) &&
                programBatch(oldState.backHandle, oldState.backExternalBatch) == Result::OK) {
                backProgrammed = oldState.backExternalBatch;
            }

            std::lock_guard<std::mutex> lock(gFusedAlsMutex);
            gFusedAls.active = false;
            gFusedAls.enabling = false;
            gFusedAls.frontHardwareActive = frontHardwareActive;
            gFusedAls.backHardwareActive = backHardwareActive;
            gFusedAls.frontProgrammedBatch = frontProgrammed;
            gFusedAls.backProgrammedBatch = backProgrammed;
            if (!frontHardwareActive) gFusedAls.frontValueValid = false;
            if (!backHardwareActive) gFusedAls.backValueValid = false;
            gFusedAls.lastOutputValid = false;
            return Result::OK;
        }

        const bool externallyActive = sourceIsFront ? oldState.frontExternallyActive
                                                    : oldState.backExternallyActive;
        const bool hardwareActive = sourceIsFront ? oldState.frontHardwareActive
                                                  : oldState.backHardwareActive;
        const int32_t sourceHandle = sourceIsFront ? oldState.frontHandle : oldState.backHandle;
        const BatchParams externalBatch = sourceIsFront ? oldState.frontExternalBatch
                                                        : oldState.backExternalBatch;
        const BatchParams oldProgrammed = sourceIsFront ? oldState.frontProgrammedBatch
                                                        : oldState.backProgrammedBatch;

        if (enabled) {
            if (externallyActive && hardwareActive) return Result::OK;
            const BatchParams effective = combineBatchParams(
                    externalBatch, oldState.active ? oldState.fusedBatch : BatchParams{});
            BatchParams programmed = oldProgrammed;
            if (!batchParamsEqual(effective, programmed)) {
                Result result = programBatch(sourceHandle, effective);
                if (result != Result::OK) return result;
                programmed = effective;
            }

            if (!hardwareActive) {
                {
                    std::lock_guard<std::mutex> lock(gFusedAlsMutex);
                    if (sourceIsFront) {
                        gFusedAls.frontExternalEnabling = true;
                        gFusedAls.frontPendingActivationEvents.clear();
                        gFusedAls.frontProgrammedBatch = programmed;
                    } else {
                        gFusedAls.backExternalEnabling = true;
                        gFusedAls.backPendingActivationEvents.clear();
                        gFusedAls.backProgrammedBatch = programmed;
                    }
                }
                Result result = activateSource(sourceHandle, true);
                if (result != Result::OK) {
                    if (oldProgrammed.valid && !batchParamsEqual(programmed, oldProgrammed) &&
                        programBatch(sourceHandle, oldProgrammed) == Result::OK) {
                        programmed = oldProgrammed;
                    }
                    std::lock_guard<std::mutex> lock(gFusedAlsMutex);
                    if (sourceIsFront) {
                        gFusedAls.frontExternalEnabling = false;
                        gFusedAls.frontExternallyActive = externallyActive;
                        gFusedAls.frontProgrammedBatch = programmed;
                        gFusedAls.frontValueValid = oldState.frontValueValid;
                        gFusedAls.frontLux = oldState.frontLux;
                        gFusedAls.frontTimestamp = oldState.frontTimestamp;
                        gFusedAls.frontPendingActivationEvents.clear();
                    } else {
                        gFusedAls.backExternalEnabling = false;
                        gFusedAls.backExternallyActive = externallyActive;
                        gFusedAls.backProgrammedBatch = programmed;
                        gFusedAls.backValueValid = oldState.backValueValid;
                        gFusedAls.backLux = oldState.backLux;
                        gFusedAls.backTimestamp = oldState.backTimestamp;
                        gFusedAls.backPendingActivationEvents.clear();
                    }
                    return result;
                }
            }

            std::lock_guard<std::mutex> lock(gFusedAlsMutex);
            std::vector<Event> pendingEvents;
            if (sourceIsFront) {
                gFusedAls.frontExternalEnabling = false;
                gFusedAls.frontExternallyActive = true;
                gFusedAls.frontHardwareActive = true;
                pendingEvents = std::move(gFusedAls.frontPendingActivationEvents);
            } else {
                gFusedAls.backExternalEnabling = false;
                gFusedAls.backExternallyActive = true;
                gFusedAls.backHardwareActive = true;
                pendingEvents = std::move(gFusedAls.backPendingActivationEvents);
            }
            if (hardwareActive && pendingEvents.empty()) {
                const bool valueValid = sourceIsFront ? gFusedAls.frontValueValid
                                                      : gFusedAls.backValueValid;
                if (valueValid) {
                    Event currentValue{};
                    currentValue.timestamp = getTimeNow();
                    currentValue.sensorHandle = sourceHandle;
                    currentValue.sensorType = sourceIsFront ? gFusedAls.frontSensorType
                                                            : gFusedAls.backSensorType;
                    if (sourceIsFront) {
                        currentValue.u.scalar = gFusedAls.frontLux;
                    } else {
                        currentValue.u.data[0] = gFusedAls.backLux;
                    }
                    pendingEvents.push_back(currentValue);
                }
            }
            if (!pendingEvents.empty()) {
                std::lock_guard<std::mutex> queueLock(mEventQueueWriteMutex);
                if (!enqueueEventsLocked(
                            pendingEvents, mEventQueue.get(), mEventQueueFlag, mSensors,
                            &mPendingWriteEventsQueue, &mSizePendingWriteEventsQueue,
                            &mMostEventsObservedPendingWriteEventsQueue,
                            kMaxSizePendingWriteEventsQueue, &mEventQueueWriteCV)) {
                    ALOGE("Failed to queue initial physical ALS events");
                }
            }
            return Result::OK;
        }

        if (!externallyActive) return Result::OK;
        if (oldState.active) {
            const BatchParams effective = oldState.fusedBatch;
            BatchParams programmed = oldProgrammed;
            if (effective.valid && !batchParamsEqual(effective, programmed) &&
                programBatch(sourceHandle, effective) == Result::OK) {
                programmed = effective;
            }
            std::lock_guard<std::mutex> lock(gFusedAlsMutex);
            if (sourceIsFront) {
                gFusedAls.frontExternallyActive = false;
                gFusedAls.frontProgrammedBatch = programmed;
            } else {
                gFusedAls.backExternallyActive = false;
                gFusedAls.backProgrammedBatch = programmed;
            }
            return Result::OK;
        }

        Result result = hardwareActive ? activateSource(sourceHandle, false) : Result::OK;
        if (result != Result::OK) return result;
        std::lock_guard<std::mutex> lock(gFusedAlsMutex);
        if (sourceIsFront) {
            gFusedAls.frontExternallyActive = false;
            gFusedAls.frontHardwareActive = false;
            gFusedAls.frontValueValid = false;
        } else {
            gFusedAls.backExternallyActive = false;
            gFusedAls.backHardwareActive = false;
            gFusedAls.backValueValid = false;
        }
        return Result::OK;
    }
#endif

    if (!isSubHalIndexValid(sensorHandle)) {
        return Result::BAD_VALUE;
    }
    Result result = getSubHalForSensorHandle(sensorHandle)
            ->activate(clearSubHalIndex(sensorHandle), enabled);

    return result;
}

Return<Result> HalProxy::initialize_2_1(
        const ::android::hardware::MQDescriptorSync<V2_1::Event>& eventQueueDescriptor,
        const ::android::hardware::MQDescriptorSync<uint32_t>& wakeLockDescriptor,
        const sp<V2_1::ISensorsCallback>& sensorsCallback) {
    sp<ISensorsCallbackWrapperBase> dynamicCallback =
            new ISensorsCallbackWrapperV2_1(sensorsCallback);

    // Create the Event FMQ from the eventQueueDescriptor. Reset the read/write positions.
    auto eventQueue =
            std::make_unique<EventMessageQueueV2_1>(eventQueueDescriptor, true /* resetPointers */);
    std::unique_ptr<EventMessageQueueWrapperBase> queue =
            std::make_unique<EventMessageQueueWrapperV2_1>(eventQueue);

    // Create the Wake Lock FMQ from the wakeLockDescriptor. Reset the read/write positions.
    auto hidlWakeLockQueue =
            std::make_unique<WakeLockMessageQueue>(wakeLockDescriptor, true /* resetPointers */);
    std::unique_ptr<WakeLockMessageQueueWrapperBase> wakeLockQueue =
            std::make_unique<WakeLockMessageQueueWrapperHidl>(hidlWakeLockQueue);

    return initializeCommon(queue, wakeLockQueue, dynamicCallback);
}

Return<Result> HalProxy::initialize(
        const ::android::hardware::MQDescriptorSync<V1_0::Event>& eventQueueDescriptor,
        const ::android::hardware::MQDescriptorSync<uint32_t>& wakeLockDescriptor,
        const sp<V2_0::ISensorsCallback>& sensorsCallback) {
    sp<ISensorsCallbackWrapperBase> dynamicCallback =
            new ISensorsCallbackWrapperV2_0(sensorsCallback);

    // Create the Event FMQ from the eventQueueDescriptor. Reset the read/write positions.
    auto eventQueue =
            std::make_unique<EventMessageQueueV2_0>(eventQueueDescriptor, true /* resetPointers */);
    std::unique_ptr<EventMessageQueueWrapperBase> queue =
            std::make_unique<EventMessageQueueWrapperV1_0>(eventQueue);

    // Create the Wake Lock FMQ from the wakeLockDescriptor. Reset the read/write positions.
    auto hidlWakeLockQueue =
            std::make_unique<WakeLockMessageQueue>(wakeLockDescriptor, true /* resetPointers */);
    std::unique_ptr<WakeLockMessageQueueWrapperBase> wakeLockQueue =
            std::make_unique<WakeLockMessageQueueWrapperHidl>(hidlWakeLockQueue);

    return initializeCommon(queue, wakeLockQueue, dynamicCallback);
}

Return<Result> HalProxy::initializeCommon(
        std::unique_ptr<EventMessageQueueWrapperBase>& eventQueue,
        std::unique_ptr<WakeLockMessageQueueWrapperBase>& wakeLockQueue,
        const sp<ISensorsCallbackWrapperBase>& sensorsCallback) {
    Result result = Result::OK;
#if defined(XIAOMI_DUAL_ALS_FUSION)
    std::lock_guard<std::recursive_mutex> controlLock(gFusedAlsControlMutex);
#endif

    stopThreads();
    resetSharedWakelock();

    // So that the pending write events queue can be cleared safely and when we start threads
    // again we do not get new events until after initialize resets the subhals.
    disableAllSensors();

#if defined(XIAOMI_DUAL_ALS_FUSION)
    {
        std::lock_guard<std::mutex> lock(gFusedAlsMutex);
        gFusedAls.active = false;
        gFusedAls.enabling = false;
        gFusedAls.frontExternalEnabling = false;
        gFusedAls.backExternalEnabling = false;
        gFusedAls.frontExternallyActive = false;
        gFusedAls.backExternallyActive = false;
        gFusedAls.frontHardwareActive = false;
        gFusedAls.backHardwareActive = false;
        gFusedAls.frontValueValid = false;
        gFusedAls.backValueValid = false;
        gFusedAls.lastOutputValid = false;
        gFusedAls.fusedBatch = {};
        gFusedAls.frontExternalBatch = {};
        gFusedAls.backExternalBatch = {};
        gFusedAls.frontProgrammedBatch = {};
        gFusedAls.backProgrammedBatch = {};
        gFusedAls.frontPendingActivationEvents.clear();
        gFusedAls.backPendingActivationEvents.clear();
        gFusedAls.nextFlushId = 1;
        gFusedAls.frontFlushRequests.clear();
        gFusedAls.backFlushRequests.clear();
        gFusedAls.fusedFlushRequests.clear();
    }
#endif

    // Clears the queue if any events were pending write before.
    mPendingWriteEventsQueue = std::queue<std::pair<std::vector<V2_1::Event>, size_t>>();
    mSizePendingWriteEventsQueue = 0;

    // Clears previously connected dynamic sensors
    mDynamicSensors.clear();

    mDynamicSensorsCallback = sensorsCallback;

    // Create the Event FMQ from the eventQueueDescriptor. Reset the read/write positions.
    mEventQueue = std::move(eventQueue);

    // Create the Wake Lock FMQ that is used by the framework to communicate whenever WAKE_UP
    // events have been successfully read and handled by the framework.
    mWakeLockQueue = std::move(wakeLockQueue);

    if (mEventQueueFlag != nullptr) {
        EventFlag::deleteEventFlag(&mEventQueueFlag);
    }
    if (mWakelockQueueFlag != nullptr) {
        EventFlag::deleteEventFlag(&mWakelockQueueFlag);
    }
    if (EventFlag::createEventFlag(mEventQueue->getEventFlagWord(), &mEventQueueFlag) != OK) {
        result = Result::BAD_VALUE;
    }
    if (EventFlag::createEventFlag(mWakeLockQueue->getEventFlagWord(), &mWakelockQueueFlag) != OK) {
        result = Result::BAD_VALUE;
    }
    if (!mDynamicSensorsCallback || !mEventQueue || !mWakeLockQueue || mEventQueueFlag == nullptr) {
        result = Result::BAD_VALUE;
    }

    mThreadsRun.store(true);

    mPendingWritesThread = std::thread(startPendingWritesThread, this);
    mWakelockThread = std::thread(startWakelockThread, this);

    for (size_t i = 0; i < mSubHalList.size(); i++) {
        Result currRes = mSubHalList[i]->initialize(this, this, i);
        if (currRes != Result::OK) {
            result = currRes;
            ALOGE("Subhal '%s' failed to initialize with reason %" PRId32 ".",
                  mSubHalList[i]->getName().c_str(), static_cast<int32_t>(currRes));
        }
    }

    mCurrentOperationMode = OperationMode::NORMAL;

    return result;
}

Return<Result> HalProxy::batch(int32_t sensorHandle, int64_t samplingPeriodNs,
                               int64_t maxReportLatencyNs) {
#if defined(XIAOMI_DUAL_ALS_FUSION)
    bool isFusedAls = false;
    bool isFusedAlsSource = false;
    bool sourceIsFront = false;
    {
        std::lock_guard<std::mutex> lock(gFusedAlsMutex);
        if (gFusedAls.configured && sensorHandle == gFusedAls.fusedHandle) {
            isFusedAls = true;
        } else if (gFusedAls.configured && sensorHandle == gFusedAls.frontHandle) {
            isFusedAlsSource = true;
            sourceIsFront = true;
        } else if (gFusedAls.configured && sensorHandle == gFusedAls.backHandle) {
            isFusedAlsSource = true;
        }
    }

    if (isFusedAls || isFusedAlsSource) {
        if (samplingPeriodNs < 0 || maxReportLatencyNs < 0) return Result::BAD_VALUE;

        std::lock_guard<std::recursive_mutex> controlLock(gFusedAlsControlMutex);
        FusedAlsState oldState;
        {
            std::lock_guard<std::mutex> lock(gFusedAlsMutex);
            oldState = gFusedAls;
        }

        const BatchParams requested{
                true,
                samplingPeriodNs,
                isFusedAls ? 0 : maxReportLatencyNs,
        };
        auto programBatch = [this](int32_t sourceHandle, const BatchParams& params) -> Result {
            return getSubHalForSensorHandle(sourceHandle)->batch(
                    clearSubHalIndex(sourceHandle), params.samplingPeriodNs,
                    params.maxReportLatencyNs);
        };

        if (isFusedAls) {
            // Do not disturb an active physical client until the fused sensor is activated.
            if (!oldState.active &&
                (oldState.frontExternallyActive || oldState.backExternallyActive)) {
                std::lock_guard<std::mutex> lock(gFusedAlsMutex);
                gFusedAls.fusedBatch = requested;
                return Result::OK;
            }

            const BatchParams frontEffective = combineBatchParams(
                    requested,
                    oldState.frontExternallyActive ? oldState.frontExternalBatch : BatchParams{});
            const BatchParams backEffective = combineBatchParams(
                    requested,
                    oldState.backExternallyActive ? oldState.backExternalBatch : BatchParams{});
            BatchParams frontProgrammed = oldState.frontProgrammedBatch;
            BatchParams backProgrammed = oldState.backProgrammedBatch;

            if (!batchParamsEqual(frontEffective, frontProgrammed)) {
                Result result = programBatch(oldState.frontHandle, frontEffective);
                if (result != Result::OK) return result;
                frontProgrammed = frontEffective;
            }
            if (!batchParamsEqual(backEffective, backProgrammed)) {
                Result result = programBatch(oldState.backHandle, backEffective);
                if (result != Result::OK) {
                    if (oldState.frontProgrammedBatch.valid &&
                        !batchParamsEqual(frontProgrammed, oldState.frontProgrammedBatch) &&
                        programBatch(oldState.frontHandle, oldState.frontProgrammedBatch) ==
                                Result::OK) {
                        frontProgrammed = oldState.frontProgrammedBatch;
                    }
                    std::lock_guard<std::mutex> lock(gFusedAlsMutex);
                    gFusedAls.frontProgrammedBatch = frontProgrammed;
                    return result;
                }
                backProgrammed = backEffective;
            }

            std::lock_guard<std::mutex> lock(gFusedAlsMutex);
            gFusedAls.fusedBatch = requested;
            gFusedAls.frontProgrammedBatch = frontProgrammed;
            gFusedAls.backProgrammedBatch = backProgrammed;
            return Result::OK;
        }

        const bool externallyActive = sourceIsFront ? oldState.frontExternallyActive
                                                    : oldState.backExternallyActive;
        if (!externallyActive && oldState.active) {
            std::lock_guard<std::mutex> lock(gFusedAlsMutex);
            if (sourceIsFront) {
                gFusedAls.frontExternalBatch = requested;
            } else {
                gFusedAls.backExternalBatch = requested;
            }
            return Result::OK;
        }

        const int32_t sourceHandle = sourceIsFront ? oldState.frontHandle : oldState.backHandle;
        const BatchParams oldProgrammed = sourceIsFront ? oldState.frontProgrammedBatch
                                                        : oldState.backProgrammedBatch;
        const BatchParams effective = combineBatchParams(
                requested, oldState.active ? oldState.fusedBatch : BatchParams{});
        Result result = batchParamsEqual(effective, oldProgrammed)
                ? Result::OK
                : programBatch(sourceHandle, effective);
        if (result != Result::OK) return result;

        std::lock_guard<std::mutex> lock(gFusedAlsMutex);
        if (sourceIsFront) {
            gFusedAls.frontExternalBatch = requested;
            gFusedAls.frontProgrammedBatch = effective;
        } else {
            gFusedAls.backExternalBatch = requested;
            gFusedAls.backProgrammedBatch = effective;
        }
        return Result::OK;
    }
#endif

    if (!isSubHalIndexValid(sensorHandle)) {
        return Result::BAD_VALUE;
    }
    return getSubHalForSensorHandle(sensorHandle)
            ->batch(clearSubHalIndex(sensorHandle), samplingPeriodNs, maxReportLatencyNs);
}

Return<Result> HalProxy::flush(int32_t sensorHandle) {
#if defined(XIAOMI_DUAL_ALS_FUSION)
    bool isFusedAls = false;
    bool isFusedAlsSource = false;
    bool sourceIsFront = false;
    {
        std::lock_guard<std::mutex> lock(gFusedAlsMutex);
        isFusedAls = gFusedAls.configured && sensorHandle == gFusedAls.fusedHandle;
        if (gFusedAls.configured && sensorHandle == gFusedAls.frontHandle) {
            isFusedAlsSource = true;
            sourceIsFront = true;
        } else if (gFusedAls.configured && sensorHandle == gFusedAls.backHandle) {
            isFusedAlsSource = true;
        }
    }
    if (isFusedAls) {
        std::lock_guard<std::recursive_mutex> controlLock(gFusedAlsControlMutex);
        int32_t frontHandle;
        int32_t backHandle;
        uint64_t flushId;
        {
            std::lock_guard<std::mutex> lock(gFusedAlsMutex);
            if (!gFusedAls.active) return Result::BAD_VALUE;
            frontHandle = gFusedAls.frontHandle;
            backHandle = gFusedAls.backHandle;
            flushId = gFusedAls.nextFlushId++;
            gFusedAls.fusedFlushRequests.push_back({flushId, false, false});
            gFusedAls.frontFlushRequests.push_back({flushId, true});
            gFusedAls.backFlushRequests.push_back({flushId, true});
        }

        auto cancelFlush = [flushId](bool frontFlushAccepted) {
            std::lock_guard<std::mutex> lock(gFusedAlsMutex);
            auto removeById = [flushId](auto* requests) {
                requests->erase(
                        std::remove_if(requests->begin(), requests->end(),
                                       [flushId](const auto& request) {
                                           return request.id == flushId;
                                       }),
                        requests->end());
            };
            // A successful source flush cannot be cancelled. Keep its entry as a tombstone so
            // the eventual callback cannot be mistaken for a later request.
            if (!frontFlushAccepted) removeById(&gFusedAls.frontFlushRequests);
            removeById(&gFusedAls.backFlushRequests);
            removeById(&gFusedAls.fusedFlushRequests);
        };

        Result result = getSubHalForSensorHandle(frontHandle)->flush(
                clearSubHalIndex(frontHandle));
        if (result != Result::OK) {
            cancelFlush(false);
            return result;
        }
        result = getSubHalForSensorHandle(backHandle)->flush(clearSubHalIndex(backHandle));
        if (result != Result::OK) {
            cancelFlush(true);
            return result;
        }
        return Result::OK;
    }
    if (isFusedAlsSource) {
        std::lock_guard<std::recursive_mutex> controlLock(gFusedAlsControlMutex);
        uint64_t flushId;
        {
            std::lock_guard<std::mutex> lock(gFusedAlsMutex);
            const bool externallyActive = sourceIsFront ? gFusedAls.frontExternallyActive
                                                        : gFusedAls.backExternallyActive;
            if (!externallyActive) return Result::BAD_VALUE;
            flushId = gFusedAls.nextFlushId++;
            std::deque<FlushRequest>& requests = sourceIsFront
                    ? gFusedAls.frontFlushRequests
                    : gFusedAls.backFlushRequests;
            requests.push_back({flushId, false});
        }

        Result result = getSubHalForSensorHandle(sensorHandle)->flush(
                clearSubHalIndex(sensorHandle));
        if (result != Result::OK) {
            std::lock_guard<std::mutex> lock(gFusedAlsMutex);
            std::deque<FlushRequest>& requests = sourceIsFront
                    ? gFusedAls.frontFlushRequests
                    : gFusedAls.backFlushRequests;
            requests.erase(
                    std::remove_if(requests.begin(), requests.end(),
                                   [flushId](const FlushRequest& request) {
                                       return request.id == flushId;
                                   }),
                    requests.end());
        }
        return result;
    }
#endif

    if (!isSubHalIndexValid(sensorHandle)) {
        return Result::BAD_VALUE;
    }
    return getSubHalForSensorHandle(sensorHandle)->flush(clearSubHalIndex(sensorHandle));
}

Return<Result> HalProxy::injectSensorData_2_1(const V2_1::Event& event) {
    return injectSensorData(convertToOldEvent(event));
}

Return<Result> HalProxy::injectSensorData(const V1_0::Event& event) {
#if defined(XIAOMI_DUAL_ALS_FUSION)
    {
        std::lock_guard<std::mutex> lock(gFusedAlsMutex);
        if (gFusedAls.configured && event.sensorHandle == gFusedAls.fusedHandle) {
            return Result::BAD_VALUE;
        }
    }
#endif

    Result result = Result::OK;
    if (mCurrentOperationMode == OperationMode::NORMAL &&
        event.sensorType != V1_0::SensorType::ADDITIONAL_INFO) {
        ALOGE("An event with type != ADDITIONAL_INFO passed to injectSensorData while operation"
              " mode was NORMAL.");
        result = Result::BAD_VALUE;
    }
    if (result == Result::OK) {
        V1_0::Event subHalEvent = event;
        if (!isSubHalIndexValid(event.sensorHandle)) {
            return Result::BAD_VALUE;
        }
        subHalEvent.sensorHandle = clearSubHalIndex(event.sensorHandle);
        result = getSubHalForSensorHandle(event.sensorHandle)
                         ->injectSensorData(convertToNewEvent(subHalEvent));
    }
    return result;
}

Return<void> HalProxy::registerDirectChannel(const SharedMemInfo& mem,
                                             ISensorsV2_0::registerDirectChannel_cb _hidl_cb) {
    if (mDirectChannelSubHal == nullptr) {
        _hidl_cb(Result::INVALID_OPERATION, -1 /* channelHandle */);
    } else {
        mDirectChannelSubHal->registerDirectChannel(mem, _hidl_cb);
    }
    return Return<void>();
}

Return<Result> HalProxy::unregisterDirectChannel(int32_t channelHandle) {
    Result result;
    if (mDirectChannelSubHal == nullptr) {
        result = Result::INVALID_OPERATION;
    } else {
        result = mDirectChannelSubHal->unregisterDirectChannel(channelHandle);
    }
    return result;
}

Return<void> HalProxy::configDirectReport(int32_t sensorHandle, int32_t channelHandle,
                                           RateLevel rate,
                                           ISensorsV2_0::configDirectReport_cb _hidl_cb) {
#if defined(XIAOMI_DUAL_ALS_FUSION)
    {
        std::lock_guard<std::mutex> lock(gFusedAlsMutex);
        if (gFusedAls.configured && sensorHandle == gFusedAls.fusedHandle) {
            _hidl_cb(Result::INVALID_OPERATION, -1 /* reportToken */);
            return Void();
        }
    }
#endif

    if (mDirectChannelSubHal == nullptr) {
        _hidl_cb(Result::INVALID_OPERATION, -1 /* reportToken */);
    } else if (sensorHandle == -1 && rate != RateLevel::STOP) {
        _hidl_cb(Result::BAD_VALUE, -1 /* reportToken */);
    } else {
        // -1 denotes all sensors should be disabled
        if (sensorHandle != -1) {
            sensorHandle = clearSubHalIndex(sensorHandle);
        }
        mDirectChannelSubHal->configDirectReport(sensorHandle, channelHandle, rate, _hidl_cb);
    }
    return Return<void>();
}

Return<void> HalProxy::debug(const hidl_handle& fd, const hidl_vec<hidl_string>& args) {
    if (fd.getNativeHandle() == nullptr || fd->numFds < 1) {
        ALOGE("%s: missing fd for writing", __FUNCTION__);
        return Void();
    }

    int writeFd = fd->data[0];

    std::ostringstream stream;
    stream << "===HalProxy===" << std::endl;
    stream << "Internal values:" << std::endl;
    stream << "  Threads are running: " << (mThreadsRun.load() ? "true" : "false") << std::endl;
    int64_t now = getTimeNow();
    stream << "  Wakelock timeout start time: " << msFromNs(now - mWakelockTimeoutStartTime)
           << " ms ago" << std::endl;
    stream << "  Wakelock timeout reset time: " << msFromNs(now - mWakelockTimeoutResetTime)
           << " ms ago" << std::endl;
    // TODO(b/142969448): Add logging for history of wakelock acquisition per subhal.
    stream << "  Wakelock ref count: " << mWakelockRefCount << std::endl;
    stream << "  # of events on pending write writes queue: " << mSizePendingWriteEventsQueue
           << std::endl;
    stream << " Most events seen on pending write events queue: "
           << mMostEventsObservedPendingWriteEventsQueue << std::endl;
    if (!mPendingWriteEventsQueue.empty()) {
        stream << "  Size of events list on front of pending writes queue: "
               << mPendingWriteEventsQueue.front().first.size() << std::endl;
    }
    stream << "  # of non-dynamic sensors across all subhals: " << mSensors.size() << std::endl;
    stream << "  # of dynamic sensors across all subhals: " << mDynamicSensors.size() << std::endl;
    stream << "SubHals (" << mSubHalList.size() << "):" << std::endl;
    for (auto& subHal : mSubHalList) {
        stream << "  Name: " << subHal->getName() << std::endl;
        stream << "  Debug dump: " << std::endl;
        android::base::WriteStringToFd(stream.str(), writeFd);
        subHal->debug(fd, args);
        stream.str("");
        stream << std::endl;
    }
    android::base::WriteStringToFd(stream.str(), writeFd);
    return Return<void>();
}

Return<void> HalProxy::onDynamicSensorsConnected(const hidl_vec<SensorInfo>& dynamicSensorsAdded,
                                                 int32_t subHalIndex) {
    std::vector<SensorInfo> sensors;
    {
        std::lock_guard<std::mutex> lock(mDynamicSensorsMutex);
        for (SensorInfo sensor : dynamicSensorsAdded) {
            if (!subHalIndexIsClear(sensor.sensorHandle)) {
                ALOGE("Dynamic sensor added %s had sensorHandle with first byte not 0.",
                      sensor.name.c_str());
            } else {
                sensor.sensorHandle = setSubHalIndex(sensor.sensorHandle, subHalIndex);
                mDynamicSensors[sensor.sensorHandle] = sensor;
                sensors.push_back(sensor);
            }
        }
    }
    mDynamicSensorsCallback->onDynamicSensorsConnected(sensors);
    return Return<void>();
}

Return<void> HalProxy::onDynamicSensorsDisconnected(
        const hidl_vec<int32_t>& dynamicSensorHandlesRemoved, int32_t subHalIndex) {
    // TODO(b/143302327): Block this call until all pending events are flushed from queue
    std::vector<int32_t> sensorHandles;
    {
        std::lock_guard<std::mutex> lock(mDynamicSensorsMutex);
        for (int32_t sensorHandle : dynamicSensorHandlesRemoved) {
            if (!subHalIndexIsClear(sensorHandle)) {
                ALOGE("Dynamic sensorHandle removed had first byte not 0.");
            } else {
                sensorHandle = setSubHalIndex(sensorHandle, subHalIndex);
                if (mDynamicSensors.find(sensorHandle) != mDynamicSensors.end()) {
                    mDynamicSensors.erase(sensorHandle);
                    sensorHandles.push_back(sensorHandle);
                }
            }
        }
    }
    mDynamicSensorsCallback->onDynamicSensorsDisconnected(sensorHandles);
    return Return<void>();
}

void HalProxy::initializeSubHalListFromConfigFile(const char* configFileName) {
    std::ifstream subHalConfigStream(configFileName);
    if (!subHalConfigStream) {
        ALOGE("Failed to load subHal config file: %s", configFileName);
    } else {
        std::string subHalLibraryFile;
        while (subHalConfigStream >> subHalLibraryFile) {
            void* handle = getHandleForSubHalSharedObject(subHalLibraryFile);
            if (handle == nullptr) {
                ALOGE("dlopen failed for library: %s", subHalLibraryFile.c_str());
            } else {
                SensorsHalGetSubHalFunc* sensorsHalGetSubHalPtr =
                        (SensorsHalGetSubHalFunc*)dlsym(handle, "sensorsHalGetSubHal");
                if (sensorsHalGetSubHalPtr != nullptr) {
                    std::function<SensorsHalGetSubHalFunc> sensorsHalGetSubHal =
                            *sensorsHalGetSubHalPtr;
                    uint32_t version;
                    ISensorsSubHalV2_0* subHal = sensorsHalGetSubHal(&version);
                    if (version != SUB_HAL_2_0_VERSION) {
                        ALOGE("SubHal version was not 2.0 for library: %s",
                              subHalLibraryFile.c_str());
                    } else {
                        ALOGV("Loaded SubHal from library: %s", subHalLibraryFile.c_str());
                        mSubHalList.push_back(std::make_unique<SubHalWrapperV2_0>(subHal));
                    }
                } else {
                    SensorsHalGetSubHalV2_1Func* getSubHalV2_1Ptr =
                            (SensorsHalGetSubHalV2_1Func*)dlsym(handle, "sensorsHalGetSubHal_2_1");

                    if (getSubHalV2_1Ptr == nullptr) {
                        ALOGE("Failed to locate sensorsHalGetSubHal function for library: %s",
                              subHalLibraryFile.c_str());
                    } else {
                        std::function<SensorsHalGetSubHalV2_1Func> sensorsHalGetSubHal_2_1 =
                                *getSubHalV2_1Ptr;
                        uint32_t version;
                        ISensorsSubHalV2_1* subHal = sensorsHalGetSubHal_2_1(&version);
                        if (version != SUB_HAL_2_1_VERSION) {
                            ALOGE("SubHal version was not 2.1 for library: %s",
                                  subHalLibraryFile.c_str());
                        } else {
                            ALOGV("Loaded SubHal from library: %s", subHalLibraryFile.c_str());
                            mSubHalList.push_back(std::make_unique<SubHalWrapperV2_1>(subHal));
                        }
                    }
                }
            }
        }
    }
}

void HalProxy::initializeSensorList() {
    for (size_t subHalIndex = 0; subHalIndex < mSubHalList.size(); subHalIndex++) {
        auto result = mSubHalList[subHalIndex]->getSensorsList([&](const auto& list) {
            for (SensorInfo sensor : list) {
                if (!subHalIndexIsClear(sensor.sensorHandle)) {
                    ALOGE("SubHal sensorHandle's first byte was not 0");
                } else {
                    ALOGV("Loaded sensor: %s", sensor.name.c_str());
                    sensor.sensorHandle = setSubHalIndex(sensor.sensorHandle, subHalIndex);
                    setDirectChannelFlags(&sensor, mSubHalList[subHalIndex]);
                    bool keep = patchXiaomiPickupSensor(sensor);
                    if (!keep) {
                        continue;
                    }

                    mSensors[sensor.sensorHandle] = sensor;
                }
            }
        });
        if (!result.isOk()) {
            ALOGE("getSensorsList call failed for SubHal: %s",
                  mSubHalList[subHalIndex]->getName().c_str());
        }
    }
}

void* HalProxy::getHandleForSubHalSharedObject(const std::string& filename) {
    static const std::string kSubHalShareObjectLocations[] = {
            "",  // Default locations will be searched
#ifdef __LP64__
            "/vendor/lib64/hw/", "/odm/lib64/hw/"
#else
            "/vendor/lib/hw/", "/odm/lib/hw/"
#endif
    };

    for (const std::string& dir : kSubHalShareObjectLocations) {
        void* handle = dlopen((dir + filename).c_str(), RTLD_NOW);
        if (handle != nullptr) {
            return handle;
        }
    }
    return nullptr;
}

void HalProxy::init() {
#if defined(XIAOMI_DUAL_ALS_FUSION)
    {
        std::lock_guard<std::mutex> lock(gFusedAlsMutex);
        gFusedAls = {};
    }
#endif

    initializeSensorList();

#if defined(XIAOMI_DUAL_ALS_FUSION)
    int32_t frontAlsHandle = -1;
    int32_t backAlsHandle = -1;
    V2_1::SensorInfo frontAlsInfo;
    V2_1::SensorInfo backAlsInfo;

    for (const auto& sensorEntry : mSensors) {
        const V2_1::SensorInfo& sensor = sensorEntry.second;
        if (sensor.flags & V1_0::SensorFlagBits::WAKE_UP) {
            continue;
        }
        if (frontAlsHandle == -1 && sensor.type == V2_1::SensorType::LIGHT) {
            frontAlsHandle = sensorEntry.first;
            frontAlsInfo = sensor;
        } else if (backAlsHandle == -1 && sensor.typeAsString == kBackAlsType) {
            backAlsHandle = sensorEntry.first;
            backAlsInfo = sensor;
        }
    }

    if (frontAlsHandle != -1 && backAlsHandle != -1 &&
        mSensors.find(kFusedAlsHandle) == mSensors.end()) {
        V2_1::SensorInfo fusedAlsInfo = frontAlsInfo;
        fusedAlsInfo.sensorHandle = kFusedAlsHandle;
        fusedAlsInfo.name = kFusedAlsName;
        fusedAlsInfo.vendor = "Xiaomi";
        fusedAlsInfo.version = 1;
        fusedAlsInfo.type = static_cast<V2_1::SensorType>(kFusedAlsSensorType);
        fusedAlsInfo.typeAsString = kFusedAlsType;
        fusedAlsInfo.maxRange = std::max(frontAlsInfo.maxRange, backAlsInfo.maxRange);
        fusedAlsInfo.power = frontAlsInfo.power + backAlsInfo.power;
        fusedAlsInfo.minDelay = 0;
        fusedAlsInfo.maxDelay = 0;
        fusedAlsInfo.fifoReservedEventCount = 0;
        fusedAlsInfo.fifoMaxEventCount = 0;
        fusedAlsInfo.requiredPermission = "";
        fusedAlsInfo.flags &= ~(static_cast<uint32_t>(V1_0::SensorFlagBits::MASK_DIRECT_REPORT) |
                                static_cast<uint32_t>(V1_0::SensorFlagBits::MASK_DIRECT_CHANNEL) |
                                static_cast<uint32_t>(V1_0::SensorFlagBits::DATA_INJECTION) |
                                static_cast<uint32_t>(V1_0::SensorFlagBits::ADDITIONAL_INFO) |
                                static_cast<uint32_t>(V1_0::SensorFlagBits::DYNAMIC_SENSOR) |
                                static_cast<uint32_t>(V1_0::SensorFlagBits::MASK_REPORTING_MODE));
        fusedAlsInfo.flags |= static_cast<uint32_t>(V1_0::SensorFlagBits::ON_CHANGE_MODE);
        mSensors.emplace(kFusedAlsHandle, fusedAlsInfo);

        {
            std::lock_guard<std::mutex> lock(gFusedAlsMutex);
            gFusedAls.fusedHandle = kFusedAlsHandle;
            gFusedAls.fusedSensorType = kFusedAlsSensorType;
            gFusedAls.frontHandle = frontAlsHandle;
            gFusedAls.backHandle = backAlsHandle;
            gFusedAls.frontSensorType = frontAlsInfo.type;
            gFusedAls.backSensorType = backAlsInfo.type;
            gFusedAls.configured = true;
        }

        ALOGI("Dual ALS fusion enabled: front handle=0x%08" PRIx32
              ", back handle=0x%08" PRIx32 ", fused handle=0x%08" PRIx32,
              static_cast<uint32_t>(frontAlsHandle), static_cast<uint32_t>(backAlsHandle),
              static_cast<uint32_t>(kFusedAlsHandle));
    } else {
        ALOGW("Dual ALS fusion unavailable: front handle=0x%08" PRIx32
              ", back handle=0x%08" PRIx32,
              static_cast<uint32_t>(frontAlsHandle), static_cast<uint32_t>(backAlsHandle));
    }
#endif
}

void HalProxy::stopThreads() {
    mThreadsRun.store(false);
    if (mEventQueueFlag != nullptr && mEventQueue != nullptr) {
        size_t numToRead = mEventQueue->availableToRead();
        std::vector<Event> events(numToRead);
        mEventQueue->read(events.data(), numToRead);
        mEventQueueFlag->wake(static_cast<uint32_t>(EventQueueFlagBits::EVENTS_READ));
    }
    if (mWakelockQueueFlag != nullptr && mWakeLockQueue != nullptr) {
        uint32_t kZero = 0;
        mWakeLockQueue->write(&kZero);
        mWakelockQueueFlag->wake(static_cast<uint32_t>(WakeLockQueueFlagBits::DATA_WRITTEN));
    }
    mWakelockCV.notify_one();
    mEventQueueWriteCV.notify_one();
    if (mPendingWritesThread.joinable()) {
        mPendingWritesThread.join();
    }
    if (mWakelockThread.joinable()) {
        mWakelockThread.join();
    }
}

void HalProxy::disableAllSensors() {
    for (const auto& sensorEntry : mSensors) {
        int32_t sensorHandle = sensorEntry.first;
        activate(sensorHandle, false /* enabled */);
    }
    std::lock_guard<std::mutex> dynamicSensorsLock(mDynamicSensorsMutex);
    for (const auto& sensorEntry : mDynamicSensors) {
        int32_t sensorHandle = sensorEntry.first;
        activate(sensorHandle, false /* enabled */);
    }
}

void HalProxy::startPendingWritesThread(HalProxy* halProxy) {
    halProxy->handlePendingWrites();
}

void HalProxy::handlePendingWrites() {
    // TODO(b/143302327): Find a way to optimize locking strategy maybe using two mutexes instead of
    // one.
    std::unique_lock<std::mutex> lock(mEventQueueWriteMutex);
    while (mThreadsRun.load()) {
        mEventQueueWriteCV.wait(
                lock, [&] { return !mPendingWriteEventsQueue.empty() || !mThreadsRun.load(); });
        if (mThreadsRun.load()) {
            std::vector<Event>& pendingWriteEvents = mPendingWriteEventsQueue.front().first;
            size_t numWakeupEvents = mPendingWriteEventsQueue.front().second;
            size_t eventQueueSize = mEventQueue->getQuantumCount();
            size_t numToWrite = std::min(pendingWriteEvents.size(), eventQueueSize);
            lock.unlock();
            if (!mEventQueue->writeBlocking(
                        pendingWriteEvents.data(), numToWrite,
                        static_cast<uint32_t>(EventQueueFlagBits::EVENTS_READ),
                        static_cast<uint32_t>(EventQueueFlagBits::READ_AND_PROCESS),
                        kPendingWriteTimeoutNs, mEventQueueFlag)) {
                ALOGE("Dropping %zu events after blockingWrite failed (is system_server running?).",
                      numToWrite);
                if (numWakeupEvents > 0) {
                    if (pendingWriteEvents.size() > eventQueueSize) {
                        decrementRefCountAndMaybeReleaseWakelock(
                                countNumWakeupEvents(pendingWriteEvents, eventQueueSize));
                    } else {
                        decrementRefCountAndMaybeReleaseWakelock(numWakeupEvents);
                    }
                }
            }
            lock.lock();
            mSizePendingWriteEventsQueue -= numToWrite;
            if (pendingWriteEvents.size() > eventQueueSize) {
                // TODO(b/143302327): Check if this erase operation is too inefficient. It will copy
                // all the events ahead of it down to fill gap off array at front after the erase.
                pendingWriteEvents.erase(pendingWriteEvents.begin(),
                                         pendingWriteEvents.begin() + eventQueueSize);
            } else {
                mPendingWriteEventsQueue.pop();
            }
        }
    }
}

void HalProxy::startWakelockThread(HalProxy* halProxy) {
    halProxy->handleWakelocks();
}

void HalProxy::handleWakelocks() {
    std::unique_lock<std::recursive_mutex> lock(mWakelockMutex);
    while (mThreadsRun.load()) {
        mWakelockCV.wait(lock, [&] { return mWakelockRefCount > 0 || !mThreadsRun.load(); });
        if (mThreadsRun.load()) {
            int64_t timeLeft;
            if (sharedWakelockDidTimeout(&timeLeft)) {
                resetSharedWakelock();
            } else {
                uint32_t numWakeLocksProcessed;
                lock.unlock();
                bool success = mWakeLockQueue->readBlocking(
                        &numWakeLocksProcessed, 1, 0,
                        static_cast<uint32_t>(WakeLockQueueFlagBits::DATA_WRITTEN), timeLeft);
                lock.lock();
                if (success) {
                    decrementRefCountAndMaybeReleaseWakelock(
                            static_cast<size_t>(numWakeLocksProcessed));
                }
            }
        }
    }
    resetSharedWakelock();
}

bool HalProxy::sharedWakelockDidTimeout(int64_t* timeLeft) {
    bool didTimeout;
    int64_t duration = getTimeNow() - mWakelockTimeoutStartTime;
    if (duration > kWakelockTimeoutNs) {
        didTimeout = true;
    } else {
        didTimeout = false;
        *timeLeft = kWakelockTimeoutNs - duration;
    }
    return didTimeout;
}

void HalProxy::resetSharedWakelock() {
    std::lock_guard<std::recursive_mutex> lockGuard(mWakelockMutex);
    decrementRefCountAndMaybeReleaseWakelock(mWakelockRefCount);
    mWakelockTimeoutResetTime = getTimeNow();
}

void HalProxy::postEventsToMessageQueue(const std::vector<Event>& events, size_t numWakeupEvents,
                                        V2_0::implementation::ScopedWakelock wakelock) {
#if defined(XIAOMI_DUAL_ALS_FUSION)
    std::vector<Event> eventsToPost(events);
    {
        std::lock_guard<std::mutex> fusedAlsLock(gFusedAlsMutex);
        if (gFusedAls.active) {
            bool sourceEventSeen = false;
            int64_t latestTimestamp = 0;
            for (const Event& event : events) {
                if (event.sensorHandle != gFusedAls.frontHandle &&
                    event.sensorHandle != gFusedAls.backHandle) {
                    continue;
                }

                const float lux = event.sensorHandle == gFusedAls.frontHandle
                        ? event.u.scalar
                        : event.u.data[0];
                if (!std::isfinite(lux) || lux < 0.0f) {
                    continue;
                }

                sourceEventSeen = true;
                latestTimestamp = std::max(latestTimestamp, event.timestamp);
                if (event.sensorHandle == gFusedAls.frontHandle) {
                    gFusedAls.frontLux = lux;
                    gFusedAls.frontValueValid = true;
                } else {
                    gFusedAls.backLux = lux;
                    gFusedAls.backValueValid = true;
                }
            }

            if (sourceEventSeen &&
                (gFusedAls.frontValueValid || gFusedAls.backValueValid)) {
                const float fusedLux = gFusedAls.frontValueValid && gFusedAls.backValueValid
                        ? std::max(gFusedAls.frontLux, gFusedAls.backLux)
                        : gFusedAls.frontValueValid ? gFusedAls.frontLux : gFusedAls.backLux;
                if (!gFusedAls.lastOutputValid || fusedLux != gFusedAls.lastOutputLux) {
                    Event fusedEvent{};
                    fusedEvent.timestamp = latestTimestamp;
                    fusedEvent.sensorHandle = gFusedAls.fusedHandle;
                    fusedEvent.sensorType = static_cast<V2_1::SensorType>(
                            gFusedAls.fusedSensorType);
                    fusedEvent.u.data[0] = fusedLux;
                    eventsToPost.push_back(fusedEvent);
                    gFusedAls.lastOutputLux = fusedLux;
                    gFusedAls.lastOutputValid = true;
                }
            }
        }
    }
#else
    const std::vector<Event>& eventsToPost = events;
#endif

    size_t numToWrite = 0;
    std::lock_guard<std::mutex> lock(mEventQueueWriteMutex);
    if (wakelock.isLocked()) {
        incrementRefCountAndMaybeAcquireWakelock(numWakeupEvents);
    }
    if (mPendingWriteEventsQueue.empty()) {
        numToWrite = std::min(eventsToPost.size(), mEventQueue->availableToWrite());
        if (numToWrite > 0) {
            if (mEventQueue->write(eventsToPost.data(), numToWrite)) {
                // TODO(b/143302327): While loop if mEventQueue->avaiableToWrite > 0 to possibly fit
                // in more writes immediately
                mEventQueueFlag->wake(static_cast<uint32_t>(EventQueueFlagBits::READ_AND_PROCESS));
            } else {
                numToWrite = 0;
            }
        }
    }
    size_t numLeft = eventsToPost.size() - numToWrite;
    if (numToWrite < eventsToPost.size() &&
        mSizePendingWriteEventsQueue + numLeft <= kMaxSizePendingWriteEventsQueue) {
        std::vector<Event> eventsLeft(eventsToPost.begin() + numToWrite, eventsToPost.end());
        mPendingWriteEventsQueue.push({eventsLeft, numWakeupEvents});
        mSizePendingWriteEventsQueue += numLeft;
        mMostEventsObservedPendingWriteEventsQueue =
                std::max(mMostEventsObservedPendingWriteEventsQueue, mSizePendingWriteEventsQueue);
        mEventQueueWriteCV.notify_one();
    }
}

bool HalProxy::incrementRefCountAndMaybeAcquireWakelock(size_t delta,
                                                        int64_t* timeoutStart /* = nullptr */) {
    if (!mThreadsRun.load()) return false;
    std::lock_guard<std::recursive_mutex> lockGuard(mWakelockMutex);
    if (mWakelockRefCount == 0) {
        acquire_wake_lock(PARTIAL_WAKE_LOCK, kWakelockName);
        mWakelockCV.notify_one();
    }
    mWakelockTimeoutStartTime = getTimeNow();
    mWakelockRefCount += delta;
    if (timeoutStart != nullptr) {
        *timeoutStart = mWakelockTimeoutStartTime;
    }
    return true;
}

void HalProxy::decrementRefCountAndMaybeReleaseWakelock(size_t delta,
                                                        int64_t timeoutStart /* = -1 */) {
    if (!mThreadsRun.load()) return;
    std::lock_guard<std::recursive_mutex> lockGuard(mWakelockMutex);
    if (delta > mWakelockRefCount) {
        ALOGE("Decrementing wakelock ref count by %zu when count is %zu", delta, mWakelockRefCount);
    }
    if (timeoutStart == -1) timeoutStart = mWakelockTimeoutResetTime;
    if (mWakelockRefCount == 0 || timeoutStart < mWakelockTimeoutResetTime) return;
    mWakelockRefCount -= std::min(mWakelockRefCount, delta);
    if (mWakelockRefCount == 0) {
        release_wake_lock(kWakelockName);
    }
}

void HalProxy::setDirectChannelFlags(SensorInfo* sensorInfo,
                                     std::shared_ptr<ISubHalWrapperBase> subHal) {
    bool sensorSupportsDirectChannel =
            (sensorInfo->flags & (V1_0::SensorFlagBits::MASK_DIRECT_REPORT |
                                  V1_0::SensorFlagBits::MASK_DIRECT_CHANNEL)) != 0;
    if (mDirectChannelSubHal == nullptr && sensorSupportsDirectChannel) {
        mDirectChannelSubHal = subHal;
    } else if (mDirectChannelSubHal != nullptr && subHal != mDirectChannelSubHal) {
        // disable direct channel capability for sensors in subHals that are not
        // the only one we will enable
        sensorInfo->flags &= ~(V1_0::SensorFlagBits::MASK_DIRECT_REPORT |
                               V1_0::SensorFlagBits::MASK_DIRECT_CHANNEL);
    }
}

std::shared_ptr<ISubHalWrapperBase> HalProxy::getSubHalForSensorHandle(int32_t sensorHandle) {
    return mSubHalList[extractSubHalIndex(sensorHandle)];
}

bool HalProxy::isSubHalIndexValid(int32_t sensorHandle) {
    return extractSubHalIndex(sensorHandle) < mSubHalList.size();
}

size_t HalProxy::countNumWakeupEvents(const std::vector<Event>& events, size_t n) {
    size_t numWakeupEvents = 0;
    for (size_t i = 0; i < n; i++) {
        int32_t sensorHandle = events[i].sensorHandle;
        if (mSensors[sensorHandle].flags & static_cast<uint32_t>(V1_0::SensorFlagBits::WAKE_UP)) {
            numWakeupEvents++;
        }
    }
    return numWakeupEvents;
}

int32_t HalProxy::clearSubHalIndex(int32_t sensorHandle) {
    return sensorHandle & (~kSensorHandleSubHalIndexMask);
}

bool HalProxy::subHalIndexIsClear(int32_t sensorHandle) {
    return (sensorHandle & kSensorHandleSubHalIndexMask) == 0;
}

}  // namespace implementation
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
