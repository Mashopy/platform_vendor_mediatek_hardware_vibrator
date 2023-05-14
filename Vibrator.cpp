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

#include "vibrator-impl/Vibrator.h"
#include <thread>

#undef LOG_TAG
#define LOG_TAG "Vibrator"

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

static constexpr int32_t kComposeDelayMaxMs = 1000;
static constexpr int32_t kComposeSizeMax = 256;
static constexpr int32_t kComposePwleSizeMax = 127;
static constexpr float kResonantFrequency = 150.0;
static constexpr float kQFactor = 11.0;
static constexpr int32_t COMPOSE_PWLE_PRIMITIVE_DURATION_MAX_MS = 16383;
static constexpr float PWLE_LEVEL_MIN = 0.0;
static constexpr float PWLE_LEVEL_MAX = 0.98256;
static constexpr float PWLE_FREQUENCY_RESOLUTION_HZ = 1.0;
static constexpr float PWLE_FREQUENCY_MIN_HZ = 140.0;
static constexpr float PWLE_FREQUENCY_MAX_HZ = 160.0;
// Use effect #1 in the waveform library for CLICK effect
static constexpr int32_t WAVEFORM_CLICK_EFFECT = 30;

// Use effect #2 in the waveform library for TICK effect
static constexpr int32_t WAVEFORM_TICK_EFFECT = 50;

// Use effect #3 in the waveform library for DOUBLE_CLICK effect
static constexpr int32_t WAVEFORM_DOUBLE_CLICK_EFFECT = 70;

// Use effect #4 in the waveform library for HEAVY_CLICK effect
static constexpr int32_t WAVEFORM_HEAVY_CLICK_EFFECT = 90;


#define TIMEOUT_STR_LEN         20

static const char THE_DEVICE[] = "/sys/class/timed_output/vibrator/enable";

static bool device_exists(const char *file) {
    int fd = -1;

    fd = TEMP_FAILURE_RETRY(open(file, O_RDWR));
    if(fd < 0) {
        return false;
    }

    close(fd);
    fd = -1;
    return true;
}

static bool vibra_exists() {
    return device_exists(THE_DEVICE);
}

static int write_value(const char *file, const char *value)
{
    int to_write = 0, written = -1, ret = 0;
    int fd = -1;

    fd = TEMP_FAILURE_RETRY(open(file, O_WRONLY));
    if (fd < 0) {
        return -errno;
    }

    to_write = strlen(value) + 1;
    written = TEMP_FAILURE_RETRY(write(fd, value, to_write));
    if (written == -1) {
        ret = -errno;
    } else if (written != to_write) {
        /* even though EAGAIN is an errno value that could be set
           by write() in some cases, none of them apply here.  So, this return
           value can be clearly identified when debugging and suggests the
           caller that it may try to call vibrator_on() again */
        ret = -EAGAIN;
    } else {
        ret = 0;
    }

    errno = 0;
    close(fd);
    fd = -1;

    return ret;
}

static int sendit(unsigned int timeout_ms)
{
    char value[TIMEOUT_STR_LEN]; /* large enough for millions of years */

    int bytes = snprintf(value, sizeof(value), "%u", timeout_ms);
    if (bytes >= sizeof(value)) return -EINVAL;
    return write_value(THE_DEVICE, value);
}

static int vibra_on(unsigned int timeout_ms)
{
    /* constant on, up to maximum allowed time */
    return sendit(timeout_ms);
}

static int vibra_off()
{
    return sendit(0);
}

static const char LED_DEVICE[] = "/sys/class/leds/vibrator";

static int write_led_file(const char *file, const char *value)
{
    char file_str[50];

    int bytes = snprintf(file_str, sizeof(file_str), "%s/%s", LED_DEVICE, file);
    if (bytes >= sizeof(file_str)) return -EINVAL;
    return write_value(file_str, value);
}

static bool vibra_led_exists()
{
    char file_str[50];

    int bytes = snprintf(file_str, sizeof(file_str), "%s/%s", LED_DEVICE, "activate");
    if (bytes >= sizeof(file_str)) return -EINVAL;
    return device_exists(file_str);
}

static int vibra_led_on( unsigned int timeout_ms)
{
    int ret;
    char value[TIMEOUT_STR_LEN]; /* large enough for millions of years */

    ret = write_led_file("state", "1");
    if (ret)
        return ret;

    int bytes = snprintf(value, sizeof(value), "%u\n", timeout_ms);
    if (bytes >= sizeof(value)) return -EINVAL;
    ret = write_led_file("duration", value);
    if (ret)
        return ret;

    return write_led_file("activate", "1");
}

static int vibra_led_off()
{
    return write_led_file("activate", "0");
}

ndk::ScopedAStatus Vibrator::getCapabilities(int32_t* _aidl_return) {
    LOG(INFO) << "Vibrator reporting capabilities";
    if (_aidl_return == nullptr) {
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
    }
#ifndef VIBR_EFFECT_SUPPORT
    *_aidl_return = 0;
#else
    *_aidl_return = IVibrator::CAP_ON_CALLBACK | IVibrator::CAP_PERFORM_CALLBACK |
                    IVibrator::CAP_AMPLITUDE_CONTROL | IVibrator::CAP_EXTERNAL_CONTROL |
                    IVibrator::CAP_EXTERNAL_AMPLITUDE_CONTROL | IVibrator::CAP_COMPOSE_EFFECTS |
                    IVibrator::CAP_ALWAYS_ON_CONTROL | IVibrator::CAP_GET_RESONANT_FREQUENCY |
                    IVibrator::CAP_GET_Q_FACTOR | IVibrator::CAP_FREQUENCY_CONTROL |
                    IVibrator::CAP_COMPOSE_PWLE_EFFECTS;
#endif

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::off() {
    ALOGD("Vibrator off");
    if (vibra_exists()) {
        ALOGD("Vibrator using timed_output");
        vibra_off();
    } else if (vibra_led_exists()) {
        ALOGD("Vibrator using LED trigger");
        vibra_led_off();
    } else {
        ALOGI("Vibrator device does not exist. Cannot start vibrator");
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::on(int32_t timeoutMs,
                                const std::shared_ptr<IVibratorCallback>& callback) {
    ALOGI("Vibrator on for timeoutMs: %d", timeoutMs);

    if (vibra_exists()) {
        ALOGD("Vibrator using timed_output");
        vibra_on(timeoutMs);
    } else if (vibra_led_exists()) {
        ALOGD("Vibrator using LED trigger");
        vibra_led_on(timeoutMs);
    } else {
        ALOGI("Vibrator device does not exist. Cannot start vibrator");
    }
    if (callback != nullptr) {
#ifndef VIBR_EFFECT_SUPPORT
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
#else
        std::thread([=] {
            ALOGD("Starting on on another thread");
            usleep(timeoutMs * 1000);
            ALOGD("Notifying on complete");
            if (!callback->onComplete().isOk()) {
                ALOGI("Failed to call onComplete");
            }
        }).detach();
    return ndk::ScopedAStatus::ok();
#endif
    } else {
        return ndk::ScopedAStatus::ok();
    }

}

ndk::ScopedAStatus Vibrator::perform(Effect effect, EffectStrength strength,
                                     const std::shared_ptr<IVibratorCallback>& callback,
                                     int32_t* _aidl_return) {
    uint32_t timeMS = 0;
    ndk::ScopedAStatus status;

    ALOGD("Vibrator perform %d, %d", (int)strength, (int)effect);
    if (callback != nullptr) {
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
    }

    if (effect != Effect::CLICK && effect != Effect::TICK
        && effect != Effect::TEXTURE_TICK && effect != Effect::DOUBLE_CLICK
        && effect != Effect::HEAVY_CLICK) {
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
    }
    if (strength != EffectStrength::LIGHT && strength != EffectStrength::MEDIUM &&
        strength != EffectStrength::STRONG) {
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
    }

    switch (effect) {
        case Effect::TEXTURE_TICK:
            timeMS = WAVEFORM_TICK_EFFECT;
            break;
        case Effect::CLICK:
            timeMS = WAVEFORM_CLICK_EFFECT;
            break;
        case Effect::DOUBLE_CLICK:
            timeMS = WAVEFORM_DOUBLE_CLICK_EFFECT;
            break;
        case Effect::TICK:
            timeMS = WAVEFORM_TICK_EFFECT;
            break;
        case Effect::HEAVY_CLICK:
            timeMS = WAVEFORM_HEAVY_CLICK_EFFECT;
            break;
        default:
           return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
    }

    status = on(timeMS, nullptr);
    if (!status.isOk()) {
        return status;
    } else {
        *_aidl_return = timeMS;
        return ndk::ScopedAStatus::ok();
    }
}

ndk::ScopedAStatus Vibrator::getSupportedEffects(std::vector<Effect>* _aidl_return) {

    *_aidl_return = {Effect::CLICK, Effect::TICK, Effect::TEXTURE_TICK,
        Effect::DOUBLE_CLICK, Effect::HEAVY_CLICK};

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::setAmplitude(float amplitude) {
    LOG(INFO) << "Vibrator set amplitude: " << amplitude;
#ifndef VIBR_EFFECT_SUPPORT
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
#else
    if (amplitude <= 0.0f || amplitude > 1.0f) {
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
    }
    return ndk::ScopedAStatus::ok();
#endif
}

ndk::ScopedAStatus Vibrator::setExternalControl(bool enabled) {
    LOG(INFO) << "Vibrator set external control: " << enabled;
#ifndef VIBR_EFFECT_SUPPORT
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
#else
    return ndk::ScopedAStatus::ok();
#endif
}

ndk::ScopedAStatus Vibrator::getCompositionDelayMax(int32_t* maxDelayMs) {
    *maxDelayMs = kComposeDelayMaxMs;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getCompositionSizeMax(int32_t* maxSize) {
    *maxSize = kComposeSizeMax;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getSupportedPrimitives(std::vector<CompositePrimitive>* supported) {
#ifndef VIBR_EFFECT_SUPPORT
    *supported = {};
#else
    *supported = {
            CompositePrimitive::NOOP,       CompositePrimitive::CLICK,
            CompositePrimitive::THUD,       CompositePrimitive::SPIN,
            CompositePrimitive::QUICK_RISE, CompositePrimitive::SLOW_RISE,
            CompositePrimitive::QUICK_FALL, CompositePrimitive::LIGHT_TICK,
            CompositePrimitive::LOW_TICK,
    };
#endif
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getPrimitiveDuration(CompositePrimitive primitive,
                                                  int32_t* durationMs) {
    if (primitive != CompositePrimitive::NOOP) {
        *durationMs = 100;
    } else {
        *durationMs = 0;
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::compose(const std::vector<CompositeEffect>& composite,
                                     const std::shared_ptr<IVibratorCallback>& callback) {
    if (composite.size() > kComposeSizeMax) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    std::vector<CompositePrimitive> supported;
    getSupportedPrimitives(&supported);

    for (auto& e : composite) {
        if (e.delayMs > kComposeDelayMaxMs) {
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
        }
        if (e.scale < 0.0f || e.scale > 1.0f) {
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
        }
        if (std::find(supported.begin(), supported.end(), e.primitive) == supported.end()) {
            return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
        }
    }

    std::thread([=] {
        LOG(INFO) << "Starting compose on another thread";

        for (auto& e : composite) {
            if (e.delayMs) {
                usleep(e.delayMs * 1000);
            }
            LOG(INFO) << "triggering primitive " << static_cast<int>(e.primitive) << " @ scale "
                      << e.scale;

            int32_t durationMs;
            getPrimitiveDuration(e.primitive, &durationMs);
            usleep(durationMs * 1000);
        }

        if (callback != nullptr) {
            LOG(INFO) << "Notifying perform complete";
            callback->onComplete();
        }
    }).detach();

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getSupportedAlwaysOnEffects(std::vector<Effect>* _aidl_return) {
    return getSupportedEffects(_aidl_return);
}

ndk::ScopedAStatus Vibrator::alwaysOnEnable(int32_t id, Effect effect, EffectStrength strength) {
    std::vector<Effect> effects;
    getSupportedAlwaysOnEffects(&effects);

    if (std::find(effects.begin(), effects.end(), effect) == effects.end()) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    } else {
        LOG(INFO) << "Enabling always-on ID " << id << " with " << toString(effect) << "/"
                  << toString(strength);
        return ndk::ScopedAStatus::ok();
    }
}

ndk::ScopedAStatus Vibrator::alwaysOnDisable(int32_t id) {
    LOG(INFO) << "Disabling always-on ID " << id;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getResonantFrequency(float *resonantFreqHz) {
    *resonantFreqHz = kResonantFrequency;
#ifndef VIBR_EFFECT_SUPPORT
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
#else
    return ndk::ScopedAStatus::ok();
#endif
}

ndk::ScopedAStatus Vibrator::getQFactor(float *qFactor) {
    *qFactor = kQFactor;
#ifndef VIBR_EFFECT_SUPPORT
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
#else
    return ndk::ScopedAStatus::ok();
#endif
}

ndk::ScopedAStatus Vibrator::getFrequencyResolution(float *freqResolutionHz) {
    *freqResolutionHz = PWLE_FREQUENCY_RESOLUTION_HZ;
#ifndef VIBR_EFFECT_SUPPORT
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
#else
    return ndk::ScopedAStatus::ok();
#endif
}

ndk::ScopedAStatus Vibrator::getFrequencyMinimum(float *freqMinimumHz) {
    *freqMinimumHz = PWLE_FREQUENCY_MIN_HZ;
#ifndef VIBR_EFFECT_SUPPORT
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
#else
    return ndk::ScopedAStatus::ok();
#endif
}

ndk::ScopedAStatus Vibrator::getBandwidthAmplitudeMap(std::vector<float> *_aidl_return) {
    // A valid array should be of size:
    //     (PWLE_FREQUENCY_MAX_HZ - PWLE_FREQUENCY_MIN_HZ) / PWLE_FREQUENCY_RESOLUTION_HZ
    *_aidl_return = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10,
                     0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.20};
#ifndef VIBR_EFFECT_SUPPORT
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
#else
    return ndk::ScopedAStatus::ok();
#endif
}

ndk::ScopedAStatus Vibrator::getPwlePrimitiveDurationMax(int32_t *durationMs) {
    *durationMs = COMPOSE_PWLE_PRIMITIVE_DURATION_MAX_MS;
#ifndef VIBR_EFFECT_SUPPORT
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
#else
    return ndk::ScopedAStatus::ok();
#endif
}

ndk::ScopedAStatus Vibrator::getPwleCompositionSizeMax(int32_t *maxSize) {
    *maxSize = kComposePwleSizeMax;
#ifndef VIBR_EFFECT_SUPPORT
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
#else
    return ndk::ScopedAStatus::ok();
#endif
}

ndk::ScopedAStatus Vibrator::getSupportedBraking(std::vector<Braking> *supported) {
    *supported = {
            Braking::NONE,
            Braking::CLAB,
    };
#ifndef VIBR_EFFECT_SUPPORT
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
#else
    return ndk::ScopedAStatus::ok();
#endif
}

void resetPreviousEndAmplitudeEndFrequency(float &prevEndAmplitude, float &prevEndFrequency) {
    const float reset = -1.0;
    prevEndAmplitude = reset;
    prevEndFrequency = reset;
}

void incrementIndex(int &index) {
    index += 1;
}

void constructActiveDefaults(std::ostringstream &pwleBuilder, const int &segmentIdx) {
    pwleBuilder << ",C" << segmentIdx << ":1";
    pwleBuilder << ",B" << segmentIdx << ":0";
    pwleBuilder << ",AR" << segmentIdx << ":0";
    pwleBuilder << ",V" << segmentIdx << ":0";
}

void constructActiveSegment(std::ostringstream &pwleBuilder, const int &segmentIdx, int duration,
                            float amplitude, float frequency) {
    pwleBuilder << ",T" << segmentIdx << ":" << duration;
    pwleBuilder << ",L" << segmentIdx << ":" << amplitude;
    pwleBuilder << ",F" << segmentIdx << ":" << frequency;
    constructActiveDefaults(pwleBuilder, segmentIdx);
}

void constructBrakingSegment(std::ostringstream &pwleBuilder, const int &segmentIdx, int duration,
                             Braking brakingType) {
    pwleBuilder << ",T" << segmentIdx << ":" << duration;
    pwleBuilder << ",L" << segmentIdx << ":" << 0;
    pwleBuilder << ",F" << segmentIdx << ":" << 0;
    pwleBuilder << ",C" << segmentIdx << ":0";
    pwleBuilder << ",B" << segmentIdx << ":"
                << static_cast<std::underlying_type<Braking>::type>(brakingType);
    pwleBuilder << ",AR" << segmentIdx << ":0";
    pwleBuilder << ",V" << segmentIdx << ":0";
}

ndk::ScopedAStatus Vibrator::composePwle(const std::vector<PrimitivePwle> &composite,
                                         const std::shared_ptr<IVibratorCallback> &callback) {
    std::ostringstream pwleBuilder;
    std::string pwleQueue;

    int compositionSizeMax;
    getPwleCompositionSizeMax(&compositionSizeMax);
    if (composite.size() <= 0 || composite.size() > compositionSizeMax) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    float prevEndAmplitude;
    float prevEndFrequency;
    resetPreviousEndAmplitudeEndFrequency(prevEndAmplitude, prevEndFrequency);

    int segmentIdx = 0;
    uint32_t totalDuration = 0;

    pwleBuilder << "S:0,WF:4,RP:0,WT:0";

    for (auto &e : composite) {
        switch (e.getTag()) {
            case PrimitivePwle::active: {
                auto active = e.get<PrimitivePwle::active>();
                if (active.duration < 0 ||
                    active.duration > COMPOSE_PWLE_PRIMITIVE_DURATION_MAX_MS) {
                    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
                }
                if (active.startAmplitude < PWLE_LEVEL_MIN ||
                    active.startAmplitude > PWLE_LEVEL_MAX ||
                    active.endAmplitude < PWLE_LEVEL_MIN || active.endAmplitude > PWLE_LEVEL_MAX) {
                    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
                }
                if (active.startFrequency < PWLE_FREQUENCY_MIN_HZ ||
                    active.startFrequency > PWLE_FREQUENCY_MAX_HZ ||
                    active.endFrequency < PWLE_FREQUENCY_MIN_HZ ||
                    active.endFrequency > PWLE_FREQUENCY_MAX_HZ) {
                    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
                }

                if (!((active.startAmplitude == prevEndAmplitude) &&
                      (active.startFrequency == prevEndFrequency))) {
                    constructActiveSegment(pwleBuilder, segmentIdx, 0, active.startAmplitude,
                                           active.startFrequency);
                    incrementIndex(segmentIdx);
                }

                constructActiveSegment(pwleBuilder, segmentIdx, active.duration,
                                       active.endAmplitude, active.endFrequency);
                incrementIndex(segmentIdx);

                prevEndAmplitude = active.endAmplitude;
                prevEndFrequency = active.endFrequency;
                totalDuration += active.duration;
                break;
            }
            case PrimitivePwle::braking: {
                auto braking = e.get<PrimitivePwle::braking>();
                if (braking.braking > Braking::CLAB) {
                    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
                }
                if (braking.duration > COMPOSE_PWLE_PRIMITIVE_DURATION_MAX_MS) {
                    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
                }

                constructBrakingSegment(pwleBuilder, segmentIdx, 0, braking.braking);
                incrementIndex(segmentIdx);

                constructBrakingSegment(pwleBuilder, segmentIdx, braking.duration, braking.braking);
                incrementIndex(segmentIdx);

                resetPreviousEndAmplitudeEndFrequency(prevEndAmplitude, prevEndFrequency);
                totalDuration += braking.duration;
                break;
            }
        }
    }

    std::thread([=] {
        LOG(INFO) << "Starting composePwle on another thread";
        usleep(totalDuration * 1000);
        if (callback != nullptr) {
            LOG(INFO) << "Notifying compose PWLE complete";
            callback->onComplete();
        }
    }).detach();

    return ndk::ScopedAStatus::ok();
}

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
