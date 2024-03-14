/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "common/sensor_alignment.h"

//#include "config/config_eeprom.h"
#include "config/config.h"
#include "config/feature.h"

//#include "drivers/dshot_command.h"
//#include "drivers/motor.h"
//#include "drivers/system.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "sensors/sensors.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/pid_init.h"
//#include "flight/rpm_filter.h"
//#include "flight/servos.h"
#include "flight/position.h"

#include "msp/msp_box.h"

#include "osd/osd.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

//#include "drivers/dshot.h"

static bool configIsDirty; /* someone indicated that the config is modified and it is not yet saved */

static bool rebootRequired = false;  // set if a config change requires a reboot to take effect

static bool eepromWriteInProgress = false;

pidProfile_t *currentPidProfile;

#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif

#ifndef DEBUG_MODE
#define DEBUG_MODE 0
#endif

#define DEFAULT_CPU_OVERCLOCK 0

#define SYSTEM_HSE_VALUE (HSE_VALUE/1000000U)


pilotConfig_t pilotConfig;

void pilotConfig_Init(void)
{
	pilotConfig.name[MAX_NAME_LENGTH + 1] = 0;
	pilotConfig.displayName[MAX_NAME_LENGTH + 1] = 0;
}

systemConfig_t systemConfig;

void systemConfig_Init(void)
{
	systemConfig.pidProfileIndex = 0;
	systemConfig.activeRateProfile = 0;
	systemConfig.debug_mode = DEBUG_MODE;
	systemConfig.task_statistics = true;
	systemConfig.rateProfile6PosSwitch = false;
	systemConfig.cpu_overclock = DEFAULT_CPU_OVERCLOCK;
	systemConfig.powerOnArmingGraceTime = 5;
	systemConfig.boardIdentifier[sizeof(TARGET_BOARD_IDENTIFIER) + 1] = 0;
	systemConfig.hseMhz = SYSTEM_HSE_VALUE;  // Only used for F4 and G4 targets
	systemConfig.configurationState = CONFIGURATION_STATE_DEFAULTS_BARE;
	systemConfig.enableStickArming = false;
}

bool isEepromWriteInProgress(void)
{
    return eepromWriteInProgress;
}

uint8_t getCurrentPidProfileIndex(void)
{
    return systemConfig.pidProfileIndex;
}

static void loadPidProfile(void)
{
    currentPidProfile = &pidProfiles[systemConfig.pidProfileIndex];
}

uint8_t getCurrentControlRateProfileIndex(void)
{
    return systemConfig.activeRateProfile;
}

uint16_t getCurrentMinthrottle(void)
{
    return motorConfig.minthrottle;
}

void resetConfig(void)
{
    //pgResetAll();

#if defined(USE_TARGET_CONFIG)
    targetConfiguration();
#endif
}

static void activateConfig(void)
{
    loadPidProfile();
    loadControlRateProfile();

    initRcProcessing();

    activeAdjustmentRangeReset();

    pidInit(currentPidProfile);

    rcControlsInit();

    failsafeReset();
#ifdef USE_ACC
    setAccelerationTrims(&accelerometerConfig.accZero);
    accInitFilters();
#endif

    imuConfigure(throttleCorrectionConfig.throttle_correction_angle, throttleCorrectionConfig.throttle_correction_value);

#if defined(USE_LED_STRIP_STATUS_MODE)
    reevaluateLedConfig();
#endif

    initActiveBoxIds();
}

static void adjustFilterLimit(uint16_t *parm, uint16_t resetValue)
{
    if (*parm > LPF_MAX_HZ) {
        *parm = resetValue;
    }
}

static void validateAndFixRatesSettings(void)
{
    for (unsigned profileIndex = 0; profileIndex < CONTROL_RATE_PROFILE_COUNT; profileIndex++) {
        const ratesType_e ratesType = controlRateProfiles[profileIndex].rates_type;
        for (unsigned axis = FD_ROLL; axis <= FD_YAW; axis++) {
            controlRateProfiles[profileIndex].rcRates[axis] = constrain(controlRateProfiles[profileIndex].rcRates[axis], 0, ratesSettingLimits[ratesType].rc_rate_limit);
            controlRateProfiles[profileIndex].rates[axis] = constrain(controlRateProfiles[profileIndex].rates[axis], 0, ratesSettingLimits[ratesType].srate_limit);
            controlRateProfiles[profileIndex].rcExpo[axis] = constrain(controlRateProfiles[profileIndex].rcExpo[axis], 0, ratesSettingLimits[ratesType].expo_limit);
        }
    }
}

static void validateAndFixPositionConfig(void)
{
    if (positionConfig.altNumSatsBaroFallback >= positionConfig.altNumSatsGpsUse) {
        positionConfig.altNumSatsGpsUse = POSITION_DEFAULT_ALT_NUM_SATS_GPS_USE;
        positionConfig.altNumSatsBaroFallback = POSITION_DEFAULT_ALT_NUM_SATS_BARO_FALLBACK;
    }
}

static void validateAndFixConfig(void)
{
#if !defined(USE_QUAD_MIXER_ONLY)
    // Reset unsupported mixer mode to default.
    // This check will be gone when motor/servo mixers are loaded dynamically
    // by configurator as a part of configuration procedure.

    mixerMode_e mixerMode = mixerConfig.mixerMode;

    if (!(mixerMode == MIXER_CUSTOM || mixerMode == MIXER_CUSTOM_AIRPLANE || mixerMode == MIXER_CUSTOM_TRI)) {
        if (mixers[mixerMode].motorCount && mixers[mixerMode].motor == NULL)
            mixerConfig.mixerMode = MIXER_CUSTOM;
#ifdef USE_SERVOS
        if (mixers[mixerMode].useServo && servoMixers[mixerMode].servoRuleCount == 0)
            mixerConfigMutable()->mixerMode = MIXER_CUSTOM_AIRPLANE;
#endif
    }
#endif

//    if (!isSerialConfigValid(serialConfig())) {
//        pgResetFn_serialConfig(serialConfigMutable());
//    }

#if defined(USE_GPS)
    const serialPortConfig_t *gpsSerial = findSerialPortConfig(FUNCTION_GPS);
    if (gpsConfig()->provider == GPS_MSP && gpsSerial) {
        serialRemovePort(gpsSerial->identifier);
    }
#endif
    if (
#if defined(USE_GPS)
        gpsConfig()->provider != GPS_MSP && !gpsSerial &&
#endif
        true) {
        featureDisableImmediate(FEATURE_GPS);
    }

    for (unsigned i = 0; i < PID_PROFILE_COUNT; i++) {
        //Fix filter settings to handle cases where an older configurator was used that
        //allowed higher cutoff limits from previous firmware versions.
        adjustFilterLimit(&pidProfiles[i].dterm_lpf1_static_hz, LPF_MAX_HZ);
        adjustFilterLimit(&pidProfiles[i].dterm_lpf2_static_hz, LPF_MAX_HZ);
        adjustFilterLimit(&pidProfiles[i].dterm_notch_hz, LPF_MAX_HZ);
        adjustFilterLimit(&pidProfiles[i].dterm_notch_cutoff, 0);

        //Prevent invalid notch cutoff
        if (pidProfiles[i].dterm_notch_cutoff >= pidProfiles[i].dterm_notch_hz) {
            pidProfiles[i].dterm_notch_hz = 0;
        }

#ifdef USE_DYN_LPF
        //Prevent invalid dynamic lowpass
        if (pidProfiles[i].dterm_lpf1_dyn_min_hz > pidProfiles[i].dterm_lpf1_dyn_max_hz) {
            pidProfiles[i].dterm_lpf1_dyn_min_hz = 0;
        }
#endif

        if (pidProfiles[i].motor_output_limit > 100 || pidProfiles[i].motor_output_limit == 0) {
            pidProfiles[i].motor_output_limit = 100;
        }

        if (pidProfiles[i].auto_profile_cell_count > MAX_AUTO_DETECT_CELL_COUNT || pidProfiles[i].auto_profile_cell_count < AUTO_PROFILE_CELL_COUNT_CHANGE) {
            pidProfiles[i].auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY;
        }

        // If the d_min value for any axis is >= the D gain then reset d_min to 0 for consistent Configurator behavior
        for (unsigned axis = 0; axis <= FD_YAW; axis++) {
            if (pidProfiles[i].d_min[axis] > pidProfiles[i].pid[axis].D) {
                pidProfiles[i].d_min[axis] = 0;
            }
        }

#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
        if (batteryConfig()->voltageMeterSource != VOLTAGE_METER_ADC) {
            pidProfilesMutable(i)->vbat_sag_compensation = 0;
        }
#endif
    }

    if (motorConfig.dev.motorPwmProtocol == PWM_TYPE_BRUSHED) {
        featureDisableImmediate(FEATURE_3D);

        if (motorConfig.mincommand < 1000) {
            motorConfig.mincommand = 1000;
        }
    }

    if ((motorConfig.dev.motorPwmProtocol == PWM_TYPE_STANDARD) && (motorConfig.dev.motorPwmRate > BRUSHLESS_MOTORS_PWM_RATE)) {
        motorConfig.dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
    }

    validateAndFixGyroConfig();

#if defined(USE_MAG)
    buildAlignmentFromStandardAlignment(&compassConfig.mag_customAlignment, compassConfig.mag_alignment);
#endif
    buildAlignmentFromStandardAlignment(&gyroConfig.customAlignment, gyroConfig.alignment);
#if defined(USE_MULTI_GYRO)
    buildAlignmentFromStandardAlignment(&gyroDeviceConfigMutable(1)->customAlignment, gyroDeviceConfig(1)->alignment);
#endif

#ifdef USE_ACC
    if (accelerometerConfig.accZero.values.roll != 0 ||
        accelerometerConfig.accZero.values.pitch != 0 ||
        accelerometerConfig.accZero.values.yaw != 0) {
        accelerometerConfig.accZero.values.calibrationCompleted = 1;
    }
#endif // USE_ACC

    if (!(featureIsConfigured(FEATURE_RX_PARALLEL_PWM) || featureIsConfigured(FEATURE_RX_PPM) || featureIsConfigured(FEATURE_RX_SERIAL) || featureIsConfigured(FEATURE_RX_MSP) || featureIsConfigured(FEATURE_RX_SPI))) {
        featureEnableImmediate(DEFAULT_RX_FEATURE);
    }

    if (featureIsConfigured(FEATURE_RX_PPM)) {
        featureDisableImmediate(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP | FEATURE_RX_SPI);
    }

    if (featureIsConfigured(FEATURE_RX_MSP)) {
        featureDisableImmediate(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM | FEATURE_RX_SPI);
    }

    if (featureIsConfigured(FEATURE_RX_SERIAL)) {
        featureDisableImmediate(FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP | FEATURE_RX_PPM | FEATURE_RX_SPI);
    }

#ifdef USE_RX_SPI
    if (featureIsConfigured(FEATURE_RX_SPI)) {
        featureDisableImmediate(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM | FEATURE_RX_MSP);
    }
#endif // USE_RX_SPI

    if (featureIsConfigured(FEATURE_RX_PARALLEL_PWM)) {
        featureDisableImmediate(FEATURE_RX_SERIAL | FEATURE_RX_MSP | FEATURE_RX_PPM | FEATURE_RX_SPI);
    }

#if defined(USE_ADC)
    if (featureIsConfigured(FEATURE_RSSI_ADC)) {
        rxConfig.rssi_channel = 0;
        rxConfig.rssi_src_frame_errors = false;
    } else
#endif
    if (rxConfig.rssi_channel
#if defined(USE_PWM) || defined(USE_PPM)
        || featureIsConfigured(FEATURE_RX_PPM) || featureIsConfigured(FEATURE_RX_PARALLEL_PWM)
#endif
        ) {
        rxConfig.rssi_src_frame_errors = false;
    }

    if (
        featureIsConfigured(FEATURE_3D) || !featureIsConfigured(FEATURE_GPS) || mixerModeIsFixedWing(mixerConfig.mixerMode)
#if !defined(USE_GPS) || !defined(USE_GPS_RESCUE)
        || true
#endif
        ) {

#ifdef USE_GPS_RESCUE
        if (failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE) {
            failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_DROP_IT;
        }
#endif

        if (isModeActivationConditionPresent(BOXGPSRESCUE)) {
            removeModeActivationCondition(BOXGPSRESCUE);
        }
    }

#if defined(USE_ESC_SENSOR)
    if (!findSerialPortConfig(FUNCTION_ESC_SENSOR)) {
        featureDisableImmediate(FEATURE_ESC_SENSOR);
    }
#endif

    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = &modeActivationConditions[i];

        if (mac->linkedTo) {
            if (mac->modeId == BOXARM || isModeActivationConditionLinked(mac->linkedTo)) {
                removeModeActivationCondition(mac->modeId);
            }
        }
    }

#if defined(USE_DSHOT_TELEMETRY) && defined(USE_DSHOT_BITBANG)
    if (motorConfig()->dev.motorPwmProtocol == PWM_TYPE_PROSHOT1000 && motorConfig()->dev.useDshotTelemetry &&
        motorConfig()->dev.useDshotBitbang == DSHOT_BITBANG_ON) {
        motorConfigMutable()->dev.useDshotBitbang = DSHOT_BITBANG_AUTO;
    }
#endif

#ifdef USE_ADC
    adcConfig.vbat.enabled = (batteryConfig.voltageMeterSource == VOLTAGE_METER_ADC);
    adcConfig.current.enabled = (batteryConfig.currentMeterSource == CURRENT_METER_ADC);

    // The FrSky D SPI RX sends RSSI_ADC_PIN (if configured) as A2
    adcConfig.rssi.enabled = featureIsEnabled(FEATURE_RSSI_ADC);
#ifdef USE_RX_SPI
    adcConfigMutable()->rssi.enabled |= (featureIsEnabled(FEATURE_RX_SPI) && rxSpiConfig()->rx_spi_protocol == RX_SPI_FRSKY_D);
#endif
#endif // USE_ADC


// clear features that are not supported.
// I have kept them all here in one place, some could be moved to sections of code above.

#ifndef USE_PPM
    featureDisableImmediate(FEATURE_RX_PPM);
#endif

#ifndef USE_SERIAL_RX
    featureDisableImmediate(FEATURE_RX_SERIAL);
#endif

#if !defined(USE_SOFTSERIAL1) && !defined(USE_SOFTSERIAL2)
    featureDisableImmediate(FEATURE_SOFTSERIAL);
#endif

#ifndef USE_RANGEFINDER
    featureDisableImmediate(FEATURE_RANGEFINDER);
#endif

#ifndef USE_TELEMETRY
    featureDisableImmediate(FEATURE_TELEMETRY);
#endif

#ifndef USE_PWM
    featureDisableImmediate(FEATURE_RX_PARALLEL_PWM);
#endif

#ifndef USE_RX_MSP
    featureDisableImmediate(FEATURE_RX_MSP);
#endif

#ifndef USE_LED_STRIP
    featureDisableImmediate(FEATURE_LED_STRIP);
#endif

#ifndef USE_DASHBOARD
    featureDisableImmediate(FEATURE_DASHBOARD);
#endif

#ifndef USE_OSD
    featureDisableImmediate(FEATURE_OSD);
#endif

#ifndef USE_SERVOS
    featureDisableImmediate(FEATURE_SERVO_TILT | FEATURE_CHANNEL_FORWARDING);
#endif

#ifndef USE_TRANSPONDER
    featureDisableImmediate(FEATURE_TRANSPONDER);
#endif

#ifndef USE_RX_SPI
    featureDisableImmediate(FEATURE_RX_SPI);
#endif

#ifndef USE_ESC_SENSOR
    featureDisableImmediate(FEATURE_ESC_SENSOR);
#endif

#if !defined(USE_ADC)
    featureDisableImmediate(FEATURE_RSSI_ADC);
#endif

#if defined(USE_BEEPER)
#ifdef USE_TIMER
    if (beeperDevConfig()->frequency && !timerGetConfiguredByTag(beeperDevConfig()->ioTag)) {
        beeperDevConfigMutable()->frequency = 0;
    }
#endif

    if (beeperConfig()->beeper_off_flags & ~BEEPER_ALLOWED_MODES) {
        beeperConfigMutable()->beeper_off_flags = 0;
    }

#ifdef USE_DSHOT
    if (beeperConfig()->dshotBeaconOffFlags & ~DSHOT_BEACON_ALLOWED_MODES) {
        beeperConfigMutable()->dshotBeaconOffFlags = 0;
    }

    if (beeperConfig()->dshotBeaconTone < DSHOT_CMD_BEACON1
        || beeperConfig()->dshotBeaconTone > DSHOT_CMD_BEACON5) {
        beeperConfigMutable()->dshotBeaconTone = DSHOT_CMD_BEACON1;
    }
#endif
#endif

    bool configuredMotorProtocolDshot = false;
    checkMotorProtocolEnabled(&motorConfig.dev, &configuredMotorProtocolDshot);
#if defined(USE_DSHOT)
    // If using DSHOT protocol disable unsynched PWM as it's meaningless
    if (configuredMotorProtocolDshot) {
        motorConfigMutable()->dev.useUnsyncedPwm = false;
    }

#if defined(USE_DSHOT_TELEMETRY)
    bool nChannelTimerUsed = false;
    for (unsigned i = 0; i < getMotorCount(); i++) {
        const ioTag_t tag = motorConfig()->dev.ioTags[i];
        if (tag) {
            const timerHardware_t *timer = timerGetConfiguredByTag(tag);
            if (timer && timer->output & TIMER_OUTPUT_N_CHANNEL) {
                nChannelTimerUsed = true;

                break;
            }
        }
    }

    if ((!configuredMotorProtocolDshot || (motorConfig()->dev.useDshotBitbang == DSHOT_BITBANG_OFF && (motorConfig()->dev.useBurstDshot == DSHOT_DMAR_ON || nChannelTimerUsed))) && motorConfig()->dev.useDshotTelemetry) {
        motorConfigMutable()->dev.useDshotTelemetry = false;
    }
#endif // USE_DSHOT_TELEMETRY
#endif // USE_DSHOT

#if defined(USE_OSD)
    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
         const uint16_t t = osdConfig.timers[i];
         if (OSD_TIMER_SRC(t) >= OSD_TIMER_SRC_COUNT ||
                 OSD_TIMER_PRECISION(t) >= OSD_TIMER_PREC_COUNT) {
             osdConfig.timers[i] = osdTimerDefault[i];
         }
     }
#endif

#if defined(USE_VTX_COMMON) && defined(USE_VTX_TABLE)
    // reset vtx band, channel, power if outside range specified by vtxtable
    if (vtxSettingsConfig()->channel > vtxTableConfig()->channels) {
        vtxSettingsConfigMutable()->channel = 0;
        if (vtxSettingsConfig()->band > 0) {
            vtxSettingsConfigMutable()->freq = 0; // band/channel determined frequency can't be valid anymore
        }
    }
    if (vtxSettingsConfig()->band > vtxTableConfig()->bands) {
        vtxSettingsConfigMutable()->band = 0;
        vtxSettingsConfigMutable()->freq = 0; // band/channel determined frequency can't be valid anymore
    }
    if (vtxSettingsConfig()->power > vtxTableConfig()->powerLevels) {
        vtxSettingsConfigMutable()->power = 0;
    }
#endif

    validateAndFixRatesSettings();  // constrain the various rates settings to limits imposed by the rates type

#if defined(USE_RX_MSP_OVERRIDE)
    if (!rxConfig()->msp_override_channels_mask) {
        removeModeActivationCondition(BOXMSPOVERRIDE);
    }

    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);
        if (mac->modeId == BOXMSPOVERRIDE && ((1 << (mac->auxChannelIndex) & (rxConfig()->msp_override_channels_mask)))) {
            rxConfigMutable()->msp_override_channels_mask &= ~(1 << (mac->auxChannelIndex + NON_AUX_CHANNEL_COUNT));
        }
    }
#endif

    //validateAndfixMotorOutputReordering(motorConfigMutable()->dev.motorOutputReordering, MAX_SUPPORTED_MOTORS);

    // validate that the minimum battery cell voltage is less than the maximum cell voltage
    // reset to defaults if not
    if (batteryConfig.vbatmincellvoltage >=  batteryConfig.vbatmaxcellvoltage) {
        batteryConfig.vbatmincellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MIN;
        batteryConfig.vbatmaxcellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MAX;
    }

#ifdef USE_MSP_DISPLAYPORT
    // validate that displayport_msp_serial is referencing a valid UART that actually has MSP enabled
    if (displayPortProfileMsp()->displayPortSerial != SERIAL_PORT_NONE) {
        const serialPortConfig_t *portConfig = serialFindPortConfiguration(displayPortProfileMsp()->displayPortSerial);
        if (!portConfig || !(portConfig->functionMask & FUNCTION_MSP)
#ifndef USE_MSP_PUSH_OVER_VCP
            || (portConfig->identifier == SERIAL_PORT_USB_VCP)
#endif
            ) {
            displayPortProfileMspMutable()->displayPortSerial = SERIAL_PORT_NONE;
        }
    }
#endif

#if defined(TARGET_VALIDATECONFIG)
    // This should be done at the end of the validation
    targetValidateConfiguration();
#endif

    validateAndFixPositionConfig();
}

void validateAndFixGyroConfig(void)
{
    // Fix gyro filter settings to handle cases where an older configurator was used that
    // allowed higher cutoff limits from previous firmware versions.
    adjustFilterLimit(&gyroConfig.gyro_lpf1_static_hz, LPF_MAX_HZ);
    adjustFilterLimit(&gyroConfig.gyro_lpf2_static_hz, LPF_MAX_HZ);
    adjustFilterLimit(&gyroConfig.gyro_soft_notch_hz_1, LPF_MAX_HZ);
    adjustFilterLimit(&gyroConfig.gyro_soft_notch_cutoff_1, 0);
    adjustFilterLimit(&gyroConfig.gyro_soft_notch_hz_2, LPF_MAX_HZ);
    adjustFilterLimit(&gyroConfig.gyro_soft_notch_cutoff_2, 0);

    // Prevent invalid notch cutoff
    if (gyroConfig.gyro_soft_notch_cutoff_1 >= gyroConfig.gyro_soft_notch_hz_1) {
        gyroConfig.gyro_soft_notch_hz_1 = 0;
    }
    if (gyroConfig.gyro_soft_notch_cutoff_2 >= gyroConfig.gyro_soft_notch_hz_2) {
        gyroConfig.gyro_soft_notch_hz_2 = 0;
    }
#ifdef USE_DYN_LPF
    //Prevent invalid dynamic lowpass filter
    if (gyroConfig.gyro_lpf1_dyn_min_hz > gyroConfig.gyro_lpf1_dyn_max_hz) {
        gyroConfig.gyro_lpf1_dyn_min_hz = 0;
    }
#endif

    if (gyro.sampleRateHz > 0) {
        float samplingTime = 1.0f / gyro.sampleRateHz;

        // check for looptime restrictions based on motor protocol. Motor times have safety margin
        float motorUpdateRestriction;

#if defined(STM32F411xE)
        /* If bidirectional DSHOT is being used on an F411 then force DSHOT300. The motor update restrictions then applied
         * will automatically consider the loop time and adjust pid_process_denom appropriately
         */
        if (motorConfig()->dev.useDshotTelemetry && (motorConfig()->dev.motorPwmProtocol == PWM_TYPE_DSHOT600)) {
            motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT300;
        }
#endif

        switch (motorConfig.dev.motorPwmProtocol) {
        case PWM_TYPE_STANDARD:
                motorUpdateRestriction = 1.0f / BRUSHLESS_MOTORS_PWM_RATE;
                break;
        case PWM_TYPE_ONESHOT125:
                motorUpdateRestriction = 0.0005f;
                break;
        case PWM_TYPE_ONESHOT42:
                motorUpdateRestriction = 0.0001f;
                break;
#ifdef USE_DSHOT
        case PWM_TYPE_DSHOT150:
                motorUpdateRestriction = 0.000250f;
                break;
        case PWM_TYPE_DSHOT300:
                motorUpdateRestriction = 0.0001f;
                break;
#endif
        default:
            motorUpdateRestriction = 0.00003125f;
            break;
        }

        if (motorConfig.dev.useUnsyncedPwm) {
            bool configuredMotorProtocolDshot = false;
            checkMotorProtocolEnabled(&motorConfig.dev, &configuredMotorProtocolDshot);
            // Prevent overriding the max rate of motors
            if (!configuredMotorProtocolDshot && motorConfig.dev.motorPwmProtocol != PWM_TYPE_STANDARD) {
                const uint32_t maxEscRate = lrintf(1.0f / motorUpdateRestriction);
                motorConfig.dev.motorPwmRate = MIN(motorConfig.dev.motorPwmRate, maxEscRate);
            }
        } else {
            const float pidLooptime = samplingTime * pidConfig.pid_process_denom;
            if (motorConfig.dev.useDshotTelemetry) {
                motorUpdateRestriction *= 2;
            }
            if (pidLooptime < motorUpdateRestriction) {
                uint8_t minPidProcessDenom = motorUpdateRestriction / samplingTime;
                if (motorUpdateRestriction / samplingTime > minPidProcessDenom) {
                    // if any fractional part then round up
                    minPidProcessDenom++;
                }
                minPidProcessDenom = constrain(minPidProcessDenom, 1, MAX_PID_PROCESS_DENOM);
                pidConfig.pid_process_denom = MAX(pidConfig.pid_process_denom, minPidProcessDenom);
            }
        }
    }

#ifdef USE_BLACKBOX
#ifndef USE_FLASHFS
    if (blackboxConfig()->device == BLACKBOX_DEVICE_FLASH) {
        blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
    }
#endif // USE_FLASHFS

    if (blackboxConfig()->device == BLACKBOX_DEVICE_SDCARD) {
#if defined(USE_SDCARD)
        if (!sdcardConfig()->mode)
#endif
        {
            blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
        }
    }
#endif // USE_BLACKBOX

    if (systemConfig.activeRateProfile >= CONTROL_RATE_PROFILE_COUNT) {
        systemConfig.activeRateProfile = 0;
    }
    loadControlRateProfile();

    if (systemConfig.pidProfileIndex >= PID_PROFILE_COUNT) {
        systemConfig.pidProfileIndex = 0;
    }
    loadPidProfile();
}

bool readEEPROM(void)
{
    suspendRxSignal();

    // Sanity check, read flash
    bool success = true;// = loadEEPROM();

    featureInit();

    validateAndFixConfig();

    activateConfig();

    resumeRxSignal();

    return success;
}

void writeUnmodifiedConfigToEEPROM(void)
{
    validateAndFixConfig();

    suspendRxSignal();
    eepromWriteInProgress = true;
    //writeConfigToEEPROM();
    eepromWriteInProgress = false;
    resumeRxSignal();
    configIsDirty = false;
}

void writeEEPROM(void)
{
#ifdef USE_RX_SPI
    rxSpiStop(); // some rx spi protocols use hardware timer, which needs to be stopped before writing to eeprom
#endif
    systemConfig.configurationState = CONFIGURATION_STATE_CONFIGURED;

    writeUnmodifiedConfigToEEPROM();
}

bool resetEEPROM(bool useCustomDefaults)
{
#if !defined(USE_CUSTOM_DEFAULTS)
    UNUSED(useCustomDefaults);
#else
    if (useCustomDefaults) {
        if (!resetConfigToCustomDefaults()) {
            return false;
        }
    } else
#endif
    {
        resetConfig();
    }

    writeUnmodifiedConfigToEEPROM();

    return true;
}

void isEEPROMStructureValid(void)
{
//    if (isEEPROMStructureValid()) {
//        return;
//    }
//    resetEEPROM(false);
}

void saveConfigAndNotify(void)
{
    // The write to EEPROM will cause a big delay in the current task, so ignore
    schedulerIgnoreTaskExecTime();

    writeEEPROM();
    readEEPROM();
    //beeperConfirmationBeeps(1);
}

void setConfigDirty(void)
{
    configIsDirty = true;
}

bool isConfigDirty(void)
{
    return configIsDirty;
}

void changePidProfileFromCellCount(uint8_t cellCount)
{
    if (currentPidProfile->auto_profile_cell_count == cellCount || currentPidProfile->auto_profile_cell_count == AUTO_PROFILE_CELL_COUNT_STAY) {
        return;
    }

    unsigned profileIndex = (systemConfig.pidProfileIndex + 1) % PID_PROFILE_COUNT;
    int matchingProfileIndex = -1;
    while (profileIndex != systemConfig.pidProfileIndex) {
        if (pidProfiles[profileIndex].auto_profile_cell_count == cellCount) {
            matchingProfileIndex = profileIndex;

            break;
        } else if (matchingProfileIndex < 0 && pidProfiles[profileIndex].auto_profile_cell_count == AUTO_PROFILE_CELL_COUNT_STAY) {
            matchingProfileIndex = profileIndex;
        }

        profileIndex = (profileIndex + 1) % PID_PROFILE_COUNT;
    }

    if (matchingProfileIndex >= 0) {
        changePidProfile(matchingProfileIndex);
    }
}

void changePidProfile(uint8_t pidProfileIndex)
{
    // The config switch will cause a big enough delay in the current task to upset the scheduler
    schedulerIgnoreTaskExecTime();

    if (pidProfileIndex < PID_PROFILE_COUNT) {
        systemConfig.pidProfileIndex = pidProfileIndex;
        loadPidProfile();

        pidInit(currentPidProfile);
        initEscEndpoints();
        mixerInitProfile();
    }

    //beeperConfirmationBeeps(pidProfileIndex + 1);
}

bool isSystemConfigured(void)
{
    return systemConfig.configurationState == CONFIGURATION_STATE_CONFIGURED;
}

void setRebootRequired(void)
{
    rebootRequired = true;
    //setArmingDisabled(ARMING_DISABLED_REBOOT_REQUIRED);
}

bool getRebootRequired(void)
{
    return rebootRequired;
}
