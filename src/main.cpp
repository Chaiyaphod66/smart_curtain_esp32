#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <BH1750.h>
#include <DHT.h>
#include <Preferences.h>
#include <esp_timer.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "secrets.h"

constexpr uint8_t MOTOR_ENA = 25;
constexpr uint8_t MOTOR_IN1 = 16;
constexpr uint8_t MOTOR_IN2 = 17;
constexpr uint8_t DHT_PIN = 4;
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t ENCODER_A_PIN = 32;
constexpr uint8_t ENCODER_B_PIN = 33;
constexpr uint8_t LIMIT_CLOSED_PIN = 27;
constexpr uint8_t LIMIT_OPEN_PIN = 26;

// FC-33 ของระบบนี้ส่ง HIGH เมื่อตรวจพบ (Active HIGH)
constexpr uint8_t LIMIT_ACTIVE_LEVEL = HIGH;
constexpr unsigned long LIMIT_DEBOUNCE_MS = 40;
constexpr bool LIMIT_SAFETY_ENABLED = true;
constexpr uint8_t LIMIT_BRAKE_DUTY = 255;
constexpr unsigned long LIMIT_BRAKE_DURATION_MS = 100;
constexpr unsigned long LIMIT_DIAGNOSTIC_SETTLE_MS = 1200;
// A starting-end limit may stay active briefly while the curtain leaves it.
// That exception must have both an encoder-distance and a time bound.
constexpr unsigned long DEPARTURE_LIMIT_RELEASE_TIMEOUT_MS = 400;
// Observed: the source sensor was active when the old 227-pulse guard tripped;
// a later status showed 467 pulses and the sensor inactive (4550-pulse travel).
// The exact release point is unknown. Keep the 400 ms bound and test this
// wider distance guard under supervision before using unattended Auto.
constexpr uint8_t DEPARTURE_LIMIT_RELEASE_TRAVEL_PERCENT = 12;
constexpr int32_t DEPARTURE_LIMIT_RELEASE_MAX_PULSES = 600;
constexpr int32_t DEPARTURE_WRONG_DIRECTION_PULSES = 4;
// เบรกเต็มกำลังช่วงสั้นเพื่อลดระยะไหลหลังถึงเป้าหมาย Encoder
constexpr uint8_t POSITION_BRAKE_DUTY = 255;
constexpr unsigned long POSITION_BRAKE_DURATION_MS = 80;
// ผลทดสอบจริงพบว่าชุดมอเตอร์ยังเคลื่อนต่อราว 5-6% หลังเริ่มสั่งเบรก
// จึงเริ่มเบรกล่วงหน้าแยกตามทิศทาง แล้วปล่อยให้แรงเฉื่อยพาไปถึงเป้าหมายจริง
constexpr uint8_t POSITION_OPEN_STOP_LEAD_PERCENT = 6;
constexpr uint8_t POSITION_CLOSE_STOP_LEAD_PERCENT = 5;

constexpr unsigned long SENSOR_INTERVAL_MS = 2500;
constexpr unsigned long LIGHT_SENSOR_RETRY_MS = 5000;
constexpr unsigned long ENCODER_REPORT_INTERVAL_MS = 250;

constexpr uint8_t PWM_CHANNEL = 0;
constexpr uint32_t PWM_FREQUENCY = 5000;
constexpr uint8_t PWM_RESOLUTION = 8;

constexpr uint8_t TEST_SPEED = 14;
// จากผลทดสอบจริง 700 ms เคลื่อนประมาณครึ่งระยะม่าน
// 250 ms จึงเป็นระยะขยับที่ปลอดภัยกว่าและใกล้ 20% ต่อครั้ง
constexpr unsigned long TEST_DURATION_MS = 250;
constexpr unsigned long LIMIT_TEST_TIMEOUT_MS = 10000;
constexpr unsigned long POSITION_TIMEOUT_MS = 15000;
constexpr unsigned long POSITION_NO_PULSE_TIMEOUT_MS = 1200;
constexpr unsigned long WIFI_TIMEOUT_MS = 15000;
constexpr int32_t MIN_CALIBRATION_TRAVEL_PULSES = 20;
constexpr int32_t MIN_POSITION_TOLERANCE_PULSES = 3;
constexpr uint32_t CALIBRATION_STORAGE_VERSION = 1;
constexpr char CALIBRATION_NAMESPACE[] = "curtain";
constexpr char CALIBRATION_VERSION_KEY[] = "calVer";
constexpr char CALIBRATION_TRAVEL_KEY[] = "travel";
constexpr uint32_t AUTO_STORAGE_VERSION = 2;
constexpr char AUTO_STORAGE_NAMESPACE[] = "autoCfg";
constexpr char AUTO_STORAGE_KEY[] = "config";
constexpr uint8_t MAX_AUTO_LIGHT_RANGES = 5;
constexpr float AUTO_POSITION_TOLERANCE_PERCENT = 1.0F;
constexpr uint8_t AUTO_GENTLE_MAX_STEP_PERCENT = 15;
constexpr uint8_t AUTO_GENTLE_MIN_STEP_PERCENT = 5;
constexpr float MODEL_A_LOW_LUX = 200.0F;
constexpr float MODEL_A_HOT_DARK_CELSIUS = 30.0F;
constexpr float MODEL_A_TEMPERATURE_HYSTERESIS_CELSIUS = 0.5F;
constexpr float MODEL_A_TEMPERATURE_THRESHOLDS[] = {26.0F, 28.0F, 30.0F, 31.0F};
constexpr float MODEL_A_HIGH_LUX_BY_BAND[] = {3000.0F, 2500.0F, 1500.0F, 900.0F, 600.0F};
constexpr char MODEL_A_PRESET_ID[] = "model-a";
constexpr char FIRMWARE_VERSION[] = "0.12.2";
constexpr char FALLBACK_AP_SSID[] = "smart_curtain";
constexpr char FALLBACK_AP_PASSWORD[] = "password123";

enum class MotorDirection : uint8_t
{
  Stopped,
  Opening,
  Closing
};

enum class AutoResponseStyle : uint8_t
{
  Standard = 0,
  Gentle = 1
};

bool isMotorRunning = false;
bool limitTestActive = false;
MotorDirection motorDirection = MotorDirection::Stopped;
unsigned long motorStartedAt = 0;
bool positionControlActive = false;
int32_t positionTargetCount = 0;
int positionTargetPercent = -1;
int8_t positionTravelDirection = 0;
int32_t positionLastEncoderCount = 0;
unsigned long positionLastPulseAt = 0;
String motorFault = "";
String motorStopReason = "manual";
String departureFaultThreshold = "";
unsigned long departureFaultElapsedMs = 0;
int32_t departureFaultDirectedPulses = 0;
bool departureFaultSourceActive = false;
bool departureLimitPending = false;
bool departureLimitReleased = false;
unsigned long departureLimitStartedAt = 0;
int32_t departureLimitStartCount = 0;
int32_t departureLimitMaxPulses = 0;
int8_t departureExpectedEncoderSign = 0;
bool accessPointFallbackActive = false;
bool stationConnectionWasReady = false;
bool networkFailSafeTriggered = false;
unsigned long lastNetworkDisconnectAt = 0;

// Diagnostic only: GPIO interrupts record an edge; they never drive the motor.
volatile uint32_t closedLimitEdgeUs = 0;
volatile uint32_t openLimitEdgeUs = 0;
volatile uint32_t closedLimitEdgeAPulses = 0;
volatile uint32_t openLimitEdgeAPulses = 0;
volatile uint32_t closedLimitEdgeSequence = 0;
volatile uint32_t openLimitEdgeSequence = 0;
volatile uint32_t closedLimitEdgeSequenceAtStart = 0;
volatile uint32_t openLimitEdgeSequenceAtStart = 0;
uint32_t previousSafetyPollUs = 0;
uint32_t maxSafetyPollGapUs = 0;
uint32_t maxWebHandlerUs = 0;

struct LimitStopDiagnostic
{
  bool available;
  bool closedSide;
  bool edgeCaptured;
  bool settled;
  bool interrupted;
  uint32_t edgeToBrakeUs;
  uint32_t edgeToBrakeAPulses;
  uint32_t brakeStartAPulses;
  uint32_t brakeEndAPulses;
  uint32_t settledAPulses;
  uint32_t brakeEndedAtMs;
  uint32_t maxSafetyPollGapUs;
  uint32_t maxWebHandlerUs;
};

LimitStopDiagnostic limitStopDiagnostic = {};

WebServer server(80);
BH1750 lightMeter;
DHT dht(DHT_PIN, DHT22);

bool lightSensorReady = false;

float currentTemperature = NAN;
float currentHumidity = NAN;
float currentLux = NAN;

unsigned long lastSensorReadAt = 0;
unsigned long lastLightSensorRetryAt = 0;
unsigned long lastEncoderReportAt = 0;

volatile int32_t encoderCount = 0;
volatile bool encoderPulseDetected = false;
volatile uint32_t encoderAPulseCount = 0;
volatile uint32_t encoderBPulseCount = 0;
int32_t lastReportedEncoderCount = 0;
uint32_t lastReportedEncoderAPulseCount = 0;
uint32_t lastReportedEncoderBPulseCount = 0;

bool calibrationClosedSet = false;
bool calibrationOpenSet = false;
int32_t calibrationClosedCount = 0;
int32_t calibrationOpenCount = 0;
bool storedCalibrationAvailable = false;
int32_t storedCalibrationTravel = 0;

struct AutoLightRange
{
  float minLux;
  float maxLux;
  int16_t targetPercent;
  bool openEnded;
};

struct AutoConfiguration
{
  uint32_t version;
  bool configured;
  bool enabled;
  AutoResponseStyle responseStyle;
  bool temperatureEnabled;
  float closeAtOrAboveCelsius;
  float reopenAtOrBelowCelsius;
  bool lightEnabled;
  uint8_t lightRangeCount;
  AutoLightRange lightRanges[MAX_AUTO_LIGHT_RANGES];
  int16_t openTargetPercent;
  int16_t closedTargetPercent;
  uint8_t confirmationReadings;
  uint16_t cooldownSeconds;
  uint32_t clientRevision;
  char presetId[24];
};

AutoConfiguration autoConfig = {};
String autoRuntimePhase = "not-configured";
String autoRuntimeReason = "Auto configuration has not been saved";
int autoCandidateTargetPercent = -1;
uint8_t autoConfirmationCount = 0;
int autoLastCommandTargetPercent = -1;
bool autoCommandActive = false;
unsigned long autoLastActionCompletedAt = 0;
uint8_t autoGentleStepPercent = AUTO_GENTLE_MAX_STEP_PERCENT;
int8_t autoGentleLastDirection = 0;
bool autoGentleResolutionLimited = false;
int8_t autoModelATemperatureBandIndex = -1;

bool isModelAConfigured()
{
  return autoConfig.configured &&
         strncmp(
             autoConfig.presetId,
             MODEL_A_PRESET_ID,
             sizeof(autoConfig.presetId)) == 0;
}

void appendAutoStatusJson(String &response);
void disableAutoForManualControl(const char *reason);

struct LimitSensorState
{
  int rawLevel;
  int stableLevel;
  unsigned long rawChangedAt;
  unsigned long lastChangedAt;
  uint32_t triggerCount;
};

LimitSensorState closedLimit = {
    HIGH, // rawLevel  — idle HIGH (INPUT_PULLUP)
    HIGH, // stableLevel
    0,
    0,
    0};

LimitSensorState openLimit = {
    HIGH, // rawLevel  — idle HIGH (INPUT_PULLUP)
    HIGH, // stableLevel
    0,
    0,
    0};

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR handleClosedLimitEdge()
{
  if (closedLimitEdgeSequence == closedLimitEdgeSequenceAtStart)
  {
    closedLimitEdgeAPulses = encoderAPulseCount;
    closedLimitEdgeUs = static_cast<uint32_t>(esp_timer_get_time());
  }
  closedLimitEdgeSequence++;
}

void IRAM_ATTR handleOpenLimitEdge()
{
  if (openLimitEdgeSequence == openLimitEdgeSequenceAtStart)
  {
    openLimitEdgeAPulses = encoderAPulseCount;
    openLimitEdgeUs = static_cast<uint32_t>(esp_timer_get_time());
  }
  openLimitEdgeSequence++;
}

struct EncoderSnapshot
{
  int32_t count;
  uint32_t aPulseCount;
  uint32_t bPulseCount;
  bool pulseDetected;
};

void IRAM_ATTR handleEncoderAPulse()
{
  const bool phaseAHigh = digitalRead(ENCODER_A_PIN) == HIGH;
  const bool phaseBHigh = digitalRead(ENCODER_B_PIN) == HIGH;

  portENTER_CRITICAL_ISR(&encoderMux);
  encoderAPulseCount++;
  encoderCount += phaseAHigh == phaseBHigh ? 1 : -1;
  encoderPulseDetected = true;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR handleEncoderBPulse()
{
  portENTER_CRITICAL_ISR(&encoderMux);
  encoderBPulseCount++;
  encoderPulseDetected = true;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

EncoderSnapshot readEncoderSnapshot()
{
  portENTER_CRITICAL(&encoderMux);
  const EncoderSnapshot snapshot = {
      encoderCount,
      encoderAPulseCount,
      encoderBPulseCount,
      encoderPulseDetected};
  portEXIT_CRITICAL(&encoderMux);

  return snapshot;
}

void setEncoderCount(int32_t value)
{
  portENTER_CRITICAL(&encoderMux);
  encoderCount = value;
  portEXIT_CRITICAL(&encoderMux);
}

int32_t getCalibrationTravelPulses()
{
  if (!calibrationClosedSet || !calibrationOpenSet)
  {
    return 0;
  }

  const int64_t difference =
      static_cast<int64_t>(calibrationOpenCount) -
      static_cast<int64_t>(calibrationClosedCount);

  return static_cast<int32_t>(difference < 0 ? -difference : difference);
}

bool isValidCalibrationTravel(int32_t travel)
{
  const int64_t magnitude =
      travel < 0
          ? -static_cast<int64_t>(travel)
          : static_cast<int64_t>(travel);

  return magnitude >= MIN_CALIBRATION_TRAVEL_PULSES;
}

void loadStoredCalibration()
{
  Preferences preferences;

  if (!preferences.begin(CALIBRATION_NAMESPACE, true))
  {
    Serial.println("Calibration storage: open failed");
    return;
  }

  const uint32_t version =
      preferences.getUInt(CALIBRATION_VERSION_KEY, 0);
  const int32_t travel =
      preferences.getInt(CALIBRATION_TRAVEL_KEY, 0);
  preferences.end();

  if (
      version != CALIBRATION_STORAGE_VERSION ||
      !isValidCalibrationTravel(travel))
  {
    storedCalibrationAvailable = false;
    storedCalibrationTravel = 0;
    Serial.println("Calibration storage: no valid saved travel");
    return;
  }

  storedCalibrationAvailable = true;
  storedCalibrationTravel = travel;

  Serial.print("Calibration storage: loaded travel ");
  Serial.println(storedCalibrationTravel);
  Serial.println("Calibration storage: homing is required");
}

bool saveStoredCalibration()
{
  if (!calibrationClosedSet || !calibrationOpenSet)
  {
    return false;
  }

  const int32_t travel =
      calibrationOpenCount - calibrationClosedCount;

  if (!isValidCalibrationTravel(travel))
  {
    return false;
  }

  Preferences preferences;

  if (!preferences.begin(CALIBRATION_NAMESPACE, false))
  {
    Serial.println("Calibration storage: open for write failed");
    return false;
  }

  const bool versionSaved =
      preferences.putUInt(
          CALIBRATION_VERSION_KEY,
          CALIBRATION_STORAGE_VERSION) == sizeof(uint32_t);
  const bool travelSaved =
      preferences.putInt(
          CALIBRATION_TRAVEL_KEY,
          travel) == sizeof(int32_t);
  preferences.end();

  if (!versionSaved || !travelSaved)
  {
    Serial.println("Calibration storage: save failed");
    return false;
  }

  storedCalibrationAvailable = true;
  storedCalibrationTravel = travel;

  Serial.print("Calibration storage: saved travel ");
  Serial.println(storedCalibrationTravel);
  return true;
}

bool clearStoredCalibration()
{
  Preferences preferences;

  if (!preferences.begin(CALIBRATION_NAMESPACE, false))
  {
    Serial.println("Calibration storage: open for clear failed");
    return false;
  }

  const bool cleared = preferences.clear();
  preferences.end();

  if (cleared)
  {
    storedCalibrationAvailable = false;
    storedCalibrationTravel = 0;
    Serial.println("Calibration storage: cleared");
  }
  else
  {
    Serial.println("Calibration storage: clear failed");
  }

  return cleared;
}

void resetAutoRuntimeState(
    const char *phase = "waiting",
    const char *reason = "Waiting for a fresh sensor reading")
{
  autoRuntimePhase = phase;
  autoRuntimeReason = reason;
  autoCandidateTargetPercent = -1;
  autoConfirmationCount = 0;
  autoLastCommandTargetPercent = -1;
  autoCommandActive = false;
  autoLastActionCompletedAt = 0;
  autoGentleStepPercent = AUTO_GENTLE_MAX_STEP_PERCENT;
  autoGentleLastDirection = 0;
  autoGentleResolutionLimited = false;
  autoModelATemperatureBandIndex = -1;
}

bool validateAutoConfiguration(
    const AutoConfiguration &configuration,
    String &errorMessage)
{
  if (!configuration.configured)
  {
    errorMessage = "Auto configuration is not complete";
    return false;
  }

  if (
      strncmp(
          configuration.presetId,
          MODEL_A_PRESET_ID,
          sizeof(configuration.presetId)) == 0)
  {
    if (
        configuration.responseStyle != AutoResponseStyle::Gentle ||
        !configuration.temperatureEnabled ||
        !configuration.lightEnabled ||
        configuration.confirmationReadings != 2 ||
        configuration.cooldownSeconds != 15)
    {
      errorMessage = "Invalid Model A configuration";
      return false;
    }
    errorMessage = "";
    return true;
  }

  if (
      configuration.responseStyle != AutoResponseStyle::Standard &&
      configuration.responseStyle != AutoResponseStyle::Gentle)
  {
    errorMessage = "Invalid Auto response style";
    return false;
  }

  if (
      !configuration.temperatureEnabled &&
      !configuration.lightEnabled)
  {
    errorMessage = "At least one sensor rule is required";
    return false;
  }

  if (configuration.temperatureEnabled)
  {
    if (
        !isfinite(configuration.closeAtOrAboveCelsius) ||
        !isfinite(configuration.reopenAtOrBelowCelsius) ||
        configuration.closeAtOrAboveCelsius < -20.0F ||
        configuration.closeAtOrAboveCelsius > 80.0F ||
        configuration.reopenAtOrBelowCelsius < -20.0F ||
        configuration.reopenAtOrBelowCelsius > 80.0F ||
        configuration.reopenAtOrBelowCelsius >=
            configuration.closeAtOrAboveCelsius)
    {
      errorMessage = "Invalid temperature thresholds";
      return false;
    }
  }

  if (configuration.lightEnabled)
  {
    if (
        configuration.lightRangeCount < 2 ||
        configuration.lightRangeCount > MAX_AUTO_LIGHT_RANGES)
    {
      errorMessage = "Light range count must be between 2 and 5";
      return false;
    }

    for (
        uint8_t index = 0;
        index < configuration.lightRangeCount;
        index++)
    {
      const AutoLightRange &range = configuration.lightRanges[index];
      const bool isLast = index == configuration.lightRangeCount - 1;

      if (
          !isfinite(range.minLux) ||
          range.minLux < 0.0F ||
          range.targetPercent < 0 ||
          range.targetPercent > 100)
      {
        errorMessage = "Invalid light range";
        return false;
      }

      if (index == 0 && range.minLux != 0.0F)
      {
        errorMessage = "First light range must start at 0 lux";
        return false;
      }

      if (isLast)
      {
        if (!range.openEnded)
        {
          errorMessage = "Last light range must be open ended";
          return false;
        }
      }
      else
      {
        const AutoLightRange &nextRange =
            configuration.lightRanges[index + 1];

        if (
            range.openEnded ||
            !isfinite(range.maxLux) ||
            range.maxLux <= range.minLux ||
            fabsf(range.maxLux - nextRange.minLux) > 0.01F ||
            nextRange.targetPercent > range.targetPercent)
        {
          errorMessage = "Light ranges must be continuous and non-increasing";
          return false;
        }
      }
    }
  }

  if (
      configuration.openTargetPercent < 0 ||
      configuration.openTargetPercent > 100 ||
      configuration.closedTargetPercent < 0 ||
      configuration.closedTargetPercent > 100 ||
      configuration.openTargetPercent <=
          configuration.closedTargetPercent)
  {
    errorMessage = "Invalid open or closed target";
    return false;
  }

  if (
      configuration.confirmationReadings < 1 ||
      configuration.confirmationReadings > 10 ||
      configuration.cooldownSeconds > 3600)
  {
    errorMessage = "Invalid confirmation or cooldown setting";
    return false;
  }

  if (configuration.presetId[0] == '\0')
  {
    errorMessage = "Preset id is required";
    return false;
  }

  errorMessage = "";
  return true;
}

bool saveAutoConfiguration()
{
  String validationError;

  if (!validateAutoConfiguration(autoConfig, validationError))
  {
    Serial.print("Auto storage: invalid configuration - ");
    Serial.println(validationError);
    return false;
  }

  Preferences preferences;

  if (!preferences.begin(AUTO_STORAGE_NAMESPACE, false))
  {
    Serial.println("Auto storage: open for write failed");
    return false;
  }

  const size_t savedBytes = preferences.putBytes(
      AUTO_STORAGE_KEY,
      &autoConfig,
      sizeof(autoConfig));
  preferences.end();

  if (savedBytes != sizeof(autoConfig))
  {
    Serial.println("Auto storage: save failed");
    return false;
  }

  Serial.print("Auto storage: saved preset ");
  Serial.print(autoConfig.presetId);
  Serial.print(" | enabled=");
  Serial.println(autoConfig.enabled ? "true" : "false");
  return true;
}

void loadStoredAutoConfiguration()
{
  Preferences preferences;

  if (!preferences.begin(AUTO_STORAGE_NAMESPACE, true))
  {
    Serial.println("Auto storage: open failed");
    return;
  }

  const size_t storedSize =
      preferences.getBytesLength(AUTO_STORAGE_KEY);
  AutoConfiguration storedConfiguration = {};
  const size_t loadedBytes =
      storedSize == sizeof(storedConfiguration)
          ? preferences.getBytes(
                AUTO_STORAGE_KEY,
                &storedConfiguration,
                sizeof(storedConfiguration))
          : 0;
  preferences.end();

  String validationError;
  if (
      loadedBytes != sizeof(storedConfiguration) ||
      storedConfiguration.version != AUTO_STORAGE_VERSION ||
      !validateAutoConfiguration(
          storedConfiguration,
          validationError))
  {
    autoConfig = {};
    resetAutoRuntimeState(
        "not-configured",
        "Auto configuration has not been saved");
    Serial.println("Auto storage: no valid saved configuration");
    return;
  }

  autoConfig = storedConfiguration;
  const bool autoWasEnabledBeforeRestart = autoConfig.enabled;
  autoConfig.enabled = false;
  resetAutoRuntimeState(
      "disabled",
      "ESP32 restarted; Manual mode required");

  // A reboot must never resume unattended movement. Keep the selected preset,
  // but persist the disabled state so every restart returns to Manual.
  if (autoWasEnabledBeforeRestart && !saveAutoConfiguration())
  {
    Serial.println(
        "Auto safety: unable to persist disabled state after restart");
  }

  Serial.print("Auto storage: loaded preset ");
  Serial.print(autoConfig.presetId);
  Serial.print(" | enabled=");
  Serial.println(autoConfig.enabled ? "true" : "false");
}

String getPositionPercentJson(int32_t count)
{
  if (!calibrationClosedSet || !calibrationOpenSet)
  {
    return "null";
  }

  const int32_t travel =
      calibrationOpenCount - calibrationClosedCount;

  if (travel == 0)
  {
    return "null";
  }

  float position =
      static_cast<float>(count - calibrationClosedCount) *
      100.0f / static_cast<float>(travel);

  position = constrain(position, 0.0f, 100.0f);
  return String(position, 1);
}

String getCalibrationStage()
{
  if (calibrationClosedSet && calibrationOpenSet)
  {
    return "ready";
  }

  if (calibrationClosedSet)
  {
    return "closed-set";
  }

  return "not-started";
}

void synchronizeClosedLimitPosition()
{
  bool restoreOpenCalibration = false;
  int32_t travelFromClosed = 0;

  if (calibrationClosedSet && calibrationOpenSet)
  {
    const int32_t currentTravel =
        calibrationOpenCount - calibrationClosedCount;

    if (isValidCalibrationTravel(currentTravel))
    {
      restoreOpenCalibration = true;
      travelFromClosed = currentTravel;
    }
  }
  else if (
      storedCalibrationAvailable &&
      isValidCalibrationTravel(storedCalibrationTravel))
  {
    restoreOpenCalibration = true;
    travelFromClosed = storedCalibrationTravel;
  }

  setEncoderCount(0);
  calibrationClosedSet = true;
  calibrationClosedCount = 0;

  if (restoreOpenCalibration)
  {
    calibrationOpenSet = true;
    calibrationOpenCount = travelFromClosed;
    Serial.println(
        "Position sync: restored saved open travel after homing");
  }
  else
  {
    calibrationOpenSet = false;
    calibrationOpenCount = 0;
  }

  Serial.println(
      "Position sync: CLOSED limit = encoder 0 (0%)");
}

void synchronizeOpenLimitPosition()
{
  if (!calibrationClosedSet)
  {
    Serial.println(
        "Position sync: OPEN ignored - home CLOSED first");
    return;
  }

  if (calibrationOpenSet)
  {
    setEncoderCount(calibrationOpenCount);
    Serial.println(
        "Position sync: OPEN limit = saved encoder count (100%)");
    return;
  }

  const EncoderSnapshot snapshot = readEncoderSnapshot();
  const int64_t difference =
      static_cast<int64_t>(snapshot.count) -
      static_cast<int64_t>(calibrationClosedCount);
  const int64_t travelPulses =
      difference < 0 ? -difference : difference;

  if (travelPulses < MIN_CALIBRATION_TRAVEL_PULSES)
  {
    Serial.println(
        "Position sync: OPEN travel is too short");
    return;
  }

  calibrationOpenCount = snapshot.count;
  calibrationOpenSet = true;
  saveStoredCalibration();

  Serial.print(
      "Position sync: OPEN limit learned as 100%, count ");
  Serial.println(calibrationOpenCount);
}

bool isLimitActive(const LimitSensorState &state)
{
  return state.stableLevel == LIMIT_ACTIVE_LEVEL;
}

bool isLimitRawActive(const LimitSensorState &state)
{
  return state.rawLevel == LIMIT_ACTIVE_LEVEL;
}

bool isLimitPinActive(uint8_t pin)
{
  return digitalRead(pin) == LIMIT_ACTIVE_LEVEL;
}

void initializeLimitSensors()
{
  pinMode(LIMIT_CLOSED_PIN, INPUT_PULLUP);
  pinMode(LIMIT_OPEN_PIN, INPUT_PULLUP);

  const unsigned long currentTime = millis();

  closedLimit.rawLevel = digitalRead(LIMIT_CLOSED_PIN);
  closedLimit.stableLevel = closedLimit.rawLevel;
  closedLimit.rawChangedAt = currentTime;
  closedLimit.lastChangedAt = currentTime;
  closedLimit.triggerCount = 0;

  openLimit.rawLevel = digitalRead(LIMIT_OPEN_PIN);
  openLimit.stableLevel = openLimit.rawLevel;
  openLimit.rawChangedAt = currentTime;
  openLimit.lastChangedAt = currentTime;
  openLimit.triggerCount = 0;

  attachInterrupt(
      digitalPinToInterrupt(LIMIT_CLOSED_PIN),
      handleClosedLimitEdge,
      RISING);
  attachInterrupt(
      digitalPinToInterrupt(LIMIT_OPEN_PIN),
      handleOpenLimitEdge,
      RISING);

  Serial.println(
      "FC-33 ready: CLOSED=GPIO27, OPEN=GPIO26");
}

void updateLimitSensor(
    LimitSensorState &state,
    uint8_t pin,
    const char *label,
    unsigned long currentTime)
{
  const int reading = digitalRead(pin);

  if (reading != state.rawLevel)
  {
    state.rawLevel = reading;
    state.rawChangedAt = currentTime;
  }

  if (
      reading != state.stableLevel &&
      currentTime - state.rawChangedAt >= LIMIT_DEBOUNCE_MS)
  {
    state.stableLevel = reading;
    state.lastChangedAt = currentTime;

    const bool active = isLimitActive(state);

    if (active)
    {
      state.triggerCount++;
    }

    Serial.print("FC-33 ");
    Serial.print(label);
    Serial.print(": ");
    Serial.print(active ? "ACTIVE" : "RELEASED");
    Serial.print(" | raw=");
    Serial.print(state.stableLevel == HIGH ? "HIGH" : "LOW");
    Serial.print(" | count=");
    Serial.println(state.triggerCount);
  }
}

void updateLimitSensors(unsigned long currentTime)
{
  updateLimitSensor(
      closedLimit,
      LIMIT_CLOSED_PIN,
      "CLOSED",
      currentTime);

  updateLimitSensor(
      openLimit,
      LIMIT_OPEN_PIN,
      "OPEN",
      currentTime);
}

void initializeEncoder()
{
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_A_PIN),
      handleEncoderAPulse,
      CHANGE);
  attachInterrupt(
      digitalPinToInterrupt(ENCODER_B_PIN),
      handleEncoderBPulse,
      CHANGE);

  Serial.println("Encoder test ready: A=GPIO32, B=GPIO33");
}

void reportEncoderActivity(unsigned long currentTime)
{
  if (
      currentTime - lastEncoderReportAt <
      ENCODER_REPORT_INTERVAL_MS)
  {
    return;
  }

  lastEncoderReportAt = currentTime;

  const EncoderSnapshot snapshot = readEncoderSnapshot();

  if (
      snapshot.count == lastReportedEncoderCount &&
      snapshot.aPulseCount == lastReportedEncoderAPulseCount &&
      snapshot.bPulseCount == lastReportedEncoderBPulseCount)
  {
    return;
  }

  lastReportedEncoderCount = snapshot.count;
  lastReportedEncoderAPulseCount = snapshot.aPulseCount;
  lastReportedEncoderBPulseCount = snapshot.bPulseCount;

  Serial.print("Encoder count: ");
  Serial.print(snapshot.count);
  Serial.print(" | A: ");
  Serial.print(snapshot.aPulseCount);
  Serial.print(" | B: ");
  Serial.println(snapshot.bPulseCount);
}

void beginLimitStopDiagnostic(bool closedSide)
{
  const EncoderSnapshot snapshot = readEncoderSnapshot();
  const uint32_t brakeRequestedAtUs =
      static_cast<uint32_t>(esp_timer_get_time());
  const uint32_t edgeSequence = closedSide
                                    ? closedLimitEdgeSequence
                                    : openLimitEdgeSequence;
  const uint32_t sequenceAtStart = closedSide
                                       ? closedLimitEdgeSequenceAtStart
                                       : openLimitEdgeSequenceAtStart;
  const uint32_t edgeUs = closedSide
                              ? closedLimitEdgeUs
                              : openLimitEdgeUs;
  const uint32_t edgeAPulses = closedSide
                                   ? closedLimitEdgeAPulses
                                   : openLimitEdgeAPulses;

  limitStopDiagnostic = {};
  limitStopDiagnostic.available = true;
  limitStopDiagnostic.closedSide = closedSide;
  limitStopDiagnostic.edgeCaptured = edgeSequence != sequenceAtStart;
  limitStopDiagnostic.brakeStartAPulses = snapshot.aPulseCount;
  limitStopDiagnostic.maxSafetyPollGapUs = maxSafetyPollGapUs;
  limitStopDiagnostic.maxWebHandlerUs = maxWebHandlerUs;

  if (limitStopDiagnostic.edgeCaptured)
  {
    limitStopDiagnostic.edgeToBrakeUs = brakeRequestedAtUs - edgeUs;
    limitStopDiagnostic.edgeToBrakeAPulses =
        snapshot.aPulseCount - edgeAPulses;
  }
}

void finishLimitStopBrakeDiagnostic()
{
  limitStopDiagnostic.brakeEndAPulses =
      readEncoderSnapshot().aPulseCount;
  limitStopDiagnostic.brakeEndedAtMs = millis();
}

void updateLimitStopDiagnostic(unsigned long currentTime)
{
  if (!limitStopDiagnostic.available ||
      limitStopDiagnostic.settled ||
      limitStopDiagnostic.interrupted ||
      isMotorRunning ||
      currentTime - limitStopDiagnostic.brakeEndedAtMs <
          LIMIT_DIAGNOSTIC_SETTLE_MS)
  {
    return;
  }

  limitStopDiagnostic.settledAPulses =
      readEncoderSnapshot().aPulseCount;
  limitStopDiagnostic.settled = true;
}

void stopMotor(
    const char *reason = "manual",
    bool useFastBrake = false,
    uint8_t brakeDuty = LIMIT_BRAKE_DUTY,
    unsigned long brakeDurationMs = LIMIT_BRAKE_DURATION_MS)
{
  if (useFastBrake && isMotorRunning)
  {
    // L298N active brake: keep ENA active briefly while both inputs are equal.
    // Used for a travel limit or a controlled position stop.
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    ledcWrite(PWM_CHANNEL, brakeDuty);
    delay(brakeDurationMs);
  }

  ledcWrite(PWM_CHANNEL, 0);

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  isMotorRunning = false;
  positionControlActive = false;
  limitTestActive = false;
  departureLimitPending = false;
  departureLimitReleased = false;
  departureExpectedEncoderSign = 0;
  motorDirection = MotorDirection::Stopped;
  motorStopReason = reason;
  Serial.println("Motor: STOP");
}

void startMotor(bool forward)
{
  positionControlActive = false;
  positionTargetPercent = -1;
  limitTestActive = false;
  motorFault = "";
  departureFaultThreshold = "";

  const bool initialClosedActive = isLimitPinActive(LIMIT_CLOSED_PIN);
  const bool initialOpenActive = isLimitPinActive(LIMIT_OPEN_PIN);
  if (LIMIT_SAFETY_ENABLED && initialClosedActive && initialOpenActive)
  {
    motorFault = "limit-conflict";
    stopMotor("limit-conflict", isMotorRunning);
    Serial.println("Motor start blocked: both limits active");
    return;
  }

  if (LIMIT_SAFETY_ENABLED &&
      (forward ? initialOpenActive : initialClosedActive))
  {
    motorFault = forward
                     ? "open-limit-active"
                     : "closed-limit-active";
    stopMotor(
        forward ? "open-limit" : "closed-limit",
        isMotorRunning);
    Serial.println(
        "Motor start blocked: requested limit is active");
    return;
  }

  if (isMotorRunning)
  {
    stopMotor();
    delay(150);
  }

  // Recheck after a direction change; a limit may have changed during braking.
  const bool closedActive = isLimitPinActive(LIMIT_CLOSED_PIN);
  const bool openActive = isLimitPinActive(LIMIT_OPEN_PIN);
  if (LIMIT_SAFETY_ENABLED && closedActive && openActive)
  {
    motorFault = "limit-conflict";
    stopMotor("limit-conflict", false);
    Serial.println("Motor start blocked: both limits active");
    return;
  }
  if (LIMIT_SAFETY_ENABLED && (forward ? openActive : closedActive))
  {
    motorFault = forward ? "open-limit-active" : "closed-limit-active";
    stopMotor(forward ? "open-limit" : "closed-limit", false);
    Serial.println("Motor start blocked: requested limit is active");
    return;
  }

  const bool sourceLimitActive = forward ? closedActive : openActive;
  departureLimitPending = LIMIT_SAFETY_ENABLED && sourceLimitActive;
  departureLimitReleased = false;
  departureExpectedEncoderSign = 0;
  if (departureLimitPending)
  {
    const int32_t signedTravel =
        calibrationClosedSet && calibrationOpenSet
            ? calibrationOpenCount - calibrationClosedCount
            : storedCalibrationTravel;
    const bool encoderDirectionKnown =
        isValidCalibrationTravel(signedTravel);
    int64_t releasePulses = DEPARTURE_LIMIT_RELEASE_MAX_PULSES;

    if (encoderDirectionKnown)
    {
      const int64_t travelMagnitude =
          signedTravel < 0 ? -static_cast<int64_t>(signedTravel)
                           : static_cast<int64_t>(signedTravel);
      releasePulses =
          travelMagnitude * DEPARTURE_LIMIT_RELEASE_TRAVEL_PERCENT / 100;
      departureExpectedEncoderSign =
          (signedTravel > 0 ? 1 : -1) * (forward ? 1 : -1);
    }
    else
    {
      // A calibration reset intentionally removes the signed travel. Allow the
      // first homing movement to leave its source limit using absolute encoder
      // travel, while retaining the same release-distance and timeout guards.
      // The commanded motor direction and the source-limit release still prove
      // that the curtain moved away from the active end stop.
      departureExpectedEncoderSign = 0;
      Serial.println(
          "Limit safety: encoder direction unknown; using guarded absolute travel");
    }

    if (releasePulses < 8) releasePulses = 8;
    if (releasePulses > DEPARTURE_LIMIT_RELEASE_MAX_PULSES)
      releasePulses = DEPARTURE_LIMIT_RELEASE_MAX_PULSES;
    departureLimitMaxPulses = static_cast<int32_t>(releasePulses);
    departureLimitStartCount = readEncoderSnapshot().count;
    departureLimitStartedAt = millis();
  }

  if (forward)
  {
    motorDirection = MotorDirection::Opening;
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    Serial.println("Motor: FORWARD test");
  }
  else
  {
    motorDirection = MotorDirection::Closing;
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    Serial.println("Motor: REVERSE test");
  }

  if (limitStopDiagnostic.available && !limitStopDiagnostic.settled)
  {
    limitStopDiagnostic.interrupted = true;
  }
  closedLimitEdgeSequenceAtStart = closedLimitEdgeSequence;
  openLimitEdgeSequenceAtStart = openLimitEdgeSequence;
  previousSafetyPollUs = static_cast<uint32_t>(esp_timer_get_time());
  maxSafetyPollGapUs = 0;
  maxWebHandlerUs = 0;
  ledcWrite(PWM_CHANNEL, TEST_SPEED);

  motorStartedAt = millis();
  isMotorRunning = true;
}

void updateLimitSafety()
{
  if (!LIMIT_SAFETY_ENABLED)
  {
    return;
  }

  if (!isMotorRunning)
  {
    return;
  }

  const uint32_t safetyPollUs =
      static_cast<uint32_t>(esp_timer_get_time());
  const uint32_t safetyGapUs = safetyPollUs - previousSafetyPollUs;
  if (safetyGapUs > maxSafetyPollGapUs)
  {
    maxSafetyPollGapUs = safetyGapUs;
  }
  previousSafetyPollUs = safetyPollUs;

  const bool closedActive = isLimitRawActive(closedLimit);
  const bool openActive   = isLimitRawActive(openLimit);

  if (closedActive && openActive)
  {
    motorFault = "limit-conflict";
    stopMotor("limit-conflict", true);
    Serial.println("Limit safety: both limits active - motor stopped");
    return;
  }

  const bool movingToClose = motorDirection == MotorDirection::Closing;
  const bool movingToOpen  = motorDirection == MotorDirection::Opening;
  if (!movingToClose && !movingToOpen)
  {
    motorFault = "motor-direction-unknown";
    stopMotor("direction-unknown", true);
    return;
  }

  // A destination limit always stops motion immediately.
  if (closedActive && movingToClose)
  {
    beginLimitStopDiagnostic(true);
    stopMotor("closed-limit", true);
    finishLimitStopBrakeDiagnostic();
    synchronizeClosedLimitPosition();
    Serial.println(
        "Limit safety: CLOSED limit active - motor stopped");
    return;
  }
  if (openActive && movingToOpen)
  {
    beginLimitStopDiagnostic(false);
    stopMotor("open-limit", true);
    finishLimitStopBrakeDiagnostic();
    synchronizeOpenLimitPosition();
    Serial.println(
        "Limit safety: OPEN limit active - motor stopped");
    return;
  }

  const bool sourceLimitActive = movingToClose ? openActive : closedActive;
  if (departureLimitPending)
  {
    const int32_t count = readEncoderSnapshot().count;
    const int64_t rawTravel =
        static_cast<int64_t>(count) - departureLimitStartCount;
    const bool encoderDirectionKnown =
        departureExpectedEncoderSign != 0;
    const int64_t directedTravel = encoderDirectionKnown
                                       ? rawTravel * departureExpectedEncoderSign
                                       : (rawTravel < 0 ? -rawTravel : rawTravel);
    if (encoderDirectionKnown &&
        directedTravel <= -DEPARTURE_WRONG_DIRECTION_PULSES)
    {
      motorFault = "departure-wrong-direction";
      stopMotor("departure-wrong-direction", true);
      Serial.println("Limit safety: wrong encoder direction while leaving source limit");
      return;
    }
    if (!sourceLimitActive)
    {
      departureLimitReleased = true;
    }
    if (sourceLimitActive && departureLimitReleased)
    {
      motorFault = "unexpected-source-limit";
      stopMotor("unexpected-source-limit", true);
      Serial.println("Limit safety: source limit reactivated - motor stopped");
      return;
    }
    if (!departureLimitReleased &&
        directedTravel >= departureLimitMaxPulses)
    {
      departureFaultThreshold = "distance";
      departureFaultElapsedMs = millis() - departureLimitStartedAt;
      departureFaultDirectedPulses = static_cast<int32_t>(directedTravel);
      departureFaultSourceActive = sourceLimitActive;
      motorFault = "departure-limit-stuck";
      stopMotor("departure-limit-stuck", true);
      Serial.println("Limit safety: source limit did not release - motor stopped");
      return;
    }
    if (departureLimitReleased && directedTravel >= 8)
    {
      departureLimitPending = false;
      return;
    }
    if (millis() - departureLimitStartedAt >=
        DEPARTURE_LIMIT_RELEASE_TIMEOUT_MS)
    {
      departureFaultThreshold = "time";
      departureFaultElapsedMs = millis() - departureLimitStartedAt;
      departureFaultDirectedPulses = static_cast<int32_t>(directedTravel);
      departureFaultSourceActive = sourceLimitActive;
      motorFault = departureLimitReleased
                       ? "departure-encoder-unverified"
                       : "departure-limit-stuck";
      stopMotor("departure-unverified", true);
      Serial.println("Limit safety: departure was not verified - motor stopped");
    }
    return;
  }

  // Once released, a source limit may not become active again during motion.
  if (sourceLimitActive)
  {
    motorFault = "unexpected-source-limit";
    stopMotor("unexpected-source-limit", true);
    Serial.println("Limit safety: unexpected source limit - motor stopped");
  }
}


int32_t getPositionTolerancePulses()
{
  const int32_t onePercent = getCalibrationTravelPulses() / 100;
  return onePercent > MIN_POSITION_TOLERANCE_PULSES
             ? onePercent
             : MIN_POSITION_TOLERANCE_PULSES;
}

bool hasReachedPositionTarget(int32_t currentCount)
{
  const int64_t error =
      static_cast<int64_t>(positionTargetCount) - currentCount;
  const int64_t absoluteError = error < 0 ? -error : error;

  if (absoluteError <= getPositionTolerancePulses())
  {
    return true;
  }

  return
      (positionTravelDirection > 0 && currentCount >= positionTargetCount) ||
      (positionTravelDirection < 0 && currentCount <= positionTargetCount);
}

void updatePositionControl(unsigned long currentTime)
{
  if (!positionControlActive || !isMotorRunning)
  {
    return;
  }

  const EncoderSnapshot snapshot = readEncoderSnapshot();

  if (hasReachedPositionTarget(snapshot.count))
  {
    Serial.println("Position control: TARGET reached");
    positionControlActive = false;
    stopMotor(
        "position-target",
        true,
        POSITION_BRAKE_DUTY,
        POSITION_BRAKE_DURATION_MS);
    return;
  }

  if (snapshot.count != positionLastEncoderCount)
  {
    positionLastEncoderCount = snapshot.count;
    positionLastPulseAt = currentTime;
  }
  else if (
      currentTime - positionLastPulseAt >=
      POSITION_NO_PULSE_TIMEOUT_MS)
  {
    motorFault = "encoder-no-pulse";
    Serial.println("Position control: STOP - encoder has no pulse");
    positionControlActive = false;
    stopMotor();
    return;
  }

  if (currentTime - motorStartedAt >= POSITION_TIMEOUT_MS)
  {
    motorFault = "position-timeout";
    Serial.println("Position control: STOP - timeout");
    positionControlActive = false;
    stopMotor();
  }
}

bool hasI2CDevice(uint8_t address)
{
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool initializeLightSensor()
{
  lightSensorReady = false;
  currentLux = NAN;

  uint8_t lightSensorAddress = 0;

  if (hasI2CDevice(0x23))
  {
    lightSensorAddress = 0x23;
  }
  else if (hasI2CDevice(0x5C))
  {
    lightSensorAddress = 0x5C;
  }

  if (lightSensorAddress == 0)
  {
    Serial.println("BH1750 not found");
    return false;
  }

  lightSensorReady = lightMeter.begin(
      BH1750::CONTINUOUS_HIGH_RES_MODE,
      lightSensorAddress,
      &Wire);

  if (!lightSensorReady)
  {
    Serial.println("BH1750 initialization failed");
    return false;
  }

  Serial.print("BH1750 ready at address 0x");
  Serial.println(lightSensorAddress, HEX);

  return true;
}

void initializeSensors()
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  dht.begin();
  initializeLightSensor();
}

void readSensors()
{
  const float temperature = dht.readTemperature();
  const float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity))
  {
    currentTemperature = NAN;
    currentHumidity = NAN;

    Serial.println("DHT22 reading failed");
  }
  else
  {
    currentTemperature = temperature;
    currentHumidity = humidity;

    Serial.print("Temperature: ");
    Serial.print(currentTemperature, 1);
    Serial.println(" C");

    Serial.print("Humidity: ");
    Serial.print(currentHumidity, 1);
    Serial.println(" %");
  }

  if (!lightSensorReady)
  {
    currentLux = NAN;

    const unsigned long currentTime = millis();

    if (
        currentTime - lastLightSensorRetryAt >=
        LIGHT_SENSOR_RETRY_MS)
    {
      lastLightSensorRetryAt = currentTime;
      initializeLightSensor();
    }

    return;
  }

  const float lux = lightMeter.readLightLevel();

  if (lux < 0)
  {
    currentLux = NAN;
    lightSensorReady = false;
    lastLightSensorRetryAt = millis();

    Serial.println("BH1750 reading failed");
    return;
  }

  currentLux = lux;

  Serial.print("Light: ");
  Serial.print(currentLux, 1);
  Serial.println(" lux");
}

String jsonNumberOrNull(float value, unsigned int decimalPlaces)
{
  if (isnan(value))
  {
    return "null";
  }

  return String(value, decimalPlaces);
}

void addCorsHeaders()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader(
      "Access-Control-Allow-Methods",
      "GET, POST, OPTIONS");
  server.sendHeader(
      "Access-Control-Allow-Headers",
      "Content-Type");
  server.sendHeader(
      "Access-Control-Allow-Private-Network",
      "true");
}

void handleCorsPreflight()
{
  addCorsHeaders();
  server.send(204, "text/plain", "");
}

void handleSensors()
{
  const bool dht22Available =
      !isnan(currentTemperature) &&
      !isnan(currentHumidity);

  String response = "{";

  response += "\"temperatureCelsius\":";
  response += jsonNumberOrNull(currentTemperature, 1);
  response += ",";

  response += "\"humidityPercent\":";
  response += jsonNumberOrNull(currentHumidity, 1);
  response += ",";

  response += "\"lightLux\":";
  response += jsonNumberOrNull(currentLux, 1);
  response += ",";

  response += "\"available\":{";

  response += "\"dht22\":";
  response += (dht22Available ? "true" : "false");
  response += ",";

  response += "\"bh1750\":";
  response += (lightSensorReady ? "true" : "false");

  response += "},";

  response += "\"sampledAtMs\":";
  response += String(lastSensorReadAt);

  response += "}";

  addCorsHeaders();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", response);
}

void handleHealth()
{
  const String motorState =
      isMotorRunning ? "running" : "stopped";
  const bool stationConnected = WiFi.status() == WL_CONNECTED;
  const String wifiMode =
      accessPointFallbackActive ? "access-point" : "station";
  const String ipAddress =
      accessPointFallbackActive
          ? WiFi.softAPIP().toString()
          : WiFi.localIP().toString();

  String response = "{";
  response += "\"status\":\"ok\",";
  response += "\"device\":\"smart-curtain-esp32\",";
  response += "\"motor\":\"" + motorState + "\",";
  response += "\"uptimeMs\":" + String(millis()) + ",";
  response += "\"wifiMode\":\"" + wifiMode + "\",";
  response += "\"wifiReady\":";
  response += (accessPointFallbackActive || stationConnected)
                  ? "true"
                  : "false";
  response += ",\"ipAddress\":\"" + ipAddress + "\",";
  response += "\"wifiRssi\":";
  response += stationConnected ? String(WiFi.RSSI()) : "null";
  response += ",\"failSafeTriggered\":";
  response += networkFailSafeTriggered ? "true" : "false";
  response += ",\"lastDisconnectMs\":";
  response += networkFailSafeTriggered
                  ? String(lastNetworkDisconnectAt)
                  : "null";
  response += "}";

  addCorsHeaders();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", response);
}

void handleStatus()
{
  const bool dht22Available =
      !isnan(currentTemperature) &&
      !isnan(currentHumidity);
  const String motorState =
      isMotorRunning ? "running" : "stopped";
  const EncoderSnapshot encoderSnapshot = readEncoderSnapshot();

  String response = "{";
  response += "\"status\":\"ok\",";
  response += "\"device\":\"smart-curtain-esp32\",";
  response += "\"firmwareVersion\":\"";
  response += FIRMWARE_VERSION;
  response += "\",";

  const bool stationConnected = WiFi.status() == WL_CONNECTED;
  const String wifiMode =
      accessPointFallbackActive ? "access-point" : "station";
  const String ipAddress =
      accessPointFallbackActive
          ? WiFi.softAPIP().toString()
          : WiFi.localIP().toString();

  response += "\"network\":{";
  response += "\"mode\":\"" + wifiMode + "\",";
  response += "\"ready\":";
  response += (accessPointFallbackActive || stationConnected)
                  ? "true"
                  : "false";
  response += ",\"ipAddress\":\"" + ipAddress + "\",";
  response += "\"rssi\":";
  response += stationConnected ? String(WiFi.RSSI()) : "null";
  response += ",\"failSafeTriggered\":";
  response += networkFailSafeTriggered ? "true" : "false";
  response += ",\"lastDisconnectMs\":";
  response += networkFailSafeTriggered
                  ? String(lastNetworkDisconnectAt)
                  : "null";
  response += "},";

  response += "\"motor\":{";
  response += "\"state\":\"" + motorState + "\",";
  response += "\"speed\":" + String(TEST_SPEED) + ",";
  response += "\"timeoutMs\":" + String(TEST_DURATION_MS) + ",";
  response += "\"positionTimeoutMs\":" +
              String(POSITION_TIMEOUT_MS) + ",";
  response += "\"limitTestTimeoutMs\":" +
              String(LIMIT_TEST_TIMEOUT_MS) + ",";
  response += "\"positionControlAvailable\":";
  response += (LIMIT_SAFETY_ENABLED ? "true" : "false");
  response += ",";
  response += "\"controlMode\":\"";
  response += positionControlActive
                  ? "position"
                  : (limitTestActive
                         ? "limit-test"
                         : (isMotorRunning ? "jog" : "idle"));
  response += "\",";
  response += "\"targetPercent\":";
  response += positionTargetPercent >= 0
                  ? String(positionTargetPercent)
                  : "null";
  response += ",\"fault\":";
  response += motorFault.length() > 0
                  ? "\"" + motorFault + "\""
                  : "null";
  response += ",\"lastStopReason\":\"";
  response += motorStopReason;
  response += "\"";
  response += ",\"departureFault\":";
  if (departureFaultThreshold.length() == 0)
  {
    response += "null";
  }
  else
  {
    response += "{\"threshold\":\"" + departureFaultThreshold + "\",";
    response += "\"elapsedMs\":" + String(departureFaultElapsedMs) + ",";
    response += "\"directedPulses\":" + String(departureFaultDirectedPulses) + ",";
    response += "\"maxPulses\":" + String(departureLimitMaxPulses) + ",";
    response += "\"sourceLimitActive\":";
    response += departureFaultSourceActive ? "true" : "false";
    response += "}";
  }
  response += "},";

  response += "\"limitStopDiagnostic\":";
  if (!limitStopDiagnostic.available)
  {
    response += "null,";
  }
  else
  {
    response += "{";
    response += "\"side\":\"";
    response += limitStopDiagnostic.closedSide ? "closed" : "open";
    response += "\",\"edgeCaptured\":";
    response += limitStopDiagnostic.edgeCaptured ? "true" : "false";
    response += ",\"edgeToBrakeUs\":";
    response += limitStopDiagnostic.edgeCaptured
                    ? String(limitStopDiagnostic.edgeToBrakeUs)
                    : "null";
    response += ",\"edgeToBrakeEncoderAEdges\":";
    response += limitStopDiagnostic.edgeCaptured
                    ? String(limitStopDiagnostic.edgeToBrakeAPulses)
                    : "null";
    response += ",\"brakeEncoderAEdges\":";
    response += String(
        limitStopDiagnostic.brakeEndAPulses -
        limitStopDiagnostic.brakeStartAPulses);
    response += ",\"afterBrakeEncoderAEdges\":";
    response += limitStopDiagnostic.settled
                    ? String(
                          limitStopDiagnostic.settledAPulses -
                          limitStopDiagnostic.brakeEndAPulses)
                    : "null";
    response += ",\"settled\":";
    response += limitStopDiagnostic.settled ? "true" : "false";
    response += ",\"interrupted\":";
    response += limitStopDiagnostic.interrupted ? "true" : "false";
    response += ",\"maxSafetyPollGapUs\":";
    response += String(limitStopDiagnostic.maxSafetyPollGapUs);
    response += ",\"maxWebHandlerUs\":";
    response += String(limitStopDiagnostic.maxWebHandlerUs);
    response += "},";
  }

  response += "\"sensors\":{";
  response += "\"dht22\":";
  response += (dht22Available ? "true" : "false");
  response += ",";
  response += "\"bh1750\":";
  response += (lightSensorReady ? "true" : "false");
  response += "},";

    response += "\"limits\":{";

  response += "\"closed\":{";
  response += "\"active\":";
  response += (isLimitActive(closedLimit) ? "true" : "false");
  response += ",";
  response += "\"rawLevel\":\"";
  response += (closedLimit.rawLevel == HIGH ? "HIGH" : "LOW");
  response += "\",";
  response += "\"triggerCount\":";
  response += String(
      static_cast<unsigned long>(closedLimit.triggerCount));
  response += ",";
  response += "\"lastChangedMs\":";
  response += String(closedLimit.lastChangedAt);
  response += "},";

  response += "\"open\":{";
  response += "\"active\":";
  response += (isLimitActive(openLimit) ? "true" : "false");
  response += ",";
  response += "\"rawLevel\":\"";
  response += (openLimit.rawLevel == HIGH ? "HIGH" : "LOW");
  response += "\",";
  response += "\"triggerCount\":";
  response += String(
      static_cast<unsigned long>(openLimit.triggerCount));
  response += ",";
  response += "\"lastChangedMs\":";
  response += String(openLimit.lastChangedAt);
  response += "},";

  response += "\"conflict\":";
  response += (
      isLimitActive(closedLimit) &&
              isLimitActive(openLimit)
          ? "true"
          : "false");
  response += ",";

  response += "\"closedStopEnabled\":";
  response += (LIMIT_SAFETY_ENABLED ? "true" : "false");
  response += ",";
  response += "\"openStopEnabled\":";
  response += (LIMIT_SAFETY_ENABLED ? "true" : "false");
  response += ",";
  response += "\"safetyEnabled\":";
  response += (LIMIT_SAFETY_ENABLED ? "true" : "false");

  response += "},";
  response += "\"encoder\":{";
  response += "\"count\":";
  response += String(static_cast<long>(encoderSnapshot.count));
  response += ",";
  response += "\"pulseDetected\":";
  response +=
      (encoderSnapshot.pulseDetected ? "true" : "false");
  response += ",";
  response += "\"aPulseCount\":";
  response += String(
      static_cast<unsigned long>(encoderSnapshot.aPulseCount));
  response += ",";
  response += "\"bPulseCount\":";
  response += String(
      static_cast<unsigned long>(encoderSnapshot.bPulseCount));
  response += ",";
  response += "\"positionPercent\":";
  response += getPositionPercentJson(encoderSnapshot.count);
  response += ",";
  response += "\"calibrated\":";
  response +=
      (calibrationClosedSet && calibrationOpenSet ? "true" : "false");
  response += "},";

  response += "\"calibration\":{";
  response += "\"stage\":\"" + getCalibrationStage() + "\",";
  response += "\"closedSet\":";
  response += (calibrationClosedSet ? "true" : "false");
  response += ",";
  response += "\"openSet\":";
  response += (calibrationOpenSet ? "true" : "false");
  response += ",";
  response += "\"closedCount\":";
  response += calibrationClosedSet
                  ? String(static_cast<long>(calibrationClosedCount))
                  : "null";
  response += ",";
  response += "\"openCount\":";
  response += calibrationOpenSet
                  ? String(static_cast<long>(calibrationOpenCount))
                  : "null";
  response += ",";
  response += "\"travelPulses\":";
  response += calibrationOpenSet
                  ? String(static_cast<long>(getCalibrationTravelPulses()))
                  : "null";
  response += ",";
  response += "\"persistentStored\":";
  response += (storedCalibrationAvailable ? "true" : "false");
  response += ",";
  response += "\"storedTravelPulses\":";
  response += storedCalibrationAvailable
                  ? String(static_cast<long>(
                        storedCalibrationTravel < 0
                            ? -static_cast<int64_t>(storedCalibrationTravel)
                            : static_cast<int64_t>(storedCalibrationTravel)))
                  : "null";
  response += ",";
  response += "\"requiresHoming\":";
  response += (calibrationClosedSet ? "false" : "true");
  response += "},";

  response += "\"auto\":";
  appendAutoStatusJson(response);
  response += ",";

  response += "\"uptimeMs\":" + String(millis());
  response += "}";

  addCorsHeaders();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", response);
}

void sendMotorResponse(const String &command)
{
  String response = "{\"ok\":true,\"command\":\"";
  response += command;
  response += "\"}";

  addCorsHeaders();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", response);
}

void sendMotorStartError(const String &command)
{
  String response = "{\"ok\":false,\"command\":\"";
  response += command;
  response += "\",\"message\":\"";
  response += motorFault.length() ? motorFault : "motor-did-not-start";
  response += "\"}";

  addCorsHeaders();
  server.sendHeader("Cache-Control", "no-store");
  server.send(409, "application/json", response);
}

void sendPositionResponse(
    int statusCode,
    bool ok,
    const String &message,
    int targetPercent)
{
  String response = "{";
  response += "\"ok\":";
  response += (ok ? "true" : "false");
  response += ",\"command\":\"position\",";
  response += "\"message\":\"" + message + "\",";
  response += "\"targetPercent\":";
  response += targetPercent >= 0 ? String(targetPercent) : "null";
  response += "}";

  addCorsHeaders();
  server.sendHeader("Cache-Control", "no-store");
  server.send(statusCode, "application/json", response);
}

void sendCalibrationResponse(
    int statusCode,
    bool ok,
    const String &action,
    const String &message)
{
  const EncoderSnapshot snapshot = readEncoderSnapshot();

  String response = "{";
  response += "\"ok\":";
  response += (ok ? "true" : "false");
  response += ",\"action\":\"" + action + "\",";
  response += "\"message\":\"" + message + "\",";
  response += "\"encoderCount\":";
  response += String(static_cast<long>(snapshot.count));
  response += ",\"stage\":\"" + getCalibrationStage() + "\"";
  response += "}";

  addCorsHeaders();
  server.sendHeader("Cache-Control", "no-store");
  server.send(statusCode, "application/json", response);
}

void handleCalibrationClosed()
{
  disableAutoForManualControl("Calibration started");
  stopMotor();
  synchronizeClosedLimitPosition();

  Serial.println("Calibration: CLOSED set to count 0");
  sendCalibrationResponse(
      200,
      true,
      "set-closed",
      "Closed position saved");
}

void handleCalibrationOpen()
{
  disableAutoForManualControl("Calibration started");
  stopMotor();

  if (!calibrationClosedSet)
  {
    sendCalibrationResponse(
        409,
        false,
        "set-open",
        "Set closed position first");
    return;
  }

  const EncoderSnapshot snapshot = readEncoderSnapshot();
  const int64_t difference =
      static_cast<int64_t>(snapshot.count) -
      static_cast<int64_t>(calibrationClosedCount);
  const int64_t travelPulses = difference < 0 ? -difference : difference;

  if (travelPulses < MIN_CALIBRATION_TRAVEL_PULSES)
  {
    sendCalibrationResponse(
        409,
        false,
        "set-open",
        "Travel is too short");
    return;
  }

  calibrationOpenCount = snapshot.count;
  calibrationOpenSet = true;
  const bool persisted = saveStoredCalibration();

  Serial.print("Calibration: OPEN set to count ");
  Serial.println(calibrationOpenCount);
  Serial.print("Calibration: travel pulses ");
  Serial.println(getCalibrationTravelPulses());

  sendCalibrationResponse(
      persisted ? 200 : 500,
      persisted,
      "set-open",
      persisted
          ? "Open position saved permanently"
          : "Open position set, but persistent save failed");
}

void handleCalibrationReset()
{
  disableAutoForManualControl("Calibration reset");
  stopMotor();

  calibrationClosedSet = false;
  calibrationOpenSet = false;
  calibrationClosedCount = 0;
  calibrationOpenCount = 0;
  const bool storageCleared = clearStoredCalibration();

  Serial.println("Calibration: RESET");
  sendCalibrationResponse(
      storageCleared ? 200 : 500,
      storageCleared,
      "reset",
      storageCleared
          ? "Calibration cleared"
          : "Runtime calibration cleared, but storage clear failed");
}

void handleMotorOpen()
{
  disableAutoForManualControl("Manual open command");
  startMotor(true);
  sendMotorResponse("open");
}

void handleMotorOpenUntilLimit()
{
  disableAutoForManualControl("Manual open-limit command");
  if (!LIMIT_SAFETY_ENABLED)
  {
    sendPositionResponse(
        409,
        false,
        "Limit sensors disabled",
        -1);
    return;
  }

  if (isLimitRawActive(closedLimit) && isLimitRawActive(openLimit))
  {
    motorFault = "limit-conflict";
    stopMotor("limit-conflict", isMotorRunning);
    sendMotorStartError("open-until-limit");
    return;
  }

  if (isLimitRawActive(openLimit))
  {
    motorFault = "";
    stopMotor("open-limit", isMotorRunning);
    synchronizeOpenLimitPosition();
    sendMotorResponse("open-until-limit");
    return;
  }

  startMotor(true);
  if (!isMotorRunning)
  {
    sendMotorStartError("open-until-limit");
    return;
  }
  limitTestActive = true;

  Serial.println(
      "Limit test: opening until OPEN sensor or timeout");
  sendMotorResponse("open-until-limit");
}

void handleMotorClose()
{
  disableAutoForManualControl("Manual close command");
  startMotor(false);
  sendMotorResponse("close");
}

void handleMotorCloseUntilLimit()
{
  disableAutoForManualControl("Manual close-limit command");
  if (!LIMIT_SAFETY_ENABLED)
  {
    sendPositionResponse(
        409,
        false,
        "Limit sensors disabled",
        -1);
    return;
  }

  if (isLimitRawActive(closedLimit) && isLimitRawActive(openLimit))
  {
    motorFault = "limit-conflict";
    stopMotor("limit-conflict", isMotorRunning);
    sendMotorStartError("close-until-limit");
    return;
  }

  if (isLimitRawActive(closedLimit))
  {
    motorFault = "";
    stopMotor("closed-limit", isMotorRunning);
    synchronizeClosedLimitPosition();
    sendMotorResponse("close-until-limit");
    return;
  }

  startMotor(false);
  if (!isMotorRunning)
  {
    sendMotorStartError("close-until-limit");
    return;
  }
  limitTestActive = true;

  Serial.println(
      "Limit test: closing until CLOSED sensor or timeout");
  sendMotorResponse("close-until-limit");
}

void handleMotorStop()
{
  disableAutoForManualControl("Manual STOP command");
  positionTargetPercent = -1;
  stopMotor();
  sendMotorResponse("stop");
}

struct PositionMoveResult
{
  int statusCode;
  bool ok;
  bool started;
  String message;
  int targetPercent;
};

PositionMoveResult startPositionMove(int requestedPercent)
{
  if (!LIMIT_SAFETY_ENABLED)
  {
    return {
        409,
        false,
        false,
        "Limit sensors disabled",
        -1};
  }

  if (!calibrationClosedSet || !calibrationOpenSet)
  {
    return {
        409,
        false,
        false,
        "Calibration required",
        -1};
  }

  if (requestedPercent < 0 || requestedPercent > 100)
  {
    return {
        400,
        false,
        false,
        "Percent out of range",
        -1};
  }

  if (
      isLimitRawActive(closedLimit) &&
      isLimitRawActive(openLimit))
  {
    return {
        409,
        false,
        false,
        "Limit sensor conflict",
        requestedPercent};
  }

  const EncoderSnapshot snapshot = readEncoderSnapshot();
  const int64_t travel =
      static_cast<int64_t>(calibrationOpenCount) -
      calibrationClosedCount;
  const int32_t requestedCount = static_cast<int32_t>(
      calibrationClosedCount +
      travel * requestedPercent / 100);

  positionTargetCount = requestedCount;
  positionTargetPercent = requestedPercent;

  const int64_t countDifference =
      static_cast<int64_t>(requestedCount) - snapshot.count;

  if (
      countDifference <= getPositionTolerancePulses() &&
      countDifference >= -getPositionTolerancePulses())
  {
    stopMotor();
    return {
        200,
        true,
        false,
        "Already at target",
        requestedPercent};
  }

  const bool movingTowardOpen = countDifference * travel > 0;

  if (
      (movingTowardOpen && isLimitRawActive(openLimit)) ||
      (!movingTowardOpen && isLimitRawActive(closedLimit)))
  {
    motorFault = "";
    stopMotor(
        movingTowardOpen ? "open-limit" : "closed-limit",
        isMotorRunning);

    if (movingTowardOpen)
    {
      synchronizeOpenLimitPosition();
    }
    else
    {
      synchronizeClosedLimitPosition();
    }

    const int endpointPercent = movingTowardOpen ? 100 : 0;
    const bool requestedEndpoint =
        requestedPercent == endpointPercent;

    return {
        requestedEndpoint ? 200 : 409,
        requestedEndpoint,
        false,
        requestedEndpoint
            ? (movingTowardOpen
                   ? "Already at open limit"
                   : "Already at closed limit")
            : (movingTowardOpen
                   ? "Open limit active"
                   : "Closed limit active"),
        requestedEndpoint
            ? endpointPercent
            : requestedPercent};
  }

  startMotor(movingTowardOpen);

  if (!isMotorRunning)
  {
    return {
        409,
        false,
        false,
        "Motor start was blocked by safety state",
        requestedPercent};
  }

  const int64_t absoluteTravel = travel < 0 ? -travel : travel;
  const int64_t absoluteDistance =
      countDifference < 0 ? -countDifference : countDifference;
  const uint8_t stopLeadPercent =
      movingTowardOpen
          ? POSITION_OPEN_STOP_LEAD_PERCENT
          : POSITION_CLOSE_STOP_LEAD_PERCENT;
  const int64_t fullStopLead =
      absoluteTravel * stopLeadPercent / 100;

  // For a very short move, never place the braking point behind the
  // current position. Half of the requested distance is the safe cap.
  const int64_t appliedStopLead =
      fullStopLead < absoluteDistance / 2
          ? fullStopLead
          : absoluteDistance / 2;
  const int8_t encoderDirection = countDifference > 0 ? 1 : -1;

  positionTargetCount = static_cast<int32_t>(
      static_cast<int64_t>(requestedCount) -
      encoderDirection * appliedStopLead);
  positionTargetPercent = requestedPercent;
  positionTravelDirection = encoderDirection;
  positionLastEncoderCount = snapshot.count;
  positionLastPulseAt = millis();
  positionControlActive = true;

  Serial.print("Position control: requested ");
  Serial.print(requestedPercent);
  Serial.print("% | requested count ");
  Serial.print(requestedCount);
  Serial.print(" | brake count ");
  Serial.print(positionTargetCount);
  Serial.print(" | lead ");
  Serial.print(appliedStopLead);
  Serial.print(" pulse | direction ");
  Serial.println(movingTowardOpen ? "open" : "close");

  return {
      200,
      true,
      true,
      "Moving to target",
      requestedPercent};
}

void handleMotorPosition()
{
  disableAutoForManualControl("Manual position command");
  if (!server.hasArg("percent"))
  {
    sendPositionResponse(400, false, "Missing percent", -1);
    return;
  }

  const String percentText = server.arg("percent");

  if (percentText.length() == 0)
  {
    sendPositionResponse(400, false, "Invalid percent", -1);
    return;
  }

  for (size_t index = 0; index < percentText.length(); index++)
  {
    if (!isDigit(percentText.charAt(index)))
    {
      sendPositionResponse(400, false, "Invalid percent", -1);
      return;
    }
  }

  const PositionMoveResult result =
      startPositionMove(percentText.toInt());
  sendPositionResponse(
      result.statusCode,
      result.ok,
      result.message,
      result.targetPercent);
}

bool parseStrictLong(
    const String &text,
    long minimum,
    long maximum,
    long &result)
{
  if (text.length() == 0)
  {
    return false;
  }

  char *endPointer = nullptr;
  const long parsed = strtol(text.c_str(), &endPointer, 10);

  if (
      endPointer == text.c_str() ||
      *endPointer != '\0' ||
      parsed < minimum ||
      parsed > maximum)
  {
    return false;
  }

  result = parsed;
  return true;
}

bool parseStrictFloat(
    const String &text,
    float minimum,
    float maximum,
    float &result)
{
  if (text.length() == 0)
  {
    return false;
  }

  char *endPointer = nullptr;
  const float parsed = strtof(text.c_str(), &endPointer);

  if (
      endPointer == text.c_str() ||
      *endPointer != '\0' ||
      !isfinite(parsed) ||
      parsed < minimum ||
      parsed > maximum)
  {
    return false;
  }

  result = parsed;
  return true;
}

bool parseBooleanArgument(const char *name, bool &result)
{
  if (!server.hasArg(name))
  {
    return false;
  }

  const String value = server.arg(name);

  if (value == "1" || value == "true")
  {
    result = true;
    return true;
  }

  if (value == "0" || value == "false")
  {
    result = false;
    return true;
  }

  return false;
}

bool isSafePresetId(const String &presetId)
{
  if (presetId.length() == 0 || presetId.length() >= 24)
  {
    return false;
  }

  for (size_t index = 0; index < presetId.length(); index++)
  {
    const char character = presetId.charAt(index);

    if (
        !isAlphaNumeric(character) &&
        character != '-' &&
        character != '_')
    {
      return false;
    }
  }

  return true;
}

unsigned long getAutoCooldownRemainingSeconds(unsigned long currentTime)
{
  if (autoLastActionCompletedAt == 0)
  {
    return 0;
  }

  const unsigned long cooldownMs =
      static_cast<unsigned long>(autoConfig.cooldownSeconds) * 1000UL;
  const unsigned long elapsed =
      currentTime - autoLastActionCompletedAt;

  if (elapsed >= cooldownMs)
  {
    return 0;
  }

  return (cooldownMs - elapsed + 999UL) / 1000UL;
}

void appendAutoStatusJson(String &response)
{
  response += "{\"configured\":";
  response += autoConfig.configured ? "true" : "false";
  response += ",\"enabled\":";
  response += autoConfig.enabled ? "true" : "false";
  response += ",\"presetId\":";
  response += autoConfig.configured
                  ? "\"" + String(autoConfig.presetId) + "\""
                  : "null";
  response += ",\"configRevision\":";
  response += autoConfig.configured
                  ? String(autoConfig.clientRevision)
                  : "null";
  response += ",\"responseStyle\":";
  response += autoConfig.configured
                  ? (autoConfig.responseStyle == AutoResponseStyle::Gentle
                         ? "\"gentle\""
                         : "\"standard\"")
                  : "null";
  response += ",\"gentleStepPercent\":";
  response += autoConfig.configured
                  ? String(autoGentleStepPercent)
                  : "null";
  response += ",\"gentleLastDirection\":";
  response += autoGentleLastDirection > 0
                  ? "\"open\""
                  : autoGentleLastDirection < 0
                        ? "\"close\""
                        : "null";
  response += ",\"gentleResolutionLimited\":";
  response += autoGentleResolutionLimited ? "true" : "false";
  response += ",\"modelATemperatureBandIndex\":";
  response += isModelAConfigured() && autoModelATemperatureBandIndex >= 0
                  ? String(autoModelATemperatureBandIndex)
                  : "null";
  response += ",\"modelAHighLux\":";
  response += isModelAConfigured() && autoModelATemperatureBandIndex >= 0
                  ? String(MODEL_A_HIGH_LUX_BY_BAND[autoModelATemperatureBandIndex], 0)
                  : "null";
  response += ",\"phase\":\"" + autoRuntimePhase + "\"";
  response += ",\"reason\":\"" + autoRuntimeReason + "\"";
  response += ",\"candidateTargetPercent\":";
  response += autoCandidateTargetPercent >= 0
                  ? String(autoCandidateTargetPercent)
                  : "null";
  response += ",\"confirmationCount\":";
  response += String(autoConfirmationCount);
  response += ",\"confirmationRequired\":";
  response += autoConfig.configured
                  ? String(autoConfig.confirmationReadings)
                  : "null";
  response += ",\"cooldownRemainingSeconds\":";
  response += String(getAutoCooldownRemainingSeconds(millis()));
  response += ",\"lastCommandTargetPercent\":";
  response += autoLastCommandTargetPercent >= 0
                  ? String(autoLastCommandTargetPercent)
                  : "null";
  response += ",\"commandActive\":";
  response += autoCommandActive ? "true" : "false";
  response += "}";
}

void sendAutoResponse(
    int statusCode,
    bool ok,
    const String &message)
{
  String response = "{\"ok\":";
  response += ok ? "true" : "false";
  response += ",\"message\":\"" + message + "\",\"auto\":";
  appendAutoStatusJson(response);
  response += "}";

  addCorsHeaders();
  server.sendHeader("Cache-Control", "no-store");
  server.send(statusCode, "application/json", response);
}

void handleAutoStatus()
{
  String response;
  appendAutoStatusJson(response);

  addCorsHeaders();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", response);
}

void disableAutoForManualControl(const char *reason)
{
  if (!autoConfig.configured || !autoConfig.enabled)
  {
    return;
  }

  autoConfig.enabled = false;
  saveAutoConfiguration();
  resetAutoRuntimeState("disabled", reason);
  Serial.print("Auto: disabled by manual control - ");
  Serial.println(reason);
}

void handleAutoConfiguration()
{
  AutoConfiguration nextConfiguration = {};
  nextConfiguration.version = AUTO_STORAGE_VERSION;
  nextConfiguration.configured = true;
  nextConfiguration.enabled = false;

  if (!server.hasArg("presetId"))
  {
    sendAutoResponse(400, false, "Missing preset id");
    return;
  }

  const String presetId = server.arg("presetId");

  if (!isSafePresetId(presetId))
  {
    sendAutoResponse(400, false, "Invalid preset id");
    return;
  }

  snprintf(
      nextConfiguration.presetId,
      sizeof(nextConfiguration.presetId),
      "%s",
      presetId.c_str());

  if (!server.hasArg("responseStyle"))
  {
    sendAutoResponse(400, false, "Missing Auto response style");
    return;
  }

  const String responseStyle = server.arg("responseStyle");
  if (responseStyle == "standard")
  {
    nextConfiguration.responseStyle = AutoResponseStyle::Standard;
  }
  else if (responseStyle == "gentle")
  {
    nextConfiguration.responseStyle = AutoResponseStyle::Gentle;
  }
  else
  {
    sendAutoResponse(400, false, "Invalid Auto response style");
    return;
  }

  if (
      !parseBooleanArgument(
          "temperatureEnabled",
          nextConfiguration.temperatureEnabled) ||
      !parseBooleanArgument(
          "lightEnabled",
          nextConfiguration.lightEnabled))
  {
    sendAutoResponse(400, false, "Invalid sensor enable flags");
    return;
  }

  if (
      !server.hasArg("closeAt") ||
      !parseStrictFloat(
          server.arg("closeAt"),
          -20.0F,
          80.0F,
          nextConfiguration.closeAtOrAboveCelsius) ||
      !server.hasArg("reopenAt") ||
      !parseStrictFloat(
          server.arg("reopenAt"),
          -20.0F,
          80.0F,
          nextConfiguration.reopenAtOrBelowCelsius))
  {
    sendAutoResponse(400, false, "Invalid temperature thresholds");
    return;
  }

  long parsedValue = 0;

  if (
      !server.hasArg("openTarget") ||
      !parseStrictLong(
          server.arg("openTarget"),
          0,
          100,
          parsedValue))
  {
    sendAutoResponse(400, false, "Invalid open target");
    return;
  }
  nextConfiguration.openTargetPercent = parsedValue;

  if (
      !server.hasArg("closedTarget") ||
      !parseStrictLong(
          server.arg("closedTarget"),
          0,
          100,
          parsedValue))
  {
    sendAutoResponse(400, false, "Invalid closed target");
    return;
  }
  nextConfiguration.closedTargetPercent = parsedValue;

  if (
      !server.hasArg("confirmation") ||
      !parseStrictLong(
          server.arg("confirmation"),
          1,
          10,
          parsedValue))
  {
    sendAutoResponse(400, false, "Invalid confirmation count");
    return;
  }
  nextConfiguration.confirmationReadings = parsedValue;

  if (
      !server.hasArg("cooldown") ||
      !parseStrictLong(
          server.arg("cooldown"),
          0,
          3600,
          parsedValue))
  {
    sendAutoResponse(400, false, "Invalid cooldown");
    return;
  }
  nextConfiguration.cooldownSeconds = parsedValue;

  if (
      !server.hasArg("revision") ||
      !parseStrictLong(
          server.arg("revision"),
          0,
          2147483647L,
          parsedValue))
  {
    sendAutoResponse(400, false, "Invalid configuration revision");
    return;
  }
  nextConfiguration.clientRevision =
      static_cast<uint32_t>(parsedValue);

  if (
      !server.hasArg("rangeCount") ||
      !parseStrictLong(
          server.arg("rangeCount"),
          0,
          MAX_AUTO_LIGHT_RANGES,
          parsedValue))
  {
    sendAutoResponse(400, false, "Invalid light range count");
    return;
  }
  nextConfiguration.lightRangeCount = parsedValue;

  for (
      uint8_t index = 0;
      index < nextConfiguration.lightRangeCount;
      index++)
  {
    const String prefix = "range" + String(index);
    const String minName = prefix + "Min";
    const String maxName = prefix + "Max";
    const String targetName = prefix + "Target";
    AutoLightRange &range = nextConfiguration.lightRanges[index];

    if (
        !server.hasArg(minName) ||
        !parseStrictFloat(
            server.arg(minName),
            0.0F,
            1000000.0F,
            range.minLux))
    {
      sendAutoResponse(400, false, "Invalid light range minimum");
      return;
    }

    if (!server.hasArg(maxName))
    {
      sendAutoResponse(400, false, "Missing light range maximum");
      return;
    }

    const String maxText = server.arg(maxName);
    range.openEnded = maxText == "null";

    if (
        !range.openEnded &&
        !parseStrictFloat(
            maxText,
            0.0F,
            1000000.0F,
            range.maxLux))
    {
      sendAutoResponse(400, false, "Invalid light range maximum");
      return;
    }

    if (
        !server.hasArg(targetName) ||
        !parseStrictLong(
            server.arg(targetName),
            0,
            100,
            parsedValue))
    {
      sendAutoResponse(400, false, "Invalid light range target");
      return;
    }
    range.targetPercent = parsedValue;
  }

  String validationError;
  if (!validateAutoConfiguration(nextConfiguration, validationError))
  {
    sendAutoResponse(400, false, validationError);
    return;
  }

  if (autoCommandActive && isMotorRunning)
  {
    stopMotor(
        "auto-config-updated",
        true,
        POSITION_BRAKE_DUTY,
        POSITION_BRAKE_DURATION_MS);
  }

  autoConfig = nextConfiguration;
  resetAutoRuntimeState(
      "dry-run",
      "Configuration saved; Auto remains disabled");

  if (!saveAutoConfiguration())
  {
    sendAutoResponse(500, false, "Unable to save Auto configuration");
    return;
  }

  sendAutoResponse(
      200,
      true,
      "Auto configuration saved in dry-run mode");
}

void handleAutoModelAConfiguration()
{
  if (isMotorRunning)
  {
    sendAutoResponse(409, false, "Stop the motor before configuring Model A");
    return;
  }

  AutoConfiguration nextConfiguration = {};
  nextConfiguration.version = AUTO_STORAGE_VERSION;
  nextConfiguration.configured = true;
  nextConfiguration.enabled = false;
  nextConfiguration.responseStyle = AutoResponseStyle::Gentle;
  nextConfiguration.temperatureEnabled = true;
  nextConfiguration.lightEnabled = true;
  nextConfiguration.confirmationReadings = 2;
  nextConfiguration.cooldownSeconds = 15;
  nextConfiguration.clientRevision = 1;
  snprintf(
      nextConfiguration.presetId,
      sizeof(nextConfiguration.presetId),
      "%s",
      MODEL_A_PRESET_ID);

  autoConfig = nextConfiguration;
  resetAutoRuntimeState(
      "dry-run",
      "Model A configured; Auto remains disabled");

  if (!saveAutoConfiguration())
  {
    sendAutoResponse(500, false, "Unable to save Model A configuration");
    return;
  }

  sendAutoResponse(200, true, "Model A configured in dry-run mode");
}

void handleAutoEnable()
{
  String validationError;
  if (!validateAutoConfiguration(autoConfig, validationError))
  {
    sendAutoResponse(409, false, validationError);
    return;
  }

  if (!LIMIT_SAFETY_ENABLED)
  {
    sendAutoResponse(409, false, "Limit safety is disabled");
    return;
  }

  if (
      isLimitRawActive(closedLimit) &&
      isLimitRawActive(openLimit))
  {
    sendAutoResponse(409, false, "Limit sensor conflict");
    return;
  }

  if (!calibrationClosedSet || !calibrationOpenSet)
  {
    sendAutoResponse(409, false, "Calibration required");
    return;
  }

  if (isMotorRunning)
  {
    sendAutoResponse(409, false, "Motor is currently running");
    return;
  }

  if (motorFault.length() > 0)
  {
    sendAutoResponse(409, false, "Clear motor fault before enabling Auto");
    return;
  }

  if (
      (autoConfig.temperatureEnabled &&
       isnan(currentTemperature)) ||
      (autoConfig.lightEnabled && isnan(currentLux)))
  {
    sendAutoResponse(409, false, "Required sensor data is unavailable");
    return;
  }

  autoConfig.enabled = true;
  resetAutoRuntimeState(
      "waiting",
      "Auto enabled; waiting for fresh sensor readings");

  if (!saveAutoConfiguration())
  {
    autoConfig.enabled = false;
    sendAutoResponse(500, false, "Unable to persist Auto state");
    return;
  }

  Serial.println("Auto: continuous mode enabled");
  sendAutoResponse(200, true, "Continuous Auto enabled");
}

void handleAutoDisable()
{
  if (autoCommandActive && isMotorRunning)
  {
    stopMotor(
        "auto-disabled",
        true,
        POSITION_BRAKE_DUTY,
        POSITION_BRAKE_DURATION_MS);
  }

  autoConfig.enabled = false;
  resetAutoRuntimeState("disabled", "Auto disabled by user");

  const bool saved =
      !autoConfig.configured || saveAutoConfiguration();
  Serial.println("Auto: continuous mode disabled");
  sendAutoResponse(
      saved ? 200 : 500,
      saved,
      saved ? "Continuous Auto disabled" : "Unable to persist Auto state");
}

bool evaluateAutoTargetPercent(
    int &targetPercent,
    String &reason)
{
  targetPercent = -1;

  if (
      autoConfig.temperatureEnabled &&
      isnan(currentTemperature))
  {
    reason = "Waiting for temperature data";
    return false;
  }

  if (autoConfig.lightEnabled && isnan(currentLux))
  {
    reason = "Waiting for light data";
    return false;
  }

  if (
      autoConfig.temperatureEnabled &&
      currentTemperature >= autoConfig.closeAtOrAboveCelsius)
  {
    targetPercent = autoConfig.closedTargetPercent;
    reason = "Temperature reached the close threshold";
    return true;
  }

  if (autoConfig.lightEnabled)
  {
    for (
        uint8_t index = 0;
        index < autoConfig.lightRangeCount;
        index++)
    {
      const AutoLightRange &range = autoConfig.lightRanges[index];
      const bool matches =
          currentLux >= range.minLux &&
          (range.openEnded || currentLux < range.maxLux);

      if (matches)
      {
        targetPercent = range.targetPercent;
        reason = "Light level matched configured range";
        return true;
      }
    }

    reason = "No configured light range matched";
    return false;
  }

  if (
      autoConfig.temperatureEnabled &&
      currentTemperature <= autoConfig.reopenAtOrBelowCelsius)
  {
    targetPercent = autoConfig.openTargetPercent;
    reason = "Temperature reached the reopen threshold";
    return true;
  }

  reason = "Temperature is inside the hold band";
  return false;
}

bool getCurrentPositionPercent(float &positionPercent)
{
  if (!calibrationClosedSet || !calibrationOpenSet)
  {
    return false;
  }

  const int32_t travel = calibrationOpenCount - calibrationClosedCount;
  if (!isValidCalibrationTravel(travel))
  {
    return false;
  }

  const EncoderSnapshot snapshot = readEncoderSnapshot();
  positionPercent =
      static_cast<float>(snapshot.count - calibrationClosedCount) *
      100.0F / static_cast<float>(travel);
  positionPercent = constrain(positionPercent, 0.0F, 100.0F);
  return true;
}

bool applyAutoResponseStyle(
    int desiredTargetPercent,
    int &commandTargetPercent,
    uint8_t &proposedStepPercent,
    int8_t &proposedDirection,
    String &reason)
{
  commandTargetPercent = desiredTargetPercent;
  proposedStepPercent = autoGentleStepPercent;
  proposedDirection = 0;

  if (autoConfig.responseStyle == AutoResponseStyle::Standard)
  {
    return true;
  }

  if (autoGentleResolutionLimited)
  {
    reason = "Gentle response is paused at minimum resolution";
    return false;
  }

  float currentPositionPercent = 0.0F;
  if (!getCurrentPositionPercent(currentPositionPercent))
  {
    reason = "Gentle response requires an Encoder position";
    return false;
  }

  const float difference = desiredTargetPercent - currentPositionPercent;
  if (fabsf(difference) <= AUTO_POSITION_TOLERANCE_PERCENT)
  {
    return true;
  }

  proposedDirection = difference > 0.0F ? 1 : -1;
  const bool isDirectionReversal =
      autoGentleLastDirection != 0 &&
      autoGentleLastDirection != proposedDirection;

  if (isDirectionReversal)
  {
    if (autoGentleStepPercent <= AUTO_GENTLE_MIN_STEP_PERCENT)
    {
      autoGentleResolutionLimited = true;
      reason = "Gentle response stopped after reversing at minimum step";
      return false;
    }

    proposedStepPercent = autoGentleStepPercent / 2;
    if (proposedStepPercent < AUTO_GENTLE_MIN_STEP_PERCENT)
    {
      proposedStepPercent = AUTO_GENTLE_MIN_STEP_PERCENT;
    }
  }

  const float step = min(
      fabsf(difference),
      static_cast<float>(proposedStepPercent));
  commandTargetPercent = constrain(
      static_cast<int>(roundf(currentPositionPercent + proposedDirection * step)),
      0,
      100);
  reason += "; gentle step " + String(proposedStepPercent) + "%";
  return true;
}

int8_t getModelATemperatureBand(float temperatureCelsius)
{
  constexpr int8_t lastBand = 4;
  int8_t band = autoModelATemperatureBandIndex;

  if (band < 0 || band > lastBand)
  {
    band = 0;
    while (
        band < lastBand &&
        temperatureCelsius >= MODEL_A_TEMPERATURE_THRESHOLDS[band])
    {
      band++;
    }
    return band;
  }

  while (
      band < lastBand &&
      temperatureCelsius >=
          MODEL_A_TEMPERATURE_THRESHOLDS[band] +
              MODEL_A_TEMPERATURE_HYSTERESIS_CELSIUS)
  {
    band++;
  }

  while (
      band > 0 &&
      temperatureCelsius <
          MODEL_A_TEMPERATURE_THRESHOLDS[band - 1] -
              MODEL_A_TEMPERATURE_HYSTERESIS_CELSIUS)
  {
    band--;
  }

  return band;
}

bool evaluateModelATargetPercent(
    int &targetPercent,
    uint8_t &proposedStepPercent,
    int8_t &proposedDirection,
    String &reason)
{
  targetPercent = -1;
  proposedStepPercent = autoGentleStepPercent;
  proposedDirection = 0;

  if (!isfinite(currentTemperature))
  {
    reason = "Model A is waiting for temperature data";
    return false;
  }

  autoModelATemperatureBandIndex =
      getModelATemperatureBand(currentTemperature);

  if (!isfinite(currentLux) || currentLux < 0.0F)
  {
    reason = "Model A is waiting for light data";
    return false;
  }

  if (autoGentleResolutionLimited)
  {
    reason = "Model A paused at minimum step; inspect position before enabling again";
    return false;
  }

  if (
      currentTemperature >= MODEL_A_HOT_DARK_CELSIUS &&
      currentLux < MODEL_A_LOW_LUX)
  {
    reason = "Model A holds because the room is hot but dark";
    return false;
  }

  const float highLux =
      MODEL_A_HIGH_LUX_BY_BAND[autoModelATemperatureBandIndex];
  if (currentLux > highLux)
  {
    proposedDirection = -1;
  }
  else if (currentLux < MODEL_A_LOW_LUX)
  {
    proposedDirection = 1;
  }
  else
  {
    reason = "Model A light level is inside the acceptable band";
    return false;
  }

  float currentPositionPercent = 0.0F;
  if (!getCurrentPositionPercent(currentPositionPercent))
  {
    reason = "Model A requires a calibrated Encoder position";
    return false;
  }

  if (
      autoGentleLastDirection != 0 &&
      autoGentleLastDirection != proposedDirection)
  {
    if (autoGentleStepPercent <= AUTO_GENTLE_MIN_STEP_PERCENT)
    {
      autoGentleResolutionLimited = true;
      reason = "Model A reversed at minimum step; Auto disabled until reviewed";
      return false;
    }

    proposedStepPercent = max(
        static_cast<uint8_t>(AUTO_GENTLE_MIN_STEP_PERCENT),
        static_cast<uint8_t>(autoGentleStepPercent / 2));
  }

  const float nextPosition = constrain(
      currentPositionPercent +
          proposedDirection * static_cast<float>(proposedStepPercent),
      0.0F,
      100.0F);
  if (
      fabsf(nextPosition - currentPositionPercent) <=
      AUTO_POSITION_TOLERANCE_PERCENT)
  {
    reason = "Model A holds at the travel boundary";
    return false;
  }

  targetPercent =
      constrain(static_cast<int>(roundf(nextPosition)), 0, 100);
  reason = proposedDirection > 0
               ? "Model A opens one step because light is low"
               : "Model A closes one step because light exceeds the temperature budget";
  return true;
}

void updateAutoRuntimeAfterSensorRead(unsigned long currentTime)
{
  if (!autoConfig.configured)
  {
    resetAutoRuntimeState(
        "not-configured",
        "Auto configuration has not been saved");
    return;
  }

  if (autoCommandActive)
  {
    autoRuntimePhase = "moving";
    autoRuntimeReason = "Auto position command is active";
    return;
  }

  const bool modelA = isModelAConfigured();
  if (modelA && autoConfig.enabled)
  {
    if (motorFault.length() > 0)
    {
      autoConfig.enabled = false;
      saveAutoConfiguration();
      autoCandidateTargetPercent = -1;
      autoConfirmationCount = 0;
      autoRuntimePhase = "fault";
      autoRuntimeReason = "Motor fault disabled Model A";
      return;
    }

    if (!calibrationClosedSet || !calibrationOpenSet)
    {
      autoCandidateTargetPercent = -1;
      autoConfirmationCount = 0;
      autoRuntimePhase = "waiting";
      autoRuntimeReason = "Model A requires Homing after power-up";
      return;
    }

    if (getAutoCooldownRemainingSeconds(currentTime) > 0)
    {
      autoCandidateTargetPercent = -1;
      autoConfirmationCount = 0;
      autoRuntimePhase = "cooldown";
      autoRuntimeReason = "Waiting for Model A cooldown";
      return;
    }
  }

  int desiredTargetPercent = -1;
  String decisionReason;
  uint8_t proposedStepPercent = autoGentleStepPercent;
  int8_t proposedDirection = 0;
  const bool hasTarget = modelA
                             ? evaluateModelATargetPercent(
                                   desiredTargetPercent,
                                   proposedStepPercent,
                                   proposedDirection,
                                   decisionReason)
                             : evaluateAutoTargetPercent(
                                   desiredTargetPercent,
                                   decisionReason);

  if (!hasTarget)
  {
    autoCandidateTargetPercent = -1;
    autoConfirmationCount = 0;
    if (modelA && autoGentleResolutionLimited && autoConfig.enabled)
    {
      autoConfig.enabled = false;
      saveAutoConfiguration();
      autoRuntimePhase = "resolution-limited";
      autoRuntimeReason = decisionReason;
      return;
    }
    autoRuntimePhase = autoConfig.enabled ? "holding" : "dry-run";
    autoRuntimeReason = decisionReason;
    return;
  }

  int nextTargetPercent = desiredTargetPercent;
  if (!modelA && !applyAutoResponseStyle(
          desiredTargetPercent,
          nextTargetPercent,
          proposedStepPercent,
          proposedDirection,
          decisionReason))
  {
    autoCandidateTargetPercent = -1;
    autoConfirmationCount = 0;
    autoRuntimePhase = autoConfig.enabled ? "holding" : "dry-run";
    autoRuntimeReason = decisionReason;
    return;
  }

  if (nextTargetPercent == autoCandidateTargetPercent)
  {
    if (autoConfirmationCount < autoConfig.confirmationReadings)
    {
      autoConfirmationCount++;
    }
  }
  else
  {
    autoCandidateTargetPercent = nextTargetPercent;
    autoConfirmationCount = 1;
  }

  autoRuntimeReason = decisionReason;

  if (autoConfirmationCount < autoConfig.confirmationReadings)
  {
    autoRuntimePhase = "confirming";
    return;
  }

  if (!autoConfig.enabled)
  {
    autoRuntimePhase = "dry-run-ready";
    return;
  }

  const unsigned long cooldownRemaining =
      getAutoCooldownRemainingSeconds(currentTime);
  if (cooldownRemaining > 0)
  {
    autoRuntimePhase = "cooldown";
    autoRuntimeReason = "Waiting for Auto cooldown";
    return;
  }

  if (motorFault.length() > 0)
  {
    autoConfig.enabled = false;
    saveAutoConfiguration();
    autoRuntimePhase = "fault";
    autoRuntimeReason = "Motor fault disabled Auto";
    return;
  }

  const PositionMoveResult result =
      startPositionMove(nextTargetPercent);

  if (!result.ok)
  {
    autoRuntimePhase = "blocked";
    autoRuntimeReason = result.message;
    return;
  }

  autoLastCommandTargetPercent = nextTargetPercent;
  autoCandidateTargetPercent = -1;
  autoConfirmationCount = 0;

  if (!result.started)
  {
    autoRuntimePhase = "holding";
    autoRuntimeReason = result.message;
    return;
  }

  if (autoConfig.responseStyle == AutoResponseStyle::Gentle)
  {
    autoGentleStepPercent = proposedStepPercent;
    autoGentleLastDirection = proposedDirection;
  }

  autoCommandActive = true;
  autoRuntimePhase = "moving";
  autoRuntimeReason = "Auto position command is active";

  Serial.print("Auto: moving to ");
  Serial.print(nextTargetPercent);
  Serial.println("%");
}

void updateAutoCommandState(unsigned long currentTime)
{
  if (!autoCommandActive || isMotorRunning)
  {
    return;
  }

  autoCommandActive = false;
  autoCandidateTargetPercent = -1;
  autoConfirmationCount = 0;

  if (motorFault.length() > 0)
  {
    autoConfig.enabled = false;
    saveAutoConfiguration();
    autoRuntimePhase = "fault";
    autoRuntimeReason = "Motor fault disabled Auto";
    Serial.println("Auto: disabled after motor fault");
    return;
  }

  autoLastActionCompletedAt = currentTime;
  autoRuntimePhase = "cooldown";
  autoRuntimeReason = "Auto movement completed; cooldown started";
  Serial.println("Auto: movement complete, cooldown started");
}

bool connectToStation()
{
  if (strlen(WIFI_SSID) == 0)
  {
    Serial.println("Wi-Fi station skipped: SSID is empty");
    return false;
  }

  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(WIFI_SSID);

  accessPointFallbackActive = false;
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const unsigned long startedAt = millis();

  while (
      WiFi.status() != WL_CONNECTED &&
      millis() - startedAt < WIFI_TIMEOUT_MS)
  {
    delay(250);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Wi-Fi station connection failed");
    WiFi.disconnect();
    return false;
  }

  Serial.println("Wi-Fi station connected");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi RSSI: ");
  Serial.println(WiFi.RSSI());
  stationConnectionWasReady = true;
  return true;
}

void applyNetworkDisconnectFailSafe(unsigned long currentTime)
{
  const bool motorWasRunning = isMotorRunning;
  stopMotor(
      "network-lost",
      motorWasRunning,
      POSITION_BRAKE_DUTY,
      POSITION_BRAKE_DURATION_MS);

  const bool autoWasEnabled = autoConfig.configured && autoConfig.enabled;
  autoConfig.enabled = false;
  resetAutoRuntimeState(
      "disabled",
      "Wi-Fi disconnected; Manual mode required");

  if (autoWasEnabled && !saveAutoConfiguration())
  {
    Serial.println(
        "Auto safety: unable to persist disabled state after Wi-Fi loss");
  }

  networkFailSafeTriggered = true;
  lastNetworkDisconnectAt = currentTime;
  Serial.println(
      "Network safety: Wi-Fi lost; motor stopped and Manual mode required");
}

void updateNetworkDisconnectSafety(unsigned long currentTime)
{
  if (accessPointFallbackActive)
  {
    return;
  }

  const bool stationConnected = WiFi.status() == WL_CONNECTED;

  if (stationConnected)
  {
    stationConnectionWasReady = true;
    return;
  }

  if (!stationConnectionWasReady)
  {
    return;
  }

  // Handle each connected -> disconnected transition once. Auto reconnect may
  // restore the network, but it never re-enables Auto automatically.
  stationConnectionWasReady = false;
  applyNetworkDisconnectFailSafe(currentTime);
}

bool startFallbackAccessPoint()
{
  Serial.println("Starting fallback Wi-Fi Access Point");

  WiFi.mode(WIFI_AP);

  const bool started = WiFi.softAP(
      FALLBACK_AP_SSID,
      FALLBACK_AP_PASSWORD);

  if (!started)
  {
    Serial.println("Access Point failed");
    return false;
  }

  accessPointFallbackActive = true;

  Serial.println("Access Point started");
  Serial.print("Wi-Fi name: ");
  Serial.println(FALLBACK_AP_SSID);

  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.softAPIP());
  return true;
}

void startNetworkAndServer()
{
  if (!connectToStation())
  {
    startFallbackAccessPoint();
  }

  server.on("/health", HTTP_GET, handleHealth);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/sensors", HTTP_GET, handleSensors);
  server.on("/auto/status", HTTP_GET, handleAutoStatus);
  server.on("/auto/config", HTTP_POST, handleAutoConfiguration);
  server.on("/auto/model-a", HTTP_POST, handleAutoModelAConfiguration);
  server.on("/auto/enable", HTTP_POST, handleAutoEnable);
  server.on("/auto/disable", HTTP_POST, handleAutoDisable);

  server.on("/status", HTTP_OPTIONS, handleCorsPreflight);
  server.on("/auto/status", HTTP_OPTIONS, handleCorsPreflight);
  server.on("/auto/config", HTTP_OPTIONS, handleCorsPreflight);
  server.on("/auto/model-a", HTTP_OPTIONS, handleCorsPreflight);
  server.on("/auto/enable", HTTP_OPTIONS, handleCorsPreflight);
  server.on("/auto/disable", HTTP_OPTIONS, handleCorsPreflight);

  server.on("/motor/open", HTTP_POST, handleMotorOpen);
  server.on(
      "/motor/open-until-limit",
      HTTP_POST,
      handleMotorOpenUntilLimit);
  server.on("/motor/close", HTTP_POST, handleMotorClose);
  server.on(
      "/motor/close-until-limit",
      HTTP_POST,
      handleMotorCloseUntilLimit);
  server.on("/motor/stop", HTTP_POST, handleMotorStop);
  server.on("/motor/position", HTTP_POST, handleMotorPosition);

  server.on(
      "/calibration/closed",
      HTTP_POST,
      handleCalibrationClosed);
  server.on(
      "/calibration/open",
      HTTP_POST,
      handleCalibrationOpen);
  server.on(
      "/calibration/reset",
      HTTP_POST,
      handleCalibrationReset);

  server.on("/motor/open", HTTP_OPTIONS, handleCorsPreflight);
  server.on(
      "/motor/open-until-limit",
      HTTP_OPTIONS,
      handleCorsPreflight);
  server.on("/motor/close", HTTP_OPTIONS, handleCorsPreflight);
  server.on(
      "/motor/close-until-limit",
      HTTP_OPTIONS,
      handleCorsPreflight);
  server.on("/motor/stop", HTTP_OPTIONS, handleCorsPreflight);
  server.on("/motor/position", HTTP_OPTIONS, handleCorsPreflight);
  server.on(
      "/calibration/closed",
      HTTP_OPTIONS,
      handleCorsPreflight);
  server.on(
      "/calibration/open",
      HTTP_OPTIONS,
      handleCorsPreflight);
  server.on(
      "/calibration/reset",
      HTTP_OPTIONS,
      handleCorsPreflight);

  server.begin();
}

void setup()
{
  // ทำให้มอเตอร์หยุดก่อนเริ่ม Wi-Fi
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_ENA, PWM_CHANNEL);

  Serial.begin(115200);
  delay(500);

  stopMotor("restart-manual");
  loadStoredCalibration();
  loadStoredAutoConfiguration();
  initializeSensors();
  initializeEncoder();
  initializeLimitSensors();
  startNetworkAndServer();

  Serial.println("ESP32 ready");
  Serial.println("o = forward, c = reverse, s = stop");
}

void loop()
{
  // Safety is checked before serving a web request so an active limit is
  // handled with the smallest possible software delay.
  unsigned long currentTime = millis();
  updateLimitSensors(currentTime);
  updateLimitSafety();
  updateNetworkDisconnectSafety(currentTime);

  const bool motorWasRunningForWeb = isMotorRunning;
  const uint32_t webStartedAtUs =
      static_cast<uint32_t>(esp_timer_get_time());
  server.handleClient();
  if (motorWasRunningForWeb && isMotorRunning)
  {
    const uint32_t webDurationUs =
        static_cast<uint32_t>(esp_timer_get_time()) - webStartedAtUs;
    if (webDurationUs > maxWebHandlerUs)
    {
      maxWebHandlerUs = webDurationUs;
    }
  }

  if (Serial.available() > 0)
  {
    const char command = Serial.read();

    if (command == 'o')
    {
      disableAutoForManualControl("Serial open command");
      startMotor(true);
    }
    else if (command == 'c')
    {
      disableAutoForManualControl("Serial close command");
      startMotor(false);
    }
    else if (command == 's')
    {
      disableAutoForManualControl("Serial STOP command");
      stopMotor();
    }
  }

  // Check again after commands because a request may have just started
  // the motor, or a limit may have changed while the request was handled.
  currentTime = millis();
  updateLimitSensors(currentTime);
  updateLimitSafety();

  if (isMotorRunning && !positionControlActive)
  {
    const unsigned long timeoutMs =
        limitTestActive
            ? LIMIT_TEST_TIMEOUT_MS
            : TEST_DURATION_MS;

    if (currentTime - motorStartedAt >= timeoutMs)
    {
      if (limitTestActive)
      {
        const bool wasOpening =
            motorDirection == MotorDirection::Opening;
        motorFault = wasOpening
                         ? "open-limit-timeout"
                         : "closed-limit-timeout";
        Serial.println(
            "Limit test: STOP - limit sensor timeout");
        stopMotor(
            wasOpening
                ? "open-limit-timeout"
                : "closed-limit-timeout",
            true);
      }
      else
      {
        stopMotor("jog-timeout");
      }
    }
  }

  updatePositionControl(currentTime);
  updateAutoCommandState(currentTime);

  reportEncoderActivity(currentTime);
  updateLimitStopDiagnostic(millis());

  // Sensor reads (DHT22 / I2C) can block the main loop. Auto does not need
  // a fresh sample while a move is in progress, so leave limit polling free.
  if (
      !isMotorRunning &&
      currentTime - lastSensorReadAt >= SENSOR_INTERVAL_MS)
  {
    lastSensorReadAt = currentTime;
    readSensors();
    currentTime = millis();
    updateLimitSensors(currentTime);
    updateLimitSafety();
    updateAutoRuntimeAfterSensorRead(currentTime);
  }
}
