/**
 * @file mr_hyde_edge_node.ino
 * @author Diego Andrés Delgado Andrade (Mr_Hyde Industries)
 * @brief Environmental and Biometric Telemetry Edge Gateway for SmartGym.
 * @version 2.77
 * @date 2026-11-15
 * 
 * @details Computer Architecture - UCACUE (EVEA Activity No. 5).
 * Implements non-blocking multisensor acquisition on the ESP32-S3 (Dual-Core Xtensa LX7),
 * moving-average digital filtering at the edge (Edge Computing), autonomous PWM/RGB control,
 * a tactical Cyberpunk 2077 graphical interface on an SSD1306 OLED, and resilient MQTT transmission
 * to HiveMQ Cloud backed by a circular FIFO buffer in SRAM.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <cmath>
#include "MAX30105.h"
#include "heartRate.h"

namespace mr_hyde::edge {

/**
 * @struct NetworkConfig
 * @brief Network connectivity, NTP time synchronization and MQTT broker parameters.
 * @note Defined as compile-time constant expressions (Flash/RODATA)
 *       to optimize SRAM usage on the microcontroller.
 */
struct NetworkConfig {
    static constexpr const char* WIFI_SSID = "CATO-WIFI-2G";         ///< SSID of the 2.4 GHz Wi-Fi network
    static constexpr const char* WIFI_PASS = "";                   ///< WPA2-PSK credential
    static constexpr const char* MQTT_BROKER = "broker.hivemq.com";         ///< Host of the public cloud broker
    static constexpr uint16_t MQTT_PORT = 1883;                             ///< Standard TCP port for MQTT without TLS
    static constexpr const char* MQTT_TOPIC = "smartgym/ucacue/diego/nodo01"; ///< Hierarchical publication topic
    static constexpr const char* DEVICE_ID = "MR_HYDE_NODE_01";             ///< Unique identifier of the Edge node
    static constexpr const char* NTP_SERVER = "pool.ntp.org";               ///< Time server for the Unix Epoch timestamp
    static constexpr long GMT_OFFSET_SEC = -5 * 3600;                       ///< Mainland Ecuador time zone (UTC-5)
    static constexpr int DAYLIGHT_OFFSET_SEC = 0;                           ///< No daylight saving time
};

/**
 * @struct HardwarePins
 * @brief Physical GPIO pin mapping on the ESP32-S3 SoC (N16R8).
 */
struct HardwarePins {
    static constexpr uint8_t I2C_SCL = 4;   ///< Clock of the shared I2C bus (OLED, MPU6050, MAX30105)
    static constexpr uint8_t I2C_SDA = 5;   ///< Data line of the shared I2C bus (400 kHz Fast Mode)
    static constexpr uint8_t ONEWIRE = 6;   ///< 1-Wire digital bus for the DS18B20 temperature probe
    static constexpr uint8_t LDR = 7;       ///< Digital input of the ambient light optical comparator
    static constexpr uint8_t GAS = 1;       ///< Analog channel ADC1_CH0 (12 bits: 0-4095) for the MQ sensor
    static constexpr uint8_t LED_R = 15;    ///< Red PWM channel of the RGB status indicator
    static constexpr uint8_t LED_G = 16;    ///< Green PWM channel of the RGB status indicator
    static constexpr uint8_t LED_B = 17;    ///< Blue PWM channel of the RGB status indicator
    static constexpr uint8_t FAN = 18;      ///< PWM output (8 bits: 0-255) to the fan power stage
};

/**
 * @struct EdgeThresholds
 * @brief Buffer sizes, physical constants and local (Edge) decision thresholds.
 */
struct EdgeThresholds {
    static constexpr uint8_t SCREEN_WIDTH = 128;         ///< OLED horizontal resolution in pixels
    static constexpr uint8_t SCREEN_HEIGHT = 64;         ///< OLED vertical resolution in pixels
    static constexpr int8_t OLED_RESET = -1;             ///< Reset shared with the board bus
    static constexpr uint8_t OLED_ADDR = 0x3C;           ///< I2C slave address of the SSD1306 controller

    static constexpr size_t MOVING_AVG_SIZE = 10;        ///< Moving-average filter window (noise smoothing)
    static constexpr size_t OFFLINE_BUFFER_SIZE = 100;   ///< Capacity of the circular FIFO queue in RAM during network outages
    static constexpr size_t ECG_WIDTH = 46;              ///< Width in pixels of the cardiographic trace on the HUD
    static constexpr uint8_t RATE_SIZE = 4;              ///< Averaging window for BPM stabilization

    static constexpr float TEMP_MIN_PWM = 21.0f;         ///< Base temperature (°C) for fan activation (30% PWM)
    static constexpr float TEMP_MAX_PWM = 28.0f;         ///< Critical temperature (°C) for maximum heat dissipation (100% PWM)
    static constexpr int GAS_ALERT_ADC = 2700;           ///< Critical air contamination threshold (~2.17V on a 12-bit ADC)
    static constexpr float VIBRATION_IMPACT_G = 1.8f;    ///< Net dynamic acceleration threshold (g) for impact alert
    static constexpr float GRAVITY_MS2 = 9.80665f;       ///< Standard Earth gravitational acceleration (m/s^2)
};

/**
 * @enum SystemState
 * @brief Finite-state machine (FSM) that governs the operational priority of the node.
 */
enum class SystemState {
    NORMAL,     ///< Nominal parameters: Green LED, fan off
    BIOMETRIA,  ///< User measuring pulse without thermal alert: Blue LED
    MODERADO,   ///< Proportional active climate control (30-90% PWM): Orange LED
    CRITICO     ///< Temperature/Gas at the limit (100% PWM) or impact > 1.8g: Red LED + Cloud Alert
};

/**
 * @struct TelemetryRecord
 * @brief Packed structure of a telemetry frame for JSON serialization or RAM storage.
 */
struct TelemetryRecord {
    unsigned long timestamp;    ///< Unix Epoch time (seconds since 1970-01-01 UTC)
    float temp_c;               ///< Filtered temperature in degrees Celsius
    int gas_adc;                ///< Filtered raw value of the analog-to-digital converter (0-4095)
    bool light_present;         ///< Room lighting state (true = lit, false = dark)
    float vibration_peak_g;     ///< Maximum vibration peak recorded in the sampling window (g)
    int bpm_user;               ///< Average heart rate of the user in beats per minute
    int fan_duty_pct;           ///< Current duty cycle applied to the fan (0-100%)
    SystemState status;         ///< Operational state of the system at the instant of capture
};

/**
 * @class EdgeNodeController
 * @brief Main controller of the Edge Gateway. Encapsulates the hardware drivers,
 *        signal filtering, the Cyberpunk graphical interface and the MQTT client.
 */
class EdgeNodeController {
private:
    // --- Hardware bus and peripheral instances ---
    WiFiClient espClient_;
    PubSubClient mqttClient_;
    Adafruit_SSD1306 display_;
    OneWire oneWire_;
    DallasTemperature tempSensor_;
    Adafruit_MPU6050 mpu_;
    MAX30105 particleSensor_;

    // --- Circular FIFO buffer in RAM for offline resilience ---
    TelemetryRecord offlineBuffer_[EdgeThresholds::OFFLINE_BUFFER_SIZE];
    size_t bufferHead_ = 0;     ///< Write index of the circular queue
    size_t bufferCount_ = 0;    ///< Current number of retained frames pending transmission

    // --- Moving-average digital filter arrays (Moving Average) ---
    float tempSamples_[EdgeThresholds::MOVING_AVG_SIZE]{};
    int gasSamples_[EdgeThresholds::MOVING_AVG_SIZE]{};
    size_t filterIndex_ = 0;
    bool filterFilled_ = false;

    // --- Photoplethysmographic processing variables (PPG - MAX30105) ---
    uint8_t rates_[EdgeThresholds::RATE_SIZE]{};
    uint8_t rateSpot_ = 0;
    long lastBeat_ = 0;
    unsigned long lastFingerSeen_ = 0;
    int beatAvg_ = 0;

    // --- Graphics buffer of the ECG oscilloscope on the OLED display ---
    uint8_t ecgHistory_[EdgeThresholds::ECG_WIDTH]{};
    bool beatSpikePending_ = false;

    // --- Time registers for the non-blocking cooperative scheduler ---
    unsigned long lastImuSample_ = 0;       ///< Inertial sampling timer at 50 Hz (20 ms)
    unsigned long lastClimateSample_ = 0;   ///< Environmental and control timer at 0.5 Hz (2000 ms)
    unsigned long lastOledRefresh_ = 0;     ///< HUD rendering timer at ~6.6 FPS (150 ms)
    unsigned long lastTelemetrySend_ = 0;   ///< Periodic MQTT publication timer at 0.1 Hz (10000 ms)
    unsigned long lastMqttReconnect_ = 0;   ///< Non-blocking network link retry timer
    unsigned long criticalFlashUntil_ = 0;  ///< Timestamp for the strobe inversion of the display on impacts
    uint8_t animFrame_ = 0;                 ///< Frame counter for the blade and trace animation

    // --- Current state of the variables processed at the edge ---
    float filteredTemp_ = 0.0f;
    int filteredGas_ = 0;
    bool hasLight_ = true;
    float currentVibrationG_ = 0.0f;
    float peakVibrationWindowG_ = 0.0f;
    int fanDutyPct_ = 0;
    SystemState currentState_ = SystemState::NORMAL;

    bool hasMPU_ = false;   ///< Flag for the physical presence of the MPU6050 accelerometer on the I2C bus
    bool hasMAX_ = false;   ///< Flag for the physical presence of the MAX30105 biometric sensor on the I2C bus

    /**
     * @brief Converts the internal state enumerator to its text representation for the JSON payload.
     */
    static const char* stateToString(SystemState state) {
        switch (state) {
            case SystemState::CRITICO:   return "CRITICO";
            case SystemState::MODERADO:  return "MODERADO";
            case SystemState::BIOMETRIA: return "BIOMETRIA";
            default:                     return "NORMAL";
        }
    }

    /**
     * @brief Modulates the RGB LED PWM channels applying automatic night dimming.
     * @param r Red channel intensity (0-255).
     * @param g Green channel intensity (0-255).
     * @param b Blue channel intensity (0-255).
     * @param ambientLight If false (darkness), reduces the duty cycle to 30% to avoid glare.
     */
    void setAdaptiveRgb(uint8_t r, uint8_t g, uint8_t b, bool ambientLight) {
        const float dimFactor = ambientLight ? 1.0f : 0.30f;
        analogWrite(HardwarePins::LED_R, static_cast<uint8_t>(r * dimFactor));
        analogWrite(HardwarePins::LED_G, static_cast<uint8_t>(g * dimFactor));
        analogWrite(HardwarePins::LED_B, static_cast<uint8_t>(b * dimFactor));
    }

    /**
     * @brief Gets the local time synchronized via NTP in HH:MM:SS format.
     */
    String getFormattedTime() const {
        struct tm timeinfo;
        if (!getLocalTime(&timeinfo, 10)) {
            return "--:--:--";
        }
        char buf[16];
        strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
        return String(buf);
    }

    /**
     * @brief Returns the real Unix Epoch timestamp, or the relative time since boot if NTP is not yet available.
     */
    unsigned long getEpochTime() const {
        time_t now;
        time(&now);
        return (now > 1700000000) ? static_cast<unsigned long>(now) : (millis() / 1000);
    }

    /**
     * @brief Inserts a record into the circular FIFO buffer in RAM when there is no link to the MQTT broker.
     * @note If the buffer reaches 100 samples, it overwrites the oldest sample to preserve the most recent ones.
     */
    void enqueueOfflineTelemetry(const TelemetryRecord& rec) {
        offlineBuffer_[bufferHead_] = rec;
        bufferHead_ = (bufferHead_ + 1) % EdgeThresholds::OFFLINE_BUFFER_SIZE;
        if (bufferCount_ < EdgeThresholds::OFFLINE_BUFFER_SIZE) {
            bufferCount_++;
        }
    }

    /**
     * @brief Serializes a TelemetryRecord structure into a standardized JSON string for the cloud.
     */
    String buildJsonPayload(const TelemetryRecord& rec) const {
        String json = "{";
        json += "\"device_id\":\"" + String(NetworkConfig::DEVICE_ID) + "\",";
        json += "\"timestamp\":" + String(rec.timestamp) + ",";
        json += "\"metrics\":{";
        json += "\"temp_c\":" + String(rec.temp_c, 1) + ",";
        json += "\"gas_adc\":" + String(rec.gas_adc) + ",";
        json += "\"light_detected\":" + String(rec.light_present ? "true" : "false") + ",";
        json += "\"vibration_peak_g\":" + String(rec.vibration_peak_g, 2) + ",";
        json += "\"bpm_user\":" + String(rec.bpm_user);
        json += "},";
        json += "\"actuators\":{";
        json += "\"fan_duty_pct\":" + String(rec.fan_duty_pct) + ",";
        json += "\"led_status\":\"" + String(stateToString(rec.status)) + "\"";
        json += "}}";
        return json;
    }

    /**
     * @brief Vector-renders the Mr_Hyde Industries isotype (sectioned Spartan Lambda)
     *        with a progressive laser sweep effect.
     * @param cx X coordinate of the emblem center.
     * @param cy Y coordinate of the emblem center.
     * @param scanLimitY Height limit of the laser sweep during the boot animation.
     */
    void drawMrHydeLogo(int cx, int cy, int scanLimitY) {
        // Outer orbital ring
        display_.drawCircle(cx, cy, 22, SSD1306_WHITE);
        display_.drawPixel(cx - 23, cy, SSD1306_WHITE);
        display_.drawPixel(cx + 23, cy, SSD1306_WHITE);

        // Upper vertex of the Lambda with inner triangular cutout
        display_.fillTriangle(cx, cy - 17, cx - 13, cy + 7, cx - 6, cy + 7, SSD1306_WHITE);
        display_.fillTriangle(cx, cy - 17, cx + 10, cy + 2, cx + 3, cy + 4, SSD1306_WHITE);
        display_.fillTriangle(cx, cy - 17, cx - 6, cy + 7, cx + 3, cy + 4, SSD1306_WHITE);
        display_.fillTriangle(cx, cy - 8, cx - 5, cy + 5, cx + 3, cy + 2, SSD1306_BLACK);

        // Rising diagonal stroke (negative cut and luminous edge)
        display_.drawLine(cx - 16, cy + 12, cx + 18, cy - 3, SSD1306_BLACK);
        display_.drawLine(cx - 16, cy + 13, cx + 18, cy - 2, SSD1306_BLACK);
        display_.drawLine(cx - 15, cy + 14, cx + 17, cy - 1, SSD1306_WHITE);
        display_.drawLine(cx - 15, cy + 15, cx + 16, cy, SSD1306_WHITE);

        // Lower supports of the Lambda
        display_.fillTriangle(cx - 15, cy + 14, cx - 17, cy + 18, cx - 7, cy + 18, SSD1306_WHITE);
        display_.fillTriangle(cx - 15, cy + 14, cx - 7, cy + 18, cx - 5, cy + 11, SSD1306_WHITE);
        display_.fillTriangle(cx + 5, cy + 6, cx + 11, cy + 4, cx + 16, cy + 18, SSD1306_WHITE);
        display_.fillTriangle(cx + 5, cy + 6, cx + 16, cy + 18, cx + 9, cy + 18, SSD1306_WHITE);

        // Vertical laser scan curtain
        if (scanLimitY < 64) {
            display_.fillRect(0, scanLimitY, 128, 64 - scanLimitY, SSD1306_BLACK);
            display_.drawFastHLine(cx - 25, scanLimitY, 50, SSD1306_WHITE);
        }
    }

    /**
     * @brief Runs the Cyberpunk 2077-style system initialization animation (visual bootloader).
     */
    void playCyberpunkBootSequence() {
        for (int y = 8; y <= 64; y += 4) {
            display_.clearDisplay();
            drawMrHydeLogo(28, 32, y);

            display_.setTextSize(1);
            display_.setTextColor(SSD1306_WHITE);
            display_.setCursor(58, 12);
            display_.println("MR_HYDE");
            display_.setCursor(58, 22);
            display_.println("INDUSTRIES");

            display_.drawFastHLine(58, 33, 66, SSD1306_WHITE);
            display_.setCursor(58, 37);
            display_.println("EDGE OS 2.77");

            const int pct = map(y, 8, 64, 0, 100);
            display_.drawRect(58, 49, 66, 7, SSD1306_WHITE);
            display_.fillRect(60, 51, map(pct, 0, 100, 0, 62), 3, SSD1306_WHITE);

            display_.display();
            delay(45);
        }

        // Neural glitch-style inverse flicker on load completion
        for (int g = 0; g < 3; g++) {
            display_.invertDisplay(g % 2 == 0);
            delay(55);
        }
        display_.invertDisplay(false);
        delay(450);
    }

    /**
     * @brief Asynchronously and without blocking manages Wi-Fi reconnection and the TCP session with the MQTT broker.
     */
    void ensureMqttConnected() {
        if (WiFi.status() != WL_CONNECTED) {
            if (millis() - lastMqttReconnect_ >= 10000) {
                lastMqttReconnect_ = millis();
                WiFi.disconnect();
                WiFi.begin(NetworkConfig::WIFI_SSID, NetworkConfig::WIFI_PASS);
            }
            return;
        }

        if (mqttClient_.connected()) return;
        if (millis() - lastMqttReconnect_ < 5000) return;
        lastMqttReconnect_ = millis();

        const String clientId = String(NetworkConfig::DEVICE_ID) + "_" + String(random(1000, 9999));
        if (mqttClient_.connect(clientId.c_str())) {
            configTime(NetworkConfig::GMT_OFFSET_SEC, NetworkConfig::DAYLIGHT_OFFSET_SEC, NetworkConfig::NTP_SERVER);
        }
    }

    /**
     * @brief Packs the current state and manages transmission to HiveMQ Cloud.
     * @details If the connection is active, it first drains in a burst (FIFO) any record stored
     *          in `offlineBuffer_` during previous disconnections and then publishes the current sample.
     *          If there is no network, it enqueues the sample in SRAM without halting local control.
     */
    void processTelemetryTransmission() {
        TelemetryRecord rec{
            getEpochTime(),
            filteredTemp_,
            filteredGas_,
            hasLight_,
            peakVibrationWindowG_,
            beatAvg_,
            fanDutyPct_,
            currentState_
        };

        if (WiFi.status() == WL_CONNECTED && mqttClient_.connected()) {
            // Chronological-order drain of the circular RAM buffer
            while (bufferCount_ > 0) {
                const size_t tail = (bufferHead_ - bufferCount_ + EdgeThresholds::OFFLINE_BUFFER_SIZE) % EdgeThresholds::OFFLINE_BUFFER_SIZE;
                const String cachedJson = buildJsonPayload(offlineBuffer_[tail]);
                if (mqttClient_.publish(NetworkConfig::MQTT_TOPIC, cachedJson.c_str())) {
                    bufferCount_--;
                } else {
                    break;
                }
            }

            // Publication of the current frame in real time
            const String payload = buildJsonPayload(rec);
            if (!mqttClient_.publish(NetworkConfig::MQTT_TOPIC, payload.c_str())) {
                enqueueOfflineTelemetry(rec);
            }
        } else {
            enqueueOfflineTelemetry(rec);
        }

        // Reset of the seismic peak holder for the next window
        peakVibrationWindowG_ = 0.0f;
    }

    /**
     * @brief Samples the MPU6050 accelerometer at 50 Hz (every 20 ms) to capture mechanical impacts.
     * @details Computes the Euclidean magnitude of the 3D acceleration vector:
     *          |a| = sqrt(ax^2 + ay^2 + az^2), subtracts the static gravitational component (9.80665 m/s^2)
     *          and normalizes the result in multiples of g-force. If it exceeds 1.8g, it triggers an immediate transmission.
     */
    void sampleImu50Hz() {
        if (!hasMPU_) return;
        sensors_event_t a, g, temp;
        mpu_.getEvent(&a, &g, &temp);

        const float magMs2 = std::sqrt(
            (a.acceleration.x * a.acceleration.x) +
            (a.acceleration.y * a.acceleration.y) +
            (a.acceleration.z * a.acceleration.z)
        );
        const float dynamicMs2 = std::fabs(magMs2 - EdgeThresholds::GRAVITY_MS2);
        currentVibrationG_ = dynamicMs2 / EdgeThresholds::GRAVITY_MS2;

        if (currentVibrationG_ > peakVibrationWindowG_) {
            peakVibrationWindowG_ = currentVibrationG_;
        }

        // Asynchronous trigger on critical event (impact detected)
        if (currentVibrationG_ > EdgeThresholds::VIBRATION_IMPACT_G) {
            criticalFlashUntil_ = millis() + 1200;
            processTelemetryTransmission();
        }
    }

    /**
     * @brief Runs continuous heartbeat pulse wave detection on the MAX30105 infrared stream.
     * @note Must be invoked on every iteration of `tick()` without blocking so the zero crossing of the heartbeat is not missed.
     */
    void updateHeartRateContinuous() {
        if (!hasMAX_) return;
        const long irValue = particleSensor_.getIR();

        // IR reflectance threshold confirming skin contact (> 35000 counts)
        if (irValue > 35000) {
            lastFingerSeen_ = millis();
            if (checkForBeat(irValue)) {
                beatSpikePending_ = true;
                const long delta = millis() - lastBeat_;
                lastBeat_ = millis();
                const float bpm = 60.0f / (delta / 1000.0f);

                // Valid human physiological window filter (35 - 220 BPM)
                if (bpm < 220.0f && bpm > 35.0f) {
                    rates_[rateSpot_++] = static_cast<uint8_t>(bpm);
                    rateSpot_ %= EdgeThresholds::RATE_SIZE;
                    int sum = 0;
                    for (uint8_t x = 0; x < EdgeThresholds::RATE_SIZE; x++) {
                        sum += rates_[x];
                    }
                    beatAvg_ = sum / EdgeThresholds::RATE_SIZE;
                }
            }
        } else if (millis() - lastFingerSeen_ > 2500) {
            // Register cleanup 2.5 seconds after the finger is removed
            beatAvg_ = 0;
            for (uint8_t x = 0; x < EdgeThresholds::RATE_SIZE; x++) {
                rates_[x] = 0;
            }
        }
    }

    /**
     * @brief Acquires slow variables (temperature, gas, light), applies the moving-average filter (N=10)
     *        and updates the local actuators (PWM fan and RGB traffic light).
     */
    void sampleClimateAndControl() {
        float rawTemp = tempSensor_.getTempCByIndex(0);
        tempSensor_.requestTemperatures(); // Asynchronous request for the next reading
        if (rawTemp <= -50.0f || rawTemp >= 125.0f) {
            rawTemp = filteredTemp_; // Discard of spurious readings from a 1-Wire disconnection (-127 °C)
        }

        const int rawGas = analogRead(HardwarePins::GAS);
        hasLight_ = (digitalRead(HardwarePins::LDR) == LOW);

        // Insertion into the sliding window (Moving Average Filter)
        tempSamples_[filterIndex_] = rawTemp;
        gasSamples_[filterIndex_] = rawGas;
        filterIndex_++;
        if (filterIndex_ >= EdgeThresholds::MOVING_AVG_SIZE) {
            filterIndex_ = 0;
            filterFilled_ = true;
        }

        const size_t count = filterFilled_ ? EdgeThresholds::MOVING_AVG_SIZE : filterIndex_;
        float sumT = 0.0f;
        long sumG = 0;
        for (size_t i = 0; i < count; i++) {
            sumT += tempSamples_[i];
            sumG += gasSamples_[i];
        }
        filteredTemp_ = sumT / count;
        filteredGas_ = sumG / count;

        // Proportional control law for forced PWM ventilation
        if (filteredTemp_ >= EdgeThresholds::TEMP_MAX_PWM || filteredGas_ >= EdgeThresholds::GAS_ALERT_ADC) {
            fanDutyPct_ = 100;
        } else if (filteredTemp_ >= EdgeThresholds::TEMP_MIN_PWM) {
            const float ratio = (filteredTemp_ - EdgeThresholds::TEMP_MIN_PWM) /
                                (EdgeThresholds::TEMP_MAX_PWM - EdgeThresholds::TEMP_MIN_PWM);
            fanDutyPct_ = 30 + static_cast<int>(ratio * 60.0f);
        } else {
            fanDutyPct_ = 0;
        }

        analogWrite(HardwarePins::FAN, map(fanDutyPct_, 0, 100, 0, 255));

        // State machine evaluation and adaptive lighting
        if (fanDutyPct_ == 100 || peakVibrationWindowG_ > EdgeThresholds::VIBRATION_IMPACT_G) {
            currentState_ = SystemState::CRITICO;
            setAdaptiveRgb(255, 0, 0, hasLight_);
        } else if (fanDutyPct_ > 0) {
            currentState_ = SystemState::MODERADO;
            setAdaptiveRgb(255, 140, 0, hasLight_);
        } else if (beatAvg_ > 0) {
            currentState_ = SystemState::BIOMETRIA;
            setAdaptiveRgb(0, 0, 255, hasLight_);
        } else {
            currentState_ = SystemState::NORMAL;
            setAdaptiveRgb(0, 255, 0, hasLight_);
        }
    }

    /**
     * @brief Renders the Cyberpunk 2077-style tactical HUD on the SSD1306 OLED display at ~6.6 FPS.
     */
    void updateOledDisplay() {
        animFrame_++;

        // Visual color-inversion alert on seismic impacts
        if (millis() < criticalFlashUntil_) {
            display_.invertDisplay((animFrame_ % 2) == 0);
        } else {
            display_.invertDisplay(false);
        }

        // Horizontal scrolling of the cardiographic trace (ECG)
        for (size_t i = 0; i < EdgeThresholds::ECG_WIDTH - 1; i++) {
            ecgHistory_[i] = ecgHistory_[i + 1];
        }
        if (beatSpikePending_) {
            ecgHistory_[EdgeThresholds::ECG_WIDTH - 1] = 13; // Simulated QRS complex on a real heartbeat
            beatSpikePending_ = false;
        } else if (millis() - lastFingerSeen_ < 1500 && lastFingerSeen_ > 0) {
            ecgHistory_[EdgeThresholds::ECG_WIDTH - 1] = 5 + (animFrame_ % 3);
        } else {
            ecgHistory_[EdgeThresholds::ECG_WIDTH - 1] = 6;
        }

        display_.clearDisplay();
        display_.setTextSize(1);

        // --- 1. Top bar: Corporate identity, NTP clock and Cloud/RAM link ---
        display_.fillRect(0, 0, 128, 10, SSD1306_WHITE);
        display_.setTextColor(SSD1306_BLACK);
        display_.setCursor(2, 1);
        display_.print("MR.HYDE ");
        display_.print(getFormattedTime());

        display_.setCursor(102, 1);
        if (mqttClient_.connected()) {
            display_.print("[MQ]");
        } else {
            display_.print("R:");
            display_.print(bufferCount_);
        }

        display_.setTextColor(SSD1306_WHITE);

        // --- 2. Left panel: Thermometry and Air Quality with level bars ---
        display_.drawRect(0, 12, 64, 35, SSD1306_WHITE);
        display_.setCursor(3, 14);
        display_.print("T:");
        display_.print(filteredTemp_, 1);
        display_.print("C");

        const int tempBar = constrain(map(static_cast<int>(filteredTemp_ * 10), 180, 320, 2, 56), 2, 56);
        display_.drawRect(3, 23, 58, 5, SSD1306_WHITE);
        display_.fillRect(4, 24, tempBar, 3, SSD1306_WHITE);

        display_.setCursor(3, 30);
        display_.print("GAS:");
        display_.print(filteredGas_);

        const int gasBar = constrain(map(filteredGas_, 0, 4095, 2, 56), 2, 56);
        display_.drawRect(3, 39, 58, 5, SSD1306_WHITE);
        display_.fillRect(4, 40, gasBar, 3, SSD1306_WHITE);

        // --- 3. Right panel: Biometrics and real-time ECG monitor ---
        display_.drawRect(66, 12, 62, 35, SSD1306_WHITE);
        display_.setCursor(69, 14);
        if (beatAvg_ > 0) {
            display_.print("BPM:");
            display_.print(beatAvg_);
        } else if (millis() - lastFingerSeen_ < 1200 && lastFingerSeen_ > 0) {
            display_.print("SCAN...");
        } else {
            display_.print("BPM:--");
        }

        display_.drawFastHLine(68, 24, 58, SSD1306_WHITE);
        for (size_t i = 1; i < EdgeThresholds::ECG_WIDTH; i++) {
            const int y1 = 43 - ecgHistory_[i - 1];
            const int y2 = 43 - ecgHistory_[i];
            display_.drawLine(72 + i - 1, y1, 72 + i, y2, SSD1306_WHITE);
        }

        // --- 4. Bottom tactical bar: Animated turbine, Photometry and Seismograph ---
        display_.drawRect(0, 49, 128, 15, SSD1306_WHITE);
        display_.setCursor(3, 53);

        char fanSpin = '|';
        if (fanDutyPct_ > 0) {
            static constexpr char blades[4] = {'|', '/', '-', '\\'};
            fanSpin = blades[(animFrame_ * (fanDutyPct_ > 60 ? 2 : 1)) % 4];
        }
        display_.print(fanSpin);
        display_.print("FAN:");
        display_.print(fanDutyPct_);
        display_.print("% ");

        display_.print(hasLight_ ? "LUX " : "DRK ");
        display_.print(currentVibrationG_, 1);
        display_.print("g");

        display_.display();
    }

public:
    /**
     * @brief Node constructor: binds the bus references for the peripheral drivers.
     */
    EdgeNodeController()
        : mqttClient_(espClient_),
          display_(EdgeThresholds::SCREEN_WIDTH, EdgeThresholds::SCREEN_HEIGHT, &Wire, EdgeThresholds::OLED_RESET),
          oneWire_(HardwarePins::ONEWIRE),
          tempSensor_(&oneWire_) {}

    /**
     * @brief Initializes I/O ports, digital buses (I2C/1-Wire), sensors and the Wi-Fi/MQTT stack.
     */
    void begin() {
        Serial.begin(115200);

        for (size_t i = 0; i < EdgeThresholds::ECG_WIDTH; i++) {
            ecgHistory_[i] = 6;
        }

        pinMode(HardwarePins::LDR, INPUT);
        pinMode(HardwarePins::FAN, OUTPUT);
        analogWrite(HardwarePins::FAN, 0);

        pinMode(HardwarePins::LED_R, OUTPUT);
        pinMode(HardwarePins::LED_G, OUTPUT);
        pinMode(HardwarePins::LED_B, OUTPUT);
        setAdaptiveRgb(255, 0, 40, true); // Corporate Crimson color during boot

        Wire.begin(HardwarePins::I2C_SDA, HardwarePins::I2C_SCL);

        // Safety lock with red blinking if the OLED I2C bus fails
        if (!display_.begin(SSD1306_SWITCHCAPVCC, EdgeThresholds::OLED_ADDR)) {
            while (true) {
                analogWrite(HardwarePins::LED_R, 255);
                delay(200);
                analogWrite(HardwarePins::LED_R, 0);
                delay(200);
            }
        }

        playCyberpunkBootSequence();

        // Asynchronous DS18B20 configuration (avoids 750 ms pauses in the main loop)
        tempSensor_.begin();
        tempSensor_.setResolution(10);
        tempSensor_.setWaitForConversion(false);
        tempSensor_.requestTemperatures();

        hasMPU_ = mpu_.begin();
        if (hasMPU_) {
            mpu_.setAccelerometerRange(MPU6050_RANGE_4_G);
        }

        if (particleSensor_.begin(Wire, I2C_SPEED_FAST)) {
            hasMAX_ = true;
            particleSensor_.setup(0x2A, 4, 2, 400, 411, 4096);
        }

        // Wireless network negotiation screen
        display_.clearDisplay();
        drawMrHydeLogo(24, 32, 64);
        display_.setCursor(54, 16);
        display_.println("NET_LINK://");
        display_.setCursor(54, 28);
        display_.println("CNT_2.4GHz");
        display_.setCursor(54, 40);
        display_.println("SYNCING...");
        display_.display();

        // RF power adjustment and WPA2 compatibility for breadboard stability
        WiFi.mode(WIFI_STA);
        WiFi.disconnect(true);
        delay(150);
        WiFi.setTxPower(WIFI_POWER_15dBm);
        WiFi.setSleep(false);
        //WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK);

        WiFi.begin(NetworkConfig::WIFI_SSID, NetworkConfig::WIFI_PASS);
        const unsigned long wifiStart = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 12000) {
            delay(200);
        }

        if (WiFi.status() == WL_CONNECTED) {
            configTime(NetworkConfig::GMT_OFFSET_SEC, NetworkConfig::DAYLIGHT_OFFSET_SEC, NetworkConfig::NTP_SERVER);
        }

        mqttClient_.setServer(NetworkConfig::MQTT_BROKER, NetworkConfig::MQTT_PORT);
        mqttClient_.setBufferSize(512); // Expanded to 512 bytes to support the full JSON

        sampleClimateAndControl();
    }

    /**
     * @brief Main cooperative scheduler (executed continuously in `loop()`).
     * @details Divides CPU time into deterministic slots without using `delay()`, ensuring
     *          that high-frequency sampling (PPG and IMU) coexists with MQTT telemetry.
     */
    void tick() {
        const unsigned long now = millis();

        // Task 1: MQTT session and TCP/IP stack maintenance
        ensureMqttConnected();
        if (mqttClient_.connected()) {
            mqttClient_.loop();
        }

        // Task 2: Continuous heartbeat sampling (highest temporal priority)
        updateHeartRateContinuous();

        // Task 3: Inertial sampling of vibrations and impacts at 50 Hz (every 20 ms)
        if (now - lastImuSample_ >= 20) {
            lastImuSample_ = now;
            sampleImu50Hz();
        }

        // Task 4: Climate sampling, digital filtering and PWM control loop at 0.5 Hz (every 2000 ms)
        if (now - lastClimateSample_ >= 2000) {
            lastClimateSample_ = now;
            sampleClimateAndControl();
        }

        // Task 5: Tactical HUD update on the OLED display at ~6.6 FPS (every 150 ms)
        if (now - lastOledRefresh_ >= 150) {
            lastOledRefresh_ = now;
            updateOledDisplay();
        }

        // Task 6: Periodic dispatch of JSON telemetry to HiveMQ Cloud at 0.1 Hz (every 10 s)
        if (now - lastTelemetrySend_ >= 10000) {
            lastTelemetrySend_ = now;
            processTelemetryTransmission();
        }
    }
};

} // namespace mr_hyde::edge

// Single global instance of the Edge controller
mr_hyde::edge::EdgeNodeController edgeNode;

void setup() {
    edgeNode.begin();
}

void loop() {
    edgeNode.tick();
}