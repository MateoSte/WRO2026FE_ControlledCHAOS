#ifndef FE_CLASSES
#define FE_CLASSES
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <Wire.h>

namespace pybricks {
namespace parameters {
enum class Direction : short {
    COUNTERCLOCKWISE = -1,
    CLOCKWISE = 1,
};
}  // namespace parameters

namespace pupdevices {

class Servo {
public:
    int pin;
    float max_deg;          // puni raspon servo motora (270 za SER0046, 180 za MG995)
    int min_us, max_us;     // širina impulsa za 0 i max_deg
    float limit_min, limit_max;   // sigurnosne granice u stupnjevima

    Servo(const Servo&) = delete;
    Servo& operator=(const Servo&) = delete;

    Servo(int pin, float max_deg = 270.0f, int min_us = 500, int max_us = 2500,
          float limit_min = 0.0f, float limit_max = -1.0f)
        : pin(pin), max_deg(max_deg), min_us(min_us), max_us(max_us),
          limit_min(limit_min),
          limit_max(limit_max < 0 ? max_deg : limit_max) {}

    // Call this in setup(), not at global scope
    void begin() {
        ledcAttach(pin, PWM_FREQ, PWM_BITS);   // 50 Hz, 14 bita
    }

    // Ide na kut 0..max_deg (ograničeno na limit_min..limit_max)
    void angle(float degrees) {
        degrees = constrain(degrees, limit_min, limit_max);
        current = degrees;
        writeMicroseconds(min_us + degrees * (max_us - min_us) / max_deg);
    }

    // Zadnji zadani kut (servo nema povratnu vezu)
    float angle() const { return current; }

    void writeMicroseconds(int us) {
        us = constrain(us, min_us, max_us);
        ledcWrite(pin, (uint32_t)us * PWM_MAX / PERIOD_US);   // 20 000 us = 50 Hz
    }

    // Prestaje slati impuls (servo se može okretati rukom, nema drhtanja)
    void release() { ledcWrite(pin, 0); }

private:
    static constexpr uint32_t PWM_FREQ  = 50;
    static constexpr uint8_t  PWM_BITS  = 14;
    static constexpr uint32_t PWM_MAX   = (1u << PWM_BITS) - 1;   // 16383
    static constexpr uint32_t PERIOD_US = 20000;

    float current = 0;
};

class Motor {
public:
    int pwma, ain1, ain2, stby, enc1, enc2;
    pybricks::parameters::Direction direction;
    float ppr, gear_ratio, counts_per_rev;
    volatile long encoderCount = 0;

    // Tuning
    float kp = 4.0f;            // PWM units per degree of error
    float kd = 0.1f;            // damping: PWM per deg/s
    int   min_pwm = 60;         // overcomes friction/deadband (skala 0..255)
    float tolerance = 2.0f;     // degrees
    uint32_t timeout_ms = 5000;

    Motor(const Motor&) = delete;             // the ISR holds 'this'
    Motor& operator=(const Motor&) = delete;

    // ppr = broj RISING bridova kanala A po okretaju MOTORA (ne 4x)
    // Ako si u testu izmjerio impulse po okretaju KOTACA, daj ppr = taj broj, gear_ratio = 1
    Motor(int pwma, int ain1, int ain2, int stby, int enc1, int enc2,
          float ppr, float gear_ratio,
          pybricks::parameters::Direction direction = pybricks::parameters::Direction::CLOCKWISE)
        : pwma(pwma), ain1(ain1), ain2(ain2), stby(stby), enc1(enc1), enc2(enc2),
          direction(direction), ppr(ppr), gear_ratio(gear_ratio)
    {
        counts_per_rev = ppr * gear_ratio;
    }

    // Call this in setup(), not at global scope
    void begin() {
        pinMode(stby, OUTPUT);
        digitalWrite(stby, LOW);              // driver ugasen dok se sve ne podesi

        pinMode(ain1, OUTPUT);
        pinMode(ain2, OUTPUT);
        ledcAttach(pwma, PWM_FREQ, PWM_BITS); // 20 kHz, 10 bita (0..1023)

        pinMode(enc1, INPUT_PULLUP);
        pinMode(enc2, INPUT_PULLUP);
        attachInterruptArg(digitalPinToInterrupt(enc1), isrTrampoline, this, RISING);

        stop();
        digitalWrite(stby, HIGH);
    }

    // ---------- Sensing ----------
    // Unwrapped angle in degrees
    float rawAngle() const {
        long count = encoderCount;            // 32-bit read je atomaran na ESP32
        float a = (count * 360.0f) / counts_per_rev;
        return a * static_cast<short>(direction);
    }

    // Wrapped 0..360
    int angle() const {
        float a = fmod(rawAngle(), 360.0f);
        return static_cast<int>(a < 0 ? a + 360.0f : a);
    }

    // ---------- Driving ----------
    void run(int speed) {                      // speed: -255..255
        speed = constrain(speed, -255, 255);
        int s = speed * static_cast<short>(direction);
        if (s == 0) {
            stop();
            return;
        }
        digitalWrite(ain1, s > 0);
        digitalWrite(ain2, s < 0);
        ledcWrite(pwma, abs(s) * PWM_MAX / 255);
    }

    void stop() {                              // coast
        ledcWrite(pwma, 0);
        digitalWrite(ain1, LOW);
        digitalWrite(ain2, LOW);
    }

    void brake() {                             // short brake (TB6612FNG)
        targetActive = false;
        digitalWrite(ain1, HIGH);
        digitalWrite(ain2, HIGH);
        ledcWrite(pwma, PWM_MAX);
    }

    // ---------- Position control ----------
    // speed = maximum PWM (1..255), angle = absolute target in degrees
    void run_target(int speed, float angle, bool wait = true) {
        targetSpeed = constrain(abs(speed), 1, 255);
        targetAngle = angle;
        targetStart = millis();
        targetActive = true;

        prevAngle = rawAngle();
        prevT = millis();
        vel = 0;

        if (wait) {
            while (update()) {
                delay(1);
            }
        }
    }

    // Call repeatedly if run_target(..., false). Returns true while still running.
    bool update() {
        if (!targetActive) return false;

        uint32_t now = millis();
        float a = rawAngle();
        if (now - prevT >= 10) {                       // velocity every 10 ms
            vel = (a - prevAngle) * 1000.0f / (now - prevT);
            prevAngle = a;
            prevT = now;
        }
        float error = targetAngle - a;

        if ((fabs(error) <= tolerance && fabs(vel) < 20) || now - targetStart > timeout_ms) {
            brake();                                   // also clears targetActive
            return false;
        }

        int out = static_cast<int>(kp * error - kd * vel);
        out = constrain(out, -targetSpeed, targetSpeed);

        if (fabs(error) > tolerance) {
            if (out > 0 && out < min_pwm) out = min(min_pwm, targetSpeed);
            if (out < 0 && out > -min_pwm) out = -min(min_pwm, targetSpeed);
        }

        run(out);
        return true;
    }

private:
    static constexpr uint32_t PWM_FREQ = 20000;
    static constexpr uint8_t  PWM_BITS = 10;
    static constexpr int      PWM_MAX  = (1 << PWM_BITS) - 1;   // 1023

    float targetAngle = 0;
    int targetSpeed = 0;
    uint32_t targetStart = 0;
    bool targetActive = false;

    float prevAngle = 0;
    uint32_t prevT = 0;
    float vel = 0;                                     // deg/s

    static void isrTrampoline(void* arg) {
        static_cast<Motor*>(arg)->handleEncoder();
    }

    // Kanal A RISING, smjer iz kanala B (isto kao u testu)
    void handleEncoder() {
        encoderCount += digitalRead(enc2) ? 1 : -1;
    }
};
}  // namespace pupdevices

namespace tools {

inline void wait(int milliseconds) {
    if (milliseconds > 0) {
        delay(milliseconds);
    }
}

class StopWatch {
public:
    StopWatch() { reset(); }

    void reset() {
        startTime = millis();
        accumulated = 0;
        running = true;
    }

    int time() const {
        uint32_t elapsed = accumulated;
        if (running) {
            elapsed += millis() - startTime;
        }
        return static_cast<int>(elapsed);
    }

    void pause() {
        if (running) {
            accumulated += millis() - startTime;
            running = false;
        }
    }

    void resume() {
        if (!running) {
            startTime = millis();
            running = true;
        }
    }

private:
    uint32_t startTime = 0;
    uint32_t accumulated = 0;
    bool running = false;
};

}  // namespace tools

namespace hubs {
class PrimeHub {
public:
    class IMU {
    public:
        bool begin(uint8_t addr = 0x68) {
            address = addr;

            writeReg(0x6B, 0x00);   // wake up (clear sleep bit)
            writeReg(0x1A, 0x03);   // digital low-pass filter ~44 Hz
            writeReg(0x1B, 0x08);   // gyro range ±500 °/s -> 65.5 LSB per °/s

            // Check the sensor answers (WHO_AM_I is 0x68)
            Wire.beginTransmission(address);
            Wire.write(0x75);
            if (Wire.endTransmission(false) != 0) return false;
            Wire.requestFrom(address, (uint8_t)1);
            if (!Wire.available() || Wire.read() != 0x68) return false;

            calibrate();
            heading_deg = 0.0f;
            lastMicros = micros();
            return true;
        }

        // Measure gyro bias while the robot is stationary.
        void calibrate(int samples = 500) {
            long sum = 0;
            for (int i = 0; i < samples; i++) {
                sum += readGyroZRaw();
                delay(2);
            }
            gyroBias = (float)sum / samples;
        }

        // Must be called often (every loop, ideally >100 Hz).
        void update() {
            uint32_t now = micros();
            float dt = (now - lastMicros) * 1e-6f;   // unsigned subtraction survives rollover
            lastMicros = now;

            float rate = (readGyroZRaw() - gyroBias) / 65.5f;   // °/s
            if (fabsf(rate) < 0.05f) rate = 0;                  // small deadband against noise
            heading_deg -= rate * dt;   // minus: sensor Z is counter-clockwise, Pybricks is clockwise-positive
        }

        void reset_heading(float value = 0.0f) {
            heading_deg = value;
        }

        // Continuous (not wrapped) angle in degrees, clockwise positive.
        float heading() const {
            return heading_deg;
        }

    private:
        uint8_t address = 0x68;
        volatile float heading_deg = 0.0f;
        float gyroBias = 0.0f;
        uint32_t lastMicros = 0;

        void writeReg(uint8_t reg, uint8_t val) {
            Wire.beginTransmission(address);
            Wire.write(reg);
            Wire.write(val);
            Wire.endTransmission();
        }

        int16_t readGyroZRaw() {
            Wire.beginTransmission(address);
            Wire.write(0x47);                       // GYRO_ZOUT_H
            Wire.endTransmission(false);
            Wire.requestFrom(address, (uint8_t)2);
            int16_t hi = Wire.read();
            int16_t lo = Wire.read();
            return (hi << 8) | lo;
        }
    };

    IMU imu;

    PrimeHub() {
        imu.begin();
    }
};
}  // namespace hubs
}  // namespace pybricks

namespace {
float clamp_f(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(max_value, value));
}

float to_degrees(float radians_value) {
    return radians_value * 180.0f / static_cast<float>(M_PI);
}
}  // namespace

class CarDriveBase {
public:
    // steer_center = kut servo motora kad su kotači ravno (npr. STEER_CENTER = 73)
    // Pozitivan kut skretanja = udesno (kazaljka na satu), kao i heading()
    CarDriveBase(
        pybricks::pupdevices::Motor& drive_motor,
        pybricks::pupdevices::Servo& steer_servo,
        float wheel_diameter,
        float axle_track,
        int default_speed,
        float steer_center,
        std::shared_ptr<pybricks::hubs::PrimeHub> hub)
        : drive_motor_(drive_motor),
          steer_servo_(steer_servo),
          wheel_diameter_(wheel_diameter),
          axle_track_(axle_track),
          default_speed_(default_speed),
          steer_center_(steer_center),
          hub_(std::move(hub)) {
    }

    // Podešavanje
    float steer_gain = 1.5f;        // stupnjeva skretanja po stupnju pogreške smjera
    int   steer_settle_ms = 250;    // koliko čekati da servo stigne na kut

    void use_gyro(bool use) {
        if (hub_) {
            hub_->imu.reset_heading(0.0f);
        }
        gyro_ = use;
    }

    void reset_gyro() {
        if (gyro_ && hub_) {
            hub_->imu.reset_heading(0.0f);
        }
    }

    // Najveći mogući kut skretanja u jednu stranu (iz granica servo motora)
    float max_steer() const {
        return std::min(steer_center_ - steer_servo_.limit_min,
                        steer_servo_.limit_max - steer_center_);
    }

    // Zakreće kotače: 0 = ravno, + = desno, - = lijevo. Vraća stvarno postavljen kut.
    float steer(float deg, bool wait = false) {
        const float m = max_steer();
        deg = clamp_f(deg, -m, m);
        steer_servo_.angle(steer_center_ + deg);
        if (wait) {
            pybricks::tools::wait(steer_settle_ms);
        }
        return deg;
    }

    // Regulacija smjera po žiroskopu (proporcionalna)
    void correct(float angle = 0.0f, float step = 30.0f, bool pr = false) {
        if (gyro_ && hub_) {
            const float limit = std::min(std::abs(step), max_steer());
            const float target_step = clamp_f(
                steer_gain * (angle - hub_->imu.heading()),
                -limit,
                limit);

            if (pr) {
                Serial.println(target_step);
            }

            steer(target_step);
        }
    }

    float distance() const {
        return drive_motor_.rawAngle() * wheel_diameter_ * static_cast<float>(M_PI) / 360.0f;
    }

    void drive(std::optional<int> speed = std::nullopt) {
        const int target_speed = speed.value_or(default_speed_);
        drive_motor_.run(target_speed);
        running_ = true;
    }

    void stop() {
        drive_motor_.stop();
        running_ = false;
    }

    void brake() {
        drive_motor_.brake();
        running_ = false;
    }

    void straight(float dist, float turn_rate = 0.0f, std::optional<int> speed = std::nullopt) {
        steer(0.0f, true);
        _straight(dist, turn_rate, speed);
    }

    void turn(float target_deg,
              float step_deg,
              float tolerance = 1.5f,
              std::optional<int> speed_steer = std::nullopt,   // zadržano radi kompatibilnosti, servo ga ne koristi
              std::optional<int> speed_drive = std::nullopt) {
        (void)speed_steer;

        if (target_deg == 0.0f) {
            return;
        }

        if (step_deg == 0.0f) {
            steer(0.0f, true);
            return;
        }

        const float ta = target_deg / std::abs(target_deg);
        const int used_speed_drive = speed_drive.value_or(default_speed_);

        // Stvarni kut skretanja ograničen je servo motorom, pa se račun radi s njim
        const float applied = steer(step_deg, true);
        const float sa = applied / std::abs(applied);
        const float step_abs = std::abs(applied);

        const float axle_track_mm = axle_track_ * 10.0f;
        const float dist = (ta * (std::abs(target_deg) + step_abs) * axle_track_mm * static_cast<float>(M_PI)) /
                          (std::tan((step_abs * static_cast<float>(M_PI)) / 180.0f) * 180.0f);

        _straight(dist, 0.0f, used_speed_drive);

        if (gyro_ && hub_) {
            bool started = false;
            int i = 0;

            while (std::abs(std::abs(hub_->imu.heading()) - std::abs(target_deg)) > tolerance) {
                hub_->imu.update();
                if (!started) {
                    drive(static_cast<int>(ta * used_speed_drive));
                }
                started = true;
                correct(sa * target_deg, step_abs, i == 0);
                pybricks::tools::wait(5);
                ++i;
                i %= 50;
            }

            brake();
            hub_->imu.reset_heading(hub_->imu.heading() - sa * target_deg);
        }
    }

    void turn_radius(float target_deg,
                     float radius,
                     float tolerance = 1.5f,
                     std::optional<int> speed_steer = std::nullopt,
                     std::optional<int> speed_drive = std::nullopt) {
        const float axle_track_mm = axle_track_ * 10.0f;
        float step_deg = to_degrees(2.0f * std::atan(axle_track_mm / (2.0f * radius)));

        if (target_deg < 0.0f) {
            step_deg = -step_deg;
        }

        turn(target_deg, step_deg, tolerance, speed_steer, speed_drive);
    }

private:
    void _straight(float dist, float turn_rate = 0.0f, std::optional<int> speed = std::nullopt) {
        running_ = true;
        const int target_speed = speed.value_or(default_speed_);

        const float wheel_circumference = wheel_diameter_ * static_cast<float>(M_PI);
        const float degrees = dist / wheel_circumference * 360.0f;
        const float actual_speed = std::abs(static_cast<float>(target_speed)) * (degrees > 0.0f ? 1.0f : -1.0f);

        const float start = drive_motor_.rawAngle();
        drive_motor_.run(static_cast<int>(actual_speed));

        while (std::abs(drive_motor_.rawAngle() - start) < std::abs(degrees)) {
            if (gyro_ && hub_) {
                hub_->imu.update();
                correct(turn_rate);
            }
            pybricks::tools::wait(5);
        }

        drive_motor_.brake();
        running_= false;
    }

    pybricks::pupdevices::Motor& drive_motor_;
    pybricks::pupdevices::Servo& steer_servo_;
    float wheel_diameter_;
    float axle_track_;
    int default_speed_;
    float steer_center_;
    std::shared_ptr<pybricks::hubs::PrimeHub> hub_;
    bool gyro_ = false;
    bool running_ = false;
};
#endif