#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

namespace pybricks {
namespace parameters {
enum class Direction : short {
    COUNTERCLOCKWISE = -1,
    CLOCKWISE = 1,
};
}  // namespace parameters

namespace pupdevices {
static const int8_t QEM[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};

class Motor {
public:
    int pwma, ain1, ain2, stby, enc1, enc2;
    pybricks::parameters::Direction direction;
    float ppr, gear_ratio, counts_per_rev;
    volatile long encoderCount = 0;
    volatile uint8_t lastState = 0;

    // Tuning
    float kp = 4.0f;            // PWM units per degree of error
    float kd = 0.1f;            // damping: PWM per deg/s
    int   min_pwm = 60;         // overcomes friction/deadband
    float tolerance = 2.0f;     // degrees
    uint32_t timeout_ms = 5000;

    Motor(const Motor&) = delete;             // the ISR holds 'this'
    Motor& operator=(const Motor&) = delete;

    Motor(int pwma, int ain1, int ain2, int stby, int enc1, int enc2,
          float ppr, float gear_ratio,
          pybricks::parameters::Direction direction = pybricks::parameters::Direction::CLOCKWISE)
        : pwma(pwma), ain1(ain1), ain2(ain2), stby(stby), enc1(enc1), enc2(enc2),
          direction(direction), ppr(ppr), gear_ratio(gear_ratio)
    {
        counts_per_rev = ppr * 4.0f * gear_ratio;
    }

    // Call this in setup(), not at global scope
    void begin() {
        pinMode(pwma, OUTPUT);
        pinMode(ain1, OUTPUT);
        pinMode(ain2, OUTPUT);
        pinMode(stby, OUTPUT);
        pinMode(enc1, INPUT_PULLUP);
        pinMode(enc2, INPUT_PULLUP);

        digitalWrite(stby, HIGH);

        lastState = (digitalRead(enc1) << 1) | digitalRead(enc2);
        attachInterruptArg(digitalPinToInterrupt(enc1), isrTrampoline, this, CHANGE);
        attachInterruptArg(digitalPinToInterrupt(enc2), isrTrampoline, this, CHANGE);

        stop();
    }

    // ---------- Sensing ----------
    // Unwrapped angle in degrees
    float rawAngle() const {
        noInterrupts();
        long count = encoderCount;
        interrupts();
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
        if (s > 0) {
            digitalWrite(ain1, HIGH);
            digitalWrite(ain2, LOW);
            analogWrite(pwma, s);
        } else if (s < 0) {
            digitalWrite(ain1, LOW);
            digitalWrite(ain2, HIGH);
            analogWrite(pwma, -s);
        } else {
            stop();
        }
    }

    void stop() {                              // coast
        analogWrite(pwma, 0);
        digitalWrite(ain1, LOW);
        digitalWrite(ain2, LOW);
    }

    void brake() {                             // short brake (TB6612FNG)
        targetActive = false;
        digitalWrite(ain1, HIGH);
        digitalWrite(ain2, HIGH);
        analogWrite(pwma, 255);
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
        if (now - prevT >= 10) {                       // velocity every 10 ms (less noisy)
            vel = (a - prevAngle) * 1000.0f / (now - prevT);
            prevAngle = a;
            prevT = now;
        }
        float error = targetAngle - a;

        // done only when close AND almost stopped, or on timeout
        if ((fabs(error) <= tolerance && fabs(vel) < 20) || now - targetStart > timeout_ms) {
            brake();                                   // also clears targetActive
            return false;
        }

        int out = static_cast<int>(kp * error - kd * vel);
        out = constrain(out, -targetSpeed, targetSpeed);

        // minimum power only while still outside the tolerance zone
        if (fabs(error) > tolerance) {
            if (out > 0 && out < min_pwm) out = min(min_pwm, targetSpeed);
            if (out < 0 && out > -min_pwm) out = -min(min_pwm, targetSpeed);
        }

        run(out);
        return true;
    }

private:
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

    void handleEncoder() {
        uint8_t state = (digitalRead(enc1) << 1) | digitalRead(enc2);
        encoderCount += QEM[(lastState << 2) | state];
        lastState = state;
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
    // Motors are taken by REFERENCE (Motor can't be copied)
    CarDriveBase(
        pybricks::pupdevices::Motor& drive_motor,
        pybricks::pupdevices::Motor& steer_motor,
        float wheel_diameter,
        float axle_track,
        int default_speed,
        int default_steer_speed,
        std::shared_ptr<pybricks::hubs::PrimeHub> hub)
        : drive_motor_(drive_motor),
          steer_motor_(steer_motor),
          wheel_diameter_(wheel_diameter),
          axle_track_(axle_track),
          default_speed_(default_speed),
          default_steer_speed_(default_steer_speed),
          hub_(std::move(hub)) {
    }

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

    void correct(float angle = 0.0f, float step = 30.0f, bool pr = false) {
        if (gyro_ && hub_) {
            const float target_step = clamp_f(
                15.0f * std::ceil((angle - hub_->imu.heading()) / 15.0f),
                -std::abs(step),
                std::abs(step));

            if (pr) {
                Serial.println(target_step);
            }

            steer_motor_.run_target(default_steer_speed_, target_step, false);
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
        steer_motor_.run_target(default_steer_speed_, 0.0f);
        _straight(dist, turn_rate, speed);
    }

    void turn(float target_deg,
              float step_deg,
              float tolerance = 1.5f,
              std::optional<int> speed_steer = std::nullopt,
              std::optional<int> speed_drive = std::nullopt) {
        if (target_deg == 0.0f) {
            return;
        }

        if (step_deg == 0.0f) {
            steer_motor_.run_target(default_steer_speed_, 0.0f);
            return;
        }

        const float ta = target_deg / std::abs(target_deg);
        const float sa = step_deg / std::abs(step_deg);
        const int used_speed_steer = speed_steer.value_or(default_steer_speed_);
        const int used_speed_drive = speed_drive.value_or(default_speed_);

        const float axle_track_mm = axle_track_ * 10.0f;
        const float dist = (ta * (std::abs(target_deg) + std::abs(step_deg)) * axle_track_mm * static_cast<float>(M_PI)) /
                          (std::tan((std::abs(step_deg) * static_cast<float>(M_PI)) / 180.0f) * 180.0f);

        steer_motor_.run_target(used_speed_steer, step_deg);
        _straight(dist, 0.0f, used_speed_drive);       // fixed argument order

        if (gyro_ && hub_) {
            bool started = false;
            int i = 0;

            while (std::abs(std::abs(hub_->imu.heading()) - std::abs(target_deg)) > tolerance) {
                if (!started) {
                    drive(static_cast<int>(ta * used_speed_drive));
                }
                started = true;
                correct(sa * target_deg, step_deg, i == 0);
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
                correct(turn_rate);
            }
            pybricks::tools::wait(5);
        }

        drive_motor_.brake();
        running_ = false;
    }

    pybricks::pupdevices::Motor& drive_motor_;
    pybricks::pupdevices::Motor& steer_motor_;
    float wheel_diameter_;
    float axle_track_;
    int default_speed_;
    int default_steer_speed_;
    std::shared_ptr<pybricks::hubs::PrimeHub> hub_;
    bool gyro_ = false;
    bool running_ = false;
};

// ---------- Your robot (one motor for now) ----------
using namespace pybricks::pupdevices;

// pwma, ain1, ain2, stby, enc1, enc2, ppr, gear_ratio  (use YOUR ppr and gear ratio)
Motor left(7, 4, 5, 10, 12, 13, 11.0f, 100.0f);

void setup() {
    Serial.begin(115200);
    left.begin();
}

void loop() {
    left.run_target(200, 360);
    delay(1000);
    Serial.println(left.rawAngle());

    left.run_target(200, 180);
    delay(1000);
    Serial.println(left.rawAngle());

    left.run_target(200, 0);
    delay(1000);
    Serial.println(left.rawAngle());
    Serial.println("---");
}