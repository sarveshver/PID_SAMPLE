#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <PID_v1.h>
#include <SimpleKalmanFilter.h>

// MPU6050 & BMP280 Sensors
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

// Kalman Filters
SimpleKalmanFilter kalmanRoll(2, 2, 0.01);
SimpleKalmanFilter kalmanPitch(2, 2, 0.01);
SimpleKalmanFilter kalmanAltitude(2, 2, 0.01);

// PID Variables
double rollSetpoint = 0, pitchSetpoint = 0, altitudeSetpoint = 10.0;
double rollInput, pitchInput, altitudeInput;
double rollOutput, pitchOutput, altitudeOutput;

// PID Gains (Tune These)
double Kp_roll = 2.0, Ki_roll = 0.02, Kd_roll = 0.8;
double Kp_pitch = 2.0, Ki_pitch = 0.02, Kd_pitch = 0.8;
double Kp_altitude = 1.5, Ki_altitude = 0.01, Kd_altitude = 0.5;

// PID Controllers
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp_roll, Ki_roll, Kd_roll, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp_pitch, Ki_pitch, Kd_pitch, DIRECT);
PID altitudePID(&altitudeInput, &altitudeOutput, &altitudeSetpoint, Kp_altitude, Ki_altitude, Kd_altitude, DIRECT);

// ESC (Electronic Speed Controller) PWM Setup
const int motor1 = 18; // Front-Left
const int motor2 = 19; // Front-Right
const int motor3 = 21; // Back-Left
const int motor4 = 22; // Back-Right
const int escFreq = 500;  // ESC PWM Frequency (500-1000Hz)
const int escResolution = 8;  // 8-bit (0-255)

// ESC Motor Speeds
int motor1Speed, motor2Speed, motor3Speed, motor4Speed;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("MPU6050 not found!");
        while (1);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Initialize BMP280
    if (!bmp.begin(0x76)) {
        Serial.println("BMP280 not found!");
        while (1);
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);

    // ESC PWM Setup
    ledcSetup(0, escFreq, escResolution);
    ledcSetup(1, escFreq, escResolution);
    ledcSetup(2, escFreq, escResolution);
    ledcSetup(3, escFreq, escResolution);
    ledcAttachPin(motor1, 0);
    ledcAttachPin(motor2, 1);
    ledcAttachPin(motor3, 2);
    ledcAttachPin(motor4, 3);

    // Start PID Controllers
    rollPID.SetMode(AUTOMATIC);
    pitchPID.SetMode(AUTOMATIC);
    altitudePID.SetMode(AUTOMATIC);
    rollPID.SetOutputLimits(-50, 50);
    pitchPID.SetOutputLimits(-50, 50);
    altitudePID.SetOutputLimits(50, 255);  // Min 50 to avoid motor stop
}

void loop() {
    // Get MPU6050 data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Raw Gyro Data
    double rawRoll = g.gyro.x * 57.2958;
    double rawPitch = g.gyro.y * 57.2958;

    // Apply Kalman Filter
    rollInput = kalmanRoll.updateEstimate(rawRoll);
    pitchInput = kalmanPitch.updateEstimate(rawPitch);

    // Get Altitude from BMP280
    double rawAltitude = bmp.readAltitude(1013.25);
    altitudeInput = kalmanAltitude.updateEstimate(rawAltitude);

    // Compute PID
    rollPID.Compute();
    pitchPID.Compute();
    altitudePID.Compute();

    // ESC Motor Mixing (Throttle + Roll + Pitch)
    motor1Speed = constrain(120 + rollOutput + pitchOutput + altitudeOutput, 50, 255);
    motor2Speed = constrain(120 - rollOutput + pitchOutput + altitudeOutput, 50, 255);
    motor3Speed = constrain(120 + rollOutput - pitchOutput + altitudeOutput, 50, 255);
    motor4Speed = constrain(120 - rollOutput - pitchOutput + altitudeOutput, 50, 255);

    // Send PWM signals to ESCs
    ledcWrite(0, motor1Speed);
    ledcWrite(1, motor2Speed);
    ledcWrite(2, motor3Speed);
    ledcWrite(3, motor4Speed);

    // Debugging
    Serial.print("Raw Roll: "); Serial.print(rawRoll);
    Serial.print(" | Kalman Roll: "); Serial.print(rollInput);
    Serial.print(" | Raw Pitch: "); Serial.print(rawPitch);
    Serial.print(" | Kalman Pitch: "); Serial.print(pitchInput);
    Serial.print(" | Raw Altitude: "); Serial.print(rawAltitude);
    Serial.print(" | Kalman Altitude: "); Serial.print(altitudeInput);
    Serial.print(" | M1: "); Serial.print(motor1Speed);
    Serial.print(" | M2: "); Serial.print(motor2Speed);
    Serial.print(" | M3: "); Serial.print(motor3Speed);
    Serial.print(" | M4: "); Serial.println(motor4Speed);

    delay(10);
}

