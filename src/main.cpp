#include <Arduino.h>
#include "TCRT5000.h"
#include "Motors.h"
#include "PID.h"

// ---------------------------- CONSTANTS ------------------------------- //

// Motor pins
#define IN1 12
#define IN2 14
#define IN3 27
#define IN4 26
constexpr int MAX_SPEED = 200;
constexpr int NORMAL_SPEED = 50;

// TCRT5000 pins
#define ledOnPin 5
#define s0 17
#define s1 18
#define s2 19
constexpr uint8_t threshold = 2000;
constexpr uint8_t numSensors = 7;
constexpr uint8_t sensorsPins[numSensors] = {25,33,32,35,34,39,36};

// Ultrasonic Pins
#define echoPin 2
#define trigPin 15

// BMI-160 Pins
#define IMU_INT1 13
#define IMU_SDA 21
#define IMU_SCL 22

// Switches
#define PB 23
#define DIP1 16
#define DIP2 4

// PID
constexpr float KP = 30.0;
constexpr float KI = 0.0;
constexpr float KD = 0.0;
constexpr float ALPHA = 0.7;
constexpr float INTEGRAL_LIMIT = 10.0;

// Else
unsigned long t = 0; // For millis()
unsigned int sensorsReading[8] = {};
unsigned long duration;
double distanceCm;

// ---------------------------- CLASSES ------------------------------- //

DualMotors<
    IN1,IN2,IN3,IN4,MAX_SPEED,0,1,2,3
> Robot;

TCRT5000 <
    ledOnPin,s0,s1,s2,threshold,numSensors,sensorsPins
> Sensors;

PID pid(KP,KI,KD,ALPHA,INTEGRAL_LIMIT);

// ---------------------------- FUNCTIONS ------------------------------- //

// Motors controller: takes correction from PID then applied it on motors.
void controlMotors(const float correction) {
    const int rightSpeed = constrain(NORMAL_SPEED - correction,-MAX_SPEED,MAX_SPEED);
    const int leftSpeed = constrain(NORMAL_SPEED + correction,-MAX_SPEED,MAX_SPEED);
    Robot.set(rightSpeed,leftSpeed);
}

// Reading Ultrasonic then return distance in cm.
double readUltrasonicCm() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
    return duration * 0.0343 / 2;
}

void ultrasonicDebug() {

    const double distanceCm = readUltrasonicCm();

    Serial.print("Distance: ");

    if (distanceCm == 0) {
        Serial.println("No Echo");
    }
    else {
        Serial.print(distanceCm);
        Serial.println(" cm");
    }
}

void motorsDebug() {
    int speed = 80;
    Robot.forward(speed);
    delay(3000);
    Robot.backward(speed);
    delay(3000);
    Robot.turnRight(speed);
    delay(3000);
    Robot.turnLeft(speed);
    delay(3000);
    Robot.stop();
    delay(1500);
}

void sensorsDebug() {
    digitalWrite(ledOnPin, HIGH);

    for (int i = 0; i < numSensors; i++) {
        Sensors.setMux(i);
        Serial.print(analogRead(sensorsPins[i]));
        Serial.print("\t");
    }
    Serial.println(" ");
}

// ---------------------------- MAIN ------------------------------- //
void setup() {
    analogSetWidth(12);
    analogSetAttenuation(ADC_11db);
    Serial.begin(115200);

    pinMode(PB, INPUT_PULLDOWN);
    pinMode(DIP1, INPUT_PULLDOWN);
    pinMode(DIP2, INPUT_PULLDOWN);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void loop() {

    // Sensors.debug();
    // delay(700);

    // motorsDebug();

    auto data = Sensors.getErrorDigital();
    double dt = (millis() - t) / 1000.0;
    if (dt <= 0) dt = 0.001;
    t = millis();
    const float pid_out = constrain(pid.computePID(data.error ,dt), -MAX_SPEED, MAX_SPEED);
    controlMotors(pid_out);

    // Serial.print("PID: ");Serial.print(pid_out);Serial.print("Error: ");Serial.println(data.error);
    // delay(1000);

}