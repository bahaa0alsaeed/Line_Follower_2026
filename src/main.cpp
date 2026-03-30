#include <Arduino.h>
#include "TCRT5000.h"
#include "Motors.h"
#include "PID.h"

// ---------------------------- CONSTANTS ------------------------------- //

// Motor pins
#define IN1 5
#define IN2 17
#define IN3 16
#define IN4 4
#define LEFT_SPEED_PIN 2
#define RIGHT_SPEED_PIN 18
constexpr int MAX_SPEED = 200;
constexpr int NORMAL_SPEED = 150;

// TCRT5000 pins
#define readingPin 36
#define ledOnPin 15
#define s0 25
#define s1 26
#define s2 27
constexpr int threshold = 2000;

// Ultrasonic Pins
#define echoPin 34
#define trigPin 32

// BMI-160 Pins
#define IMU_INT1 19
#define IMU_INT2 23
#define IMU_SDA 21
#define IMU_SCL 22

// Switches
#define PB 33
#define DIP1 13
#define DIP2 12
#define DIP3 14

// PID
constexpr float KP = 2.0;
constexpr float KI = 0.0;
constexpr float KD = 0.0;
constexpr float INTEGRAL_LIMIT = 10.0;

// Else
unsigned long t = 0; // For millis()
unsigned int sensorsReading[8] = {};
unsigned long duration;
double distanceCm;

// ---------------------------- CLASSES ------------------------------- //

DualMotors<
    IN1,IN2,RIGHT_SPEED_PIN,IN3,IN4,LEFT_SPEED_PIN,MAX_SPEED
> Robot;

TCRT5000 <
    readingPin,ledOnPin,s0,s1,s2,threshold
> Sensors;

PID pid(KP,KI,KD,INTEGRAL_LIMIT);

// ---------------------------- FUNCTIONS ------------------------------- //

// Motors controller: takes correction from PID then applied it on motors.
void controlMotors(const float correction) {
    const int rightSpeed = constrain(MAX_SPEED - correction,-MAX_SPEED,MAX_SPEED);
    const int leftSpeed = constrain(MAX_SPEED + correction,-MAX_SPEED,MAX_SPEED);
    Robot.forward(rightSpeed,leftSpeed);
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

void sensorsDebug() {
    // Reading TCRT5000
    for (int i=0; i<8; i++) {
        Sensors.setMux(i);
        sensorsReading[i] = Sensors.averageRead();
    }
    // Reading Ultrasonic
    distanceCm = readUltrasonicCm();

    // Printing
    Serial.print("IR Sensors: [ ");
    for (int i = 0; i < 8; i++) {
        Serial.print(sensorsReading[i]);
        if (i < 7) {
            Serial.print(", ");
        }
    }
    Serial.print(" ]");

    Serial.print(" | Distance: ");
    Serial.print(distanceCm);
    Serial.println(" cm");

    Serial.println(" ");
    delay(500);
}

void motorsDebug() {
    Robot.forward(100);
    delay(1000);
    Robot.backward(100);
    delay(1000);
    Robot.turnRight(100);
    delay(1000);
    Robot.turnLeft(100);
    delay(1000);
    Robot.stop();
    delay(500);
}

// ---------------------------- MAIN ------------------------------- //
void setup() {
    analogSetWidth(12);
    analogSetAttenuation(ADC_11db);
    Serial.begin(115200);

    pinMode(PB, INPUT_PULLDOWN);   pinMode(DIP1, INPUT_PULLDOWN);
    pinMode(DIP2, INPUT_PULLDOWN); pinMode(DIP3, INPUT_PULLDOWN);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

}

void loop() {

    // sensorsDebug();
    // motorsDebug();

    const float error = Sensors.getError();
    double dt = (millis() - t) / 1000.0;
    if (dt <= 0) dt = 0.001;
    t = millis();
    const float pid_out = constrain(pid.computePID(error ,dt), -MAX_SPEED, MAX_SPEED);
    controlMotors(pid_out);
}