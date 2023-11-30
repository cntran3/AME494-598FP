#include <Wire.h>
#include <ArduPID.h>

ArduPID heat;
ArduPID cool;

double Temp_H, Temp_C;
double PID_H, PID_C;

double Tc = 5;
double p = 22.2;
double i = 1.08;
double d = 114;

int ThermistorPin = 34;  // Change this to a valid analog pin on ESP32
int Vo;
float R1 = 10000;
float logR2, R2, Tf;
float c1 = 0.3063689350e-03, c2 = 2.750609090e-04, c3 = -0.7805212717e-07;

void setup() {
  pinMode(25, OUTPUT);  // LED
  pinMode(21, OUTPUT);  // RELAY1
  pinMode(19, OUTPUT);  // RELAY2
  pinMode(2, OUTPUT);   // PELTIER
  pinMode(ThermistorPin, INPUT);   // PELTIER

  Serial.begin(115200);

  heat.begin(&Temp_H, &PID_H, &Tc, p, i, d);
  heat.setSampleTime(10);
  heat.setOutputLimits(0, 255);
  heat.setBias(255.0 / 2.0);
  heat.setWindUpLimits(-10, 10);
  heat.start();

  cool.begin(&Temp_C, &PID_C, &Tc, p, i, d);
  cool.setSampleTime(10);
  cool.setOutputLimits(0, 255);
  cool.setBias(255.0 / 2.0);
  cool.setWindUpLimits(-10, 10);
  cool.start();
}

void loop() {
  Vo = analogRead(ThermistorPin);
  R2 = R1 * Vo / (4095 - Vo);  // Adjust for 12-bit ADC
  logR2 = log(R2);
  Temp_H = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2)) - 273.15;

  Serial.print("Temperature: ");
  Serial.print(Temp_H);
  Serial.print("/");
  Serial.print(Tc);
  Serial.println(" C");


  if (Tc > 25) {
    Serial.println("heating");
    heat.compute();
    digitalWrite(21, LOW);
    digitalWrite(19, LOW);
    analogWrite(2, PID_H);
    Serial.println(PID_H);
  } else if (Tc < 25) {
    Serial.println("cooling");
    Temp_C = Tc - Temp_H + Tc;
    cool.compute();
    digitalWrite(21, HIGH);
    digitalWrite(19, HIGH);
    analogWrite(2, PID_C);
    Serial.println(PID_C);
  } else {
    Serial.println("Set Temperature");
  }

  delay(1000);
}
