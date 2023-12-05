#include <Wire.h>
#include <ArduPID.h>

ArduPID heat;
ArduPID cool;

double Temp_H, Temp_C;
double PID_H, PID_C;

double Tc = 0;  // Initialize to 0 initially
double p = 22.2;
double i = 1.08;
double d = 114;

const int ThermistorPin = 2;  // Use pin 5 for analog input on ESP32-C3 DevModule by Liligo
int Vo;
float R1 = 100000;
float logR2, R2, Tf;
float c1 = 0.8271998108e-03, c2 = 2.087906265e-04, c3 = 0.8061925470e-07;

void setup() 
{
  pinMode(ThermistorPin, INPUT);
  pinMode(5, OUTPUT);

  Serial.begin(115200);

  // Ask the user to input the initial target temperature
  while (!setTargetTemperature()) 
  {
    Serial.println("Invalid input. Please enter a valid target temperature within the range of 5 to 80.");
  }

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

void loop() 
{
  checkUserInput();

  if (Tc > 0) 
  {  // Check if Tc has been set by the user
    Vo = analogRead(ThermistorPin);
    R2 = R1 * Vo / (4095 - Vo);  // Adjust for 12-bit ADC
    logR2 = log(R2);
    Temp_H = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2)) - 273.15;

    Serial.print("Temperature: ");
    Serial.print(Temp_H);
    Serial.print("/");
    Serial.print(Tc);
    Serial.println(" C");

    if (Tc > 25) 
    {
      Serial.println("heating");
      heat.compute();
      PID_H = 255 - PID_H;
      analogWrite(4, PID_H);  // Use pin 5 for your application
      Serial.println(PID_H);
    } 
    else if (Tc < 25) 
    {
      Serial.println("cooling");
      Temp_C = Tc - Temp_H + Tc;
      cool.compute();
      PID_C = 255 - PID_C;
      analogWrite(4, PID_C);  // Use pin 5 for your application
      Serial.println(PID_C);
    } 
    else 
    {
      Serial.println("Set Temperature");
    }

    delay(1000);
  }
}

void checkUserInput() 
{
  if (Serial.available() > 0) 
  {
    String input = Serial.readStringUntil('\n');

    if (input.equalsIgnoreCase("stop")) 
    {
      Serial.println("Temperature control operation stopped. Enter a new target temperature to resume.");
      Tc = 0;  // Reset target temperature
    } 
    else 
    {
      float newTc = input.toFloat();  // Convert input to float

      if (isnan(newTc) || newTc < 5 || newTc > 80) {
        Serial.println("Invalid input. Please enter a valid number within the range of 5 to 80.");
      }
      else 
      {
        Tc = newTc;  // Set the new target temperature
      }
    }
  }
}

bool setTargetTemperature() 
{
  Serial.println("Enter the initial target temperature (in Celsius): ");
  while (Serial.available() == 0) {
    // Wait for user input
  }

  // Read user input
  String input = Serial.readStringUntil('\n');

  if (input.equalsIgnoreCase("stop")) 
  {
    Serial.println("Temperature control operation stopped.");
    return false;
  }

  float initialTc = input.toFloat();  // Convert input to float

  // Check if the input is a valid float and within the range of 5 to 80
  if (isnan(initialTc) || initialTc <= 5 || initialTc >= 80) 
  {
    Serial.println("Invalid input. Please enter a valid number within the range of 5 to 80.");
    return false;
  }

  Tc = initialTc;  // Set the initial target temperature
  return true;
}
