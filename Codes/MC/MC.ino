#include <Wire.h>
#include <ArduPID.h>
#include <PubSubClient.h>
#include <WiFi.h>

const char* ssid = "Home";
const char* password = "sundevils";
const char* mqtt = "192.168.0.237";

WiFiClient extruder; // Declare the extruder variable
PubSubClient client(extruder);

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
  connectToWiFi();
  client.setServer(mqtt, 1883);
  client.setCallback(callback);

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
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  if (Tc > 0)
  {
    // Check if Tc has been set by the user
    Vo = analogRead(ThermistorPin);
    R2 = R1 * Vo / (4095 - Vo); // Adjust for 12-bit ADC
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
      analogWrite(4, PID_H); // Use pin 5 for your application
      Serial.println(PID_H);
    }
    else if (Tc < 25)
    {
      Serial.println("cooling");
      Temp_C = Tc - Temp_H + Tc;
      cool.compute();
      PID_C = 255 - PID_C;
      analogWrite(4, PID_C); // Use pin 5 for your application
      Serial.println(PID_C);
    }
    else
    {
      Serial.println("Set Temperature");
    }

    delay(1000);
  }
}

void connectToWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

void callback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");

  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);

  if (messageTemp.equals("stop"))
  {
    // Stop the temperature control operation
    Tc = 0;
    analogWrite(4, 0); // Turn off heating/cooling
    Serial.println("Temperature control operation stopped.");
  }
  else
  {
    // Convert the incoming message to a float and set it as the new target temperature
    float newTc = messageTemp.toFloat();
    if (!isnan(newTc) && newTc >= 5 && newTc <= 80)
    {
      Tc = newTc;
      Serial.print("New target temperature set to: ");
      Serial.println(Tc);
    }
    else
    {
      Serial.println("Invalid temperature received. Ignoring.");
    }
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient"))
    {
      Serial.println("connected");
      // Subscribe to the topic
      client.subscribe("temp/command");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
