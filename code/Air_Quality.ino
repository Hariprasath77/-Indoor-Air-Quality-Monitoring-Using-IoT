#define BLYNK_TEMPLATE_ID "TMPL3lOe6MApV"
#define BLYNK_TEMPLATE_NAME "Air Quality Monitoring"
#define BLYNK_AUTH_TOKEN "gfIqhm4TmglaEUT_-leZwtXGB583nSrI"
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);
#include "DHT.h"
#define DHTPIN 4
#define DHTTYPE DHT11
#include <LiquidCrystal.h>
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);
DHT dht(DHTPIN, DHTTYPE);

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "PROJECT";
char pass[] = "11111111";

#define measurePin 39 //Connect dust sensor to Arduino A0 pin
#define ledPower 13   //Connect 3 led driver pins of dust sensor to Arduino D2
int samplingTime = 280; // time required to sample signal coming out   of the sensor
int deltaTime = 40; //
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

WidgetLCD lcd1(V0);

#define mq135 36
#define mq7 34
#define buzzer 22

#define m1 25
#define m2 26
#define m3 27
#define m4 14

int Speed = 204;  // 0 - 255.

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lcd.begin(16, 2);
  dht.begin();
  Blynk.begin(auth, ssid, pass);
  ss.begin(GPSBaud);
  pinMode(mq135, INPUT);
  pinMode(mq135, INPUT);
  pinMode(ledPower, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();

  lcd.setCursor(0, 0);
  lcd.print("  Air Quality  ");
  lcd.setCursor(0, 1);
  lcd.print("  Monitoring  ");
  lcd1.print(0, 0, "  Air Quality  ");
  lcd1.print(0, 1, "  Monitoring   ");
  delay(5000);
  lcd.clear();
  while(1){
  int Gas135 = analogRead(mq135);
  int Gas7 = analogRead(mq7);

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);

  digitalWrite(ledPower, LOW); // power on the LED
  delayMicroseconds(samplingTime);
  voMeasured = analogRead(measurePin); // read the dust value
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower, HIGH); // turn the LED off
  delayMicroseconds(sleepTime);
  calcVoltage = voMeasured * (5.0 / 1024.0);
  dustDensity = 170 * calcVoltage - 0.1;

  Serial.println(F("Temperature: "));
  Serial.println(t);
  Serial.println(F("Humidity: "));
  Serial.println(h);
  Blynk.virtualWrite(V1, Gas135);
  Blynk.virtualWrite(V2, Gas7);
  Blynk.virtualWrite(V3, t);
  Blynk.virtualWrite(V4, h);
  Blynk.virtualWrite(V9, dustDensity);

  if (Gas135 >= 4050) {
    lcd.clear();
    lcd1.clear();
    delay(100);
    lcd.setCursor(5, 0);
    lcd.print("MQ135");
    lcd.setCursor(3, 1);
    lcd.print("Gas is HIGH");
    lcd1.print(5, 0, "MQ135");
    lcd1.print(3, 1, "Gas is HIGH");
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(100);
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(100);
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(100);
    lcd.clear();
    lcd1.clear();
  } else if (Gas7 >= 2000) {
    lcd.clear();
    lcd1.clear();
    delay(100);
    lcd.setCursor(6, 0);
    lcd.print("MQ7");
    lcd.setCursor(3, 1);
    lcd.print("Gas is HIGH");
    lcd1.print(6, 0, "MQ7");
    lcd1.print(3, 1, "Gas is HIGH");
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(100);
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(100);
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(100);
    lcd.clear();
    lcd1.clear();
  } else if (t >= 40) {
    lcd.clear();
    lcd1.clear();
    delay(100);
    lcd.setCursor(3, 0);
    lcd.print("Temp is HIGH");
    lcd1.print(3, 0, "Temp is HIGH");
    delay(2000);
    lcd.clear();
    lcd1.clear();
  } else if (h >= 80) {
    lcd.clear();
    lcd1.clear();
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Humidity is HIGH");
    lcd1.print(0, 0, "Humidity is HIGH");
    delay(2000);
    lcd.clear();
    lcd1.clear();
  }
  else if (dustDensity >= 800) {
    lcd.clear();
    lcd1.clear();
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("Dust is HIGH");
    lcd1.print(0, 0, "Dust is HIGH");
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(100);
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(100);
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(100);
    lcd.clear();
    lcd1.clear();
  }
  else {
    while (ss.available() > 0)
  {
    gps.encode(ss.read());
    if (gps.location.isUpdated())
   {
      Serial.println("Location");
      Serial.print("Latitude= "); 
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude= "); 
      Serial.println(gps.location.lng(), 6);
      delay(2000);
    }
  }
    lcd.setCursor(0, 0);
    lcd.print("Lati:");
    lcd.setCursor(5, 0);
    lcd.print(gps.location.lat());
    lcd.setCursor(0, 1);
    lcd.print("Long:");
    lcd.setCursor(5, 1);
    lcd.print(gps.location.lng());
    lcd1.print(0, 0, "Lati:");
    lcd1.print(5, 0, gps.location.lat());
    lcd1.print(0, 1, "Long:76°53'02.2E");
    lcd1.print(5, 1, gps.location.lng());
  }
  }
}
BLYNK_WRITE(V5) {
  if (param.asInt() == 1) {
    forward();
  } else {
    Stop();
  }
}
BLYNK_WRITE(V6) {
  if (param.asInt() == 1) {
    back();
  } else {
    Stop();
  }
}
BLYNK_WRITE(V7) {
  if (param.asInt() == 1) {
    right();
  } else {
    Stop();
  }
}
BLYNK_WRITE(V8) {
  if (param.asInt() == 1) {
    left();
  } else {
    Stop();
  }
}

void forward() {
  digitalWrite(m1, Speed);
  digitalWrite(m3, Speed);
}

void back() {
  digitalWrite(m2, Speed);
  digitalWrite(m4, Speed);
}

void right() {
  digitalWrite(m2, Speed);
  digitalWrite(m3, Speed);
}

void left() {
  digitalWrite(m1, Speed);
  digitalWrite(m4, Speed);
}
void Stop() {
  digitalWrite(m1, 0);
  digitalWrite(m2, 0);
  digitalWrite(m3, 0);
  digitalWrite(m4, 0);
}
