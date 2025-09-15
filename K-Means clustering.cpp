#include <Wire.h>    
#include <Adafruit_GFX.h>    
#include <Adafruit_SSD1306.h>    
#include "DHT.h"    
    
// ---------- OLED ----------    
#define SCREEN_WIDTH 128    
#define SCREEN_HEIGHT 64    
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);    
const uint8_t OLED_ADDR = 0x3C;    
    
// ---------- DHT22 ----------    
#define DHTPIN 4    
#define DHTTYPE DHT22    
DHT dht(DHTPIN, DHTTYPE);    
    
// ---------- MQ135 ----------    
#define MQ_PIN 34 // ADC1_CH6    
    
// ---------- Normalisasi ----------    
float mq_min = 200.0, mq_max = 3500.0;   // ADC raw    
float t_min  = 10.0,  t_max  = 50.0;     // °C    
float h_min  = 20.0,  h_max  = 90.0;     // %RH    
    
// ---------- KMeans ----------    
struct Centroid {    
  float mq, suhu, hum;    
};    
    
Centroid centroids[3] = {    
  {0.2, 0.5, 0.6},   // Good    
  {0.5, 0.6, 0.5},   // Moderate    
  {0.85, 0.7, 0.4}   // Bad    
};    
    
float clamp01(float v){ if(v<0) return 0; if(v>1) return 1; return v; }    
float nrm(float x, float mn, float mx){ return clamp01((x - mn) / (mx - mn)); }    
    
float distance3(float mq, float t, float h, Centroid c) {    
  return sqrt(pow(mq - c.mq, 2) + pow(t - c.suhu, 2) + pow(h - c.hum, 2));    
}    
    
int classifyKMeans(float mq, float t, float h) {    
  float minDist = 9999;    
  int cluster = 0;    
  for (int i = 0; i < 3; i++) {    
    float d = distance3(mq, t, h, centroids[i]);    
    if (d < minDist) {    
      minDist = d;    
      cluster = i;    
    }    
  }    
  return cluster;    
}    
    
// ---------- Hybrid dengan MQ + Suhu + Humidity ----------    
String hybridStatus(float mq, float suhu, float hum) {    
  if (mq < 300 && suhu >= 20 && suhu <= 37 && hum >= 40 && hum <= 60) {    
    return "GOOD";    
  } else if (mq > 1000 || ((suhu < 20 || suhu > 37) && (hum < 40 || hum > 60))) {    
    return "BAD";    
  } else {    
    return "MODERATE";    
  }    
}    
    
// ---------- Fungsi untuk teks tengah ----------    
void drawCenteredText(const char* text, int textSize) {    
  display.clearDisplay();    
  display.setTextSize(textSize);    
  display.setTextColor(SSD1306_WHITE);    
    
  int16_t x1, y1;    
  uint16_t w, h;    
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);    
    
  int x = (SCREEN_WIDTH - w) / 2;    
  int y = (SCREEN_HEIGHT - h) / 2;    
    
  display.setCursor(x, y);    
  display.println(text);    
  display.display();    
}    
    
void setup() {    
  Serial.begin(115200);    
  dht.begin();    
    
  analogReadResolution(12); // 0..4095    
  analogSetPinAttenuation(MQ_PIN, ADC_11db);    
    
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {    
    for (;;);    
  }    
    
  // ---------- Opening Splash ----------    
  drawCenteredText("Monitoring", 1);    
  delay(1500);    
  drawCenteredText("Kualitas Udara", 1);    
  delay(1500);    
  drawCenteredText("Developed by", 1);    
  delay(2000);    
  drawCenteredText("BProject Developer", 1);    
  delay(2000);    
  drawCenteredText("Warming up sensor", 1);    
  delay(6000);    
}    
    
void loop() {    
  // --- Baca sensor ---    
  int raw = analogRead(MQ_PIN);    
  float mq = (float)raw;    
  float suhu = dht.readTemperature() - 5.0;  // KALIBRASI -5 °C    
  float hum = dht.readHumidity();    
    
  if (isnan(suhu) || isnan(hum)) {    
    suhu = 28;    
    hum = 60;    
  }    
    
  // --- Normalisasi ---    
  float mq_n = nrm(mq, mq_min, mq_max);    
  float t_n  = nrm(suhu, t_min, t_max);    
  float h_n  = nrm(hum, h_min, h_max);    
    
  // --- KMeans cluster ---    
  int cluster = classifyKMeans(mq_n, t_n, h_n);    
    
  // --- Hybrid status ---    
  String status = hybridStatus(mq, suhu, hum);    
    
  // --- Tampil OLED ---    
  display.clearDisplay();    
  display.setTextSize(1);    
  display.setTextColor(SSD1306_WHITE);    
    
  display.setCursor(0, 0);    
  display.println("Air Quality Monitor");    
    
  display.setCursor(0, 12);    
  display.print("MQ: "); display.println(mq);    
    
  display.setCursor(0, 22);    
  display.print("T: "); display.print(suhu); display.println(" C");    
    
  display.setCursor(0, 32);    
  display.print("H: "); display.print(hum); display.println(" %");    
    
  display.setCursor(0, 44);    
  display.print("KMeans: ");    
  if (cluster == 0) display.println("Good");    
  else if (cluster == 1) display.println("Moderate");    
  else display.println("Bad");    
    
  display.setCursor(0, 54);    
  display.print("Hybrid: "); display.println(status);    
    
  display.display();    
    
  // --- Serial Monitor ---    
  Serial.printf("RAW MQ=%d  T=%.2fC  H=%.2f%%  -> Cluster=%d  Hybrid=%s\n",    
                raw, suhu, hum, cluster, status.c_str());    
    
  delay(2000);    
}