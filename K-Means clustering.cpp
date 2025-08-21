#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHT.h"

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
const uint8_t OLED_ADDR = 0x3C; // kebanyakan 0x3C

// ---------- DHT22 ----------
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ---------- MQ135 ----------
#define MQ_PIN 34 // ADC1_CH6
// gunakan divider 100k : 100k dari AO -> GPIO34 -> GND

// ---------- Normalisasi (min-max awal, akan di-update) ----------
float mq_min = 200.0, mq_max = 3500.0;   // ADC raw (akan berubah)
float t_min  = 20.0,  t_max  = 40.0;     // °C
float h_min  = 20.0,  h_max  = 90.0;     // %RH

// ---------- K-Means (streaming) ----------
const int K = 3;               // 0=Good, 1=Moderate, 2=Bad
float C[K][3];                 // centroid[k][feature]
bool  centroidsInit = false;
float alpha = 0.05;            // learning rate online update

// ---------- Helper ----------
float clamp01(float v){ if(v<0) return 0; if(v>1) return 1; return v; }
float nrm(float x, float mn, float mx){
  if (mx - mn < 1e-6) return 0.5;
  return clamp01((x - mn) / (mx - mn));
}
float dist3(const float a[3], const float b[3]){
  float d0=a[0]-b[0], d1=a[1]-b[1], d2=a[2]-b[2];
  return sqrtf(d0*d0 + d1*d1 + d2*d2);
}
int argmin3(float d0, float d1, float d2){
  if(d0 <= d1 && d0 <= d2) return 0;
  if(d1 <= d0 && d1 <= d2) return 1;
  return 2;
}

void drawUI(int cluster, float mq, float t, float h){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  const char* names[3] = {"GOOD", "MODERATE", "BAD"};
  display.setCursor(0,0);
  display.print("Air Cluster: ");
  display.println(names[cluster]);

  // Bars
  int barW = 100, x0=14;
  // MQ bar
  display.setCursor(0,16); display.print("MQ ");
  int w_mq = (int)(barW * nrm(mq, mq_min, mq_max));
  display.drawRect(x0, 16, barW, 8, SSD1306_WHITE);
  display.fillRect(x0, 16, w_mq, 8, SSD1306_WHITE);

  // Temp bar
  display.setCursor(0,30); display.print("T  ");
  int w_t = (int)(barW * nrm(t, t_min, t_max));
  display.drawRect(x0, 30, barW, 8, SSD1306_WHITE);
  display.fillRect(x0, 30, w_t, 8, SSD1306_WHITE);

  // Hum bar
  display.setCursor(0,44); display.print("H  ");
  int w_h = (int)(barW * nrm(h, h_min, h_max));
  display.drawRect(x0, 44, barW, 8, SSD1306_WHITE);
  display.fillRect(x0, 44, w_h, 8, SSD1306_WHITE);

  display.display();
}

void initCentroids(float mq, float t, float h){
  // inisialisasi pakai tebakan masuk akal (setelah normalisasi)
  float v[3] = {
    nrm(mq, mq_min, mq_max),
    nrm(t,  t_min,  t_max),
    nrm(h,  h_min,  h_max)
  };
  // Good: mq rendah, t & h moderat
  C[0][0] = 0.2; C[0][1] = 0.5; C[0][2] = 0.6;
  // Moderate: tengah
  C[1][0] = 0.5; C[1][1] = 0.6; C[1][2] = 0.5;
  // Bad: mq tinggi, humidity bisa rendah/tinggi → ambil 0.4
  C[2][0] = 0.85; C[2][1] = 0.7; C[2][2] = 0.4;

  // “nudge” sedikit ke pembacaan awal biar adaptif
  for(int k=0;k<K;k++){
    for(int j=0;j<3;j++){
      C[k][j] = 0.8f*C[k][j] + 0.2f*v[j];
    }
  }
  centroidsInit = true;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // I2C + OLED
  Wire.begin(21,22);
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)){
    // gagal OLED: tetap jalan headless
  }
  display.clearDisplay(); display.display();

  // DHT
  dht.begin();

  // ADC ESP32
  analogReadResolution(12);           // 0..4095
  analogSetPinAttenuation(MQ_PIN, ADC_11db); // ~3.6V full-scale

  // Splash
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0); display.println("Air Cluster (K-Means)");
  display.setCursor(0,12); display.println("Warm-up sensor...");
  display.display();

  // optional: pendek pemanasan awal
  delay(3000);
}

void loop() {
  // Baca sensor
  int raw = analogRead(MQ_PIN);           // 0..4095 (sudah set 11dB)
  float mq = (float)raw;

  float t = dht.readTemperature();        // Celsius
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {             // retry sederhana
    delay(200);
    t = dht.readTemperature();
    h = dht.readHumidity();
  }
  if (isnan(t) || isnan(h)) { t = 28; h = 60; } // fallback aman

  // Update rentang min-max pelan (agar adaptif, anti-noise)
  mq_min = fminf(mq_min, mq); mq_max = fmaxf(mq_max, mq);
  t_min  = fminf(t_min,  t);  t_max  = fmaxf(t_max,  t);
  h_min  = fminf(h_min,  h);  h_max  = fmaxf(h_max,  h);

  // Normalisasi
  float v[3] = {
    nrm(mq, mq_min, mq_max),
    nrm(t,  t_min,  t_max),
    nrm(h,  h_min,  h_max)
  };

  // Init centroid kalau belum
  if(!centroidsInit) initCentroids(mq, t, h);

  // Tentukan cluster terdekat
  float d0 = dist3(v, C[0]);
  float d1 = dist3(v, C[1]);
  float d2 = dist3(v, C[2]);
  int cluster = argmin3(d0,d1,d2);

  // Online update centroid: C_k <- (1-alpha)*C_k + alpha*v
  for(int j=0;j<3;j++){
    C[cluster][j] = (1.0f - alpha)*C[cluster][j] + alpha*v[j];
  }

  // Tampilkan
  drawUI(cluster, mq, t, h);

  // Debug
  Serial.printf("RAW MQ=%d  T=%.2f  H=%.2f  -> cluster=%d\n", raw, t, h, cluster);

  delay(600); // ~1.6 Hz
}