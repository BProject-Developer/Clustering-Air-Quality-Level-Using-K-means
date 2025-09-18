#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHT.h"

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
const uint8_t OLED_ADDR = 0x3C;

// DHT22
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// MQ135
#define MQ_PIN 34 // ADC1_CH6

// Normalisasi (rentang kasar awal)
float mq_min = 200.0, mq_max = 3500.0;
float t_min  = 10.0,  t_max  = 50.0;
float h_min  = 20.0,  h_max  = 90.0;

// Dataset buffer
const int DATASET_SIZE = 50;
float mq_data[DATASET_SIZE];
float t_data[DATASET_SIZE];
float h_data[DATASET_SIZE];
int collected = 0;
bool training_done = false;

// Centroid untuk 3 cluster
struct Centroid {
  float mq, t, h;
};
Centroid centroids[3];

// Fungsi utilitas
float clamp01(float v) { if (v < 0) return 0; if (v > 1) return 1; return v; }
float nrm(float x, float mn, float mx) { return clamp01((x - mn) / (mx - mn)); }

float distance3(float mq, float t, float h, Centroid c) {
  return sqrt(pow(mq - c.mq, 2) + pow(t - c.t, 2) + pow(h - c.h, 2));
}

int nearestCluster(float mq, float t, float h) {
  float minDist = 9999; int idx = 0;
  for (int i = 0; i < 3; i++) {
    float d = distance3(mq, t, h, centroids[i]);
    if (d < minDist) { minDist = d; idx = i; }
  }
  return idx;
}

// Hybrid rule
String hybridStatus(float mq, float suhu, float hum) {
  if (mq < 300 && suhu >= 20 && suhu <= 37 && hum >= 40 && hum <= 60) {
    return "GOOD";
  } else if (mq > 1000 || ((suhu < 20 || suhu > 37) && (hum < 40 || hum > 60))) {
    return "BAD";
  } else {
    return "MODERATE";
  }
}

// Center text
void drawCenteredText(const char* text, int textSize, int delayMs) {
  display.clearDisplay();
  display.setTextSize(textSize);
  display.setTextColor(SSD1306_WHITE);
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - w) / 2;
  int y = (SCREEN_HEIGHT - h) / 2;
  display.setCursor(x, y);
  display.println(text);
  display.display();
  delay(delayMs);
}

// K-Means training
void trainKMeans() {
  // Inisialisasi centroid random (ambil dari dataset)
  centroids[0] = { mq_data[0], t_data[0], h_data[0] };
  centroids[1] = { mq_data[DATASET_SIZE/2], t_data[DATASET_SIZE/2], h_data[DATASET_SIZE/2] };
  centroids[2] = { mq_data[DATASET_SIZE-1], t_data[DATASET_SIZE-1], h_data[DATASET_SIZE-1] };

  bool changed = true;
  int maxIter = 30, iter = 0;

  while (changed && iter < maxIter) {
    changed = false;
    float sum_mq[3] = {0}, sum_t[3] = {0}, sum_h[3] = {0};
    int count[3] = {0};

    // Assign data ke cluster terdekat
    for (int i = 0; i < DATASET_SIZE; i++) {
      int c = nearestCluster(mq_data[i], t_data[i], h_data[i]);
      sum_mq[c] += mq_data[i];
      sum_t[c]  += t_data[i];
      sum_h[c]  += h_data[i];
      count[c]++;
    }

    // Update centroid
    for (int c = 0; c < 3; c++) {
      if (count[c] == 0) continue;
      float new_mq = sum_mq[c] / count[c];
      float new_t  = sum_t[c] / count[c];
      float new_h  = sum_h[c] / count[c];
      if (fabs(new_mq - centroids[c].mq) > 0.001 ||
          fabs(new_t - centroids[c].t) > 0.001 ||
          fabs(new_h - centroids[c].h) > 0.001) {
        changed = true;
      }
      centroids[c] = { new_mq, new_t, new_h };
    }
    iter++;
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  analogReadResolution(12);
  analogSetPinAttenuation(MQ_PIN, ADC_11db);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    for (;;);
  }

  // Opening text
  drawCenteredText("Monitoring", 1, 1500);
  drawCenteredText("Kualitas Udara", 1, 1500);
  drawCenteredText("Developed by", 1, 2000);
  drawCenteredText("BProject Developer", 1, 2000);
  drawCenteredText("Warming up sensor", 1, 6000);

  // Proses K-Means
  drawCenteredText("Collecting Data", 1, 2000);
  while (collected < DATASET_SIZE) {
    int raw = analogRead(MQ_PIN);
    float mq = nrm((float)raw, mq_min, mq_max);
    float suhu = nrm(dht.readTemperature() - 5.0, t_min, t_max);
    float hum  = nrm(dht.readHumidity(), h_min, h_max);
    if (!isnan(suhu) && !isnan(hum)) {
      mq_data[collected] = mq;
      t_data[collected] = suhu;
      h_data[collected] = hum;
      collected++;
    }
    delay(100);
  }

  drawCenteredText("Normalizing", 1, 1500);
  drawCenteredText("Training", 1, 2000);
  trainKMeans();
  drawCenteredText("Evaluating", 1, 1500);
  drawCenteredText("Done", 1, 1500);

  training_done = true;
}

void loop() {
  int raw = analogRead(MQ_PIN);
  float mq = (float)raw;
  float suhu = dht.readTemperature() - 5.0;
  float hum = dht.readHumidity();

  if (isnan(suhu) || isnan(hum)) {
    suhu = 28; hum = 60;
  }

  // Normalisasi untuk clustering
  float mq_n = nrm(mq, mq_min, mq_max);
  float t_n  = nrm(suhu, t_min, t_max);
  float h_n  = nrm(hum, h_min, h_max);

  int cluster = nearestCluster(mq_n, t_n, h_n);
  String status = hybridStatus(mq, suhu, hum);

  // OLED tampilan default
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

  Serial.printf("RAW MQ=%d  T=%.2fC  H=%.2f%%  -> Cluster=%d  Hybrid=%s\n",
                raw, suhu, hum, cluster, status.c_str());

  delay(500);
}