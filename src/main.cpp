/*
 * SMART BRAKING SYSTEM - Hamilton 44
 * FreeRTOS on ESP32 | Wokwi
 * Fitur: Multi-Sensor Fusion, ESA, Adaptive Braking, RTOS
*/
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ESP32Servo.h>

// === PIN ===
#define TRIG_F   5
#define ECHO_F   18
#define TRIG_L   16
#define ECHO_L   17
#define TRIG_R   32
#define ECHO_R   33
#define LED_G    25
#define LED_R    26
#define BUZ_PIN  27
#define BRAKE_PIN 14
#define STEER_PIN 13
#define BTN_PIN  19
#define DHT_PIN  4
#define POT_PIN  34

// === BUZZER LEDC (channel terpisah dari servo) ===
#define BUZ_CH   4

// === OBJEK HARDWARE ===
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHT_PIN, DHT22);
Servo brakeServo, steerServo;

// === IPC HANDLES ===
SemaphoreHandle_t xMutexLCD;
SemaphoreHandle_t xSemEmg;
QueueHandle_t xQueueThrottle;

// === SHARED DATA ===
volatile int dFront = 400, dLeft = 400, dRight = 400;
volatile int critDist = 100;
volatile bool emgActive = false;
volatile int humidity_val = 0;

// === ISR: Tombol Darurat ===
void IRAM_ATTR emgISR() {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(xSemEmg, &woken);
  if (woken) portYIELD_FROM_ISR();
}

// === BUZZER (LEDC-based, aman untuk ESP32Servo) ===
void buzOn(int freq)  { ledcWriteTone(BUZ_CH, freq); }
void buzOff()         { ledcWriteTone(BUZ_CH, 0); }

// === BACA ULTRASONIK ===
int bacaSensor(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 15000);
  return (dur == 0) ? 400 : (int)(dur * 0.034 / 2);
}

// --- DEKLARASI TASK (FUNGSI) ---
void TaskEnv(void *pvParameters);
void TaskUI(void *pvParameters);
void TaskTele(void *pvParameters);
void TaskRadar(void *pvParameters);
void TaskAEB(void *pvParameters);

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(LED_G, OUTPUT);  pinMode(LED_R, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);

  // Buzzer LEDC (channel 4, tidak bentrok dengan servo channel 0-1)
  ledcSetup(BUZ_CH, 2000, 8);
  ledcAttachPin(BUZ_PIN, BUZ_CH);

  // Servo
  brakeServo.attach(BRAKE_PIN);  brakeServo.write(0);
  steerServo.attach(STEER_PIN);  steerServo.write(90);

  // Sensor & LCD
  dht.begin();
  lcd.init(); lcd.backlight();
  lcd.print("Hamilton 44");
  lcd.setCursor(0, 1); lcd.print("Booting RTOS...");
  delay(1500); lcd.clear();

  // ISR
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), emgISR, FALLING);

  // IPC
  xMutexLCD      = xSemaphoreCreateMutex();
  xSemEmg        = xSemaphoreCreateBinary();
  xQueueThrottle = xQueueCreate(1, sizeof(int));

  // === TASK SCHEDULER ===
  xTaskCreatePinnedToCore(TaskEnv,    "Env",  3072, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskUI,     "UI",   3072, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskTele,   "Tele", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskRadar,  "Rada", 3072, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskAEB,    "AEB",  3072, NULL, 4, NULL, 1);

  Serial.println("[SETUP] RTOS Ready.");
}

void loop() { vTaskDelay(portMAX_DELAY); }

// ==================== TASK 1: Environment (P1) ====================
// DHT22 kelembapan -> adaptive braking distance
void TaskEnv(void *p) {
  for (;;) {
    if (!emgActive) {
      float h = dht.readHumidity();
      if (!isnan(h)) {
        humidity_val = (int)h;
        critDist = (h > 80.0) ? 150 : 100;
      }
      Serial.print("[ENV] H:"); Serial.print(h);
      Serial.print("% Lim:"); Serial.println(critDist);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ==================== TASK 2: Telemetry ADC (P2) ====================
// Baca throttle potentiometer -> kirim via Queue
void TaskTele(void *p) {
  for (;;) {
    if (!emgActive) {
      int val = analogRead(POT_PIN);
      xQueueOverwrite(xQueueThrottle, &val);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ==================== TASK 3: Radar Fusion (P3) ====================
// Baca 3 sensor ultrasonik -> trigger emergency jika terlalu dekat
void TaskRadar(void *p) {
  for (;;) {
    if (!emgActive) {
      dFront = bacaSensor(TRIG_F, ECHO_F);
      vTaskDelay(pdMS_TO_TICKS(15));
      dLeft  = bacaSensor(TRIG_L, ECHO_L);
      vTaskDelay(pdMS_TO_TICKS(15));
      dRight = bacaSensor(TRIG_R, ECHO_R);

      Serial.print("[RADAR] F:"); Serial.print(dFront);
      Serial.print(" L:"); Serial.print(dLeft);
      Serial.print(" R:"); Serial.println(dRight);

      if (dFront < critDist) {
        xSemaphoreGive(xSemEmg);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ==================== TASK 4: Dashboard UI (P1) ====================
// LCD display + LED status
void TaskUI(void *p) {
  int thr = 0;
  for (;;) {
    if (!emgActive) {
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, LOW);
      xQueueReceive(xQueueThrottle, &thr, 0);

      if (xSemaphoreTake(xMutexLCD, pdMS_TO_TICKS(200)) == pdTRUE) {
        lcd.setCursor(0, 0);
        lcd.print("D:"); lcd.print(dFront);
        lcd.print("cm T:"); lcd.print(map(thr, 0, 4095, 0, 100));
        lcd.print("%   ");
        lcd.setCursor(0, 1);
        lcd.print("Lim:"); lcd.print(critDist);
        lcd.print(" H:"); lcd.print(humidity_val);
        lcd.print("%   ");
        xSemaphoreGive(xMutexLCD);
      }
      Serial.print("[METRIK] Sisa RAM Task UI (Words): ");
      Serial.println(uxTaskGetStackHighWaterMark(NULL));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ==================== TASK 5: AEB + ESA (P4 - Kritis) ====================
// Emergency Braking + Evasive Steering Assist
void TaskAEB(void *p) {
  for (;;) {
    if (xSemaphoreTake(xSemEmg, portMAX_DELAY) == pdTRUE) {
      emgActive = true;
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_R, HIGH);

      // Snapshot sensor data
      int f = dFront, l = dLeft, r = dRight;
      
      // --- ESA: Keputusan Menghindar ---
      const char* aksi;
      if (l > 200 && l >= r) {
        steerServo.write(45);  brakeServo.write(45);
        aksi = "EVADE LEFT!";
      } else if (r > 200) {
        steerServo.write(135); brakeServo.write(45);
        aksi = "EVADE RIGHT!";
      } else {
        steerServo.write(90);  brakeServo.write(90);
        aksi = "FULL BRAKE!";
      }

      // LCD Update (Mutex)
      if (xSemaphoreTake(xMutexLCD, pdMS_TO_TICKS(500)) == pdTRUE) {
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("!! DANGER !!");
        lcd.setCursor(0, 1); lcd.print(aksi);
        xSemaphoreGive(xMutexLCD);
      }

      Serial.print("[AEB] "); Serial.print(aksi);
      Serial.print(" F:"); Serial.print(f);
      Serial.print(" L:"); Serial.print(l);
      Serial.print(" R:"); Serial.println(r);

      // Alarm buzzer (LEDC)
      for (int i = 0; i < 5; i++) {
        buzOn(1000);  vTaskDelay(pdMS_TO_TICKS(200));
        buzOff();     vTaskDelay(pdMS_TO_TICKS(200));
      }

      // Recovery
      steerServo.write(90);
      brakeServo.write(0);

      if (xSemaphoreTake(xMutexLCD, pdMS_TO_TICKS(500)) == pdTRUE) {
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("Recovering...");
        xSemaphoreGive(xMutexLCD);
      }

      vTaskDelay(pdMS_TO_TICKS(1000));

      if (xSemaphoreTake(xMutexLCD, pdMS_TO_TICKS(500)) == pdTRUE) {
        lcd.clear();
        xSemaphoreGive(xMutexLCD);
      }

      emgActive = false;
      Serial.println("[AEB] Recovery OK.\n");
    }
  }
}