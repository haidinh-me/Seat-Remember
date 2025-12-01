#include <Arduino.h>
#include <EEPROM.h>
#include <ESP32Servo.h>

// ------------------- Hardware mapping ------------------------
constexpr uint8_t POT_FB = 15;  // forward/backward potentiometer
constexpr uint8_t POT_UD = 4;   // up/down potentiometer

constexpr uint8_t PIN_SET = 25;  // buttons (active‑low)
constexpr uint8_t PIN_B1 = 12;
constexpr uint8_t PIN_B2 = 14;

constexpr uint8_t SERVO_UP = 23;
constexpr uint8_t SERVO_DOWN = 22;

constexpr uint8_t LED_PIN_SET = 32;  // LED
constexpr uint8_t LED_PIN_B1 = 27;
constexpr uint8_t LED_PIN_B2 = 26;

struct MotorPins {
  uint8_t in1, in2;
};
MotorPins motorFB{ 18, 19};  // motor channel
MotorPins motorUD{ 5, 17};
static Servo myservo;
constexpr uint8_t SERVO_PIN = 13;

// ------------------- Control constants ----------------------
constexpr uint16_t EEPROM_SIZE = 128;
constexpr uint16_t ADC_TOLERANCE = 15;  // dead‑band (≈0.5 %)
constexpr uint8_t MIN_PWM = 80;         // motor starts spinning here
constexpr uint8_t MAX_PWM = 255;
constexpr uint32_t LONGPRESS_MS = 2000;  // hold SET ≥1 s

// ------------------- Data structures ------------------------
struct SeatPos {
  uint16_t fb, ud, recline;
} __attribute__((packed));

enum class Mode : uint8_t { NORMAL,
                            LEARN };
enum class BtnEvent : uint8_t { SET_DOWN,
                                SET_UP,
                                BTN1,
                                BTN2 };

// ------------------- Globals --------------------------------
static QueueHandle_t btnQueue;
static Mode g_mode = Mode::NORMAL;
static bool g_autoMove = false;  // true = đang tự quay tới preset
static SeatPos g_target;         // mục tiêu khi autoMove
static bool g_setHeld = false;
static uint16_t targetAngle = 0;
static TickType_t g_setPressTick = 0;
static TickType_t g_learnTick = 0;

// ------------------- Forward declarations -------------------
SeatPos readADC();
void driveMotor(const MotorPins&, int8_t dir);
bool moveAxis(const MotorPins&, int32_t now, int32_t tgt);
void writePreset(uint8_t id, const SeatPos& p);
SeatPos readPreset(uint8_t id);
void servoTask(void*);

// ------------------- GPIO ISRs ------------------------------
static volatile uint32_t lastIsr_us_set = 0,
                         lastIsr_us_b1 = 0,
                         lastIsr_us_b2 = 0;
constexpr uint32_t DEBOUNCE_US = 20000;

void IRAM_ATTR isrSET() {
  uint32_t now_us = micros();
  if (now_us - lastIsr_us_b1 < DEBOUNCE_US / 10) return;
  lastIsr_us_set = now_us;
  bool pressed = (digitalRead(PIN_SET) == LOW);
  BtnEvent ev = pressed ? BtnEvent::SET_DOWN : BtnEvent::SET_UP;
  xQueueSendFromISR(btnQueue, &ev, nullptr);
}

void IRAM_ATTR isrB1() {
  uint32_t now_us = micros();
  if (now_us - lastIsr_us_b1 < DEBOUNCE_US) return;
  lastIsr_us_b1 = now_us;
  BtnEvent ev = BtnEvent::BTN1;
  xQueueSendFromISR(btnQueue, &ev, nullptr);
}

void IRAM_ATTR isrB2() {
  uint32_t now_us = micros();
  if (now_us - lastIsr_us_b2 < DEBOUNCE_US) return;
  lastIsr_us_b2 = now_us;
  BtnEvent ev = BtnEvent::BTN2;
  xQueueSendFromISR(btnQueue, &ev, nullptr);
}

// ------------------- FreeRTOS button task -------------------
void buttonTask(void*) {
  BtnEvent ev;
  for (;;) {
    // 1) Process queued events ----------------------------------------
    while (xQueueReceive(btnQueue, &ev, 0) == pdPASS) {
      switch (ev) {
        case BtnEvent::SET_DOWN:
          g_setHeld = true;
          g_setPressTick = xTaskGetTickCount();
          break;
        case BtnEvent::SET_UP:
          g_setHeld = false;
          break;

        case BtnEvent::BTN1:
          if (g_mode == Mode::LEARN) {
            writePreset(0, readADC());
            digitalWrite(LED_PIN_B1, HIGH);
            vTaskDelay(pdMS_TO_TICKS(500));
            digitalWrite(LED_PIN_B1, LOW);
            Serial.println("Saved preset 1");
            g_mode = Mode::NORMAL;
          } else if (!g_autoMove) {  // start AUTO only if idle
            g_target = readPreset(0);
            g_autoMove = true;
            Serial.println("Auto-move to preset 1");
          }
          break;

        case BtnEvent::BTN2:
          if (g_mode == Mode::LEARN) {
            writePreset(1, readADC());
            digitalWrite(LED_PIN_B2, HIGH);
            vTaskDelay(pdMS_TO_TICKS(500));
            digitalWrite(LED_PIN_B2, LOW);
            Serial.println("Saved preset 2");
            g_mode = Mode::NORMAL;
          } else if (!g_autoMove) {
            g_target = readPreset(1);
            g_autoMove = true;
            Serial.println("Auto-move to preset 2");
          }
          break;
      }
    }

    // 2) Detect long‑press SET to enter LEARN -------------------------
    if (g_mode == Mode::NORMAL && g_setHeld && (xTaskGetTickCount() - g_setPressTick) > pdMS_TO_TICKS(LONGPRESS_MS)) {
      g_mode = Mode::LEARN;
      g_learnTick = xTaskGetTickCount();
      Serial.println("== ENTER LEARN MODE ==");
    }

    // 3) Exit LEARN MODE
    if (g_mode == Mode::LEARN && !g_setHeld) {
      g_mode = Mode::NORMAL;
      Serial.println("== LEARN MODE OUT ==");
    }
    if (g_mode == Mode::LEARN) digitalWrite(LED_PIN_SET, HIGH);
    else digitalWrite(LED_PIN_SET, LOW);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
// ------------------- Servo task -----------------------
void servoTask(void*) {
  for (;;) {
    // Trong NORMAL:
    if (g_mode == Mode::NORMAL && !g_autoMove) {
      if (!digitalRead(SERVO_UP)) {
        targetAngle = min<uint16_t>(targetAngle + 5, 180);
        Serial.println("SERVO_UP: " + String(targetAngle));
        myservo.attach(SERVO_PIN, 500, 2400);
      }
      if (!digitalRead(SERVO_DOWN)) {
        targetAngle = targetAngle ? targetAngle - 5 : 0;
        Serial.println("SERVO_DOWN: " + String(targetAngle));
        myservo.attach(SERVO_PIN, 500, 2400);
      }
    }
    // Trong AUTO‑MOVE:
    else if (g_autoMove) {
      targetAngle = g_target.recline;
      myservo.attach(SERVO_PIN, 500, 2400);
    }
    myservo.write(targetAngle);
    vTaskDelay(pdMS_TO_TICKS(50));  // 50 Hz update
  }
}


// ------------------- Setup ----------------------------------
void setup() {
  Serial.begin(115200);

  analogReadResolution(12);

  myservo.attach(SERVO_PIN, 500, 2400);

  pinMode(PIN_SET, INPUT_PULLUP);
  pinMode(PIN_B1, INPUT_PULLUP);
  pinMode(PIN_B2, INPUT_PULLUP);
  pinMode(SERVO_UP, INPUT_PULLUP);
  pinMode(SERVO_DOWN, INPUT_PULLUP);
  pinMode(POT_FB,INPUT);
  pinMode(POT_UD,INPUT);
  pinMode(LED_PIN_SET, OUTPUT);
  pinMode(LED_PIN_B1, OUTPUT);
  pinMode(LED_PIN_B2, OUTPUT);

  auto initMotor = [](MotorPins& m) {
    pinMode(m.in1, OUTPUT);
    pinMode(m.in2, OUTPUT);
  };
  initMotor(motorFB);
  initMotor(motorUD);

  EEPROM.begin(EEPROM_SIZE);

  btnQueue = xQueueCreate(10, sizeof(BtnEvent));
  xTaskCreatePinnedToCore(buttonTask, "ButtonTask", 2048, nullptr, 5, nullptr, APP_CPU_NUM);
  xTaskCreatePinnedToCore(servoTask, "ServoTask", 2048, nullptr, 4, nullptr, APP_CPU_NUM);

  attachInterrupt(PIN_SET, isrSET, CHANGE);
  attachInterrupt(PIN_B1, isrB1, FALLING);
  attachInterrupt(PIN_B2, isrB2, FALLING);

  // No auto‑move at boot
  g_autoMove = false;
}

// ------------------- Main loop (motor control) -------------
void loop() {

  if (g_autoMove) {
    SeatPos now = readADC();
    // ---- Điều khiển tự động tới preset ----
    bool doneFB = moveAxis(motorFB, now.fb, g_target.fb);
    bool doneUD = moveAxis(motorUD, now.ud, g_target.ud);

    if (doneFB && doneUD) {
      g_autoMove = false;  // reached!
      Serial.println("== Arrived preset ==\n");
    }
  }
  vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz loop
}

// ------------------- Helper functions ----------------------
SeatPos readADC() {
  return {
    static_cast<uint16_t>(analogRead(POT_FB)),
    static_cast<uint16_t>(analogRead(POT_UD)),
    targetAngle
  };
}

void driveMotor(const MotorPins& m, int8_t dir) {
  if (dir > 0) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
  } else if (dir < 0) {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
  } else {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
  }
}

// Return true if axis is within tolerance (done)
bool moveAxis(const MotorPins& m, int32_t now, int32_t tgt) {
  int32_t delta = tgt - now;
  if (abs(delta) <= ADC_TOLERANCE) {
    driveMotor(m, 0);
    return true;
  }
  driveMotor(m, (delta > 0) ? +1 : -1);
  return false;
}

void writePreset(uint8_t id, const SeatPos& p) {
  EEPROM.put(id * sizeof(SeatPos), p);
  EEPROM.commit();
}

SeatPos readPreset(uint8_t id) {
  SeatPos p;
  EEPROM.get(id * sizeof(SeatPos), p);
  return p;
}
