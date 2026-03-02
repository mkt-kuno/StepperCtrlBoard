#include <Arduino.h>
#include <FastAccelStepper.h>

#ifdef USE_LCD
#include "NonBlockingLcd.h"
static NonBlockingLcd lcd;
void updateLcdContent(bool ena, bool dir, bool fast, float speed,
                      bool mode, bool limited);
#endif

#define UART_RX        (0)
#define UART_TX        (1)
#define SW_LIMIT_CW    (2)
#define SW_LIMIT_CCW   (3)
#define SW_MODE        (4)
#define SW_COM_ENA     (5)
#define SW_MAN_ENA     (6)
#define SW_COM_DIR     (7)
#define SW_MAN_DIR     (8)
#define SW_MAN_FAST    (9)
#define MOTOR_STEP    (10)
#define MOTOR_DIR     (11)
#define MOTOR_ENA     (12)
#define LED_SYSTEM    (13)
#define LED_MODE      (A0) // 14
#define LED_ENA       (A1) // 15
#define LED_DIR       (A2) // 16
#define LED_FAST      (A3) // 17
#define I2C_SDA       (A4) // 18
#define I2C_SCL       (A5) // 19
#define ADC_COM_SPEED (A6)
#define ADC_MAN_SPEED (A7)

// ステップ周波数の範囲 (Hz)
#define STEP_FREQ_MIN    50
#define STEP_FREQ_MAX  5000
// FAST モード時の倍率
#define STEP_FAST_MULT   10
// FastAccelStepper (AVR, 1stepper) の上限は 50000Hz
#define STEP_FREQ_LIMIT  50000UL

// ADC電圧の閾値 (V) — この電圧以下ではモーター停止
#define SPEED_THRESHOLD 0.5f

// 加速度 (steps/s²) — 大きいほど速度変化が即応的
#define STEP_ACCEL  100000UL

// --- オーバーサンプリング ADC (チャンネル別バッファ) ---
#define ADC_OS_NUM 16
struct AdcOsState {
  uint16_t buf[ADC_OS_NUM];
  uint8_t  ptr;
};

static AdcOsState adcManState = {{0}, 0};
static AdcOsState adcComState = {{0}, 0};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper       *stepper = NULL;

void setup() {
  // Limit switches (CW / CCW)
  pinMode(SW_LIMIT_CW,   INPUT_PULLUP);
  pinMode(SW_LIMIT_CCW,  INPUT_PULLUP);
  // MODE
  pinMode(SW_MODE,       INPUT_PULLUP);
  // MAN_ENA,MAN_DIR
  pinMode(SW_MAN_ENA,    INPUT_PULLUP);
  pinMode(SW_MAN_DIR,    INPUT_PULLUP);
  // LED_MODE, LED_ENA, LED_DIR, LED_FAST
  pinMode(LED_MODE,   OUTPUT);
  pinMode(LED_ENA,    OUTPUT);
  pinMode(LED_DIR,    OUTPUT);
  pinMode(LED_FAST,   OUTPUT);
  pinMode(LED_SYSTEM, OUTPUT);
  // MOTOR_ENA / MOTOR_DIR は手動制御
  pinMode(MOTOR_ENA,  OUTPUT);
  pinMode(MOTOR_DIR,  OUTPUT);
  // MOTOR_STEP は FastAccelStepper が管理 (NPN トランジスタによるパルス反転はハードウェア側で対応)

  // FastAccelStepper 初期化 (Timer1 使用)
  engine.init();
  stepper = engine.stepperConnectToPin(MOTOR_STEP);
  if (stepper) {
    stepper->setAcceleration(STEP_ACCEL);
  }

  // SW_MAN_FAST (pin 9 = OC1A) は Timer1 初期化後に設定
  // engine.init() が Timer1 を再構成するため、先に設定すると上書きされる可能性がある
  pinMode(SW_MAN_FAST, INPUT_PULLUP);

#ifdef USE_LCD
  lcd.init();
#endif
}

float analogReadOS(uint8_t pin, AdcOsState* st) {
  st->buf[st->ptr++] = analogRead(pin);
  if (st->ptr >= ADC_OS_NUM) st->ptr = 0;
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_OS_NUM; i++) sum += st->buf[i];
  return 5.0f * sum / 1024.0f / ADC_OS_NUM;
}

// --- 前回の状態を保持して無駄な再設定を避ける ---
static bool     prevRunning = false;
static uint32_t prevFreq    = 0;
static bool     prevDir     = false;

void loop() {
  bool  ena  = false;
  bool  dir  = false;
  bool  fast = false;
  float speed = 0;
  float man_speed = analogReadOS(ADC_MAN_SPEED, &adcManState);
  float com_speed = analogReadOS(ADC_COM_SPEED, &adcComState);
  bool  mode      = digitalRead(SW_MODE);
  bool  limit_cw  = !digitalRead(SW_LIMIT_CW);
  bool  limit_ccw = !digitalRead(SW_LIMIT_CCW);

  // Computer
  if (mode) {
    ena   = digitalRead(SW_COM_ENA);
    dir   = digitalRead(SW_COM_DIR);
    fast  = false;
    speed = com_speed;
  }
  // Manual
  else {
    ena   = digitalRead(SW_MAN_ENA);
    dir   = digitalRead(SW_MAN_DIR);
    fast  = digitalRead(SW_MAN_FAST);
    speed = man_speed;
  }

  // リミットスイッチ: dir=true → CW, dir=false → CCW
  // 該当方向のリミットに当たっている場合はその方向の動作のみ禁止 (逆方向は許可)
  bool limited = (limit_cw && dir) || (limit_ccw && !dir);

  // Update LED
  digitalWrite(LED_MODE, mode);
  digitalWrite(LED_ENA,  ena);
  digitalWrite(LED_DIR,  dir);
  digitalWrite(LED_FAST, fast);

  // for debug
  digitalWrite(LED_SYSTEM, speed > 0.5f ? HIGH : LOW);

  // --- ステップパルス制御 (FastAccelStepper) ---
  bool shouldRun = ena && (speed > SPEED_THRESHOLD) && !limited;

  // 方向変更 or 停止時のみパルス停止 → 出力変更 → 再始動の順序を保証
  // FAST 切替は速度変更のみなので force-stop 不要 (加減速で対応)
  if (prevRunning && (dir != prevDir || !shouldRun)) {
    stepper->forceStopAndNewPosition(0);
    prevRunning = false;
  }

  // MOTOR 出力更新 (パルス停止後に変更 → TB6600 セットアップタイム確保)
  // NPN トランジスタで反転されるため論理反転
  digitalWrite(MOTOR_ENA, !ena);
  digitalWrite(MOTOR_DIR, !dir);
  prevDir  = dir;

  if (shouldRun) {
    // 電圧 → 周波数 に変換
    float ratio = (speed - SPEED_THRESHOLD) / (5.0f - SPEED_THRESHOLD);
    if (ratio > 1.0f) ratio = 1.0f;
    uint32_t freq = (uint32_t)(STEP_FREQ_MIN + ratio * (STEP_FREQ_MAX - STEP_FREQ_MIN));

    // FAST モード (ライブラリ上限でクランプ)
    if (fast) freq *= STEP_FAST_MULT;
    if (freq > STEP_FREQ_LIMIT) freq = STEP_FREQ_LIMIT;

    // 状態が変化した場合のみ再設定
    if (!prevRunning || freq != prevFreq) {
      stepper->setSpeedInHz(freq);
      stepper->runForward();  // 物理方向は MOTOR_DIR ピンで決定
      prevFreq    = freq;
      prevRunning = true;
    }
  }

#ifdef USE_LCD
  updateLcdContent(ena, dir, fast, speed, mode, limited);
  lcd.update();
#endif
}

#ifdef USE_LCD
// LCD表示内容を更新するヘルパー関数
// 必要に応じてカスタマイズしてください
void updateLcdContent(bool ena, bool dir, bool fast, float speed,
                      bool mode, bool limited) {
  // 1行目: モード・状態
  char line[LCD_COLS + 1];
  snprintf(line, sizeof(line), "%s %s %s%s",
           mode ? "COM" : "MAN",
           ena  ? "RUN" : "STP",
           dir  ? "CW " : "CCW",
           fast ? " FST" : "");
  lcd.setText(0, line);

  // 2行目: 速度・リミット
  if (limited) {
    lcd.setText(1, "** LIMITED **   ");
  } else {
    int sv = (int)(speed * 100);
    snprintf(line, sizeof(line), "SPD:%d.%02dV       ", sv / 100, sv % 100);
    lcd.setText(1, line);
  }
}
#endif
