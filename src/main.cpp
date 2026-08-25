#include <Arduino.h>
#include <FastAccelStepper.h>
#include <FastGPIO.h>

#include "NonBlockingLcd.h"
static NonBlockingLcd lcd;
void updateLcdContent(bool ena, bool dir, bool fast, float speed,
                      bool mode, bool limited, uint32_t freq);

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
#define LED_FAST      (13)
#define LED_MODE      (A0) // 14
#define LED_ENA       (A1) // 15
#define LED_DIR       (A2) // 16
#define LED_SYSTEM    (A3) // 17
#define I2C_SDA       (A4) // 18
#define I2C_SCL       (A5) // 19
#define ADC_COM_SPEED (A6)
#define ADC_MAN_SPEED (A7)

// ステップ周波数の範囲 (Hz)
#define STEP_FREQ_MIN   12.5
#define STEP_FREQ_MAX   5000
// FAST モード時の倍率
#define STEP_FAST_MULT    10
// FastAccelStepper (AVR, 1stepper) の上限は 50kHz
#define STEP_FREQ_LIMIT  50000UL

// ADC電圧の閾値 (V) — この電圧以下ではモーター停止
#define SPEED_THRESHOLD 1.0f

// 加速度 (steps/s²) — 大きいほど速度変化が即応的
#define STEP_ACCEL  100000UL

// --- オーバーサンプリング ADC (ISR内累積) ---
#define ADC_OS_SHIFT 6
#define ADC_OS_NUM   (1 << ADC_OS_SHIFT)  // 64サンプル累積

// --- ADC割り込み駆動 ---
// A6 = MUX[3:0] = 0110, A7 = MUX[3:0] = 0111
// 64サンプルの累積和を保持 (10bit×64 = 最大65472 → 16bitに収まる)
static volatile uint16_t adc_sum[2];      // [0]=A6(COM), [1]=A7(MAN)
static volatile uint8_t  adc_ch = 6;      // 現在の変換チャンネル (6 or 7)

ISR(ADC_vect) {
  static uint16_t acc = 0;
  static uint8_t  cnt = 0;
  uint16_t val = ADC; // ADCL + ADCH を読み取り
  if (cnt == 0) {
    // チャンネル切替直後の初回変換はダミー (S/Hクロストーク対策で捨てる)
    acc = 0;
    cnt = 1;
  } else {
    acc += val;
    if (cnt >= ADC_OS_NUM) {
      // 64サンプル完了 → 公開してチャンネル切替
      adc_sum[adc_ch - 6] = acc;
      adc_ch ^= 1; // 6 <-> 7
      ADMUX = (ADMUX & 0xF0) | (adc_ch & 0x0F);
      cnt = 0;
    } else {
      cnt++;
    }
  }
  ADCSRA |= (1 << ADSC);
}

static void adc_init() {
  // AVcc基準, チャンネル6
  ADMUX  = (1 << REFS0) | 6;
  // ADC有効, 割り込み有効, プリスケーラ128 (125kHz @ 16MHz)
  ADCSRA = (1 << ADEN) | (1 << ADIE)
         | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  // 最初の変換開始
  adc_ch = 6;
  ADCSRA |= (1 << ADSC);
}

static float adc_read_volts(uint8_t ch) {
  // ch: 0=A6(COM), 1=A7(MAN)
  uint16_t sum;
  cli();
  sum = adc_sum[ch];
  sei();
  // 累積和のまま電圧変換 — サブLSB分解能を保持 (実効約13bit)
  return 5.0f * sum / (1024.0f * ADC_OS_NUM);
}

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

  lcd.begin();

  // ADC割り込み初期化 (engine.init()後に呼ぶ — Arduino ADC設定の上書き回避)
  adc_init();
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
  float man_speed = adc_read_volts(1); // ch1 = A7(MAN)
  float com_speed = adc_read_volts(0); // ch0 = A6(COM)
  bool  mode      =  FastGPIO::Pin<SW_MODE>::isInputHigh();
  bool  limit_cw  = !FastGPIO::Pin<SW_LIMIT_CW>::isInputHigh();
  bool  limit_ccw = !FastGPIO::Pin<SW_LIMIT_CCW>::isInputHigh();

  // Computer
  if (mode) {
    ena   = FastGPIO::Pin<SW_COM_ENA>::isInputHigh();
    dir   = FastGPIO::Pin<SW_COM_DIR>::isInputHigh();
    fast  = false;
    speed = com_speed;
  }
  // Manual
  else {
    ena   = FastGPIO::Pin<SW_MAN_ENA>::isInputHigh();
    dir   = FastGPIO::Pin<SW_MAN_DIR>::isInputHigh();
    fast  = FastGPIO::Pin<SW_MAN_FAST>::isInputHigh();
    speed = man_speed;
  }

  // リミットスイッチ: dir=true → CW, dir=false → CCW
  // 該当方向のリミットに当たっている場合はその方向の動作のみ禁止 (逆方向は許可)
  bool limited = (limit_cw && dir) || (limit_ccw && !dir);

  // Update LED
  FastGPIO::Pin<LED_MODE>::setOutputValue(mode);
  FastGPIO::Pin<LED_ENA>::setOutputValue(ena);
  FastGPIO::Pin<LED_DIR>::setOutputValue(dir);
  FastGPIO::Pin<LED_FAST>::setOutputValue(fast);

  // for debug
  FastGPIO::Pin<LED_SYSTEM>::setOutputValue(speed > SPEED_THRESHOLD);

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
  FastGPIO::Pin<MOTOR_ENA>::setOutputValue(!ena);
  FastGPIO::Pin<MOTOR_DIR>::setOutputValue(!dir);
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

  uint32_t activeFreq = prevRunning ? prevFreq : 0;
  updateLcdContent(ena, dir, fast, speed, mode, limited, activeFreq);
  lcd.update();
}

// LCD表示内容を更新するヘルパー関数
// 必要に応じてカスタマイズしてください
void updateLcdContent(bool ena, bool dir, bool fast, float speed,
                      bool mode, bool limited, uint32_t freq) {
  // 1行目: モード・状態 (snprintf不使用 — SRAM節約)
  static char line[17];
  char* p = line;
  memcpy(p, mode ? "COM" : "MAN", 3); p += 3; *p++ = ' ';
  memcpy(p, ena  ? "RUN" : "STP", 3); p += 3; *p++ = ' ';
  memcpy(p, dir  ? "CW " : "CCW", 3); p += 3; *p++ = ' ';
  if (fast) { memcpy(p, "FAST", 4); p += 4; }
  *p = '\0';
  lcd.setText(0, line);

  // 2行目: 電圧 + PPS (またはリミット)
  if (limited) {
    lcd.setText(1, "** LIMITED **   ");
  } else {
    int sv = (int)(speed * 100.0f + 0.5f); // 10mV単位に丸め
    if (sv < 0) sv = 0;

    // 手動文字列構築 — snprintf のスタック消費を回避
    // "X.YY V ZZZZZ PPS" (16文字)
    static char buf[17];
    // 電圧部: "X.YY V " (7文字) — 1mV位はADC分解能未満のため非表示
    buf[0] = '0' + (char)(sv / 100);
    buf[1] = '.';
    buf[2] = '0' + (char)((sv / 10) % 10);
    buf[3] = '0' + (char)(sv % 10);
    buf[4] = ' ';
    buf[5] = 'V';
    buf[6] = ' ';
    // PPS部: "ZZZZZ PPS" (9文字) — 5桁右詰め・小数点なし
    uint32_t f = freq;
    for (int i = 11; i >= 7; i--) {
      buf[i] = (f > 0 || i == 11) ? ('0' + (char)(f % 10)) : ' ';
      f /= 10;
    }
    buf[12] = ' ';
    buf[13] = 'P';
    buf[14] = 'P';
    buf[15] = 'S';
    buf[16] = '\0';
    lcd.setText(1, buf);
  }
}
