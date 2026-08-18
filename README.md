# NextGenCtrlBox

Arduino Nano ベースのステッピングモータ制御ボックス。手動スイッチ操作と外部コンピュータ制御の 2 モードに対応し、ボリュームによる速度調整・リミットスイッチによる安全停止機能を備えます。  
動作の安定のために、Arduino をUSB Serial経由（with Bootloader）ではなく、 Arduino-as-ICSPでICSP経由(without Bootloader)で書き込むことを強く推奨します  
Auduino-as-ICSPの接続時は、2x3(6pin)の極性にはくれぐれも注意してください(燃えます)。  
表示には5V 動作のI2C接続 1602 液晶を使用します。極性にはくれぐれも注意してください(燃えます)。  

## 機能一覧

- **Manual / Computer 切替** — MODE スイッチで操作モードを選択
- **可変速度制御** — ボリューム (ADC) による無段階速度調整 (50–5,000 Hz)
- **FAST モード** — 手動モード時に速度を 10 倍に引き上げ (最大 50,000 Hz)
- **CW / CCW リミットスイッチ** — 該当方向のみ停止、逆方向は許可
- **LED ステータス表示** — MODE / ENA / DIR / FAST の各状態を LED で表示
- **オーバーサンプリング ADC** — 16 サンプル移動平均でノイズを低減
- **FastAccelStepper** — ハードウェアタイマ (Timer1) による正確なパルス生成
- **FastGPIO** — デジタル入出力を直接レジスタ操作で高速化
- **ADC 割り込み駆動** — `ADC_vect` ISR による非同期 ADC 変換でループブロッキングを排除

## ハードウェア構成

### ピンアサイン (Arduino Nano)

| ピン | 名称 | 機能 |
|------|------|------|
| D0 | UART_RX | シリアル受信 (予約) |
| D1 | UART_TX | シリアル送信 (予約) |
| D2 | SW_LIMIT_CW | CW リミットスイッチ (INPUT_PULLUP) |
| D3 | SW_LIMIT_CCW | CCW リミットスイッチ (INPUT_PULLUP) |
| D4 | SW_MODE | Manual/Computer 切替 (INPUT_PULLUP) |
| D5 | SW_COM_ENA | Computer モード イネーブル |
| D6 | SW_MAN_ENA | Manual モード イネーブル (INPUT_PULLUP) |
| D7 | SW_COM_DIR | Computer モード 方向 |
| D8 | SW_MAN_DIR | Manual モード 方向 (INPUT_PULLUP) |
| D9 | SW_MAN_FAST | FAST モードスイッチ (INPUT_PULLUP) |
| D10 | MOTOR_STEP | ステップパルス出力 (FastAccelStepper) |
| D11 | MOTOR_DIR | モータ方向出力 |
| D12 | MOTOR_ENA | モータイネーブル出力 |
| D13 | LED_FAST | FAST 表示 LED |
| A0 | LED_MODE | MODE 表示 LED |
| A1 | LED_ENA | ENA 表示 LED |
| A2 | LED_DIR | DIR 表示 LED |
| A3 | LED_SYSTEM | システム LED (デバッグ) |
| A4 | I2C_SDA | I2C データ (予約) |
| A5 | I2C_SCL | I2C クロック (予約) |
| A6 | ADC_COM_SPEED | Computer モード速度ボリューム |
| A7 | ADC_MAN_SPEED | Manual モード速度ボリューム |

### 回路の注意点

MOTOR_ENA / MOTOR_DIR / MOTOR_STEP の各信号は **NPN トランジスタで反転** されて TB6600 ドライバに入力されます。そのためファームウェア上では論理を反転して出力しています:

```
Arduino ──[信号]──> NPN トランジスタ ──[反転信号]──> TB6600
```

- `FastGPIO::Pin<MOTOR_ENA>::setOutputValue(!ena)` — HIGH で無効、LOW で有効
- `FastGPIO::Pin<MOTOR_DIR>::setOutputValue(!dir)` — 反転後の論理で方向を決定

## パフォーマンス

FastGPIO と ADC 割り込みにより、メインループの所要時間を大幅に短縮しています。

| 処理 | 標準 Arduino | 最適化後 |
|------|-------------|---------|
| digitalRead × 8 | ~40 us | ~1 us (FastGPIO) |
| digitalWrite × 7 | ~35 us | ~0.9 us (FastGPIO) |
| analogRead × 2 | ~200 us | ~0.25 us (ADC 割り込み) |

## ビルド・書き込み

### 必要なもの

- [PlatformIO](https://platformio.org/) (CLI または VS Code 拡張)
- USBasp プログラマ
- Arduino Nano (ATmega328P)

### コマンド

```bash
# ビルド
pio run

# 書き込み (USBasp)
pio run --target upload
```

## ライセンス

MIT License
