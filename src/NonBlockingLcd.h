#pragma once

#include <Arduino.h>
#include <Wire.h>

#define LCD_COLS 16
#define LCD_ROWS  2
#define LCD_TOTAL (LCD_COLS * LCD_ROWS)

// HD44780 commands
#define LCD_CMD_CLEARDISPLAY   0x01
#define LCD_CMD_RETURNHOME     0x02
#define LCD_CMD_ENTRYMODESET   0x06  // increment, no shift
#define LCD_CMD_DISPLAYOFF     0x08  // display off
#define LCD_CMD_DISPLAYON      0x0C  // display on, cursor off, blink off
#define LCD_CMD_FUNCTIONSET    0x28  // 4-bit, 2-line, 5x8

// PCF8574 bit mapping
#define LCD_BIT_RS  0x01
#define LCD_BIT_RW  0x02
#define LCD_BIT_EN  0x04
#define LCD_BIT_BL  0x08  // backlight

// I2C scan range for PCF8574 / PCF8574A LCD backpacks
static constexpr uint8_t LCD_SCAN_ADDRS[] = {
  0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,  // PCF8574
  0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,  // PCF8574A
};
static constexpr uint8_t LCD_SCAN_COUNT = sizeof(LCD_SCAN_ADDRS);

class NonBlockingLcd {
public:
  // Call once in setup(). Returns immediately.
  // Non-blocking flow:
  //   Wire.begin -> I2C scan (0x20-0x27, 0x38-0x3F) ->
  //     if found: HD44780 init sequence.
  //     if not found: stays inactive (setText/update are no-op).
  void begin() {
    state_ = INIT_WIRE;
    waitUntil_ = millis() + 100;  // power-on settle before touching the bus
    scanPos_ = 0;
    bl_ = LCD_BIT_BL;
    active_ = false;
    addr_ = 0;
  }

  // No-op if scan found no LCD.
  void setText(uint8_t row, const char* text) {
    if (!active_ || row >= LCD_ROWS) return;
    uint8_t offset = row * LCD_COLS;
    uint8_t i = 0;
    for (; i < LCD_COLS && text[i] != '\0'; i++) {
      back_[offset + i] = text[i];
    }
    for (; i < LCD_COLS; i++) {
      back_[offset + i] = ' ';
    }
  }

  void setChar(uint8_t row, uint8_t col, char ch) {
    if (!active_ || row >= LCD_ROWS || col >= LCD_COLS) return;
    back_[row * LCD_COLS + col] = ch;
  }

  // True if a usable LCD was detected at boot.
  bool isActive() const { return active_; }

  // True if init completed (READY state).
  bool isReady() const { return state_ == READY; }

  // Call every loop iteration. Non-blocking.
  bool update() {
    if (state_ < READY) return initStep();
    if (!active_) return false;

    // Diff-scan: find one dirty char and write it
    for (uint8_t i = 0; i < LCD_TOTAL; i++) {
      uint8_t pos = (scanPos_ + i) % LCD_TOTAL;
      if (front_[pos] != back_[pos]) {
        front_[pos] = back_[pos];
        setCursor(pos % LCD_COLS, pos / LCD_COLS);
        writeData(front_[pos]);
        scanPos_ = (pos + 1) % LCD_TOTAL;
        return true;
      }
    }
    scanPos_ = 0;
    return false;
  }

private:
  // Init state machine states
  enum State : uint8_t {
    INIT_WIRE = 0,     // Wire.begin + setClock + set short timeout
    INIT_SCAN,         // probe 0x20-0x27 and 0x38-0x3F for an LCD
    INIT_BL,           // backlight on, wait 1000ms for LCD power-up
    INIT_8BIT_1,       // first 0x03 write, wait >4.1ms
    INIT_8BIT_2,       // second 0x03 write, wait >4.1ms
    INIT_8BIT_3,       // third 0x03 write, wait >150us
    INIT_4BIT,         // switch to 4-bit mode, wait
    INIT_FUNCSET,      // function set (0x28)
    INIT_DISPOFF,      // display off (0x08)
    INIT_CLEAR,        // clear display (0x01), wait >1.52ms
    INIT_ENTRYMODE,    // entry mode set (0x06)
    INIT_DISPON,       // display on (0x0C)
    INIT_HOME,         // return home (0x02), wait >1.52ms + buffer init
    READY              // normal operation
  };

  State    state_     = READY;
  uint8_t  bl_        = LCD_BIT_BL;
  uint32_t waitUntil_ = 0;
  uint8_t  scanPos_   = 0;
  uint8_t  addr_      = 0;   // I2C address discovered by scan
  bool     active_    = false;

  char front_[LCD_TOTAL];
  char back_[LCD_TOTAL];

  // --- Init state machine ---
  bool initStep() {
    // If we're waiting for a delay, just return
    if (waitUntil_ && millis() < waitUntil_) return false;
    waitUntil_ = 0;

    switch (state_) {
      case INIT_WIRE:
        Wire.begin();
        Wire.setClock(100000);  // PCF8574 の最大 I2C クロックは 100kHz
        #if defined(WIRE_HAS_TIMEOUT) && WIRE_HAS_TIMEOUT > 0
        Wire.setWireTimeout(1000);  // 1ms — 短め: scan を高速化 + バス固着対策
        #endif
        waitUntil_ = millis() + 50;
        state_ = INIT_SCAN;
        break;

      case INIT_SCAN: {
        // Probe 16 candidate addresses.
        // NACK は即時、ACK は <1ms、setWireTimeout(1ms) が上限。
        for (uint8_t i = 0; i < LCD_SCAN_COUNT; i++) {
          uint8_t a = LCD_SCAN_ADDRS[i];
          Wire.beginTransmission(a);
          if (Wire.endTransmission() == 0) {
            addr_ = a;
            active_ = true;
            break;
          }
        }
        if (active_) {
          waitUntil_ = millis() + 50;
          state_ = INIT_BL;
        } else {
          // 液晶らしきものが無かった — 非アクティブで固定
          state_ = READY;
        }
        break;
      }

      case INIT_BL:
        expanderWrite(bl_);
        waitUntil_ = millis() + 1000;  // LCD power-up wait
        state_ = INIT_8BIT_1;
        break;

      case INIT_8BIT_1:
        write4bits(0x03 << 4);
        waitUntil_ = millis() + 5;     // >4.1ms
        state_ = INIT_8BIT_2;
        break;

      case INIT_8BIT_2:
        write4bits(0x03 << 4);
        waitUntil_ = millis() + 5;     // >4.1ms
        state_ = INIT_8BIT_3;
        break;

      case INIT_8BIT_3:
        write4bits(0x03 << 4);
        waitUntil_ = millis() + 1;     // >150us
        state_ = INIT_4BIT;
        break;

      case INIT_4BIT:
        write4bits(0x02 << 4);         // switch to 4-bit
        waitUntil_ = millis() + 1;     // wait for mode switch
        state_ = INIT_FUNCSET;
        break;

      case INIT_FUNCSET:
        command(LCD_CMD_FUNCTIONSET);   // 4-bit, 2-line, 5x8
        waitUntil_ = millis() + 1;
        state_ = INIT_DISPOFF;
        break;

      case INIT_DISPOFF:
        command(LCD_CMD_DISPLAYOFF);    // display off before clear
        waitUntil_ = millis() + 1;
        state_ = INIT_CLEAR;
        break;

      case INIT_CLEAR:
        command(LCD_CMD_CLEARDISPLAY);
        waitUntil_ = millis() + 3;     // >1.52ms
        state_ = INIT_ENTRYMODE;
        break;

      case INIT_ENTRYMODE:
        command(LCD_CMD_ENTRYMODESET);
        waitUntil_ = millis() + 1;
        state_ = INIT_DISPON;
        break;

      case INIT_DISPON:
        command(LCD_CMD_DISPLAYON);
        waitUntil_ = millis() + 1;
        state_ = INIT_HOME;
        break;

      case INIT_HOME:
        command(LCD_CMD_RETURNHOME);
        waitUntil_ = millis() + 3;     // >1.52ms
        memset(front_, ' ', LCD_TOTAL);
        memset(back_,  ' ', LCD_TOTAL);
        state_ = READY;
        break;

      default:
        break;
    }
    return false;
  }

  // --- I2C low-level (PCF8574 + HD44780) ---

  void i2cWrite(uint8_t data) {
    Wire.beginTransmission(addr_);
    Wire.write(data);
    Wire.endTransmission();
  }

  void expanderWrite(uint8_t data) {
    i2cWrite(data | bl_);
  }

  void pulseEnable(uint8_t data) {
    expanderWrite(data | LCD_BIT_EN);
    delayMicroseconds(1);   // Enable pulse width >450ns
    expanderWrite(data & ~LCD_BIT_EN);
    delayMicroseconds(50);  // Command execution time
  }

  void write4bits(uint8_t value) {
    expanderWrite(value);
    pulseEnable(value);
  }

  void send(uint8_t value, uint8_t mode) {
    uint8_t hi = value & 0xF0;
    uint8_t lo = (value << 4) & 0xF0;
    write4bits(hi | mode);
    write4bits(lo | mode);
  }

  void command(uint8_t cmd) {
    send(cmd, 0);           // RS=0
  }

  void writeData(uint8_t data) {
    send(data, LCD_BIT_RS); // RS=1
  }

  void setCursor(uint8_t col, uint8_t row) {
    static const uint8_t rowOffsets[] = {0x00, 0x40};
    command(0x80 | (col + rowOffsets[row]));
  }
};
