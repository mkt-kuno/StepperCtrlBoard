#pragma once

#include <LiquidCrystal_I2C.h>

#ifndef LCD_I2C_ADDR
#define LCD_I2C_ADDR 0x27
#endif

#define LCD_COLS 16
#define LCD_ROWS  2
#define LCD_TOTAL (LCD_COLS * LCD_ROWS)

class NonBlockingLcd {
public:
  void init() {
    lcd_.begin(LCD_COLS, LCD_ROWS);
    lcd_.backlight();
    lcd_.clear();
    memset(front_, ' ', LCD_TOTAL);
    memset(back_,  ' ', LCD_TOTAL);
    scanPos_ = 0;
  }

  // 行単位でバッファ更新 (row: 0 or 1)
  void setText(uint8_t row, const char* text) {
    if (row >= LCD_ROWS) return;
    uint8_t offset = row * LCD_COLS;
    uint8_t i = 0;
    for (; i < LCD_COLS && text[i] != '\0'; i++) {
      back_[offset + i] = text[i];
    }
    for (; i < LCD_COLS; i++) {
      back_[offset + i] = ' ';
    }
  }

  // 1文字単位でバッファ更新
  void setChar(uint8_t row, uint8_t col, char ch) {
    if (row >= LCD_ROWS || col >= LCD_COLS) return;
    back_[row * LCD_COLS + col] = ch;
  }

  // 毎ループ呼び出し: dirty な1文字だけ実際にLCDへ書き込む
  // 戻り値: 書き込みを行った場合 true
  bool update() {
    for (uint8_t i = 0; i < LCD_TOTAL; i++) {
      uint8_t pos = (scanPos_ + i) % LCD_TOTAL;
      if (front_[pos] != back_[pos]) {
        front_[pos] = back_[pos];
        lcd_.setCursor(pos % LCD_COLS, pos / LCD_COLS);
        lcd_.write(front_[pos]);
        scanPos_ = (pos + 1) % LCD_TOTAL;
        return true;
      }
    }
    scanPos_ = 0;
    return false;
  }

private:
  LiquidCrystal_I2C lcd_{LCD_I2C_ADDR, LCD_COLS, LCD_ROWS};
  char front_[LCD_TOTAL]; // 現在LCDに表示中の内容
  char back_[LCD_TOTAL];  // 次に表示したい内容
  uint8_t scanPos_;       // スキャン位置
};
