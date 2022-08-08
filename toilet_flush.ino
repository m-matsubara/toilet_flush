/**
 * Toilet flush   Copyright © 2022 m.matsubara
 * 
 * ## 概要
 *   距離センサーを用いてトイレの着座と離席を判断し、離席後一定時間たったら（タンクの水がたまる時間待つ）トイレを流す。
 * 
 * ## 参考
 *   M5StickC(ESP32)で赤外線リモコンを作ろう
 *   https://qiita.com/coppercele/items/ed91646944ca28ff0c07
 * 
 *   M5StickCで人感センサー (人感センサーは赤外線量の変化を見るので、トイレの着座判定には不向きのため使えない)
 *   https://make-iot.com/2020/12/11/m5stickc%E3%81%A7%E4%BA%BA%E6%84%9F%E3%82%BB%E3%83%B3%E3%82%B5%E3%83%BC/
 * 
 *   M5StickC/examples/Hat/TOF/TOF.ino (距離センサー)
 *   https://github.com/m5stack/M5StickC/blob/master/examples/Hat/TOF/TOF.ino
 *   
 *   M5StickCであそぶ 〜6軸センサを使う〜
 *   https://make-muda.net/2019/09/6932/
 *   
 *   M5Displayクラスの使い方
 *   https://lang-ship.com/reference/unofficial/M5StickC/Tips/M5Display/
 * 
 *   M5StickCでの省電力ノウハウ
 *   https://lang-ship.com/blog/work/m5stickc-power-saving/
 *
 * ## ライブラリ
 *   
 *   M5StickCPlus      0.0.8  : M5Stack
 *   IRremoteESP8266   2.8.2  : David Conran, Mark Szabo, Sebastien Warin, Roi Dayan, Massimiliano Pinto, Christian Nilsson
 *   VL53L0X           1.3.1  : pololu
 *   
 */

#include <Arduino.h>
#include <M5StickCPlus.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <VL53L0X.h>
#include <Wire.h>

// 赤外線LED接続端子定数
// const uint16_t IR_LED = 9;  // 内蔵赤外線 LED
const uint16_t IR_LED = 32; // Grove端子 IR REMOTE 赤外線I/O

// 定数
const int SITDOWN_TIMER   = 120000; // 長時間着座タイマー
const int COUNTDOWN_TIMER = 120000; // カウントダウンタイマー(ms)（離席後時間経過後にトイレフラッシュ）
//const int SITDOWN_TIMER   = 12000; // 長時間着座タイマー
//const int COUNTDOWN_TIMER = 12000; // カウントダウンタイマー(ms)（離席後時間経過後にトイレフラッシュ）

// ステータス
int status = 0; // 0:待機中, 1:着座確認（長時間着座待ち）, 2:長時間着座(離席待ち) 3:カウントダウン, 4:手動カウントダウン
// 一部のステータスで使用するステータスが変更された時刻
unsigned long timeValue = 0;

// 赤外線送信クラス
IRsend irsend(IR_LED);  // Set the GPIO to be used to sending the message.

// 画面利用フラグ
boolean lcdUseFlag = false; // デフォルト消灯

/**
 * 画面初期化
 */
void initDisplay() {
  // 解像度：135x240
  //M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(5, 0, 2);
  M5.Lcd.println("Lixil flush v1.0");
}

/**
 * ディスプレイON
 */
void displayOn() {
  lcdUseFlag = true;
  M5.Axp.SetLDO2(lcdUseFlag);
}

/**
 * ディスプレイOFF
 */
void displayOff() {
  lcdUseFlag = false;
  M5.Axp.SetLDO2(lcdUseFlag);
}

/**
 * ディスプレイON/OFFトグル
 */
void displayToggle() {
  lcdUseFlag = false;
  M5.Axp.SetLDO2(lcdUseFlag);
}

/**
 * トイレフラッシュ（大）関数(INAX)
 */
void flush() {
  // CPU の速度を240Mhz に戻さないと赤外線が送れない
  setCpuFrequencyMhz(240);
  M5.Lcd.fillScreen(BLUE);
  irsend.sendInax(0x5C30CF);
  delay(500);
  initDisplay();
  // CPU速度を10Mhzに変更
  setCpuFrequencyMhz(10);
}


void setup() {
  // M5初期化
  M5.begin();
  
  // LCD明るさ
  M5.Axp.ScreenBreath(10); // 6より下はかなり見づらく、消費電力もあまり落ちないらしい
  M5.Axp.SetLDO2(lcdUseFlag);

  // ディスプレイ初期化
  initDisplay();
  
  // 赤外線LEDの初期化
  irsend.begin();

  // 6軸センサ初期化
  M5.Imu.Init();
  
  // CPUスピードを10MHzに変更
  setCpuFrequencyMhz(10);
}

void loop() {
  M5.update();

  // ボタン処理
  if (M5.BtnA.wasReleased()) {
    // M5 ボタンが押されたらディスプレイ点灯・トイレフラッシュ・トイレフラッシュキャンセル
    if (lcdUseFlag == false) {
      displayOn();
    } else if (status != 4) {
      status = 4; // 手動カウントダウン 
      timeValue = millis();
      displayOn();
    } else {
      status = 0; // 強制的に待機モードに変更
      displayOff();
    }
  }
  if (M5.BtnB.wasReleased()) {
    // 画面向かって右側のボタンが押されたら、LCD ON/OFF トグル
    displayToggle();
  }

  // 加速度取得
  float accX = 0;
  float accY = 0;
  float accZ = 0;
  M5.Imu.getAccelData(&accX, &accY, &accZ);

  // 着座判定
  //sitOnFlg = digitalRead(PIR_IO);
  //sitOnFlg = (analogRead(PIR_IO) > 2000);
  boolean sitOnFlg = (accY < 0.5);  // LCDを上に向けた状態で 0.0, USB Type-Cコネクタを下に向けて立てた状態で 1.0

  // 判定処理
  switch (status) {
  case 0:   // 待機中 
    if (sitOnFlg) {
      status = 1; // 着座した
      timeValue = millis();
    }
    break;
  case 1:   // 着座確認（長時間着座待ち）
    if (sitOnFlg == false) {
      status = 0;
    } else if (millis() - timeValue >= SITDOWN_TIMER) {
      status = 2;
      timeValue = millis();
    }
    break;
  case 2:   // 長時間着座(離席待ち)
    if (sitOnFlg == false) {
      status = 3; // 離席した…カウントダウン
      timeValue = millis();
      displayOn();
    }
    break;
  case 3:   // カウントダウン
    if (sitOnFlg) {
      status = 2; // 着座した(長時間着座(離席待ち)に戻る)
    } else if (millis() - timeValue >= COUNTDOWN_TIMER) {
      flush();
      status = 0;
      displayOff();
    }
    break;
  case 4:   // 手動カウントダウン
    if (millis() - timeValue >= COUNTDOWN_TIMER) {
      flush();
      status = 0;
      lcdUseFlag = false;
      M5.Axp.SetLDO2(lcdUseFlag);
    }
    break;
  }

  // ステータスの表示
  M5.Lcd.setCursor(5, 30, 4);
  switch (status) {
  case 0:   // 待機中 
    M5.Lcd.print("Waiting      ");
    break;
  case 1:   // 着座確認（長時間着座待ち）
    M5.Lcd.print("Sit on       ");
    break;
  case 2:   // 長時間着座(離席待ち)
    M5.Lcd.print("Sit on *     ");
    break;
  case 3:   // カウントダウン
    M5.Lcd.print("Cnt-dwn      ");
    break;
  case 4:   // 手動カウントダウン
    M5.Lcd.print("Cnt-dwn      ");
    break;
  }

  M5.Lcd.setCursor(5, 60, 7);
  if (status == 3 || status == 4) {
    int secTime = (COUNTDOWN_TIMER - (millis() - timeValue)) / 1000;
    if (secTime < 0)
      secTime = 0;  // タイミングによってはマイナスになってしまうこともある
    M5.Lcd.printf("%.03ds   ", secTime);
  } else {
    M5.Lcd.print("        ");
  }

  // 加速度のデバッグ表示
  M5.Lcd.setCursor(5, 208, 2);
  M5.Lcd.printf("Acc:\n  %.2f %.2f %.2f   ", accX, accY, accZ);

  delay(50);
}
