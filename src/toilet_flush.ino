/**
 * Toilet flush (for Lixil)  Copyright © 2022 m.matsubara
 * 
 * ## 概要
 *   配管上の問題でトイレ詰まりをよく起こすトイレにおいて、使用後に念入りに流すため、離席後一定時間たったら（離席時に流すが、タンクに水がたまるのを待つ）、再度タイマーでトイレを流す（大）。
 *   
 * ## 対象機材（トイレ）
 * * リモコンで流す動作可能なINAXシャワートイレ (プレアス DT-CL114A・CH184A用リモコンで確認)
 *   (リモコンのコマンドが異なる場合、送信コマンドを赤外線センサで解析する必要があります。)
 *
 * ## 必要機材（このプログラムを動作させるのに使用する機材）
 * * M5StickC Plus
 * * M5Stack用赤外線送受信ユニット [U002] (本体の赤外線LEDが使える場合は不要・送信コマンドを赤外線センサで解析する必要がある場合は必要)
 * * M5StickC ToF Hat（必須ではない。使用した場合は、M5ボタンの代わりに距離センサーで操作可能)
 * * M5StickC PIR Hat（必須ではない。使用した場合は、接近するとディスプレイをONにできる)
 * 
 * ## 必要ライブラリ
 *   M5StickCPlus      0.0.8  : M5Stack
 *   IRremoteESP8266   2.8.2  : David Conran, Mark Szabo, Sebastien Warin, Roi Dayan, Massimiliano Pinto, Christian Nilsson
 *   VL53L0X           1.3.1  : pololu
 *   LovyanGFX         0.4.18 : lovyan03
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
 *   【M5Stack】第二回 LCDの使い方 全集
 *   https://shizenkarasuzon.hatenablog.com/entry/2020/05/21/012555
 *   
 *   LovyanGFX入門 その1 基本描画系
 *   https://lang-ship.com/blog/work/lovyangfx-1/
 *
 */

// アプリケーション名
#define APPLICATION_NAME "Toilet flush v1.0"

// デバッグの時定義する
//#define DEBUG

// NVS領域をクリアする時定義する
//#define ERASE_NVS

// LovyanGFX 使用時定義する
//#define USE_LOVYANGFX 

// 外付け赤外線LEDを使用する時定義する（M5Stack用赤外線送受信ユニット(GROVE互換端子)）
#define USE_EXTERNAL_IR_LED

// 内蔵赤外線LEDを使用する時定義する
//#define USE_INTERNAL_IR_LED


// 流すコマンド
#define FLUSH_COMMAND 0x5C30CF		// INAX ながす（大）コマンド
//#define FLUSH_COMMAND 0x5C32CD	// INAX ながす（小）コマンド

#include <stdlib.h>
#include <Arduino.h>
#include <M5StickCPlus.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <Preferences.h>

#ifdef ERASE_NVS
#include <nvs_flash.h>
#endif

// LovyanGFX の設定
#ifdef USE_LOVYANGFX
#define LGFX_AUTODETECT
#define LGFX_M5STICK_C
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
#endif

#include "Menu.h"


// 赤外線LED接続端子定数
const uint16_t IR_LED_EXTERNAL = 32; // M5Stack用赤外線送受信ユニット(GROVE互換端子)
const boolean  IR_LED_EXTERNAL_INVERTED = false; // M5Stack用赤外線送受信ユニット(GROVE互換端子)は、1出力で点灯
const uint16_t IR_LED_INTERNAL = 9;  // 内蔵赤外線 LED
const boolean  IR_LED_INTERNAL_INVERTED = true; // 内蔵赤外線 LED は、0出力で点灯

// PIR HAT 接続端子定数
const uint16_t PIR = 36;  // 人感センサー

#ifdef USE_LOVYANGFX
// 色定義
const int32_t CL_BLACK       = TFT_BLACK;
const int32_t CL_NAVY        = TFT_NAVY;
const int32_t CL_DARKGREEN   = TFT_DARKGREEN;
const int32_t CL_DARKCYAN    = TFT_DARKCYAN;
const int32_t CL_MAROON      = TFT_MAROON;
const int32_t CL_PURPLE      = TFT_PURPLE;
const int32_t CL_OLIVE       = TFT_OLIVE;
const int32_t CL_LIGHTGREY   = TFT_LIGHTGREY;
const int32_t CL_DARKGREY    = TFT_DARKGREY;
const int32_t CL_BLUE        = TFT_BLUE;
const int32_t CL_GREEN       = TFT_GREEN;
const int32_t CL_CYAN        = TFT_CYAN;
const int32_t CL_RED         = TFT_RED;
const int32_t CL_MAGENTA     = TFT_MAGENTA;
const int32_t CL_YELLOW      = TFT_YELLOW;
const int32_t CL_WHITE       = TFT_WHITE;
const int32_t CL_ORANGE      = TFT_ORANGE;
const int32_t CL_GREENYELLOW = TFT_GREENYELLOW;
const int32_t CL_PINK        = TFT_PINK;
const int32_t CL_BROWN       = TFT_BROWN;
const int32_t CL_GOLD        = TFT_GOLD;
const int32_t CL_SILVER      = TFT_SILVER;
const int32_t CL_SKYBLUE     = TFT_SKYBLUE;
const int32_t CL_VIOLET      = TFT_VIOLET;
//const int32_t CL_TRANSPARENT = TFT_TRANSPARENT;
#else
// 色定義
const int32_t CL_BLACK       = BLACK;
const int32_t CL_NAVY        = NAVY;
const int32_t CL_DARKGREEN   = DARKGREEN;
const int32_t CL_DARKCYAN    = DARKCYAN;
const int32_t CL_MAROON      = MAROON;
const int32_t CL_PURPLE      = PURPLE;
const int32_t CL_OLIVE       = OLIVE;
const int32_t CL_LIGHTGREY   = LIGHTGREY;
const int32_t CL_DARKGREY    = DARKGREY;
const int32_t CL_BLUE        = BLUE;
const int32_t CL_GREEN       = GREEN;
const int32_t CL_CYAN        = CYAN;
const int32_t CL_RED         = RED;
const int32_t CL_MAGENTA     = MAGENTA;
const int32_t CL_YELLOW      = YELLOW;
const int32_t CL_WHITE       = WHITE;
const int32_t CL_ORANGE      = ORANGE;
const int32_t CL_GREENYELLOW = GREENYELLOW;
const int32_t CL_PINK        = PINK;
const int32_t CL_BROWN       = 0x9A60;
const int32_t CL_GOLD        = 0xFEA0;
const int32_t CL_SILVER      = 0xC618;
const int32_t CL_SKYBLUE     = 0x015C;
const int32_t CL_VIOLET      = 0x0120;
#endif

// 赤外線送信クラス
#ifdef USE_EXTERNAL_IR_LED
IRsend irsendExternal(IR_LED_EXTERNAL, IR_LED_EXTERNAL_INVERTED); // M5Stack用赤外線送受信ユニット(GROVE互換端子)
#endif
#ifdef USE_INTERNAL_IR_LED
IRsend irsendInternal(IR_LED_INTERNAL, IR_LED_INTERNAL_INVERTED); // 内蔵赤外線 LED
#endif

// 距離計(ToFセンサー)
VL53L0X rangefinder;

#ifdef USE_LOVYANGFX
// LovyanGFX
static LGFX lcd;
#else
#undef lcd // M5StickCPlus.h で定義されている
#define lcd M5.Lcd
#endif

// メニュー
MenuSet menuSet;
Menu sitonThresholdMenu("Sit-on threshold");
Menu countdownTimerMenu("Countdown Timer");
Menu characterMenu("Character");

// 設定値・長時間着座タイマー(ms)
int32_t sitonThreshold   = 60000;
// 設定値・カウントダウンタイマー(ms)（離席後時間経過後にトイレフラッシュ）
int32_t countdownTimer = 120000; 
// 設定値・キャラクタインデックス
int32_t characterIndex = 0;

// loop処理の時刻（loop()関数の中で更新）
uint32_t timeValue = millis();

// ステータス
enum class Status { Waiting, SitOn, SitOnLong, Countdown, ManualCountdown };
Status status = Status::Waiting;
// 一部のステータスで使用するステータスが変更された時刻
uint32_t timeChangeStatus = 0;

//　着座判定
boolean sitOnFlg = false;

// ディスプレイを点灯した時刻
uint32_t timeDisplayOn = 0;

// 距離計(ToF HAT)を利用するか
boolean rangefinderUseFlag = false;

// 人感センサー(PIR HAT)を利用するか
boolean pirUseFlag = false;

// 画面利用フラグ
boolean displayOnFlag = false; // デフォルト消灯

// アニメーションカウンター
int animeCounter = 0;
// アニメーションタイマー
uint32_t timeAnime = 0;

// ToFセンサーの検出距離
uint16_t distanceToF = 0;

/**
 * 設定を読み込む
 */ 
void loadSetting() {
/*
  Preferences pref;
  pref.begin("toilet_flush", false);
  characterIndex = pref.getInt("characterIndex", 0);
  sitonThreshold = pref.getInt("sitonThreshold", 60000);
  countdownTimer = pref.getInt("countdownTimer", 90000);
  pref.end();
*/
}

/**
 * 設定を保存する。
 */ 
void saveSetting() {
  Preferences pref;
  pref.begin("toilet_flush", false);
//pref.clear();
  pref.putInt("characterIndex", characterIndex);
  pref.putInt("sitonThreshold", sitonThreshold);
  pref.putInt("countdownTimer", countdownTimer);
  pref.end();
}


/**
 * 画面初期化
 */
void initDisplay() {
  // 解像度：135x240
  //lcd.setRotation(2);
  lcd.fillScreen(CL_BLACK);

  lcd.fillRect(0, 0, 135, 18, CL_NAVY);
  lcd.setTextColor(CL_WHITE, CL_NAVY);
  lcd.setCursor(5, 0, 2);
  lcd.println(APPLICATION_NAME);

  lcd.setTextColor(CL_WHITE, CL_BLACK);
}

/**
 * アニメーションのキャラクタ表示（モノアイ）
 */
void drawAnimeAiMonoEye() {
    // M5stickC Plusのディスプレイはアスペクト比が1.0ではない。
    // そのまま描くと縦に少しつぶれた形となる。このため、縦方向に1.067を掛ける
    // (物理サイズ: 2.50cm x 1.50cm , ドット数: 240px x 135 px)
    int offsetX = 0;
//    if (animeCounter % 10 == 3 || animeCounter % 10 == 5 || animeCounter % 10 == 6)
//      offsetX = 3;
//    if (animeCounter % 10 == 9)
//      offsetX = -7;
    int32_t edgeColor = CL_RED;
    int32_t outColor = CL_ORANGE;
    int32_t inColor = CL_YELLOW;
    if (characterIndex == 1) {
      edgeColor = CL_NAVY;
      outColor = CL_BLUE;
      inColor = CL_CYAN;
    }
    if (animeCounter % 2 == 0) {
      lcd.fillEllipse(66, 160, 33, 33 * 1.067, CL_BLACK);
      lcd.fillEllipse(66, 160, 30, 30 * 1.067, edgeColor);
      lcd.fillEllipse(66, 160, 29, 29 * 1.067, outColor);
      lcd.fillEllipse(66 + offsetX, 160, 15, 15 * 1.067, inColor);
    } else {
      lcd.fillEllipse(66, 160, 33, 33 * 1.067, edgeColor);
      lcd.fillEllipse(66, 160, 32, 32 * 1.067, outColor);
      lcd.fillEllipse(66 + offsetX, 160, 12, 12 * 1.067, inColor);
    }
    animeCounter++;
}


/**
 * アニメーションのキャラクタ表示（アニメ風の両目）
 */
void drawAnimeBothEyes() {
    int offsetX = 0;
    if (animeCounter % 12 == 8)
      offsetX = 6;
    if (animeCounter % 12 == 10)
      offsetX = -6;
    int32_t edgeColor = CL_NAVY;
    int32_t outColor = CL_WHITE;
    int32_t inColor = CL_MAROON;
    if (rangefinderUseFlag) {
      edgeColor = CL_RED;
      inColor = CL_NAVY;
    }
    if (animeCounter % 12 == 1 || animeCounter % 12 == 4 || animeCounter % 12 == 6) {
      lcd.fillEllipse(43, 160, 24, 30, CL_BLACK);
      lcd.fillEllipse(92, 160, 24, 30, CL_BLACK);
      lcd.fillRect(43 - 20 + offsetX, 160 + 1, 20 * 2, 4, CL_DARKGREY);
      lcd.fillRect(92 - 20 + offsetX, 160 + 1, 20 * 2, 4, CL_DARKGREY);
    } else {
      lcd.fillEllipse(43, 160, 24, 30, outColor);
      lcd.fillEllipse(92, 160, 24, 30, outColor);
      lcd.fillCircle(43 + offsetX, 160, 12, inColor);
      lcd.fillCircle(92 + offsetX, 160, 12, inColor);
    }
    animeCounter++;
}

void drawAnime() {
  switch (characterIndex) {
    case 0:
    case 1:
      drawAnimeAiMonoEye();
      break;
    case 2:
      drawAnimeBothEyes();
      break;
  }
}

/**
 * スプラッシュ画面
 */
void displaySplash() {
  initDisplay();

  lcd.setCursor(5, 20, 4);
  lcd.println("Hello");
  lcd.println("          toilet");

  lcd.setCursor(5, 208, 2);
  lcd.println("Copyright (C)");
  lcd.println("  2022 m.matsubara");

  for (int idx = 0; idx < 4; idx++) {
    drawAnime();
    delay(1000);
  }

  animeCounter = 0;
}

/**
 * ディスプレイON
 */
void displayOn() {
  displayOnFlag = true;
  M5.Axp.SetLDO2(displayOnFlag);
  timeDisplayOn = timeValue;

  // CPU速度を240Mhzに変更
  setCpuFrequencyMhz(240);
}

/**
 * ディスプレイOFF
 */
void displayOff() {
  displayOnFlag = false;
  M5.Axp.SetLDO2(displayOnFlag);

  // CPU速度を10Mhzに変更
  setCpuFrequencyMhz(10);
}

/**
 * ディスプレイON/OFFトグル
 */
void displayToggle() {
  displayOnFlag = !displayOnFlag;
  M5.Axp.SetLDO2(displayOnFlag);
}

/**
 * トイレフラッシュ（大）関数(INAX)
 */
void flush() {
  displayOn();
  // CPU の速度が10Mhzのままだと赤外線が送れない
  setCpuFrequencyMhz(240);
  lcd.fillScreen(CL_LIGHTGREY);
  lcd.setTextColor(CL_BLUE, CL_LIGHTGREY);
  lcd.setCursor(20, 127, 4);
  lcd.println("FLUSH !!");
  lcd.setTextColor(CL_WHITE, CL_BLACK);

#ifdef USE_EXTERNAL_IR_LED
  irsendExternal.sendInax(FLUSH_COMMAND);
#endif
  delay(500);
#ifdef USE_INTERNAL_IR_LED
  irsendInternal.sendInax(FLUSH_COMMAND);
#endif

  // 白い泡が下に流れるイメージのアニメーション
  for (int y = -20; y < 240; y++) {
    for (int idx = 0; idx < 10; idx++) {
      int r = rand() % 10 + 1;
      int x = rand() % 135;
      lcd.fillCircle(x, y + 20, r, CL_WHITE);
    }
    for (int idx = 0; idx < 10; idx++) {
      int r = rand() % 10 + 1;
      int x = rand() % 135;
      lcd.fillCircle(x, y, r, CL_BLUE);
    }
    lcd.fillRect(0, y - 11, 135, 2, CL_BLUE);
    delay(2);
  }
  
  initDisplay();

  // CPU速度を戻す
  if (displayOnFlag == false) {
    setCpuFrequencyMhz(10);
  }
}


// ステータス変更
void changeStatus(Status newStatus) {
  status = newStatus;
  timeChangeStatus = timeValue;
  if (status != Status::Waiting)
    displayOn();
}


void setup() {
#ifdef ERASE_NVS
  nvs_flash_erase(); // erase the NVS partition and...
  nvs_flash_init(); // initialize the NVS partition.
  while(true);
#endif

  // M5初期化
  M5.begin();

  // 設定値の読み込み
  loadSetting();

  // 500000us = 500msのスリープタイマー設定
  esp_sleep_enable_timer_wakeup(500000);

#ifdef LGFX_AUTODETECT
  lcd.init();
  lcd.setBrightness(200);
#endif
  
  // LCD明るさ
  M5.Axp.ScreenBreath(10); // 6より下はかなり見づらく、消費電力もあまり落ちないらしい
  lcd.fillScreen(CL_BLACK);

  // 外付け赤外線LEDの初期化
#ifdef USE_EXTERNAL_IR_LED
  irsendExternal.begin();
#endif

  // 内蔵赤外線LEDの初期化
#ifdef USE_INTERNAL_IR_LED
  irsendInternal.begin();
#endif

  // 距離計の初期化
  Wire.begin(0, 26, 100000UL); // I2C of HAT Connection
  rangefinder.setTimeout(500);
  if (rangefinder.init()) {
    rangefinderUseFlag = true;
    rangefinder.startContinuous();
#ifdef DEBUG
    Serial.println("use rangefinder.");
#endif
  } else {
    // 距離計が初期化できない場合、人感センサーを利用する。（接続確認はできない）
    pinMode(36, INPUT_PULLDOWN);
    pirUseFlag = true;
#ifdef DEBUG
    Serial.println("unuse rangefinder.");
    Serial.println("use PIR HAT.");
#endif
  }

  // 6軸センサ初期化
  M5.Imu.Init();

  changeStatus(Status::Waiting);

  // ディスプレイ初期化
  displayOn();
  displaySplash();
  initDisplay();
  displayOff();

  sitonThresholdMenu.addMenuItem("30 s", "30000");
  sitonThresholdMenu.addMenuItem("60 s", "60000");
  sitonThresholdMenu.addMenuItem("90 s", "90000");
  sitonThresholdMenu.addMenuItem("120 s", "120000");
  sitonThresholdMenu.addMenuItem("150 s", "150000");
  sitonThresholdMenu.addMenuItem("180 s", "180000");
  menuSet.addMenu(&sitonThresholdMenu);
  countdownTimerMenu.addMenuItem("30 s", "30000");
  countdownTimerMenu.addMenuItem("60 s", "60000");
  countdownTimerMenu.addMenuItem("90 s", "90000");
  countdownTimerMenu.addMenuItem("120 s", "120000");
  countdownTimerMenu.addMenuItem("150 s", "150000");
  countdownTimerMenu.addMenuItem("180 s", "180000");
  menuSet.addMenu(&countdownTimerMenu);
  characterMenu.addMenuItem("Mono-eye", "0");
  characterMenu.addMenuItem("Mono-eye(Blue)", "1");
  characterMenu.addMenuItem("Both-eyes", "2");
  menuSet.addMenu(&characterMenu);

  // CPUスピードを10MHzに変更
  setCpuFrequencyMhz(10);
}

void loop() {
  if (menuSet.isStarted()) {
    if (menuSet.loop() == false) {
      // メニュー終了
      initDisplay();
      sitonThreshold = atoi(sitonThresholdMenu.getValue()); 
      countdownTimer = atoi(countdownTimerMenu.getValue()); 
      characterIndex = atoi(characterMenu.getValue()); 
      saveSetting();
    } else {
      // メニュー継続
      return;
    }
  }
  // 処理時刻の更新
  timeValue = millis();

  M5.update();

  // 加速度取得
  float accX = 0;
  float accY = 0;
  float accZ = 0;
  M5.Imu.getAccelData(&accX, &accY, &accZ);

  // 着座判定
  //sitOnFlg = digitalRead(PIR_IO); // PIR センサーは人の動きがないと検出できないため着座判定には不向き。
  // accY: (LCDを上に向けた状態で 0.0, USB Type-Cコネクタを下に向けて立てた状態で 1.0)
  if (sitOnFlg)
    sitOnFlg = (accY < 0.52);
  else
    sitOnFlg = (accY < 0.48);

  // ボタン値取得
  boolean btnA = M5.BtnA.wasPressed();
  boolean btnB = M5.BtnB.wasPressed();
  boolean btnPower = (M5.Axp.GetBtnPress() != 0);

  // 電源関係取得
  float voltageBat = M5.Axp.GetBatVoltage();
  float currentBat = M5.Axp.GetBatCurrent();
  float voltageBus = M5.Axp.GetVBusVoltage();
  float currentBus = M5.Axp.GetVBusCurrent();
#ifdef DEBUG
  Serial.print("Bat(V).  ");
  Serial.println(voltageBat);
  Serial.print("Bat(A).  ");
  Serial.println(currentBat);
  Serial.print("V-in(V). ");
  Serial.println(voltageBus);
  Serial.print("V-in(A). ");
  Serial.println(currentBus);
#endif  
  // 距離計をボタンAと同じ扱いにする(本体を立てた状態かつ20cm以下で押下扱い)
  if (rangefinderUseFlag) {
    uint16_t distance = rangefinder.readRangeContinuousMillimeters();
    if ((accY > 0.75) && (distance <= 200 && distanceToF > 220)) {    // sitOnFlg ではなく、(accY > 0.75) で判定するのは、倒し初めで何かに反応するのを避けるため
      btnA = true;
    }
    distanceToF = distance;
  }

  // 人感センサによるディスプレイON判定
  if (pirUseFlag) {
    if (digitalRead(PIR)) {
      displayOn();      
    }
  }

  // ディスプレイ消灯判定
  if (displayOnFlag && status == Status::Waiting) {
    if (timeValue - timeDisplayOn > 30000) {
      displayOff();
    }
  }
  
  // ボタン処理
  if (btnA) {
    // A ボタンが押されたらディスプレイ点灯・トイレフラッシュ・トイレフラッシュキャンセル
    if (displayOnFlag == false) {
      displayOn();
    } else if (status != Status::Countdown && status != Status::ManualCountdown) {
      changeStatus(Status::ManualCountdown); // 手動カウントダウン 
    } else {
      changeStatus(Status::Waiting); // 強制的に待機モードに変更
      displayOff();
    }
  }
  if (btnB) {
    // B ボタンが押されたら、メニュー表示
    char buff[10];
    itoa(sitonThreshold, buff, 10);
    sitonThresholdMenu.setValue(buff);
    itoa(countdownTimer, buff, 10);
    countdownTimerMenu.setValue(buff);
    itoa(characterIndex, buff, 10);
    characterMenu.setValue(buff);
    menuSet.start();
    return;
  }
  if (btnPower) {
    // 電源ボタンを押すと（6秒未満）リセット
    lcd.fillScreen(BLACK);
    esp_restart();
  }

  // ステータス判定処理
  switch (status) {
  case Status::Waiting:   // 待機中 
    if (sitOnFlg) {
      changeStatus(Status::SitOn); // 着座した
    }
    break;
  case Status::SitOn:   // 着座確認（長時間着座待ち）
    if (sitOnFlg == false) {
      changeStatus(Status::Waiting);
    } else if (timeValue - timeChangeStatus >= sitonThreshold) {
      changeStatus(Status::SitOnLong);
    }
    break;
  case Status::SitOnLong:   // 長時間着座(離席待ち)
    if (sitOnFlg == false) {
      changeStatus(Status::Countdown); // 離席した…カウントダウン
    }
    break;
  case Status::Countdown:   // カウントダウン
#ifdef DEBUG
    Serial.print("timeValue: ");
    Serial.print(timeValue);
    Serial.print(", timeChangeStatus: ");
    Serial.println(timeChangeStatus);
#endif
    if (sitOnFlg) {
      changeStatus(Status::SitOnLong); // 着座した(長時間着座(離席待ち)に戻る)
    } else if (timeValue - timeChangeStatus >= countdownTimer) {
      flush();
      changeStatus(Status::Waiting);
    }
    break;
  case Status::ManualCountdown:   // 手動カウントダウン
#ifdef DEBUG
    Serial.print("timeValue: ");
    Serial.print(timeValue);
    Serial.print(", timeChangeStatus: ");
    Serial.println(timeChangeStatus);
#endif
    if (timeValue - timeChangeStatus >= countdownTimer) {
      flush();
      changeStatus(Status::Waiting);
    }
    break;
  }

  if (displayOnFlag) {
    // ステータスの表示
    lcd.setCursor(5, 30, 4);
    switch (status) {
    case Status::Waiting:   // 待機中 
      lcd.print("Welcome.        ");
      break;
    case Status::SitOn:   // 着座確認（長時間着座待ち）
      lcd.print("Sit on          ");
      break;
    case Status::SitOnLong:   // 長時間着座(離席待ち)
      lcd.print("Sit on *        ");
      break;
    case Status::Countdown:   // カウントダウン
      lcd.print("Cnt-dwn         ");
      break;
    case Status::ManualCountdown:   // 手動カウントダウン
      lcd.print("Cnt-dwn         ");
      break;
    }
  
    // カウントダウンタイマー表示
    lcd.setCursor(20, 60, 7);
    if (status == Status::Countdown || status == Status::ManualCountdown) {
      int secTime = (countdownTimer - (timeValue - timeChangeStatus)) / 1000;
      if (secTime < 0)
        secTime = 0;  // タイミングによってはマイナスになってしまうこともある
      lcd.printf("%.03d", secTime);
    } else {
      lcd.print("        ");
    }
  
    // アニメーション
    if (displayOnFlag && (timeValue - timeAnime >= 1000)) {
      drawAnime();
      timeAnime = timeValue;

    // ステータスバー・加速度の表示
/*
      lcd.setTextColor(LIGHTGREY, BLACK);
      lcd.setCursor(5, 208, 2);
      lcd.printf("Posture:\n  %+.2f %+.2f %+.2f   ", accX, accY, accZ);
      lcd.setTextColor(CL_WHITE, CL_BLACK);
*/
/*
      int width = 130 * accY;
      lcd.fillRect(0, 235, width, 5, CL_DARKGREY);
      lcd.fillRect(width, 235, 130 - width, 5, CL_BLACK);
*/
      // ステータスバー・電源関連の情報表示
      lcd.fillRect(0, 224, 135, 20, CL_NAVY);
      int xIcon = 5;
      int yIcon = 226;
      lcd.setTextColor(CL_LIGHTGREY, CL_NAVY);
      lcd.setCursor(42, 224, 2);
      if (voltageBus >= 3.0) {
        // 外部電源（コンセントのアイコン）
        lcd.fillRect(xIcon  + 4, yIcon + 5, 7, 2, CL_DARKGREEN);
        lcd.fillCircle(xIcon + 16, yIcon + 5, 5, CL_DARKGREEN);
        lcd.fillRect(xIcon + 17, yIcon + 0, 5, 11, CL_DARKGREEN);
        lcd.drawLine(xIcon + 22, yIcon + 3, xIcon + 28, yIcon + 3, CL_DARKGREEN);
        lcd.drawLine(xIcon + 22, yIcon + 8, xIcon + 28, yIcon + 8, CL_DARKGREEN);
        
        lcd.printf(" %.2fV %.0fmA", voltageBus, currentBus);
      } else {
        // 内部電源（電池のアイコン）
        lcd.drawRect(xIcon +  4, yIcon +  0, 21, 11, CL_DARKGREEN);
        lcd.fillRect(xIcon + 25, yIcon +  3,  3,  5, CL_DARKGREEN);
        if (voltageBat >= 3.5)
          lcd.fillRect(xIcon +  6, yIcon +  2, 5, 7, CL_DARKGREEN);
        if (voltageBat >= 3.7)
          lcd.fillRect(xIcon +  12, yIcon +  2, 5, 7, CL_DARKGREEN);
        if (voltageBat >= 3.9)
          lcd.fillRect(xIcon +  18, yIcon +  2, 5, 7, CL_DARKGREEN);
        
        lcd.printf("%.2fV %.0fmA ", voltageBat, currentBat * -1.0);
      }
      lcd.setTextColor(CL_WHITE, CL_BLACK);
    }
  }

  if (displayOnFlag) {
    delay(10);
  } else {
    // 500ms 待つ（ボタン操作性ちょっと悪い）
    delay(500);
    //esp_light_sleep_start();  
  }
}
