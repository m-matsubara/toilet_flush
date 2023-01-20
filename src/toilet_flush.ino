/**
 * Toilet flush  Copyright © 2022 m.matsubara
 * 
 * ## 概要
 *   配管上の問題でトイレ詰まりをよく起こすトイレにおいて、使用後に念入りに流すため、離席後一定時間たったら（離席時に流すが、タンクに水がたまるのを待つ）、再度タイマーでトイレを流す（大）。
 *   
 * ## 対象機材（トイレ）
 * * リモコンで流す動作可能なシャワートイレ (INAX プレアス DT-CL114A・CH184A用リモコンで確認)
 *   (リモコンのコマンドが異なる場合、送信コマンドを赤外線センサで解析する必要があります。)
 *
 * ## 必要機材（このプログラムを動作させるのに使用する機材）
 * * M5StickC Plus
 * * M5Stack用赤外線送受信ユニット [U002] (本体の赤外線LEDが使える場合は不要・送信コマンドを赤外線センサで解析する必要がある場合は必要)
 * * M5StickC ToF Hat（必須ではない。使用した場合は、M5ボタンの代わりに距離センサーで操作可能)
 * * M5StickC PIR Hat（必須ではない。使用した場合は、接近するとディスプレイをONにできる)
 * 
 */

// アプリケーション名
#define APPLICATION_NAME "Toilet flush v1.1.0"

// デバッグの時定義する
//#define DEBUG

// LovyanGFX 使用時定義する
//#define USE_LOVYANGFX 

// 外付け赤外線LEDを使用する時定義する（M5Stack用赤外線送受信ユニット(GROVE互換端子)）
#define USE_EXTERNAL_IR_LED

// 内蔵赤外線LEDを使用する時定義する
//#define USE_INTERNAL_IR_LED

// 流すコマンド
// TOTO (機種不明)
//#define FLUSH_IR_COMMAND_TYPE decode_type_t::TOTO
//#define FLUSH_IR_COMMAND_CODE 0xD0D00		            // ながす（大）コマンド
//#define FLUSH_IR_COMMAND_BITS 24

// INAX (プレアス DT-CL114A・CH184A)
#define FLUSH_IR_COMMAND_TYPE decode_type_t::INAX
#define FLUSH_IR_COMMAND_CODE 0x5C30CF		            // ながす（大）コマンド
//#define FLUSH_IR_COMMAND_CODE 0x5C32CD	            // ながす（小）コマンド
#define FLUSH_IR_COMMAND_BITS 24

// Panasonic (CH 1101 リモコン)
//#define FLUSH_IR_COMMAND_TYPE decode_type_t::PANASONIC
//#define FLUSH_IR_COMMAND_CODE 0x344A95A38AF1          // ながす（大）コマンド
//#define FLUSH_IR_COMMAND_BITS 48

#include <stdlib.h>
#include <Arduino.h>
#include <M5StickCPlus.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <Preferences.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <nvs_flash.h>

#include "lcd.h"
#include "Menu.h"

// 設定値・長時間着座タイマー(ms)のデフォルト値
const int32_t SITON_THRESHOLD_DEFAULT   = 60000;
// 設定値・カウントダウンタイマー(ms)（離席後時間経過後にトイレフラッシュ）のデフォルト値
const int32_t COUNTDOWN_TIMER_DEFAULT = 120000; 

// キャラクタ名の一覧
const String CHARACTER_NAME_NONE = "None";
const String CHARACTER_NAME_MONO_EYE_ORANGE = "Mono-eye(Orange)";
const String CHARACTER_NAME_MONO_EYE_BLUE = "Mono-eye(Blue)";
const String CHARACTER_NAME_BOTH_EYES = "Both-eyes";

// ボタン長押しの境界時間
const uint32_t BUTTON_LONG_PRESS_THRESHOLD = 3000;

// 5分の長さのミリ秒
const uint32_t FIVE_MIN_MILLIS = 300000;
// 10分の長さのミリ秒
const uint32_t TEN_MIN_MILLIS = 600000;


// 赤外線LED接続端子
const uint16_t IR_LED_EXTERNAL = 32; // M5Stack用赤外線送受信ユニット(GROVE互換端子)
const boolean  IR_LED_EXTERNAL_INVERTED = false; // M5Stack用赤外線送受信ユニット(GROVE互換端子)は、1出力で点灯
const uint16_t IR_LED_INTERNAL = 9;  // 内蔵赤外線 LED
const boolean  IR_LED_INTERNAL_INVERTED = true; // 内蔵赤外線 LED は、0出力で点灯
// 赤外線センサー接続端子
const uint16_t IR_SENSOR = 33;       // M5Stack用赤外線送受信ユニット(GROVE互換端子)

// PIR HAT 接続端子定数
const uint16_t PIR = 36;  // 人感センサー

// 赤外線送信クラス
#ifdef USE_EXTERNAL_IR_LED
IRsend irsendExternal(IR_LED_EXTERNAL, IR_LED_EXTERNAL_INVERTED); // M5Stack用赤外線送受信ユニット(GROVE互換端子)
#endif
#ifdef USE_INTERNAL_IR_LED
IRsend irsendInternal(IR_LED_INTERNAL, IR_LED_INTERNAL_INVERTED); // 内蔵赤外線 LED
#endif

// 赤外線受信クラス
IRrecv irrecv(IR_SENSOR, 1024, 50, true);	// 引数は、IRrecvDumpV2 を参考にした

// 赤外線コマンド
decode_type_t irCommandType = FLUSH_IR_COMMAND_TYPE;
uint64_t irCommandCode = FLUSH_IR_COMMAND_CODE;
uint16_t irCommandBits = FLUSH_IR_COMMAND_BITS;
uint16_t *irCommandBuff = NULL;
uint16_t irCommandBuffLen = 0;

// 距離計(ToFセンサー)
VL53L0X rangefinder;

// メニュー
MenuSet menuSet;
Menu sitonThresholdMenu("Sit-on threshold");
Menu countdownTimerMenu("Countdown Timer");
Menu countdown2TimerMenu("Countdown2 Timer");
Menu characterMenu("Character");
Menu lcdBrightnessMenu("LCD Brightness");

// 設定値・長時間着座タイマー(ms)
int32_t sitonThreshold   = SITON_THRESHOLD_DEFAULT;
// 設定値・カウントダウンタイマー(ms)（離席後時間経過後にトイレフラッシュ）
int32_t countdownTimer = COUNTDOWN_TIMER_DEFAULT; 
// 設定値・カウントダウンタイマー2(ms)（離席後時間経過後にトイレフラッシュ）
int32_t countdown2Timer = 0; 
// 設定値・キャラクタインデックス
String characterName = CHARACTER_NAME_MONO_EYE_ORANGE;
// 設定値・LCD明るさ
int32_t lcdBrightness = 8;

// loop処理の時刻（loop()関数の中で更新）
uint32_t timeValue = millis();

// ステータス
enum class Status {
  Waiting           // 待ち状態
  , SitOn           // 着座
  , SitOnLong       // 着座（長時間）
  , Countdown       // カウントダウン
  , Countdown2      // カウントダウン(2回目)
  , ManualCountdown // マニュアルカウントダウン
  , CleaningCountdown5  // 掃除モードカウントダウン(5分)
  , CleaningCountdown10 // 掃除モードカウントダウン(10分)
};
Status status = Status::Waiting;
// ステータスが変更された時刻
uint32_t timeChangeStatus = 0;

// 着座判定
boolean sitOnFlg = false;

// ディスプレイを点灯した時刻
uint32_t timeDisplayOn = 0;

// ボタンAを押下した時刻
uint32_t timeButtonAPressed = 0;

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

// 設定
Preferences pref;

// 赤外線コマンド学習モードの時 true
boolean isIRReceiveMode = false;

// ループごとにインクリメントする
uint32_t loopCounter = 0;

/**
 * 設定を読み込む
 */ 
void loadSetting() {
  // 2022/08/20 現在、PlatformIO では platformio.ini に以下の記述がないと Preferences クラスが正しく動作しない
  // platform = https://github.com/tasmota/platform-espressif32/releases/download/v2.0.2idf/platform-espressif32-2.0.2.zip

  pref.begin("toilet_flush", false);

  // メニューにより設定される項目
  sitonThreshold  = pref.getInt("sitonThreshold", SITON_THRESHOLD_DEFAULT);
  countdownTimer  = pref.getInt("countdownTimer", COUNTDOWN_TIMER_DEFAULT);
  countdown2Timer = pref.getInt("countdown2Timer", 0);
  characterName   = pref.getString("characterName", CHARACTER_NAME_MONO_EYE_ORANGE);
  lcdBrightness   = pref.getInt("lcdBrightness", 10);
  Serial.printf("Settings\n");
  Serial.printf("  sitonThreshold  = %d\n", sitonThreshold);
  Serial.printf("  countdownTimer  = %d\n", countdownTimer);
  Serial.printf("  countdown2Timer = %d\n", countdown2Timer);
  Serial.printf("  characterName   = %s\n", characterName.c_str());
  Serial.printf("  lcdBrightness   = %d\n", lcdBrightness);

  // 以下は赤外線受信モードで設定される項目
  irCommandType    = (decode_type_t)pref.getInt("irCommandType", FLUSH_IR_COMMAND_TYPE);
  irCommandCode    = pref.getInt("irCommandCode", FLUSH_IR_COMMAND_CODE);
  irCommandBits    = pref.getInt("irCommandBits", FLUSH_IR_COMMAND_BITS);
  irCommandBuffLen = pref.getInt("irCommandLen", 0);
  if (irCommandBuff != NULL) {
    delete[] irCommandBuff;
    irCommandBuff = NULL;
  }
  if (irCommandBuffLen != 0) {
    irCommandBuff = new uint16_t[irCommandBuffLen];
    pref.getBytes("irCommandBuff", (void *)irCommandBuff, irCommandBuffLen * sizeof(uint16_t));
  }
  Serial.printf("  irCommandType = %d (%s)\n", irCommandType, typeToString(irCommandType).c_str());
  Serial.printf("  irCommandCode = 0x%x\n", irCommandCode);
  Serial.printf("  irCommandBits = %d\n", irCommandBits);
  Serial.printf("  irCommandBuffLen = %d\n", irCommandBuffLen);
  if (irCommandBuffLen != 0) {
    Serial.printf("  irCommandBuff[%u] = {", irCommandBuffLen);
    for (int i = 0; i < irCommandBuffLen; i++) {
      if (i == 0)
        Serial.printf("%u", irCommandBuff[i]);
      else
        Serial.printf(", %u", irCommandBuff[i]);
    }
    Serial.println("}");
  }
  pref.end();
}

/**
 * 設定を保存する。
 */ 
void saveSetting() {
  pref.begin("toilet_flush", false);
  pref.putInt("sitonThreshold", sitonThreshold);
  pref.putInt("countdownTimer", countdownTimer);
  pref.putInt("countdown2Timer", countdown2Timer);
  pref.putString("characterName", characterName);
  pref.putInt("lcdBrightness",  lcdBrightness);
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
    if (characterName == CHARACTER_NAME_MONO_EYE_BLUE) {
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
  if ((characterName == CHARACTER_NAME_MONO_EYE_ORANGE) || (characterName == CHARACTER_NAME_MONO_EYE_BLUE)) {
    drawAnimeAiMonoEye();
  } else if (characterName == CHARACTER_NAME_BOTH_EYES) {
    drawAnimeBothEyes();
  } else {
    // CHARACTER_NAME_NONE
  }
}

/**
 * スプラッシュ画面（起動音つき）
 */
void displaySplash() {
  initDisplay();

  lcd.setCursor(5, 20, 4);
  lcd.println("Hello");
  lcd.println("          toilet");

  lcd.setCursor(5, 208, 2);
  lcd.println("Copyright (C)");
  lcd.println("  2022 m.matsubara");

  // アニメーションと起動音
  // 起動音は、最初の `M5.Beep.tone(2000)` の呼び出しでなぜか音が出ないので、10ms後にもう一度音を出している。
  for (int idx = 0; idx < 400; idx++) {
    if (idx % 100 == 0)
      drawAnime();
    if (idx == 0)
      M5.Beep.tone(2000); // 起動時１回目の音はなぜか出ない
    else if (idx == 1)
      M5.Beep.tone(2000); // ピ (120ms)
    else if (idx == 13)
      M5.Beep.tone(1000); // ポッ (120ms)
    else if (idx == 25)
      M5.Beep.mute();     // 消音
    delay(10);
  }

  animeCounter = 0;
}

/**
 * ディスプレイON
 */
void displayOn() {
  boolean prevDisplayOnFlag = displayOnFlag;
  displayOnFlag = true;
  M5.Axp.SetLDO2(displayOnFlag);
  timeDisplayOn = timeValue;

  // LCD明るさ
  M5.Axp.ScreenBreath(lcdBrightness); // 6より下はかなり見づらく、消費電力もあまり落ちないらしい

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
 * トイレフラッシュ（大）関数
 */
void flush() {
  setCpuFrequencyMhz(240);  // CPUクロックが低いままだと送信が安定しない

  displayOn();
  lcd.fillScreen(CL_LIGHTGREY);
  lcd.setTextColor(CL_BLUE, CL_LIGHTGREY);
  lcd.setCursor(20, 127, 4);
  lcd.println("FLUSH !!");
  lcd.setTextColor(CL_WHITE, CL_BLACK);

#ifdef USE_EXTERNAL_IR_LED
  if (irCommandType != decode_type_t::UNKNOWN) {
    // TODO ir_Panasonic.h の kPanasonicFreq を 367000 → 38000 に変更しないと、Panasonicのトイレは反応しないかも
    irsendExternal.send(irCommandType, irCommandCode, irCommandBits);
  } else {
    irsendExternal.sendRaw((const uint16_t *)irCommandBuff, irCommandBuffLen, 38);
    Serial.printf("command[%u] = {", irCommandBuffLen);
    for (int i = 0; i < irCommandBuffLen; i++) {
      if (i == 0)
        Serial.printf("%u", irCommandBuff[i]);
      else
        Serial.printf(", %u", irCommandBuff[i]);
    }
    Serial.println("}");
  }
#endif
  delay(500);
#ifdef USE_INTERNAL_IR_LED
  if (irCommandType != decode_type_t::UNKNOWN)
    irsendInternal.send(irCommandType, irCommandCode, irCommandBits);
  else
    irsendInternal.sendRaw((const uint16_t *)irCommandBuff, irCommandBuffLen, 38);
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

/*
 * ステータス変更
 */
void changeStatus(Status newStatus) {
  status = newStatus;
  timeChangeStatus = timeValue;
  if (status != Status::Waiting)
    displayOn();
}

/*
 * 赤外線受信モードでボタン描画
 */
void irRecvButtonDraw() {
  lcd.fillRoundRect(90, 140, 40, 20, 5, CL_ORANGE);
  lcd.drawLine(120, 150, 135, 125, CL_ORANGE);
  lcd.setTextColor(CL_BLACK, CL_ORANGE);
  lcd.setCursor(95, 142, 2);
  lcd.print("Save");

  lcd.fillRoundRect(10, 210, 115, 20, 5, CL_ORANGE);
  lcd.drawLine(68, 230, 68, 240, CL_ORANGE);
  lcd.setTextColor(CL_BLACK, CL_ORANGE);
  lcd.setCursor(40, 210, 2);
  lcd.print("Send test");

  lcd.setTextColor(CL_WHITE, CL_BLACK);
}

/*
 * 赤外線受信モードの初期化
 */
void irRecvSetup() {
  // 赤外線受信のための定数設定(IRrecvDumpV2より取得)
  irrecv.setUnknownThreshold(12); // この値より短いON/OFFの値を無視する閾値
  irrecv.setTolerance(25);        // 許容範囲
  // 受信開始
  irrecv.enableIRIn();

  lcd.fillRect(0, 20, 135, 40, CL_BLACK);
  lcd.setTextColor(CL_CYAN, CL_BLACK);
  lcd.setCursor(5, 20, 2);
  lcd.print("Send from \n  remote control.");

  irRecvButtonDraw();
}

/**
  * 赤外線コマンド学習モードのループ処理
  */
void irRecvLoop() {
  // 赤外線受信結果
  decode_results results;
  M5.update();
  if (irrecv.decode(&results)) {
    // 受信したコマンドを解析
    String typeName = typeToString(results.decode_type, results.repeat);
    String commandCodeStr = resultToHexidecimal(&results);
    uint64_t commandCode = strtol(commandCodeStr.c_str() , NULL, 16);
    if (results.repeat == false && results.overflow == false) {
      if (results.decode_type == decode_type_t::UNKNOWN) {
        // 受信したコマンドを表示(UNKNOWN)
        lcd.fillRect(0, 60, 135, 80, CL_BLACK);
        irRecvButtonDraw();

        irCommandType = results.decode_type;
        irCommandCode = commandCode;
        irCommandBits = results.bits;
        if (irCommandBuff != NULL)
          delete[] irCommandBuff;
        irCommandBuff = resultToRawArray(&results);
        irCommandBuffLen = getCorrectedRawLength(&results);

        // 受信したコマンドを表示
        lcd.setCursor(5, 60, 2);
        lcd.printf("type: unknown");
        lcd.setCursor(5, 100, 2);
        lcd.printf("cmd len: %d word", irCommandBuffLen);

        Serial.printf("command[%u] = {", irCommandBuffLen);
        for (int i = 0; i < irCommandBuffLen; i++) {
          if (i == 0)
            Serial.printf("%u", irCommandBuff[i]);
          else
            Serial.printf(", %u", irCommandBuff[i]);
        }
        Serial.println("}");
      } else if (commandCode != 0) {
        lcd.fillRect(0, 60, 135, 80, CL_BLACK);
        irRecvButtonDraw();

        irCommandType = results.decode_type;
        irCommandCode = commandCode;
        irCommandBits = results.bits;

        // UNKNOWNでない場合は、コマンドバッファは削除（保存もしない）
        if (irCommandBuff != NULL)
          delete[] irCommandBuff;
        irCommandBuff = NULL;
        irCommandBuffLen = 0;

        // 受信したコマンドを表示(UNKNOWN でなければ)
        lcd.setCursor(5, 60, 2);
        lcd.printf("type: %s", typeName.c_str());
        lcd.setCursor(5, 100, 2);
        lcd.printf("cmd: %s", commandCodeStr.c_str());
      }
    }
    irrecv.resume();
  }

  if (M5.BtnA.wasPressed()) {
    irrecv.disableIRIn(); // 自身の赤外線コマンドを受信してしまったりするのでいったん無効化
    lcd.fillCircle(115, 220, 5, CL_RED);
#ifdef USE_EXTERNAL_IR_LED
  if (irCommandType != decode_type_t::UNKNOWN)
    irsendExternal.send(irCommandType, irCommandCode, irCommandBits);
  else
    irsendExternal.sendRaw(irCommandBuff, irCommandBuffLen, 38);
#endif
    delay(500);
#ifdef USE_INTERNAL_IR_LED
  if (irCommandType != decode_type_t::UNKNOWN)
    irsendInternal.send(irCommandType, irCommandCode, irCommandBits);
  else
    irsendInternal.sendRaw(irCommandBuff, irCommandBuffLen, 38);
#endif
    lcd.fillCircle(115, 220, 5, CL_ORANGE);
    irrecv.enableIRIn();
  }
  if (M5.BtnB.wasPressed()) {
    // コマンド保存
    pref.begin("toilet_flush", false);
    pref.putInt("irCommandType", irCommandType);
    pref.putInt("irCommandCode", irCommandCode);
    pref.putInt("irCommandBits", irCommandBits);
    if (irCommandBuff != NULL) {
      pref.putBytes("irCommandBuff", (void *)irCommandBuff, irCommandBuffLen * sizeof(uint16_t));
      pref.putInt("irCommandLen", irCommandBuffLen);
    } else {
      pref.remove("irCommandBuff");
      pref.remove("irCommandLen");
    }
    pref.end();
    // コマンドの表示を消去
    lcd.fillRect(0, 60, 135, 80, CL_BLACK);
    irRecvButtonDraw();
  }
  if (M5.Axp.GetBtnPress() != 0) {
    lcd.fillScreen(BLACK);
    esp_restart();
  }
  delay(10);
}


void setup() {
  // M5初期化
  M5.begin();
  Serial.begin(115200, SERIAL_8N1);  
  Serial.println(APPLICATION_NAME);
  Serial.println("  Copyright (C) 2002 m.matsubara");
  if (M5.BtnA.isPressed()) {
    // 赤外線コマンド学習モード
    isIRReceiveMode = true;
  }
  if (M5.BtnB.isPressed()) {
    // 設定クリア
    nvs_flash_erase(); // erase the NVS partition and...
    nvs_flash_init(); // initialize the NVS partition.
  }

  // 設定値の読み込み
  loadSetting();

  // 500000us = 500msのスリープタイマー設定
  esp_sleep_enable_timer_wakeup(500000);

#ifdef LGFX_AUTODETECT
  lcd.init();
  lcd.setBrightness(200);
#endif
  
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

  // メニュー初期化
  sitonThresholdMenu.addMenuItem("0 s", "0");
  sitonThresholdMenu.addMenuItem("30 s", "30000");
  sitonThresholdMenu.addMenuItem("60 s", "60000");
  sitonThresholdMenu.addMenuItem("90 s", "90000");
  sitonThresholdMenu.addMenuItem("120 s", "120000");
  menuSet.addMenu(&sitonThresholdMenu);
  countdownTimerMenu.addMenuItem("0 s", "0");
  countdownTimerMenu.addMenuItem("30 s", "30000");
  countdownTimerMenu.addMenuItem("60 s", "60000");
  countdownTimerMenu.addMenuItem("90 s", "90000");
  countdownTimerMenu.addMenuItem("120 s", "120000");
  countdownTimerMenu.addMenuItem("150 s", "150000");
  countdownTimerMenu.addMenuItem("180 s", "180000");
  menuSet.addMenu(&countdownTimerMenu);
  countdown2TimerMenu.addMenuItem("None", "0");
  countdown2TimerMenu.addMenuItem("30 s", "30000");
  countdown2TimerMenu.addMenuItem("60 s", "60000");
  countdown2TimerMenu.addMenuItem("90 s", "90000");
  countdown2TimerMenu.addMenuItem("120 s", "120000");
  countdown2TimerMenu.addMenuItem("150 s", "150000");
  countdown2TimerMenu.addMenuItem("180 s", "180000");
  menuSet.addMenu(&countdown2TimerMenu);
  characterMenu.addMenuItem("None",           CHARACTER_NAME_NONE.c_str());
  characterMenu.addMenuItem("Mono-eye",       CHARACTER_NAME_MONO_EYE_ORANGE.c_str());
  characterMenu.addMenuItem("Mono-eye(Blue)", CHARACTER_NAME_MONO_EYE_BLUE.c_str());
  characterMenu.addMenuItem("Both-eyes",      CHARACTER_NAME_BOTH_EYES.c_str());
  menuSet.addMenu(&characterMenu);
  lcdBrightnessMenu.addMenuItem("Dark", "8");
  //lcdBrightnessMenu.addMenuItem("Slightly dark", "9");
  lcdBrightnessMenu.addMenuItem("Normal", "10");
  //lcdBrightnessMenu.addMenuItem("Slightly bright", "11");
  lcdBrightnessMenu.addMenuItem("Bright", "12");
  menuSet.addMenu(&lcdBrightnessMenu);


  changeStatus(Status::Waiting);
  if (isIRReceiveMode == false) {
    // ディスプレイ初期化
    displayOn();
    displaySplash();
    initDisplay();
    displayOff();
  } else {
    // ディスプレイ初期化
    displayOn();
    initDisplay();
    // 赤外線受信モード
    irRecvSetup();
  }
}

/**
  * 通常モードのループ
  */
void normalLoop() {
  // メニューの処理
  if (menuSet.isStarted()) {
    if (menuSet.loop() == false) {
      // メニュー終了
      initDisplay();
      if (menuSet.isModified()) {
        // なにか変更されていたら、設定値を保存する。（メニュー画面中にリセットすると、設定値は保存されない。）
        sitonThreshold = atoi(sitonThresholdMenu.getValue()); 
        countdownTimer = atoi(countdownTimerMenu.getValue()); 
        countdown2Timer = atoi(countdown2TimerMenu.getValue()); 
        characterName = characterMenu.getValue(); 
        lcdBrightness  = atoi(lcdBrightnessMenu.getValue()); 
        saveSetting();
      }

      // ディスプレイの明るさを設定するために、displayOn() を呼び出す。
      displayOn();
    } else {
      // メニュー継続
      return;
    }
  }

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
    sitOnFlg = (accY < 0.53);
  else
    sitOnFlg = (accY < 0.47);

  // ボタン値取得
  boolean btnA = M5.BtnA.wasPressed();
  boolean btnB = M5.BtnB.wasPressed();
  boolean btnPower = (M5.Axp.GetBtnPress() != 0);
  // 距離計をボタンAと同じ扱いにする(本体を立てた状態かつ20cm以下で押下扱い)
  if (rangefinderUseFlag) {
    uint16_t distance = rangefinder.readRangeContinuousMillimeters();
    if ((accY > 0.75) && (distance <= 200 && distanceToF > 220)) {    // sitOnFlg ではなく、(accY > 0.75) で判定するのは、倒し初めで何かに反応するのを避けるため
      btnA = true;
    }
    distanceToF = distance;
  }
  if (btnA)
    timeButtonAPressed = timeValue;

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
    // A ボタンが押されたらディスプレイ点灯→マニュアルカウントダウン→
    if (displayOnFlag == false) {
      displayOn();
    } else if (status == Status::ManualCountdown) {
      changeStatus(Status::CleaningCountdown5); // 掃除モードカウントダウン(5分)
    } else if (status == Status::CleaningCountdown5) {
      changeStatus(Status::CleaningCountdown10); // 掃除モードカウントダウン(10分)
    } else if (status == Status::CleaningCountdown10) {
      changeStatus(Status::Waiting); // 強制的に待機モードに変更
      displayOff();
    } else {
      changeStatus(Status::ManualCountdown); // 手動カウントダウン 
    }
  }
  if (btnB) {
    // B ボタンが押されたら、メニュー表示
    char buff[10];
    itoa(sitonThreshold, buff, 10);
    sitonThresholdMenu.setValue(buff);
    itoa(countdownTimer, buff, 10);
    countdownTimerMenu.setValue(buff);
    itoa(countdown2Timer, buff, 10);
    countdown2TimerMenu.setValue(buff);
    characterMenu.setValue(characterName.c_str());
    itoa(lcdBrightness, buff, 10);
    lcdBrightnessMenu.setValue(buff);
    displayOn();
    menuSet.start();
    return;
  }
  if (btnPower) {
    // 電源ボタンを押すと（6秒未満）リセット
    lcd.fillScreen(BLACK);
    esp_restart();
  }

  // ボタンAの長押し
  if (M5.BtnA.isPressed() && (timeValue - timeButtonAPressed >= BUTTON_LONG_PRESS_THRESHOLD)) {
    flush();
    changeStatus(Status::Waiting);
    timeButtonAPressed = timeValue;
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
    if (sitOnFlg) {
      changeStatus(Status::SitOnLong); // 着座した(長時間着座(離席待ち)に戻る)
    } else if (timeValue - timeChangeStatus >= countdownTimer) {
      flush();
      if (countdown2Timer != 0)
        changeStatus(Status::Countdown2);
      else
        changeStatus(Status::Waiting);
    }
    break;
  case Status::Countdown2:   // カウントダウン(2回目)
    if (sitOnFlg) {
      changeStatus(Status::SitOnLong); // 着座した(長時間着座(離席待ち)に戻る)
    } else if (timeValue - timeChangeStatus >= countdown2Timer) {
      flush();
      changeStatus(Status::Waiting);
    }
    break;
  case Status::ManualCountdown:   // 手動カウントダウン
    if (timeValue - timeChangeStatus >= countdownTimer) {
      flush();
      changeStatus(Status::Waiting);
    }
    break;
  case Status::CleaningCountdown5:   // 掃除モード手動カウントダウン(5分)
    if (timeValue - timeChangeStatus >= FIVE_MIN_MILLIS) {
      flush();
      changeStatus(Status::Waiting);
    }
    break;
  case Status::CleaningCountdown10:   // 掃除モード手動カウントダウン(10分)
    if (timeValue - timeChangeStatus >= TEN_MIN_MILLIS) {
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
      lcd.print("Sit on           ");
      break;
    case Status::SitOnLong:   // 長時間着座(離席待ち)
      lcd.print("Sit on *         ");
      break;
    case Status::Countdown:   // カウントダウン
      lcd.print("Cnt-dwn         ");
      break;
    case Status::Countdown2:   // カウントダウン(2回目)
      lcd.print("Cnt-dwn(2)      ");
      break;
    case Status::ManualCountdown:   // 手動カウントダウン
      lcd.print("Cnt-dwn(M)      ");
      break;
    case Status::CleaningCountdown5:   // 掃除モード手動カウントダウン(5分)
      lcd.print("Cleaning(C)     ");
      break;
    case Status::CleaningCountdown10:   // 掃除モード手動カウントダウン(10分)
      lcd.print("Cleaning        ");
      break;
    }
  
    // カウントダウンタイマー表示
    lcd.setCursor(20, 60, 7);
    if (status == Status::Countdown || status == Status::ManualCountdown) {
      int secTime = (countdownTimer - (timeValue - timeChangeStatus) + 999) / 1000;
      lcd.printf("%.03d", secTime);
    } else if (status == Status::Countdown2) {
      int secTime = (countdown2Timer - (timeValue - timeChangeStatus) + 999) / 1000;
      lcd.printf("%.03d", secTime);
    } else if (status == Status::CleaningCountdown5) {
      int secTime = (FIVE_MIN_MILLIS - (timeValue - timeChangeStatus) + 999) / 1000;
      lcd.printf("%.03d", secTime);
    } else if (status == Status::CleaningCountdown10) {
      int secTime = (TEN_MIN_MILLIS - (timeValue - timeChangeStatus) + 999) / 1000;
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
    // 200ms 待つ
    delay(200);
    //esp_light_sleep_start();  
  }
}


void loop() {
  // 処理時刻の更新
  timeValue = millis();

  if (isIRReceiveMode)
    irRecvLoop();
  else
    normalLoop();
  loopCounter++;
}