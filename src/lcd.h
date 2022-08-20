#ifndef _LCD_H_
#define _LCD_H_

#include <stdlib.h>
#include <Arduino.h>


// LovyanGFX の設定
#ifdef USE_LOVYANGFX
#define LGFX_AUTODETECT
#define LGFX_M5STICK_C
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
#endif

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

#ifdef USE_LOVYANGFX
// LovyanGFX
static LGFX lcd;
#else
#undef lcd // M5StickCPlus.h で定義されている
#define lcd M5.Lcd
#endif

#endif