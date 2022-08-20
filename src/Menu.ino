#include <string>
#include <M5StickCPlus.h>
#include "Menu.h"


void Menu::setValue(const char * value) {
    for (size_t idx = 0; idx < this->getSize(); idx++) {
        MenuItem *pMenuItem = this->getMenuItem(idx);
        if (strcmp(value, pMenuItem->getValue()) == 0) {
            this->selectedIdx = idx;
Serial.print("select menu index: [");
Serial.print(this->selectedIdx);
Serial.print("] : ");
Serial.println(value);

            break;
        }

    }
}

const char *Menu::getValue() {
    MenuItem *pMenuItem = this->getMenuItem(this->selectedIdx);
    if (pMenuItem == NULL)
        return NULL;
    return pMenuItem->getValue();
}

// loopで呼び出すべき関数。メニューが継続する場合、trueを返す
bool MenuSet::loop() {
    if (started == false) {
        // メニューは終了している
        return false;
    }

    M5.update();

    // 現在処理中のメニュー
    Menu *pMenu = this->menuList[this->menuIdx];

    if (M5.BtnA.wasPressed()) {
        // ボタンAで次のメニュー項目選択
        pMenu->nextItem();
        draw();
    } else if (M5.BtnB.wasPressed()) {
        // ボタンBで次のメニューへ、最後のメニューの場合は、メニュー終了
        this->menuIdx++;
        if (this->menuIdx >= this->menuList.size()) {
            this->menuIdx = 0;
            this->started = false;
            return false;   // メニュー終了
        }
        draw();
    }
        
    delay(10);
    return true;   // メニュー処理は継続
}


 // メニューの描画
 void MenuSet::draw() {
    M5.Lcd.fillScreen(BLACK);

    Menu *pMenu = this->menuList[this->menuIdx];
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(3, 0, 2);
    M5.Lcd.printf("Menu (%u/%u)\n", this->menuIdx + 1, this->menuList.size());
    M5.Lcd.setCursor(10, 18, 2);
    M5.Lcd.println(pMenu->getTitle());

    int y = 45;
    for (size_t idx = 0; idx < pMenu->getSize(); idx++) {
        MenuItem *pMenuItem = pMenu->getMenuItem(idx);
        M5.Lcd.setCursor(15, y, 2);
        if (idx == pMenu->getSelectedIdx()) {
          M5.Lcd.fillRect(15, y, 110, 18, WHITE);
          M5.Lcd.setTextColor(BLACK, WHITE);
        } else {
          M5.Lcd.setTextColor(WHITE, BLACK);
        }
        M5.Lcd.println(pMenuItem->getCaption());
        y += 18;
    }
    M5.Lcd.drawRoundRect(10, 40, 120, pMenu->getSize() * 18 + 10, 3, WHITE);

}

bool MenuSet::start() {
    if (this->menuList.size() == 0) {
        return false;
    }
    this->menuIdx = 0;
    this->started = true;
    draw();
    return loop();
}
