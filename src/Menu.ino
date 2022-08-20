#include <string>
#include <M5StickCPlus.h>
#include "lcd.h"
#include "Menu.h"


void Menu::setValue(const char * value) {
    for (size_t idx = 0; idx < this->getSize(); idx++) {
        MenuItem *pMenuItem = this->getMenuItem(idx);
        if (strcmp(value, pMenuItem->getValue()) == 0) {
            this->selectedIdx = idx;
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
    } else if (M5.Axp.GetBtnPress() != 0) {
        // 電源ボタンを押すと（6秒未満）リセット
        lcd.fillScreen(BLACK);
        esp_restart();
    }
        
    delay(10);
    return true;   // メニュー処理は継続
}


 // メニューの描画
 void MenuSet::draw() {
    lcd.fillScreen(CL_BLACK);

    Menu *pMenu = this->menuList[this->menuIdx];
    lcd.fillRect(0, 0, 135, 16, CL_NAVY);
    lcd.setTextColor(WHITE, CL_NAVY);
    lcd.setCursor(5, 0, 2);
    lcd.printf("Menu (%u/%u)\n", this->menuIdx + 1, this->menuList.size());
    lcd.setTextColor(CL_WHITE, CL_BLACK);
    lcd.setCursor(10, 18, 2);
    lcd.println(pMenu->getTitle());

    int y = 45;
    for (size_t idx = 0; idx < pMenu->getSize(); idx++) {
        MenuItem *pMenuItem = pMenu->getMenuItem(idx);
        lcd.setCursor(20, y, 2);
        if (idx == pMenu->getSelectedIdx()) {
          lcd.fillRect(15, y, 110, 18, CL_WHITE);
          lcd.setTextColor(CL_BLACK, CL_WHITE);
        } else {
          lcd.setTextColor(CL_WHITE, CL_BLACK);
        }
        lcd.println(pMenuItem->getCaption());
        y += 18;
    }
    lcd.drawRoundRect(10, 40, 120, pMenu->getSize() * 18 + 10, 3, CL_WHITE);
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
