#ifndef _MENU_H_
#define _MENU_H_

#include <vector>

/**
 * メニュー項目
 */
class MenuItem {
private:
    const char *caption;
    const char *value;
public:
    MenuItem(const char *caption, const char *value) {
        setCaption(caption);
        setValue(value);
    }

    // メニュー項目のキャプション
    void setCaption(const char *caption) { this->caption = caption; }
    const char *getCaption() { return this->caption; }
    // メニュー項目の値
    void setValue(const char *value) { this->value = value; }
    const char *getValue() { return value; }
 };

/**
 * メニュー
 */
class Menu {
private:
    const char *title;
    std::vector<MenuItem *> menuItemList;
    size_t selectedIdx;

public:
    Menu(const char *title) { 
        this->title = title; 
        this->selectedIdx = 0; 
    }

    // メニュータイトル取得
    const char *getTitle() { return title; }
    // 選択中のメニュー項目インデックス
    void setSelectedIdx(size_t selectedIdx) { this->selectedIdx = selectedIdx; }
    size_t getSelectedIdx() { return this->selectedIdx; }
    // メニュー項目数
    size_t getSize() { return menuItemList.size(); }

    void addMenuItem(const char * caption, const char * value) { 
        menuItemList.push_back(new MenuItem(caption, value)); 
    }

    // メニュー項目の取得
    MenuItem *getMenuItem(size_t idx) { return menuItemList[idx]; }
    // 次のメニュー項目へ（サイクリック）
    void nextItem() { selectedIdx++; selectedIdx %= menuItemList.size(); }

    // 現在選択中のメニュー項目の value 値
    void setValue(const char * value);
    const char *getValue();
};

/**
 * メニューのセット（複数のメニュー項目）
 *   start() でメニュー開始(メインの loop() から毎回呼び出す)
 *   ボタンAでメニュー項目を切り替え
 *   ボタンBで次のメニューへ切り替え、またはメニュー終了
 */

class MenuSet {
private:
    std::vector<Menu *>menuList;
    size_t menuIdx;
    bool started;
    size_t prevMenuSelectedIdx;
    bool modified;

public:
    MenuSet() { menuIdx = 0; started = false; prevMenuSelectedIdx = 0; modified = false;}

    // メニューが開始しているか
    bool isStarted() { return this->started; }

    // メニュー実行の結果、値が変更されたものがあるか
    bool isModified() { return this->modified; }

    // メニューの追加
    void addMenu(Menu *menu) { this->menuList.push_back(menu); }

    // loopで呼び出すべき関数。メニューが継続する場合、trueを返す
    bool loop();
    // メニューの描画
    void draw();

    // メニューを開始する。(メインの loop() からこのクラスの loop() を呼び出すようにする。)
    bool start();
};

#endif