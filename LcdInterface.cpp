
#include "LcdInterface.h"
#include "./ui_files/ui.h"


void lcd2_updateBoxId(const char* data)
{
    lv_label_set_text(ui_boxID, data);
}



void lcd2_updateMoneybutton1(const char *data){
    lv_label_set_text(ui_money1,data);
}
void lcd2_updateMoneybutton2(const char *data){
    lv_label_set_text(ui_money2,data);
}
void lcd2_updateMoneybutton3(const char *data){
    lv_label_set_text(ui_money3,data);
}
void lcd2_updateMoneybutton4(const char *data){
    lv_label_set_text(ui_money4,data);
}

void lcd2_updateMoneybutton5(const char *data){
    lv_label_set_text(ui_money5,data);
}
void lcd2_updateMoneybutton6(const char *data){
    lv_label_set_text(ui_money6,data);
}
void lcd2_updateTitleButton(const char *data){
    lv_label_set_text(ui_titleButton,data);
}
void lcd2_RssiValue(int rssi){
    
}

void lcd2_titleQR(const char *data){
    lv_label_set_text(ui_titleQR,data);
}
void lcd1_bankCode(const char *data){
    lv_label_set_text(ui_bankCode,data);
}
void lcd1_bankAccount(const char *data){
    lv_label_set_text(ui_bankAccount,data);
}
void lcd5_inputPulse(int pulse){
    
}