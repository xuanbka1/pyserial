
#ifndef __LCD_INTERFACE_H__
#define __LCD_INTERFACE_H__






void lcd2_updateBoxId(const char* data);
void lcd2_updateMoneybutton1(const char *data);
void lcd2_updateMoneybutton2(const char *data);
void lcd2_updateMoneybutton3(const char *data);
void lcd2_updateMoneybutton4(const char *data);
void lcd2_updateMoneybutton5(const char *data);
void lcd2_updateMoneybutton6(const char *data);
void lcd2_updateTitleButton(const char *data);
void lcd2_RssiValue(int rssi);
void lcd2_titleQR(const char *data);



void lcd1_bankCode(const char *data);
void lcd1_bankAccount(const char *data);








#endif