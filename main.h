#ifndef __MAIN_H__
#define __MAIN_H__

#include "Arduino.h"

const char* ssid = "FFT-VT";
const char* password = "11235813";

#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320
#define FONT_SIZE 2
#define QR_CONTENT_SIZE 512

#define  QR_WIDTH   172
#define  QR_HEIGHT  172

// define PIN //
#define BUTTON_PIN   0  // Định nghĩa chân kết nối nút nhấn
#define FULSE_OUT    27
#define FULSE_INPUT  22

#define BUZZER_PIN   21


String Terminal_ID = "";
String topic_request = "";
String topic_cash = "";
String topic_qr = "";
String topic_status ="";
String topic_bdsd = "";


String topic_cash_prefix = "vietqr/cash/";
String topic_request_prefix = "vietqr/request/";
String topic_qr_prefix = "vietqr/response/";
String topic_status_prefix ="vietqr/response-status/";
String topic_bdsd_prefix = "vietqr/bdsd/";

char topic_handlerbox_pub[128];
char topic_handlerbox_sub[128];

String qr_topic_prefix = "vietqr/boxId/";
String sync_topic_prefix = "/vqr/handle-box";


//const char* msg_get = "{\"amount\":5000,\"content\":\"PaymentForOrder\",\"bankAccount\":\"9690194300493\",\"bankCode\":\"MB\",\"userBankName\":\"TrieuNgocXuan\",\"transType\":\"C\",\"orderId\":\"ORD12345XYZ\",\"terminalCode\":\"VVB121511\",\"serviceCode\":\"SVC001\",\"additionalData\":[{\"additionalData1\":\"xuantest\"}]}";
const char *mqtt_broker = "112.78.1.209";
const char *mqtt_username = "vietqrprodAdmin123";
const char *mqtt_password = "vietqrbns123";
const int   mqtt_port = 1883;


#define RXD1              16
#define TXD1              17
#define PPP_MODEM_RESET   4

const int beePin = 27; // buzzer pin //

String apn     = "";
String gprsUser = "";
String gprsPass= "";

// state machine connect //
enum CONNECT_STATUS_ENUM
{
    E_POWER_UP,
    E_INIT_INTERNET,
    E_CONNECTING, 
    E_CONENNECTED,
};

#endif
