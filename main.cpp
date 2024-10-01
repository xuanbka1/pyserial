#include <Arduino.h>
#include "main.h"
#include "WiFi.h"
#include "time.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <lvgl.h>
#include <vector>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <TFT_eSPI.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <XPT2046_Touchscreen.h>
#include <pubsubclient.h>
#include "sys/time.h"
#include <stdlib.h>
#include <WiFi.h>
#include "./ui_files/ui.h"
#include "ArduinoJson.h"
#include "ClientHandler.h"
#include "Config.h"
#include "LcdInterface.h"
#include "stdio.h"

#include <TinyGSM.h>
#include <TinyGsmClient.h>

#define MQTT_LTE  true
#define MQTT_WIFI  false

static char qrContent[QR_CONTENT_SIZE] = {0};
static uint8_t activeAccountOK = 0;
extern lv_obj_t * ui_Label2;

TFT_eSPI tft = TFT_eSPI();
SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);
uint16_t touchScreenMinimumX = 200, touchScreenMaximumX = 3700, touchScreenMinimumY = 240, touchScreenMaximumY = 3800;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[SCREEN_WIDTH * 10];
TaskHandle_t ntConnectTaskHandler;
TaskHandle_t ntTaskMqttClient;


char mqtt_buffer[1024];
bool flag = false;
lv_obj_t * qrCode = NULL;

/* define  mqtt client */
#if (MQTT_WIFI)
WiFiClient espClient;
PubSubClient client(espClient);
#endif

#if (MQTT_LTE)
TinyGsm modem(Serial1);
TinyGsmClient gsmClient(modem);
PubSubClient client(gsmClient);

void ModemHandler_init();
void ModemHandler_connect_gprs();

#endif

extern int32_t rssi;
lv_event_t e;
uint32_t lastReconnectAttempt = 0;

unsigned long start;
static inline String time() {
    return "..." + String((millis() - start) / 1000) + 's';
}

static void log(String info) {
    Serial.println(info);
}

// CONNECT_STATUS_ENUM ConnectStatus = E_POWER_UP;

void display_QR();
 void buttonClick(void);
void ui_event_Button11(lv_event_t * e);
void mqtt_callback(char *topic, byte *payload, unsigned int length);
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
void Receive_handler();
const char* LayQR_Content(const char *data);
void setup();
static void timer_callback(lv_timer_t *timer);
void timeout_BankSuccess(uint16_t seconds);
void TrangThaiThanhToan();
void parseMessage(char* json);
void loop();
void buttonConfig_Handler(void);
void timerLVGL_Handler(void);
void lcd_init();
void fulse_generator(uint32_t amount);
void buzzerRun();
void ClientReportCash( void);
//   s
static int genFulseAction = 0;
static int NumberFulse = 0;
static int startFulse  = 0;
static uint32_t bdsd_moneyValue = 0;

static boolean BLOCK_READING = false;

static boolean buzzerActive =  false;
void fulse_generator_nonBlocking(void);

void fulse_read_nonBlocking(void);
static uint32_t TickTimerReadInput = 0;
static uint32_t TickModemHandler = 0;

static bool resetAccountAction = false;
//setting
//
void check_spinner_timeout();
void switch_to_screen1();
static bool spinner_active = false;
static bool qrcode_active = false;
static uint32_t spinner_start_time = 0; // Variable to store when the spinner starts
static uint32_t qrcode_start_time = 0;
void check_spinner_timeout() {
    if (spinner_active) {
        // Check if 10 seconds have passed
        if (lv_tick_get() - spinner_start_time >= 10000) {
            switch_to_screen1();
        }
    }

    // if(qrcode_active)
    // {
    //   if (lv_tick_get() - qrcode_start_time >= 10000) {
    //         switch_to_screen1();
    //     }
    // }
}
void switch_to_screen1() {
    if (spinner_active) {    
        lv_scr_load(ui_Screen1); 
        spinner_active = false;
    }
}


//
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  uint16_t *color565 = (uint16_t *)malloc(w * h * sizeof(uint16_t));
  for (uint32_t i = 0; i < w * h; i++) {
    color565[i] = lv_color_to16(color_p[i]);
  }
  tft.pushColors(color565, w * h, true);
  free(color565);

  tft.endWrite();

  lv_disp_flush_ready(disp);
}
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touchscreen.touched())
  {
    TS_Point p = touchscreen.getPoint();

    if (p.x < touchScreenMinimumX)
      touchScreenMinimumX = p.x;
    if (p.x > touchScreenMaximumX)
      touchScreenMaximumX = p.x;
    if (p.y < touchScreenMinimumY)
      touchScreenMinimumY = p.y;
    if (p.y > touchScreenMaximumY)
      touchScreenMaximumY = p.y;

    data->point.x = map(p.x, touchScreenMinimumX, touchScreenMaximumX, 1, SCREEN_WIDTH);
    data->point.y = map(p.y, touchScreenMinimumY, touchScreenMaximumY, 1, SCREEN_HEIGHT);
    data->state = LV_INDEV_STATE_PR;
 }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}


// uart receiver code //
void Receive_handler(){
  return;
}

const char* LayQR_Content(const char *data) {
    const char *key = "\"qrCode\":\"";
    const char *startPos = strstr(data, key);
    if (startPos == NULL) {
        Serial.println(" QRcode is NULL -> error respose from vietQR");
        return NULL;
    }
    startPos += strlen(key);
    const char *endPos = strchr(startPos, '"');
    if (endPos == NULL) {
        Serial.println("Không tìm thấy kết thúc của giá trị 'qrCode'.");
        return NULL;
    }
    int qrLength = endPos - startPos;

    if (qrLength < QR_CONTENT_SIZE) {
        strncpy(qrContent, startPos, qrLength);
        qrContent[qrLength] = '\0';
        return qrContent;
    } else {
        Serial.println("Chiều dài của giá trị 'qrCode' quá lớn.");
        return NULL;
    }
}

void loop2 (void* pvParameters) {
  while (1) {
    lv_han
  }
}

/**
 * @brief  setup function 
 * 
 */
void setup()
{
 
  // init fulse //
  //pinMode(beePin, OUTPUT);  // buzzer is output //
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(FULSE_OUT, OUTPUT);
  digitalWrite(FULSE_OUT, HIGH);
  pinMode(FULSE_INPUT, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);

  lcd_init();
  Serial.begin(115200);  // serial main -> for log //
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);  // uart for AT Command //

  xTaskCreatePinnedToCore (
    loop2,     // Function to implement the task
    "loop2",   // Name of the task
    1000,      // Stack size in words
    NULL,      // Task input parameter
    0,         // Priority of the task
    NULL,      // Task handle.
    0          // Core where the task should run
  );

  lv_task_handler();
  // init modem 4G //

  

   #if MQTT_WIFI
   WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the Wi-Fi network");
  #endif

  #if MQTT_LTE
     Serial.print(" init modem......");
     ModemHandler_init();
     ModemHandler_connect_gprs();
  #endif



    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(mqtt_callback);
    client.setBufferSize(1024);

    clientHandler.init();
    config.init();
   // load config //
    String setting = config.readSetting();
    printf(" loading setting %s\n\r", setting.c_str());

    Serial.println(" read back file config in the first bootup ");
    // if error --> set default value //


    Serial.println(config.readBankAccount().c_str());
    Serial.println(config.readbankCode().c_str());
    Serial.println(config.readUserBankName().c_str());


    while (!client.connected()) {
        String client_id = "esp32";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Public  broker connected");

            client.subscribe(clientHandler.getSyncBoxsTopic().c_str());

        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }

  //lcd_init();
  
  // push the first packet for get sync //
  JSONVar payload;
  payload["macAddr"] = clientHandler.getMacAddress();
  payload["checkSum"] = clientHandler.calculateChecksum();
  Serial.println("checksum");
  Serial.println(JSON.stringify(payload).c_str());

  client.publish(clientHandler.sync_topic.c_str(), JSON.stringify(payload).c_str());
  Serial.println(" publish sync data in the fissrt time");
}


typedef struct {
    uint16_t time_left;
} timer_data_t;


static void timer_callback(lv_timer_t *timer)
{
    timer_data_t *data = (timer_data_t *)timer->user_data;
    if (data->time_left > 0) {
        data->time_left--;
    }
    if (data->time_left == 0) {

        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen1_screen_init);
        free(data);
        lv_timer_del(timer);
    }
}


void timeout_BankSuccess(uint16_t seconds)
{
    lv_timer_t *timer = lv_timer_create(timer_callback, 1000, NULL);
    timer_data_t *data = (timer_data_t *)malloc(sizeof(timer_data_t));
    if (data == NULL) {
        return;
    }
    data->time_left = seconds;
    timer->user_data = data;
}

 lv_obj_t * qr = NULL;
 const char* qrdata_rcv;

/*********************************************************************************************************************
 * @brief  mqtt cllback function
 * 
 * @param topic 
 * @param payload 
 * @param length 
 **********************************************************************************************************************/
 void mqtt_callback(char *topic, byte *payload, unsigned int length)
 {
   Serial.print("Message arrived in topic: ");
   Serial.println(topic);
   Serial.print("Message:");
   char message[length + 1];
   strncpy(message, (char *)payload, length);
   message[length] = '\0';

   Serial.println(message);

   String str = String((char *)payload);
   JSONVar jsondata = JSON.parse(str);

   if (strcmp(topic, (topic_qr).c_str()) == 0)
   {
     // _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen2_screen_init);
     qrdata_rcv = LayQR_Content(message);
     Serial.println("Data QR Received:");
     Serial.println(qrdata_rcv);
     if (qrdata_rcv != NULL)
     {
       Serial.println("mqtt_callback -> QR is NULL");
     }
   }

   // topic bien dong so du
   if (strcmp(topic, (topic_bdsd).c_str()) == 0)
   {
     //String str = static_cast<const char*>(jsondata["amount"]);
     Serial.print("-->amount: ");
     //Serial.println(str.c_str());
     

     //float moneyVal  = jsondata["amount"];
     uint32_t amount =  (uint32_t)(static_cast<double>(jsondata["amount"]));
     printf(" amount bdsd = %d \n\r", amount);
     bdsd_moneyValue =  amount;
     genFulseAction = 1;
     qrcode_active = false;

    //   uint32_t amount = atol(str.c_str());
    //  fulse_generator(amount);
    _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen3_screen_init); 
    timeout_BankSuccess(10);
   }
   // topic lay ma QRBoxID //
   // get certigicate
   if (String(topic) == clientHandler.getSyncBoxsTopic())
   {

     clientHandler.setBoxId(static_cast<const char *>(jsondata["boxId"]));       // get BOX ID base 64
     clientHandler.setTerminalBox(static_cast<const char *>(jsondata["boxId"])); // get terminalBox after convert base64 to char

     clientHandler.setQrCertificate(static_cast<const char *>(jsondata["qrCertificate"])); // get terminalBox after convert base64 to char
     Serial.print("-->qrCertificate: ");
     Serial.println(clientHandler.getQrCertificate().c_str());

     String topic = qr_topic_prefix + clientHandler.getBoxId();
     Serial.println(topic.c_str()); // sync topic
     if (client.subscribe(topic.c_str(), 1))
     {
       Serial.print("sub sync Certificate: ");                 // In ra "Arduino"
       Serial.println(topic.c_str());                          // sync topic
       Serial.print("terminal box: ");                         // In ra "Arduino"
       Serial.println(clientHandler.getTerminalBox().c_str()); // box ID: VVB123456

       // store to config //
       config.writeQrCertificate(clientHandler.getQrCertificate().c_str());
       config.writeBoxId(clientHandler.getBoxId().c_str());

       // update all topic publish//
       topic_cash = topic_cash_prefix + clientHandler.getTerminalBox();
       topic_request = topic_request_prefix + clientHandler.getTerminalBox();
       topic_request = topic_request_prefix + clientHandler.getTerminalBox();
       topic_qr = topic_qr_prefix + clientHandler.getTerminalBox();
       topic_status = topic_status_prefix + clientHandler.getTerminalBox();
       topic_bdsd = topic_bdsd_prefix + clientHandler.getTerminalBox();
       // clientHandler.getBoxId().c_str();

       if (client.subscribe(topic_qr.c_str(), 1))
         printf(" sub topic_qr_prefix ok: %s \n\r", topic_qr.c_str());
       if (client.subscribe(topic_status.c_str(), 1))
         printf(" sub topic_status_prefix ok: %s \n\r", topic_status.c_str());
       if (client.subscribe(topic_bdsd.c_str(), 1))
         printf(" sub topic_bdsd_prefix ok: %s  \n\r", topic_bdsd.c_str());

       lcd2_updateBoxId(clientHandler.getTerminalBox().c_str());

       return;
     }
   }

   // sync TerminalBox and BoxID
   if (jsondata.hasOwnProperty("notificationType"))
   {
     String bankAccount = static_cast<const char *>(jsondata["bankAccount"]);
     String bankCode = static_cast<const char *>(jsondata["bankCode"]);
     String userBankName = static_cast<const char *>(jsondata["userBankName"]);

    //  Serial.println(bankAccount.c_str()); 
    //  Serial.println(bankShortName.c_str()); 
    //  Serial.println(userBankName.c_str()); 

     config.writeBankAccount(bankAccount.c_str());
     config.writebankCode(bankCode.c_str());
     config.writeUserBankName(userBankName.c_str());
     
    Serial.println(" read back file config");


     // store to config //
    resetAccountAction = false;
    
    // update Back Account + Bank user name //
    
    String bankAcc = config.readbankCode() + ": " + config.readBankAccount();
    lv_label_set_text(ui_bankCode, bankAcc.c_str());
    lv_label_set_text(ui_bankAccount, config.readUserBankName().c_str());

    Serial.println(config.readBankAccount().c_str());
    Serial.println(config.readbankCode().c_str());
    Serial.println(config.readUserBankName().c_str());

    activeAccountOK = 1;

    // delayMicroseconds(5000000);
  
    // _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen2_screen_init);
    // qr = lv_qrcode_create(ui_Screen2, max(QR_WIDTH, QR_HEIGHT), lv_color_hex(0x000000), lv_color_hex(0xfffff));
    // lv_qrcode_update(qr, (const uint8_t *)clientHandler.getQrCertificate().c_str(), strlen(clientHandler.getQrCertificate().c_str()));
    // lv_obj_set_size(qr, QR_WIDTH, QR_HEIGHT);
    // lv_obj_set_pos(qr, (240 - QR_WIDTH) / 2, (320 - QR_HEIGHT) / 2);
    // lv_obj_clear_flag(qr, LV_OBJ_FLAG_HIDDEN);
  }
    Serial.println("---------------------------------------------------------");
 }

void ClientRequestQR(int money, const char* bankAccount, const char* bankCode, const char* userBankName )
{
  sprintf(mqtt_buffer, "{\"amount\":%d,\"content\":\"PaymentForOrder\",\"bankAccount\":\"%s\",\"bankCode\":\"%s\",\"userBankName\":\
       \"%s\",\"transType\":\"C\",\"orderId\":\"ORD12345XYZ\",\"terminalCode\":\"%s\",\"serviceCode\":\"SVC001\",\"additionalData\":\
       [{\"additionalData1\":\"maygapthu\"}]}", money, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str(), clientHandler.getTerminalBox().c_str());
  
       printf("topic pub: %s \n\r", topic_request.c_str());
       client.publish(topic_request.c_str(), mqtt_buffer);
       buzzerRun();  
}

void ClientReportCash( int cashValue)
{
  sprintf(mqtt_buffer, "{\"amount\":%d,\"content\":\"PaymentForOrder\",\"bankAccount\":\"%s\",\"bankCode\":\"%s\",\"userBankName\":\
       \"%s\",\"transType\":\"C\",\"orderId\":\"ORD12345XYZ\",\"terminalCode\":\"%s\",\"serviceCode\":\"SVC001\",\"additionalData\":\
       [{\"additionalData1\":\"tienmat\"}]}", cashValue, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str(), clientHandler.getTerminalBox().c_str());
  

       printf("topic pub: %s \n\r", topic_cash.c_str());
       client.publish(topic_cash.c_str(), mqtt_buffer);
}

void display_QR()
{
  if (qrdata_rcv != NULL)
  {
    qr = lv_qrcode_create(ui_Screen2, max(QR_WIDTH, QR_HEIGHT), lv_color_hex(0x000000), lv_color_hex(0xffffff));
    lv_qrcode_update(qr, (const uint8_t *)qrdata_rcv, strlen(qrdata_rcv));
    lv_obj_set_size(qr, QR_WIDTH, QR_HEIGHT);
    lv_obj_set_pos(qr, (240 - QR_WIDTH) / 2, (320 - QR_HEIGHT) / 2);
    lv_obj_clear_flag(qr, LV_OBJ_FLAG_HIDDEN);
    // timeout_BankSuccess(30);
    qrcode_active = true;
    qrcode_start_time = lv_tick_get();
    printf(" create timeout for QRcode \n\r");
    screen2_setTimeout(60);
  }
  qrdata_rcv = NULL;
}

static unsigned long  buttonTimer = 0;
void loop()
{
  if (!client.connected())
  {
    log("=== MQTT NOT CONNECTED ===");
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 3000L)
    {
      lastReconnectAttempt = t;
      String client_id = "esp32_LTE_";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
      {
        Serial.println("Public  broker connected");
        client.subscribe(clientHandler.getSyncBoxsTopic().c_str());

        JSONVar payload;
        payload["macAddr"] = clientHandler.getMacAddress();
        payload["checkSum"] = clientHandler.calculateChecksum();
        Serial.println("checksum");
        Serial.println(JSON.stringify(payload).c_str());

        client.publish(clientHandler.sync_topic.c_str(), JSON.stringify(payload).c_str());
        Serial.println("--> re publish for lost connect --> re-syncbox");
      }
      else
      {
        Serial.print("failed with state ");
        Serial.print(client.state());
        delay(2000);
      }
    }
  }
  else
  {
    client.loop();
  }


  Receive_handler();
  lv_task_handler();
  display_QR();
  buttonConfig_Handler();
  timerLVGL_Handler();
  fulse_generator_nonBlocking();
  fulse_read_nonBlocking();

  //check_spinner_timeout();

  static uint32_t signalTick = 0;
  if(millis() - signalTick > 5000)
  {
    signalTick = millis();
    int csq = modem.getSignalQuality();
    rssi = csq*100/31;
    //printf("Signal quality: %d % \n\r", rssi);
  }

  if(genFulseAction == 1 && bdsd_moneyValue > 0)
  {
    printf("main gen fulse control \n\r");
    genFulseAction = 0;
    fulse_generator(bdsd_moneyValue);
    bdsd_moneyValue = 0;
    printf("-----> finish \n\r");
  }

}

/**
 * @brief button reset config handler
 * 
 */
void buttonConfig_Handler(void)
{
  static uint8_t buttonCounter = 0;
  if (millis() - buttonTimer > 100)
  {
    buttonTimer = millis();
    int buttonState = digitalRead(BUTTON_PIN);
    // Kiểm tra trạng thái
    delayMicroseconds(100);
    if (buttonState == LOW)
    {
      buttonCounter++;
      Serial.println("Button Pressed"); // In ra khi nút được nhấn
      if(buttonCounter > 30)
      {   
          buttonCounter = 0;
          Serial.println("Goto reset account device \n\r"); // In ra khi nút được nhấn
          // while (1)
          // {
          //   /* code */
          // }
          _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen2_screen_init);
           qr = lv_qrcode_create(ui_Screen2, max(QR_WIDTH, QR_HEIGHT), lv_color_hex(0x000000), lv_color_hex(0xeeeeee));
           lv_qrcode_update(qr, (const uint8_t *)clientHandler.getQrCertificate().c_str(), strlen(clientHandler.getQrCertificate().c_str()));
           lv_obj_set_size(qr, QR_WIDTH, QR_HEIGHT);
           lv_obj_set_pos(qr, (240 - QR_WIDTH) / 2, (320 - QR_HEIGHT) / 2);
           lv_obj_clear_flag(qr, LV_OBJ_FLAG_HIDDEN);
           resetAccountAction = true;
      }
    }
  }
}

void lcd_init()
{
  tft.init();
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  touchscreen.setRotation(0);
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, SCREEN_WIDTH * 10);
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_WIDTH;
  disp_drv.ver_res = SCREEN_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
  ui_init();
  Serial.println(F("Setup done!"));
  // rssi = WiFi.RSSI();
  // Serial.print("RSSI: ");
  // Serial.println(rssi);
  delay(1000);
}

void create_spinner()
{
  if (!spinner_active)
  {
    ui_Spinner2 = lv_spinner_create(ui_Screen2, 1000, 90);
    lv_obj_set_width(ui_Spinner2, 60);
    lv_obj_set_height(ui_Spinner2, 60);
    lv_obj_set_x(ui_Spinner2, 0);
    lv_obj_set_y(ui_Spinner2, 0);
    lv_obj_set_align(ui_Spinner2, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Spinner2, LV_OBJ_FLAG_CLICKABLE); // Flags

    spinner_active = true;
    spinner_start_time = lv_tick_get();
    screen2_setTimeout(30);
  }
}

/* implement event click to select monay value */
void ui_event_ClickMoney1(lv_event_t * e)
{
   lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_RELEASED) {
      tone(BUZZER_PIN, 0, 150);
      Serial.println(" ui_event_ClickMoney1");
      ClientRequestQR(10000, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str());
      create_spinner();
    }
}

void ui_event_ClickMoney2(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_RELEASED) {
       Serial.println(" ui_event_ClickMoney2");
      ClientRequestQR(20000, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str());
      create_spinner();
      
    }
}
void ui_event_ClickMoney3(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_RELEASED) {
       Serial.println(" ui_event_ClickMoney3");
      ClientRequestQR(50000, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str());
      create_spinner();
      
    }
}
void ui_event_ClickMoney4(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED)
  {
    Serial.println(" ui_event_ClickMoney4");
    ClientRequestQR(100000, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str());
    create_spinner();
    
  }
}
void ui_event_ClickMoney5(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED)
  {
    Serial.println(" ui_event_ClickMoney5");
    ClientRequestQR(200000, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str());
    create_spinner();
    
  }
}
void ui_event_ClickMoney6(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED)
  {
    Serial.println(" ui_event_ClickMoney6");
    ClientRequestQR(500000, config.bankAccount.c_str(), config.bankCode.c_str(), config.userBankName.c_str());
    create_spinner();
  }
}

void ui_event_ClearQrCode(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED)
  {
    if (qr != NULL)
    {
      lv_obj_del(qr);
      qr = NULL;
    }
  }
}

// password is ok //
void submit_password_event_cb(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_RELEASED) {
        const char *input_password = lv_textarea_get_text(password_input);
        if (strcmp(input_password,"1234") == 0) {
          // lv_obj_del(ui_Screen5);
          lv_textarea_set_text(password_input, "");
          _ui_screen_change(&ui_Screen6, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen6_screen_init);

          // add old config  to lcd //
          char buffer[8];
          itoa(config.Ton, buffer, 10);
          lv_textarea_set_text(ui_TextAreaTon, buffer);

          itoa(config.Toff, buffer, 10);
          lv_textarea_set_text(ui_TextAreaToff, buffer);

          itoa(config.fulse_10K, buffer, 10);
          lv_textarea_set_text(ui_TextArea10k, buffer);

          itoa(config.fulse_20K, buffer, 10);
          lv_textarea_set_text(ui_TextArea20k, buffer);

          itoa(config.fulse_50K, buffer, 10);
          lv_textarea_set_text(ui_TextArea50k, buffer);

          itoa(config.fulse_100K, buffer, 10);
          lv_textarea_set_text(ui_TextArea100k, buffer);

          itoa(config.fulse_200K, buffer, 10);
          lv_textarea_set_text(ui_TextArea200k, buffer);

          itoa(config.fulse_500K, buffer, 10);
          lv_textarea_set_text(ui_TextArea500k, buffer);

          Serial.println("Correct password");

        } else {
            lv_label_set_text(password_label, "Wrong Password!");
        }
    }
}
void ui_event_selectCancel(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_RELEASED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen5_screen_init);
    }
}
static bool timer_created = false;
 void change_screen(lv_timer_t * timer)
{
    _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen1_screen_init);
    timer_created = false; 
 lv_timer_del(timer);
}

void ui_event_seclectSave(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    
    if(event_code == LV_EVENT_RELEASED) {
      timer_created = true;
        const char *Toff = lv_textarea_get_text(ui_TextAreaToff);
        const char *Ton = lv_textarea_get_text(ui_TextAreaTon);
        const char *Area_Text10K = lv_textarea_get_text(ui_TextArea10k);
        const char *Area_Text20K = lv_textarea_get_text(ui_TextArea20k);
        const char *Area_Text50K = lv_textarea_get_text(ui_TextArea50k);
        
        const char *Area_Text100K = lv_textarea_get_text(ui_TextArea100k);
        const char *Area_Text200K = lv_textarea_get_text(ui_TextArea200k);
        const char *Area_Text500K = lv_textarea_get_text(ui_TextArea500k);

        printf("Ton: %s\n", Ton);
        printf("Toff: %s\n", Toff);
        printf("10k: %s\n", Area_Text10K);
        printf("20k: %s\n", Area_Text20K);
        printf("50k: %s\n", Area_Text50K);
        printf("100k: %s\n", Area_Text100K);
        printf("200k: %s\n", Area_Text200K);
        printf("500k: %s\n", Area_Text500K);
        lv_obj_set_style_text_color(ui_Label2, lv_color_hex(0xFF0000), 0);
        lv_label_set_text(ui_Label2, "Thanh Cong");
 
        lv_timer_create(change_screen, 3000, NULL);

        // save data to memory //
        // save to memory //
        String dataContent ="";
        StaticJsonDocument<256> settingJson;
        // JSONVar payload;
        settingJson["Ton"] = atoi(Ton);
        settingJson["Toff"] = atoi(Toff);
        settingJson["10k"] = atoi(Area_Text10K);
        settingJson["20k"] = atoi(Area_Text20K);
        settingJson["50k"] = atoi(Area_Text50K);
        settingJson["100k"] = atoi(Area_Text100K);
        settingJson["200k"] = atoi(Area_Text200K);
        settingJson["500k"] = atoi(Area_Text500K);

        serializeJson(settingJson, dataContent);
        printf("json, %s \n", dataContent.c_str());

        config.writeSetting(dataContent.c_str());
    }
}

void ui_event_ResetAccount(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    if(event_code == LV_EVENT_RELEASED) {
       printf(" reset account event \n\r");
       //CERT-VVB-VlZCMTIxNTExODgxM0JGMEM1QTYwVmlldFFSQm94QWNjZXNzS2V5//
      //  if (qrdata_rcv != NULL)
      //  {
         qr = lv_qrcode_create(ui_Screen4, max(QR_WIDTH, QR_HEIGHT), lv_color_hex(0x000000), lv_color_hex(0xffffff));
         lv_qrcode_update(qr, (const uint8_t *)clientHandler.getQrCertificate().c_str(), strlen(clientHandler.getQrCertificate().c_str()));
         lv_obj_set_size(qr, QR_WIDTH, QR_HEIGHT);
         lv_obj_set_pos(qr, (240 - QR_WIDTH) / 2, (320 - QR_HEIGHT) / 2);
         lv_obj_clear_flag(qr, LV_OBJ_FLAG_HIDDEN);
         // timeout_BankSuccess(30);
       //}
      //qrdata_rcv = NULL;
    }
}

void timerLVGL_Handler(void)
{
  static uint32_t cnt = 0;
  static uint32_t timerCounter = 0;
  if (millis() - timerCounter > 1000)
  {
    timerCounter = millis();
    int csq = modem.getSignalQuality();
    //printf("Signal quality: %d", csq);

    if (activeAccountOK == 1)
    {
      cnt++;
      if (cnt == 5)
      {
        activeAccountOK = 0;
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen1_screen_init);
      }
    }
    else
      cnt = 0;
  }
}

void fulse_generator(u32_t amount)
{
  int fulse = 0;
  uint32_t fulseLow = config.Toff *1000;
  uint32_t fulseHigh = config.Ton * 1000;
   switch (amount)
   {
   case 10000:
    fulse = config.fulse_10K;
    break;

  case 20000:
    fulse = config.fulse_20K;
    break;
  case 50000:
    fulse = config.fulse_50K;
    break;
  case 100000:
    fulse = config.fulse_100K;
    break;
  case 200000:
    fulse = config.fulse_200K;
    break;
  case 500000:
    fulse = config.fulse_500K;
    break;
  default:
    fulse = 0;
    break;
   }

   if(fulse)
   {
      printf("number fulse= %d\r\n", fulse);
   }

  BLOCK_READING = true;
  for(uint32_t i = 0; i < fulse ; i++)
  {
    digitalWrite(FULSE_OUT, LOW);
    delayMicroseconds(fulseLow);
    digitalWrite(FULSE_OUT, HIGH);
    delayMicroseconds(fulseHigh);
  }
  BLOCK_READING = false;
}

// /**
//  * @brief gen fulse no blocking
//  * 
//  */
  static uint32_t buzzerTick = 0;
  static uint8_t buzzerCnt =0;
void fulse_generator_nonBlocking(void)
{
  //  if (millis() - buzzerTick > 5)
  // {
  //   buzzerTick = millis();
    
  //   if(buzzerActive)
  //   {
  //      digitalWrite(BUZZER_PIN, LOW);
  //      buzzerCnt++;
  //      if(buzzerCnt > 10)
  //      {
  //        buzzerActive = false;
  //        buzzerCnt = 0;
  //        digitalWrite(BUZZER_PIN, HIGH);
  //      }
       
  //   }
  // } 
}

static uint8_t lastState  = HIGH;
static uint8_t fulseCounter = 0;
static uint16_t fulseTimeout = 0;
void fulse_read_nonBlocking(void)

{
  if(BLOCK_READING)
     return;
  if (millis() - TickTimerReadInput > 1000)
  {
    TickTimerReadInput = millis();
    int buttonState = digitalRead(FULSE_INPUT);
    // Kiểm tra trạng thái
    //delayMicroseconds(100);
    if (buttonState == LOW && lastState == HIGH)
    {
        fulseTimeout = 0;
        fulseCounter++;
        lastState = LOW;
    }
    if (buttonState == HIGH && lastState == LOW)
    {
       lastState = HIGH;
    }
    if(buttonState == HIGH)
    {
       fulseTimeout++; 
       if(fulseTimeout > 200)
       {
          if(fulseCounter != 0)
          {
             printf(" timeout --> fulse = %d \n\r", fulseCounter);
             // publish data to server //
             ClientReportCash(fulseCounter);
          }
          fulseCounter = 0;
       }
    }
  }
}

/** modem fuction*/
void ModemHandler_init()
{
  pinMode(PPP_MODEM_RESET, OUTPUT);    // accept for all Module 4G combine RST and PWR//
  digitalWrite(PPP_MODEM_RESET, LOW);
  delay(250);
  digitalWrite(PPP_MODEM_RESET, HIGH);
  delay(100);
  modem.restart();

  //modem.sendAT
  //modem.init();

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);
}

void ModemHandler_connect_gprs()
{
      log("Waiting for network...." + time());
    if (!modem.waitForNetwork()) {
        log("fail" + time());
        delay(10000);
        return;
    }
    if (modem.isNetworkConnected()) {
        log("Network connected" + time());
    }
    log("GPRS connect..." + time());
    if (!modem.gprsConnect(apn.c_str(), gprsUser.c_str(), gprsPass.c_str())) {
        log("fail");
        delay(10000);
        return;
    }
    if (modem.isGprsConnected()) {
        log("GPRS connected");
    }
}

void buzzerRun()
{
  // digitalWrite(BUZZER_PIN, LOW);
  // delayMicroseconds(1000);
  // digitalWrite(BUZZER_PIN, HIGH);

}

// void DeviceConnectHandler(void)
// {
//   if (millis() - TickModemHandler > 1000)
//   {
//     TickTimerReadInput = millis();
//     if(ConnectStatus == E_POWER_UP)
//     {
//        ModemHandler_init();
//     }
//   }
// }