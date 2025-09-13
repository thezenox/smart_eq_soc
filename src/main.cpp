#include <Arduino.h>
#include <SPI.h>
#include "ACAN2517.h"
#include "can.h"
#include "sensor.h"
#include "structs.h"

#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>


WiFiMulti wifiMulti;

const char* mqttServer = "";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";

#define DEV_NAME "smart_eq"

WiFiClient espClient;
PubSubClient mqtt(espClient);
uint32_t last_online_pub = 0;

#define topic_online "stat/smart_eq/online"
#define topic_sensor "sensor/smart_eq/"


  #define CAN_NO_INT // Disables usage of interrupts for MCP2517 to avoid isr problems
  #define HWVERSION (char*)"HWV2"

  #define CAN_CS 5
  #define CAN_INT 34

  #define GPS_TX 32
  #define GPS_RX 35

  #define IO27 27
  #define IO33 33
  #define EXT_RX_PIN 3 // RX0
  #define EXT_TX_PIN 1 // TX0



  //LSM6DS3 imu(IMU_CS);
  //BMP280 bmp(BARO_CS); // hardware SPI
  ACAN2517 can (CAN_CS, SPI, 255) ;
  ACAN2517Filters filters;
  uint32_t can_last_receive;
  uint32_t can_last_send_start=0;
  bool can_ok = false;
  uint32_t last_pub_volt =0;
  uint32_t last_can_msg = 0;

  bool en_wakeoncan = true;


bool mqtt_publish_sig(String topic, float value){
    Serial.printf("%s : %0.2f\n", topic.c_str(), value);

    char to[255]; 
    strcpy(to, topic_sensor);
    strcat(to, topic.c_str());

    char val[55];
    sprintf(val,"%0.2f", value);
    mqtt.publish(to, val);
    return true;
}
  

bool CAN2_init() {
  //initialize CAN Module
  int error = 0;

// TCAN334G
// INT0 = CAN STBY (high for standby)
// INT1 = CAN_SHDN (high for shutdown)


  pinMode (CAN_CS, OUTPUT);
  digitalWrite (CAN_CS, HIGH);
  SPI.begin (18, 19, 23); //
  SPI.setFrequency(8000000);
  delay(1);

  if(!can.en_transceiver2517FD()){
    Serial.print(F("[ERROR] Failed setting CAN Tranceiver\n"));
  }

  if(!can.wake2517FD()){
    Serial.print(F("[ERROR] Failed waking MCP2517\n"));
  }

  can.reset2517FD();
  delay(20);

ACAN2517Settings can_settings (ACAN2517Settings::OSC_40MHz, 500 * 1000); // CAN2_SPEED
can_settings.mRequestedMode = ACAN2517Settings::ListenOnly; // Normal20B; // Select loopback mode
int ret=1;

do {

    if(error > 0){
      can.reset2517FD(); 
      delay(20);} // reset before second try
#ifdef CAN_NO_INT
    ret = can.begin (can_settings, NULL, filters);
#else
    ret = can.begin (can_settings, [] { can.isr(); });
#endif
  error++;
  // Serial.println(ret);
} while ( ret && error <= 3);
if(ret == 0){}
else {Serial.print(F("MCP begin error: ")); Serial.println(ret);}
//Serial.print("MCP begin error: : "); Serial.println(ret);
  can.en_transceiver2517FD();
  //if(can_listen_only){enable_can_listen_only();}
  delay(5);

if(error >3){Serial.print(F("[ERROR] CAN setup error.\n"));  return false;}
return true;
}

bool enable_can_listen_only(){
    can.en_transceiver2517FD();
    if(!can.set_listen_only2517FD()){
        Serial.print(F("[ERROR] CAN set listen only mode failed\n"));
        //can_listen_only = false;
      } else {Serial.print(F("[INFO] CAN set to listen only mode\n"));
      //can_listen_only = true;
    }
    
    return true;
}

bool disable_can_listen_only(){
  can.en_transceiver2517FD();
    if(!can.set_normal_mode2517FD()){
      Serial.print(F("[ERROR] CAN set normal mode failed\n"));
      CAN2_init(); // reinit to get known state
    } else {
      Serial.print(F("[INFO] CAN set to normal 2.0 mode\n"));
      //can_listen_only = false;
    }
  
  return true;
}

unsigned char bit_reverse(unsigned char b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}


void can2_rest_filter() {
  // mcp2515.setFilterMask((MCP2515::MASK) 0, 0, 0); // Check all ID bits on filter 0 and 1
  // mcp2515.setFilterMask((MCP2515::MASK) 1, 0, 0); // Check all ID bits on filter 2,3 4 5
  // mcp2515.setFilter((MCP2515::RXF) 0, 0, 0);
  // mcp2515.setFilter((MCP2515::RXF) 1, 0, 0);
  // mcp2515.setFilter((MCP2515::RXF) 2, 0, 0);
  // mcp2515.setFilter((MCP2515::RXF) 3, 0, 0);
  // mcp2515.setFilter((MCP2515::RXF) 4, 0, 0);
  // mcp2515.setFilter((MCP2515::RXF) 5, 0, 0);
  // mcp2515.setNormalMode();
}


void CAN_powerdown() {
bool p = false;
    //Serial.println("Starting SPI");
    digitalWrite (CAN_CS, HIGH);
    pinMode (CAN_CS, OUTPUT);
    SPI.begin (18, 19, 23); //
    SPI.setFrequency(8000000);
    delay(1);
  
  int retry = 0;
  if(en_wakeoncan ){ // 
    // enable wake on can
    p = can.en_wake_on_can2517FD();
    if(p){
      //pin_wakeup_mask |= 0x400000000; // GPIO 34 (2^34) --> active low, can not be changed, not possible with ext1 wakeup
      Serial.print(F("[INFO] Enabled Wake on CAN\n"));
      return; // do not step into powerdown
    } else {
      Serial.print(F("[ERROR] Failed to enable Wake on CAN. Normal Powerdown\n"));
      en_wakeoncan=false; //if fails, diasable it
    }
  }

    can.dis_transceiver2517FD();
    while (!p && retry <= 3) {
      if(retry > 0){delay(5);} // wait before retry
      p = can.sleep2517FD();
      retry++;
    }

  if(!p) {Serial.print(F("[ERROR] MCP2517 Power down failed\n"));}
}


bool can_send_msg(CANMessage canOut, int protocol){ // prtocol is an additional protocol layer in addition to isotp (like BMW) 0=none, 1=bmw
  bool st = false;

    st = can.tryToSend(canOut);
  //if(st){print_can_frame(&canOut, false);}
  return st;
}



bool can_receive(){
  CANMessage canMsg;
  CANMessage* canIn = &canMsg;
  //if(!can.available()){
  //    return false;
  //}
  if (can.available ()) {
    can.receive (*canIn);
    digitalWrite(LED,0);
    //Serial.printf("%03X [%i] %02X %02X %02X %02X %02X %02X %02X %02X\r\n", canIn->id, canIn->len, canIn->data[0], canIn->data[1], canIn->data[2], canIn->data[3], canIn->data[4], canIn->data[5], canIn->data[6],canIn->data[7]);
    digitalWrite(LED,1);

    int sig = parse_canmsg(canIn);
    if(sig){
      mqtt_publish_sig(can_signals[sig].name, can_signals[sig].value);
    }

    last_can_msg = millis();
  }
  return true;
}



void send_frame(){
        CANMessage m;
      m.id = 0x1200FD7F;
      m.ext = true;
      m.len = 8;
      m.data[0] = 0x05;
      m.data[1] = 0x70;
      m.data[2] = 0x00;
      m.data[3] = 0x00;
      m.data[4] = 0x07;
      m.data[5] = 0x01;
      m.data[6] = 0x84;
      m.data[7] = 0x43;
      can.tryToSend(m);
}


void deepsleep(bool motion, bool timer, bool en_wakeoncan){
  CAN_powerdown();
  uint32_t sleepstartstime = millis();

  // setting this to floating makes sd init fail after deepsleep.
  // rtc_gpio_isolate(GPIO_NUM_13); // high
  // rtc_gpio_isolate(GPIO_NUM_15); // high
  // rtc_gpio_isolate(GPIO_NUM_35); // gps rx, float
  // rtc_gpio_isolate(GPIO_NUM_4); // low
  // rtc_gpio_isolate(GPIO_NUM_2); // low
  // rtc_gpio_isolate(GPIO_NUM_36); // float
  // rtc_gpio_isolate(GPIO_NUM_19); // float
  // rtc_gpio_isolate(GPIO_NUM_23); // float
  // rtc_gpio_isolate(GPIO_NUM_18); // float

  imu_enable_motion_detection();
  //set_sleep_time();

  Serial.print("[INFO] Now Sleeping\n\n\n");
  pinMode(LED,INPUT); // disable LED

  WiFi.disconnect(true); // esp_wifi_disconnect();
  if(motion){ 
    esp_sleep_enable_ext1_wakeup(pin_wakeup_mask, ESP_EXT1_WAKEUP_ANY_HIGH); // set pin wakeup
  } if(en_wakeoncan){
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, LOW);
    //  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON); // RTC IO sensors and ULP co-processor.
    //  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_AUTO); //RTC slow memory.
    //  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_AUTO); // RTC fast memory.
    //  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_AUTO); // XTAL oscillator.
 
// ESP_PD_OPTION_OFF // Power down the power domain in sleep mode.
// ESP_PD_OPTION_ON // Keep power domain enabled during sleep mode.
// ESP_PD_OPTION_AUTO // Keep power domain enabled in sleep mode, if it is needed by one of the wakeup options. Otherwise power it down.

     // rtc_gpio_init(GPIO_NUM_39);
     // rtc_gpio_set_direction(GPIO_NUM_39, RTC_GPIO_MODE_INPUT_ONLY);
     // if( esp_sleep_enable_ext0_wakeup(GPIO_NUM_39, 1) != ESP_OK ){Serial.println(FF("GPIO 39 Wakeup init failed"));} // active high

     // rtc_gpio_init(GPIO_NUM_34);
     // rtc_gpio_set_direction(GPIO_NUM_34, RTC_GPIO_MODE_INPUT_ONLY);
    if( esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0) != ESP_OK ){Serial.println("GPIO 34 Wakeup init failed");} // active low
  }
  
  gettimeofday(&time_last_deepsleep, NULL); // set last deepsleep start
  switch_power(false);
  delay(130); // wait to imu to set interrupt pin
  esp_deep_sleep_start();  // By default, esp_deep_sleep_start() will power down all RTC power domains which are not needed by the enabled wakeup sources. 
                           // To override this behaviour, esp_sleep_pd_config() function is provided.
                           // https://esp32.com/viewtopic.php?t=1133
}



void setup(){
  pinMode(V_IMP,OUTPUT);
  digitalWrite(V_IMP,1); // Charge capacitor
  delay(1000);
  pinMode(LED,OUTPUT);
  digitalWrite(LED,1);
  //pinMode(V_IN_PIN,INPUT); 
  pinMode(V33_SWITCHED,OUTPUT); 
  switch_power(false); // turn off GPS (and SD/MCP2551)
  //digitalWrite(GPS_TX,0);
  pinMode(GPS_TX,INPUT);
  //digitalWrite(GPS_TX,1);

  pinMode(V_IMP,OUTPUT);
  digitalWrite(V_IMP,1); // Charge capacitor
  //pinMode(IO33,INPUT_PULLUP); // Used as Hadware ID pin while no SIM inteface is present
  //pinMode(IO27,INPUT_PULLUP); // Used as Hadware ID pin while no SIM inteface is present
  pinMode(IMU_INT,INPUT);
  pinMode(CAN_INT,INPUT);

  Serial.begin(115200,SERIAL_8N1,3, 1); // RX & TX

  adc_sample_voltage();
  get_wakeup_source(true);

  imu.beginCore();
  imu.enable_normal_mode();
  imu_config();

  ArduinoOTA.setHostname(DEV_NAME);
  ArduinoOTA.setPassword("admin");

  // add multiple lines for each wifi 
  wifiMulti.addAP("ssid", "password");


  mqtt.setServer(mqttServer, mqttPort);

  // set can filter
  for (uint32_t i=0 ; i<num_signals ; i++) {
    filters.appendFrameFilter (kStandard, can_signals[i].can_id, NULL);
  }

  if(CAN2_init()){
    can_ok = true;
  }
  Serial.println("Starting");
}

void mqtt_callback(char* topic, byte *payload, unsigned int length){
  char payload_c [length+1];
  memcpy(payload_c,payload,length);
  Serial.println(topic);
  Serial.println(payload_c);
}

bool running = false;


void loop(){

can.poll ();
can_receive();

//if(millis() - can_last_send_start > 2000){
//  send_frame_jog();
//  running = true;
//  can_last_send_start = millis();
//}

//if(running && millis() - can_last_send_start > 800){
//  //send_frame_stop();
//  running = false;
//  //can_last_send_stop = millis();
//}

if(millis() - last_pub_volt > 5000){
if(adc_sample_voltage()){
  if(mqtt_publish_sig("lv_volt", lv_volt)){
    last_pub_volt = millis();
  }
}
}

if(millis()-last_can_msg > 5000){
  deepsleep(true, false, true);
}



if (wifiMulti.run() == WL_CONNECTED) {

  ArduinoOTA.handle(); 
  mqtt.loop();

  if(!mqtt.connected()){
    
    if(mqtt.connect(DEV_NAME,mqttUser,mqttPassword,topic_online,1,1,"offline")){
      mqtt.setCallback(mqtt_callback);
      ArduinoOTA.begin();
      //mqtt.subscribe(topic_cmd);
      mqtt.publish(topic_online, "online");
      last_online_pub=millis();
      Serial.println("MQTT Connected");
    }
  }

  if(millis()- last_online_pub > 3*60000){
    last_online_pub = millis();
    mqtt.publish(topic_online, "online");
  }
}
}
