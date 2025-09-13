#pragma once
#include <Arduino.h>
#include "LSM6DSM.h"
#include <esp32/rom/rtc.h>
#include "driver/rtc_io.h"

#define IMU_CS 22
#define BARO_CS 25
#define IMU_INT 39
#define V33_SWITCHED 26

#define LED 0
#define OUTPUT_ON 1

LSM6DS3 imu(IMU_CS);

#define V_IMP 21
#define V_IN_PIN 36
RTC_DATA_ATTR float voltage_factor = 8.89560641;
RTC_DATA_ATTR float voltage_offset =1416;
float lv_volt=0;

uint64_t pin_wakeup_mask=0;
RTC_DATA_ATTR timeval time_last_deepsleep; // RTC time the last deepsleep was entered
float IMU_ax, IMU_ay, IMU_az, IMU_gx, IMU_gy, IMU_gz, IMU_hx, IMU_hy, IMU_hz, IMU_t, IMU_altitiude, IMU_pressure, IMU_temp;

void switch_power(bool state){
	if(state){digitalWrite(V33_SWITCHED, OUTPUT_ON);}
	else {digitalWrite(V33_SWITCHED, !OUTPUT_ON);}
}

bool adc_sample_voltage(){
  double volt;
  float delta_v;
  static float history_v = 0;
  static float v_filter = 0;
  static int e_count = 0; // voltage reading error counter until error flag is set

  //analogRead(V_IN_PIN);// read before so setup ADC correctly
  if(!adcAttachPin(V_IN_PIN)){ // pin 36
    return false;
  }
  //adc_power_acquire(); // see https://github.com/espressif/arduino-esp32/blob/8ee5f0a11e5423a018e0f89146e05074466274db/tools/sdk/esp32s3/include/driver/include/driver/adc.h
  portDISABLE_INTERRUPTS();
  uint32_t sttime= micros();
  digitalWrite(V_IMP,0);

  // https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
  //double aval = analogRead(V_IN_PIN);
  double aval = analogRead(V_IN_PIN); // (double) adc1_get_raw( (adc1_channel_t) 0);
  uint32_t dt = micros()-sttime;
  portENABLE_INTERRUPTS();

  digitalWrite(V_IMP,1);
  //adc_power_release();


  volt = (aval*voltage_factor + voltage_offset);
  //Serial.printf("aval: %f, dt: %u, volt: %f\n",aval ,dt, volt/1000);
  if(aval == -1) { return false;}

  //volt = aval*8.89560641 + voltage_offset ;
  // FET window is 80µs @30V. longer on lower voltage

  if(dt < 130){ //75 -> 93 --> 113 --> 130 && ((abs(settings.voltage_raw - aval) < 400 )|| (settings.voltage_raw < 300)) ){
    lv_volt=volt/1000;
    e_count = 0; // reset error if sucess

  // calc voltage change
    if(!history_v){history_v = lv_volt;} // set first value
    if(abs(delta_v) < 10){ // clip to 10V
      delta_v = lv_volt - history_v;
      history_v = lv_volt;
      v_filter = (v_filter + delta_v) * 0.99;
      //IMU[d_volt_change] = v_filter;
    } else {delta_v = delta_v*0.99;}

    //Serial.printf("Voltage-Change: %0.2f\n", IMU[d_volt_change]);

  // Serial.print(dt); Serial.print("\t"); Serial.print(aval); Serial.print("\t"); Serial.println(lv_volt);
  // initstates.init_Voltage=true;
    return true;
  } //else { Serial.printf("aval: %f, dt: %u, volt: %f\n",aval ,dt, volt/1000);}
  if(lv_volt < 2.9 && millis() > 5000){ // no voltage reading after decent amount of time
    e_count ++;
    if(e_count == 5) {Serial.printf("Voltage reading not working 5x: %u %0.2f\n", dt,  lv_volt);}
  }
return false; // sample invalid due to timeout
}

bool IMU_getmotion(){
    if(digitalRead(IMU_INT)){ // new data ready
    imu.read_all(&IMU_ax, &IMU_ay, &IMU_az, &IMU_gx, &IMU_gy, &IMU_gz); // 63µs with temp @ 8 Mhz, 58ms without @ 10Mhz

        //for ( int i=0; i< 14; i++){ Serial.print(data[i], HEX); Serial.print(" ");}
        //Serial.println(" ");

    return true;
    }
    //else {Serial.print(millis()); Serial.println(" IMU Data not ready");}
    return false;
}


bool imu_config(){ // needs spi to be reserved

      imu.settings.gyroEnabled = 1;  //Can be 0 or 1
      imu.settings.gyroRange = 500;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
      imu.settings.gyroSampleRate = 1666;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
      imu.settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
      //imu.settings[gy]roFifoEnabled = 0;  //Set to include gyro in FIFO
      //imu.settings[gy]roFifoDecimation = 0;  //set 1 for on /1
    
      imu.settings.accelEnabled = 1;
      imu.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
      imu.settings.accelSampleRate = 6664;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
      imu.settings.accelBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
      //imu.settings.accelFifoEnabled = 0;  //Set to include accelerometer in the FIFO
      //imu.settings.accelFifoDecimation = 0;  //set 1 for on /1
      imu.settings.tempEnabled = 1;
      
        //Non-basic mode settings
      imu.settings.commMode = 1;
    
      if (imu.configure() != 0) {Serial.print("[ERROR] Error Configuring LSM6DS3M\n"); return false;}
      return true;
    }

    void imu_enable_motion_detection(){
        int p =-1;
          imu.disable_dri(); // make V1 suitable solution
          delay(5); // wait some time
          IMU_getmotion(); // read last value to clear interrupt flag if set
    
        //esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER); // Disable wakeup timer ESP_SLEEP_WAKEUP_TIMER

          int res = imu.enable_motion_detection(8);
          rtc_gpio_init(GPIO_NUM_39);
          pin_wakeup_mask |= 0x8000000000; // GPIO 39 (2^39)
          p=0;

        
            if(p!=0)
              {Serial.printf("[ERROR] IMU Power down failed: %i\n", p);}
            else {
              //DEBUG_t("[INFO] MPU Powered down\n");
          }
        }


    bool get_wakeup_source(bool print){
        esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
        //Wake_source ws=WAKESOURCE_NONE;
        uint64_t bitmask;
            switch(wakeup_reason){
            case ESP_SLEEP_WAKEUP_EXT0 : 
                Serial.printf("[INFO] Wakeup caused by CAN. Voltage: %0.2f\n", lv_volt);
                //ws=WAKESOURCE_CAN; 
                break;
            case ESP_SLEEP_WAKEUP_EXT1 : {
                bitmask = esp_sleep_get_ext1_wakeup_status();
                //char tmp[25]; sprintf(tmp, "%lli\n",  bitmask); DEBUG_t("[INFO] Wakeup mask: "); DEBUG(tmp);
                if(bitmask == 0x400000000){
                //  ws=WAKESOURCE_CAN; 
                    if(print){Serial.print("[INFO] Wakeup caused by CAN\n");}
                }
                if(bitmask == 0x8000000000){
                //ws=WAKESOURCE_IMU; 
                //num_wakeup_imu = num_wakeup+TIMERWAKEUP_AFTER_IMU; 
                if(print){Serial.printf("[INFO] Wakeup caused by IMU. Voltage: %0.2f\n", lv_volt);}}
                if(bitmask == 0x8400000000 ){
                //ws=WAKESOURCE_IMU;
                if(print){Serial.printf("[INFO] Wakeup caused by IMU and CAN. Voltage: %0.2f\n", lv_volt);}}
                break;}
            case ESP_SLEEP_WAKEUP_TIMER : {
                //ws = WAKESOURCE_TIMER; 
                if(print){Serial.print("[INFO] Wakeup caused by timer\n");} break;}
            //case ESP_SLEEP_WAKEUP_TOUCHPAD : DEBUG_t("[INFO] Wakeup caused by touchpad\n"); break;
            case ESP_SLEEP_WAKEUP_ULP : if(print){Serial.print("[INFO] Wakeup caused by ULP program\n");} break;
            default : if(print){Serial.printf("[INFO] Wakeup was not caused by deep sleep\n");} break;
            }
        return false;
        }