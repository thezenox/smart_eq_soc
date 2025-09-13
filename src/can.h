#include <Arduino.h>
#include "structs.h"

#define PUBLISH_INTERVAL 10000


float bit_to_float(uint8_t* data, CAN_Signal sig ){
  uint8_t bytes = (sig.length + (sig.startbit%8))/8 + 1*((((sig.length + (sig.startbit%8))%8)>0));
  uint8_t startbyte = sig.startbit/8;
  uint64_t len_mask = 0xFFFFFFFFFFFFFFFF >> (64-sig.length);
  uint64_t pre =0; 
  if(sig.byteorder == LSB){
    for (int i = 0 ; i < bytes; i++){
      //Serial.print("LSB ");Serial.print(i+startbyte);Serial.print(":");Serial.print(data[i+startbyte],HEX); Serial.print("<<");Serial.print(i*8); Serial.print("  ");
      pre += (uint64_t)(data[i+startbyte]) << (i*8) ;
    }
    //Serial.println();
  } else { // MSB
    for (int i = bytes-1 ; i > -1; i--){
    //Serial.print("MSB ");Serial.print(i+startbyte);Serial.print(":"); Serial.print(data[i+startbyte],HEX); Serial.print("<<");Serial.print(((bytes-1)-i)*8); Serial.print("  ");
    pre += (uint64_t)(data[i+startbyte]) << ( ((bytes-1)-i)*8) ;
    }
    //Serial.println();
  }
  pre = (pre >> (sig.startbit%8)) & len_mask;

  //Serial.println(pre,HEX);

if(!sig.isfloat){
  if(sig.sign) {
    if(pre & (1 << (sig.length-1))){ // negative
      //Serial.print("val: ");Serial.println(((float)(((double)pre - (double)(len_mask))*mult)/div)+add);
      return ((float)(((double)pre - (double)(len_mask))*sig.mult)/sig.div)+sig.add;
    } else {
      //Serial.print("val: "); Serial.println(((float)((double)(pre)*mult)/div)+add);
      return ((float)((double)(pre)*sig.mult)/sig.div)+sig.add;
    }
  }
  if(!sig.sign) {return (float)((double)( ((double)(pre)) *sig.mult)/sig.div)+sig.add;}
} else { // float
  float out = *((float *)&pre);
  return (float)((out * sig.mult)/sig.div)+sig.add;
}
return 0; // error
}

bool is_neg(int in){
  if(in < 0) {return true;}
  return false;
}



int parse_canmsg(CANMessage* inmsg){
    for (int i = 0; i< num_signals; i++){
        if(can_signals[i].can_id == inmsg->id){
            if(millis()-can_signals[i].last_pub > PUBLISH_INTERVAL){
                can_signals[i].value = bit_to_float(inmsg->data, can_signals[i]);
                can_signals[i].last_pub = millis();
                return i;
            }
        }
    }
    return 0;
     
}