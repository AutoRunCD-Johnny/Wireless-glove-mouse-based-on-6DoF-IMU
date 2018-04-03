#include <SPI.h>
#include <Mouse.h>
#include <Keyboard.h>
#include "RF24.h"


RF24 radio(8, 9); // CE, CSN
byte address[6] = "012345";
byte text[11] = {1,3,4,5,5,3,2,7,7,9,2};
long t0;
bool key_state[3]={0};
int count=0;

typedef struct{
  /*volatile float accel[3];
  volatile float gyro[3];*/
  signed char xyPos[2];
  signed char scroll;
  byte command;
  //b0 is L-key. b1 is R-key. b2 is M-key. 
  //b3 represent either horizontal scroll or vertical scroll
  //b4 is zoom in
  //b5 is zoom out
}message_tx;
message_tx message;

void receive(){
    digitalWrite(7,HIGH);
    radio.read(&message, sizeof(message));
    //-------mouse------//
    if(message.command&0x8){
    Keyboard.press(KEY_RIGHT_SHIFT);
    if(count==0)
    Mouse.move(message.xyPos[0],message.xyPos[1],message.scroll);
    else
    Mouse.move(0,0,message.scroll);
    Keyboard.release(KEY_RIGHT_SHIFT);
    }
    else{
    if(count==0)
    Mouse.move(message.xyPos[0],message.xyPos[1],message.scroll);
    else
    Mouse.move(0,0,message.scroll);
    }
    
    if(message.command&0x1 && !key_state[0]){
    Mouse.press(MOUSE_LEFT);
    key_state[0]=true;
    count=1;
    }
    else if(!(message.command&0x1) && key_state[0]){
    Mouse.release(MOUSE_LEFT);
    key_state[0]=false;
    }
    if(message.command&0x2 && !key_state[1]){
    Mouse.press(MOUSE_RIGHT);
    key_state[1]=true;
    count=1;
    }
    else if(!(message.command&0x2) && key_state[1]){
    Mouse.release(MOUSE_RIGHT);
    key_state[1]=false;
    }
    if(message.command&0x4 && !key_state[2]){
    Mouse.press(MOUSE_MIDDLE);
    key_state[2]=true;
    count=1;
    }
    else if(!(message.command&0x4) && key_state[2]){
    Mouse.release(MOUSE_MIDDLE);
    key_state[2]=false;
    }
    /*
    if(message.command&0x10){//zoom in
    Keyboard.press(KEY_RIGHT_CTRL);
    Mouse.move(0,0,2);//scroll up
    Keyboard.release(KEY_RIGHT_CTRL);  
    }
    if(message.command&0x20){//zoom out
    Keyboard.press(KEY_RIGHT_CTRL);
    Mouse.move(0,0,-2);//scroll up
    Keyboard.release(KEY_RIGHT_CTRL);  
    }
    */
    if(count>=1 && count<70)//stop moving duration time while initial click
    count++;
    if(count==70)
    count=0;
    digitalWrite(7,LOW);
}


void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  //radio.setPayloadSize(25);//default is 32Bytes
  pinMode(7,OUTPUT);
  digitalWrite(7,LOW);
  attachInterrupt(digitalPinToInterrupt(3),receive,FALLING);
}

void loop() {
  /*if (radio.available()) {
    digitalWrite(7,HIGH);
    radio.read(&message, sizeof(message));
    //-------mouse------//
    if(message.command&0x8){
    Keyboard.press(KEY_RIGHT_SHIFT);
    if(count==0)
    Mouse.move(message.xyPos[0],message.xyPos[1],message.scroll);
    else
    Mouse.move(0,0,message.scroll);
    Keyboard.release(KEY_RIGHT_SHIFT);
    }
    else{
    if(count==0)
    Mouse.move(message.xyPos[0],message.xyPos[1],message.scroll);
    else
    Mouse.move(0,0,message.scroll);
    }
    
    if(message.command&0x1 && !key_state[0]){
    Mouse.press(MOUSE_LEFT);
    key_state[0]=true;
    count=1;
    }
    else if(!(message.command&0x1) && key_state[0]){
    Mouse.release(MOUSE_LEFT);
    key_state[0]=false;
    }
    if(message.command&0x2 && !key_state[1]){
    Mouse.press(MOUSE_RIGHT);
    key_state[1]=true;
    count=1;
    }
    else if(!(message.command&0x2) && key_state[1]){
    Mouse.release(MOUSE_RIGHT);
    key_state[1]=false;
    }
    if(message.command&0x4 && !key_state[2]){
    Mouse.press(MOUSE_MIDDLE);
    key_state[2]=true;
    count=1;
    }
    else if(!(message.command&0x4) && key_state[2]){
    Mouse.release(MOUSE_MIDDLE);
    key_state[2]=false;
    }
    
    if(message.command&0x10){//zoom in
    Keyboard.press(KEY_RIGHT_CTRL);
    Mouse.move(0,0,2);//scroll up
    Keyboard.release(KEY_RIGHT_CTRL);  
    }
    if(message.command&0x20){//zoom out
    Keyboard.press(KEY_RIGHT_CTRL);
    Mouse.move(0,0,-2);//scroll up
    Keyboard.release(KEY_RIGHT_CTRL);  
    }

    if(count>=1 && count<70)//stop moving duration time while initial click
    count++;
    if(count==70)
    count=0;
    
  }
  else{
  //Serial.println("no data");
  digitalWrite(7,LOW);
  }
*/
}

