#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
//#include <MPU6050_6Axis_MotionApps20.h>
#include <math.h>
#include"RF24.h"
//#include"printf.h"


#define afs 0 // 16384/g  2048/g
#define gfs 0 //131/deg/s 16.8/deg/s
#define hpf 2
#define accel_err 20
#define gyro_err 5


MPU6050 mpu;
float Ax,Ay,Az,Ax_buf=0,Ay_buf=0,Az_buf=0,Wx,Wy,Wz,Wx_buf=0,Wy_buf=0,Wz_buf=0,pitch=0,roll=0,pitch_buf=0,roll_buf=0;
float a_buf[3][10]={{0}},g_buf[3][4]={{0}};
bool calib_done=false,flag=false,button_state[3]={0},button[3]={0},zoom_state=true,zoom_buf=true;
long offset[6]={0},button_avg[3]={0},zoom_avg=0,mean[6]={0};
int i,ax,ay,az,gx,gy,gz,dt=4,button_raw_buf[3]={0},zoom_raw,zoom_raw_buf=0;
long t0,t1;
byte cursor_sensitivity=32,scroll_sensitivity=1,scroll_count=0;

//nrf24l01
RF24 radio(8, 9); // CE, CSN
byte address[6] = "012345";

typedef struct{
  /*volatile float accel[3];
  volatile float gyro[3];*/
  volatile signed char xyPos[2];
  volatile signed char scroll;
  volatile byte command;
}message_tx;
message_tx message;


void key_calibration(long button_avg[],long* zoom_avg){
  for(i=0;i<1024;i++){
  button_avg[0]+=analogRead(A0);
  button_avg[1]+=analogRead(A1);
  button_avg[2]+=analogRead(A2);
  zoom_avg+=analogRead(A3);
  }
  button_avg[0]>>=10;button_avg[1]>>=10;button_avg[2]>>=10;
  *zoom_avg>>=10;
}

void calibration(long offset[],bool* calib_done){
  *calib_done=true;
  mean[0]=0;mean[1]=0;mean[2]=0;
  mean[3]=0;mean[4]=0;mean[5]=0;
  for(i=0;i<1124;i++){
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  if(i>100){
  mean[0]+=ax;mean[1]+=ay;mean[2]+=az;mean[3]+=gx;mean[4]+=gy;mean[5]+=gz;
  }
  delay(1);
  }
  mean[0]>>=10;mean[1]>>=10;mean[2]>>=10;mean[3]>>=10;mean[4]>>=10;mean[5]>>=10;
  mean[2]-=16384;
  if((abs(mean[0])>accel_err)||(abs(mean[1])>accel_err)||(abs(mean[2])>accel_err)||(abs(mean[3])>gyro_err)||(abs(mean[4])>gyro_err)||(abs(mean[5])>gyro_err))
  *calib_done=false;
  if(abs(mean[0])>accel_err){
  offset[0]=offset[0]+(mean[0]>>3);
  mpu.setXAccelOffset(-offset[0]);
  }
  if(abs(mean[1])>accel_err){
  offset[1]=offset[1]+(mean[1]>>3);
  mpu.setYAccelOffset(-offset[1]);
  }
  if(abs(mean[2])>accel_err){
  offset[2]=offset[2]+(mean[2]>>3);
  mpu.setZAccelOffset(-offset[2]);  
  }
  if(abs(mean[3])>gyro_err){
  offset[3]=offset[3]+(mean[3]>>2);
  mpu.setXGyroOffset(-offset[3]);  
  }
  if(abs(mean[4])>gyro_err){
  offset[4]=offset[4]+(mean[4]>>2);
  mpu.setYGyroOffset(-offset[4]);  
  }
  if(abs(mean[5])>gyro_err){
  offset[5]=offset[5]+(mean[5]>>2);
  mpu.setZGyroOffset(-offset[5]);  
  }
  for(i=0;i<6;i++)
  Serial.println(mean[i]);
  /*
  if(calib_done==false)
  calibration(offset);*/
}

void raw(float* Ax,float* Ay,float* Az,float* Wx,float* Wy,float* Wz,long offset[]){
  int ax,ay,az,gx,gy,gz;
  long t0;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);//about 428us
  *Ax=ax/16384.0;
  *Ay=ay/16384.0;
  *Az=az/16384.0;
  *Wx=gx*PI/(131.0*180);
  *Wy=gy*PI/(131.0*180);
  *Wz=gz*PI/(131.0*180);
}

void orientation(float Ax,float Ay,float Az,float Wx,float Wy,float Wz,float Wx_buf,float Wy_buf,float Wz_buf,float dt,float* pitch,float* roll){
  float accl_pitch=-atan(Ax/sqrt(Ay*Ay+Az*Az));
  float accl_roll=atan(Ay/sqrt(Ax*Ax+Az*Az));
  float accl_yaw=atan(sqrt(Ax*Ax+Ay*Ay)/Az);
  float gyro_pitch=*pitch+0.001*dt*(Wy+Wy_buf)/2;
  float gyro_roll=*roll+0.001*dt*(Wx+Wx_buf)/2;
  *pitch=accl_pitch*0.03+0.97*gyro_pitch;
  *roll=accl_roll*0.03+0.97*gyro_roll;
}

/*
void velocity(float Ax,float Ay,float Az,float yaw,float pitch,float roll,float dt,float* hori,float* verti){
  Ax=(Ax-Ax_buf)*dt;
  Ay=(Ay-Ay_buf)*dt;
  Az=(Az-Az_buf)*dt;
  Ax=Ax*cos(pitch)*cos(yaw) - Ay*cos(roll)*sin(yaw) + Az*sin(roll)*sin(yaw) + Az*cos(roll)*cos(yaw)*sin(pitch) + Ay*cos(yaw)*sin(pitch)*sin(roll);
  Ay=Ay*cos(roll)*cos(yaw) + Ax*cos(pitch)*sin(yaw) - Az*cos(yaw)*sin(roll) + Az*cos(roll)*sin(pitch)*sin(yaw) + Ay*sin(pitch)*sin(roll)*sin(yaw);
  Az=Az*cos(pitch)*cos(roll) - Ax*sin(pitch) + Ay*cos(pitch)*sin(roll);
  float Vx=dt*Ax,Vy=dt*Ay,Vz=dt*Az;
  *hori+=Vx;
  *verti+=Vz;
}
*/

void high_pass(float x,float buf[],float* y){
  int i;
  for(i=1;i<4;i++)
    buf[i-1]=buf[i];
  buf[3]=x;
  //*y=-0.2441*buf[3]-0.7441*buf[2]+0.7441*buf[1]+0.2441*buf[0];//about 5hz hpf
  *y=-0.2008*buf[3]-0.7008*buf[2]+0.7008*buf[1]+0.2008*buf[0];//about 15hz
}

void low_pass(float x,float buf[],float* y){
  int i;
  float sum;
  for(i=1;i<5;i++)
    buf[i-1]=buf[i];
  buf[4]=x;
  *y=(buf[0]+buf[1]+buf[2]+buf[3]+buf[4])*0.2;
}

void LR_click(int button_raw_buf[],long int button_avg[],bool button_state[],bool button[],message_tx *message){
  int L_raw=analogRead(A0), R_raw=analogRead(A1), M_raw=analogRead(A2);
  int L_d=L_raw-button_raw_buf[0],R_d=R_raw-button_raw_buf[1],M_d=M_raw-button_raw_buf[2];//diff to check edge
  //Serial.println(L_raw);
  //Serial.println(L_d);
  //Serial.println(button[0]);
  //
  if(L_d<-50)
  button_state[0]=true;
  if(L_d>30)
  button_state[0]=false;
  if(button_state[0]){
    if(L_raw-button_avg[0]<-200){//threshold of 
    button[0]=1;
    //Mouse.press(MOUSE_LEFT);
    message->command|=0x1;
    }
  }
  if(button_state[0]==false){
    if(L_raw-button_avg[0]>-30){
    button[0]=0;
    //Mouse.release(MOUSE_LEFT);
    message->command&=0xFE;
    }
  }
  //
  if(R_d<-50)
  button_state[1]=true;
  if(R_d>30)
  button_state[1]=false;
  if(button_state[1]){
    if(R_raw-button_avg[1]<-200){//threshold of 
    button[1]=1;
    //Mouse.press(MOUSE_LEFT);
    message->command|=0x2;
    }
  }
  if(button_state[1]==false){
    if(R_raw-button_avg[1]>-30){
    button[1]=0;
    //Mouse.release(MOUSE_LEFT);
    message->command&=0xFD;
    }
  }
  //
  if(M_d<-50)
  button_state[2]=true;
  if(M_d>30)
  button_state[2]=false;
  if(button_state[2]){
    if(M_raw-button_avg[2]<-200){//threshold of 
    button[2]=1;
    //Mouse.press(MOUSE_LEFT);
    message->command|=0x4;
    }
  }
  if(button_state[2]==false){
    if(M_raw-button_avg[2]>-30){
    button[2]=0;
    //Mouse.release(MOUSE_LEFT);
    message->command&=0xFB;
    }
  }

  button_raw_buf[0]=L_raw;
  button_raw_buf[1]=R_raw;
  button_raw_buf[2]=M_raw;
}

/*
void h_scoll(){
  Keyboard.press(KEY_RIGHT_SHIFT);
  Mouse.move(0,0,2,0);
  Keyboard.release(KEY_RIGHT_SHIFT);
}*/

/*
void zoom(){
  Serial.println("zoom in");
  Keyboard.press(KEY_RIGHT_CTRL);
  Mouse.move(0,0,2,0);//scroll up
  Keyboard.release(KEY_RIGHT_CTRL);
}*/

void setup() {
  pinMode(5,INPUT_PULLUP);//cursor sensitive switch
  pinMode(4,INPUT_PULLUP);//scroll sensitive switch
  pinMode(3,OUTPUT);
  digitalWrite(3,HIGH);//initialize sign LED
  ADCSRA&=0b11111100; // change the adc clk prescale from 128 to 16 (lower adc latency)
  key_calibration(button_avg,&zoom_avg);
  Wire.begin();
  Serial.begin(115200);
  mpu.initialize();
  while(!mpu.testConnection()){Serial.println("no mpu");}
  mpu.setFullScaleAccelRange(afs);
  mpu.setFullScaleGyroRange(gfs);
  //mpu.setDHPFMode(hpf);
  //mpu.setDLPFMode(1);
  while(!calib_done){calibration(offset,&calib_done);}
  Serial.println("calibration done");
  for(i=0;i<6;i++)
  Serial.println(-offset[i]);
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(address);
  radio.stopListening();
  digitalWrite(3,LOW);
 // printf_begin();
 // radio.printDetails();
  
}

void loop() {
   //zoom_state=digitalRead(10);
   //t0=micros();
   //sensitive switch check
   if(digitalRead(5)==HIGH)
   cursor_sensitivity=32;
   else
   cursor_sensitivity=24;
   if(digitalRead(4)==HIGH)
   scroll_sensitivity=1;
   else
   scroll_sensitivity=4;
   //collect data from mpu
   raw(&Ax,&Ay,&Az,&Wx,&Wy,&Wz,offset);
   low_pass(Ax,a_buf[0],&Ax);
   low_pass(Ay,a_buf[1],&Ay);
   low_pass(Az,a_buf[2],&Az);
   high_pass(Wx,g_buf[0],&Wx);
   high_pass(Wy,g_buf[1],&Wy);
   orientation(Ax,Ay,Az,Wx,Wy,Wz,Wx_buf,Wy_buf,Wz_buf,(float)dt,&pitch,&roll);
  
   /*
   Serial.print("x acc=");Serial.print(Ax);Serial.print("g\n");
   Serial.print("y acc=");Serial.print(Ay);Serial.print("g\n");
   Serial.print("z acc=");Serial.print(Az);Serial.print("g\n");
   Serial.print("Wx=");Serial.print(Wx);Serial.print("rad/s\n");
   Serial.print("Wy=");Serial.print(Wy);Serial.print("rad/s\n");
   Serial.print("Wz=");Serial.print(Wz);Serial.print("rad/s\n");*/
    
   
   if(abs(pitch*(180.0/PI))<10 && abs(Az-Az_buf)>0.22 && scroll_count==0){
    message.command&=0xF7;
    scroll_count=1;
    if(Az>1.2){
      message.scroll=-scroll_sensitivity;
      Serial.println("pos v_scroll, page down");
    }    
    else if(Az<0.5){
      message.scroll=scroll_sensitivity;
      Serial.println("neg v_scroll, page up");
    }
   }
   if(abs(pitch*(180.0/PI))<20 && abs(Ax)>1.3 && scroll_count==0){
    scroll_count=1;
    message.command|=0x8;
    if(Ax>0){
      message.scroll=-scroll_sensitivity;
      Serial.println("pos h_scroll, go right");
    }    
    else{
      message.scroll=scroll_sensitivity;
      Serial.println("neg h_scroll, go left");
    }
   }
   
   //Serial.print("roll=");Serial.println(roll*(180.0/PI));
   //Serial.print("pitch=");Serial.println(pitch*(180.0/PI));
   //Serial.println(Az);
   //Serial.println((int)(40*(pitch-pitch_buf)*180/PI));
   
   LR_click(button_raw_buf,button_avg,button_state,button,&message);
   message.xyPos[0]=(int)(cursor_sensitivity*(pitch-pitch_buf)*180/PI)&0xFF;
   message.xyPos[1]=(int)(-cursor_sensitivity*(roll-roll_buf)*180/PI)&0xFF;
   
   pitch_buf=pitch;
   roll_buf=roll;
   Ax_buf=Ax;Ay_buf=Ay;Az_buf=Az;
   Wx_buf=Wx;Wy_buf=Wy;Wz_buf=Wz;
   
   //Serial.println(zoom_state);
   /*if(!zoom_state&&zoom_buf){
   zoom();
   }*/
   
   zoom_buf=zoom_state;

   if(scroll_count>=1 && scroll_count<100)
   scroll_count++;
   if(scroll_count==100)
   scroll_count=0;
   
   //sending message
   radio.txStandBy();
   if (!radio.write( &message, sizeof(message) )){
       //Serial.println("failed");
       //radio.printDetails();
  }
   

   message.scroll=0;//clear scroll value
   message.command&=0xC7;//clear zoom-in/out and horizontal scroll bit sate
   
   //t1=micros()-t0;
   //delayMicroseconds(dt*1000-(micros()-t0));//dt ms
   //Serial.print("duration: ");Serial.println(t1);
}
