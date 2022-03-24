# include <Wire.h>
# include "MPU6050.h"
# include "I2Cdev.h"
#include <SoftwareSerial.h>
# define degconvert 57.2957786
MPU6050 mpu;
SoftwareSerial mySerial(10, 11);

uint32_t timer;
uint32_t timer2;
uint32_t timer3;
uint32_t timer4;
double compAngleX;
double omega;
double roll;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
const int MPU_addr = 0x68;
double gyro_kot;

const byte pin_A = 1;
volatile unsigned char encoder_A;
volatile unsigned char encoder_A_prej=0;
volatile double i=0;
const int pin_Q = 4;
volatile unsigned char izhod_Q;
volatile unsigned char izhod_Q_prej = 0;

const byte pin_B = 0;
volatile unsigned char encoder_B;
volatile unsigned char encoder_B_prej =0;
volatile double j=0;
const int pin_Q2 = 8;
volatile unsigned char izhod_Q2;
volatile unsigned char izhod_Q2_prej=0;

int set1 = 13;
int set2 = 9;
int set3 = 6;
int set4 = 5;

double H_signal_1 = 0;
double H_signali = 0;
double H_signal_2 = 0;
double H_dt = 0;
uint32_t H_cas = 0;
double H_hitrost = 0;
double H_napaka = 0;
double H_referenca = 0;
double H_integral = 0;
double H_diferencial = 0;
double H_napaka_prej = 0;
double H_pwm = 0;

double B_kot = 0;
double B_referenca = 0;
double B_napaka = 0;
double B_integral = 0;
double B_diferencial = 0;
double B_napaka_prej = 0;
double B_pwm = 0;
double B1_pwm = 0;
double B2_pwm = 0;



double M1_signal_1 = 0;
double M1_signali = 0;
double M1_signal_2 = 0;
double M1_dt = 0;
uint32_t M1_cas = 0;
double M1_hitrost = 0;
double M1_napaka = 0;
double M1_integral = 0;
double M1_diferencial = 0;
double M1_napaka_prej = 0;
double M1_pwm = 0;

double M2_signal_1 = 0;
double M2_signali = 0;
double M2_signal_2 = 0;
double M2_dt = 0;
uint32_t M2_cas = 0;
double M2_hitrost = 0;
double M2_napaka = 0;
double M2_integral = 0;
double M2_diferencial = 0;
double M2_napaka_prej = 0;
double M2_pwm = 0;

double T1 = 0;
double T2 = 0;
int T10=0;
double gyro_kot2=0;

int a = 0;
int b = 0;

char incomingByte = 0;
int kontroler = 0;

double t2=0;
double t3=0;
double t4=0;

double H_prop;
double H_int;
double H_dif;

double B_prop;
double B_int;
double B_dif;

double M1_prop;
double M1_int;
double M1_dif;

double M2_prop;
double M2_int;
double M2_dif;

double del = 1.7;

void setup() {
mySerial.begin(38400); 

pinMode (pin_A, INPUT);
pinMode (pin_Q, INPUT);
pinMode (pin_B, INPUT);
pinMode (pin_Q2, INPUT);
attachInterrupt(digitalPinToInterrupt(pin_A), pin_ISR, RISING);
attachInterrupt(digitalPinToInterrupt(pin_B), pin_ISR2, RISING);

pinMode(set1, OUTPUT);
pinMode(set2, OUTPUT);
pinMode(set3, OUTPUT);
pinMode(set4, OUTPUT);  

Wire.begin();
Wire.setClock(400000UL); 
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);  
Wire.write(0);  
Wire.endTransmission(true);

mpu.setXAccelOffset(-3948) ;
mpu.setYAccelOffset(-2400);
mpu.setZAccelOffset(1399) ;
  
mpu.setXGyroOffset(-70) ;
mpu.setYGyroOffset(25) ;
mpu.setZGyroOffset(-6) ;
  
timer = micros();
timer2 = micros();
timer3 = micros();
timer4 = micros();
timer2 = micros();
M1_cas = micros();
M2_cas = micros();
H_cas = micros();



}

void pin_ISR(){ 
  izhod_Q = digitalRead (pin_Q);

    if (!izhod_Q) {i++;
     }
    else {i--;
     } 
  } 


void pin_ISR2(){

  izhod_Q2 = digitalRead (pin_Q2);
    if (!izhod_Q2) {j--;
     }
    else {j++;
     }       
}

void loop() {

if(mySerial.available()>0)
  {
     incomingByte = mySerial.read();
     kontroler = incomingByte - '0';
     if (kontroler == 1){
       H_referenca = 0.3;
       del=1;
        }
      else if ((kontroler == 5)) {
        
        H_referenca = 0;
        del=1.7;
        }
      else if ((kontroler == 4)) {
        
        H_referenca = -0.3;
        del=1;
        }  
        }

H_signal_1 = (i+j)/2.0;
H_signali = H_signal_1 - H_signal_2;
H_signal_2 = H_signal_1;
H_dt =(double)(micros() - H_cas);
H_cas = micros();
H_hitrost = (0.000500293*H_signali)/(H_dt*0.000001);

t2 = (double)(micros() - timer2) / 1000000; 
timer2 = micros();
H_napaka =  H_referenca - H_hitrost;
if ((abs (H_napaka_prej)/del) < (abs (H_napaka))) {H_integral+=H_napaka*t2;}
else {H_integral=H_napaka*t2;}
H_diferencial = (H_napaka - H_napaka_prej)/t2;
H_prop = 4*H_napaka;
H_int = 23*H_integral;
H_dif = 0*H_diferencial;
H_pwm = (H_prop + H_int + H_dif);
H_napaka_prej = H_napaka;

T1 = 0; 


//--------------------------------------------------------------------

while (T1<6){

Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();     
AcY=Wire.read()<<8|Wire.read(); 
AcZ=Wire.read()<<8|Wire.read(); 
Tmp=Wire.read()<<8|Wire.read();
GyX=Wire.read()<<8|Wire.read();  
GyY=Wire.read()<<8|Wire.read();
GyZ=Wire.read()<<8|Wire.read();
  
double roll = atan2(AcY, AcZ)*degconvert;
double t = (double)(micros() - timer) / 1000000; 
timer = micros();
double omega = GyX/131.0;
gyro_kot = (omega*t);
gyro_kot2 = gyro_kot2+gyro_kot;
compAngleX = 0.995 * (compAngleX + gyro_kot) + (0.005 * roll); 


T10++;
if (T10>30){
  double timer33 = micros();
  mySerial.print(timer33);
 mySerial.print(" ");
 mySerial.print(roll);
 mySerial.print(" ");
  mySerial.print(gyro_kot2);
 mySerial.print(" ");
 mySerial.println(compAngleX);
 T10=0;
  }


t3 = (double)(micros() - timer3) / 1000000; 
timer3 = micros();
B_kot=compAngleX;
B_referenca=H_pwm;
B_napaka = B_referenca - B_kot;
B_integral += B_napaka*t3; 
B_diferencial = (B_napaka - B_napaka_prej)/t3;
B_integral = constrain(B_integral, -2L, 2L);
B_prop = 17*B_napaka;
B_int = 70*B_integral;
B_dif = 0.17*B_diferencial;
B_pwm = (B_prop + B_int + B_dif);
B_napaka_prej = B_napaka;

B1_pwm = B_pwm;
B2_pwm = B_pwm;

if (kontroler == 3) {
     B1_pwm = B_pwm*0.8;
     B2_pwm = B_pwm*1.2;
     del=1;
     } 
else if (kontroler == 2) {
    B2_pwm = B_pwm*0.8;
    B1_pwm = B_pwm*1.2;
    del=1;
    }    
else if (kontroler == 5) {
    B2_pwm = B_pwm;
    B1_pwm = B_pwm;
    del=1.7;
    } 

T1++;
T2 = 0;

//--------------------------------------------------------------------

while (T2<3){

M1_signal_1 = i;
M1_signali = M1_signal_1 - M1_signal_2;
M1_signal_2 = M1_signal_1;
M1_dt =(double) (micros() - M1_cas);
M1_cas = micros();
M1_hitrost = (0.000500293*M1_signali)/(M1_dt*0.000001);

M2_signal_1 = j;
M2_signali = M2_signal_1 - M2_signal_2;
M2_signal_2 = M2_signal_1;
M2_dt =(double) (micros() - M2_cas);
M2_cas = micros();
M2_hitrost = (0.000500293*M2_signali)/(M2_dt*0.000001);

t4 = (double)(micros() - timer4) / 1000000; 
timer4 = micros();

M1_napaka = B1_pwm - M1_hitrost;
M1_integral = M1_integral+M1_napaka*t4;  
M1_diferencial = (M1_napaka - M1_napaka_prej)/t4;
M1_integral = constrain(M1_integral, -5L, 5L);
M1_prop = 1.1 * M1_napaka;
M1_int = 0.6 * M1_integral;
M1_dif = 0.01 * M1_diferencial;
M1_pwm = (M1_prop + M1_int + M1_dif);
M1_napaka_prej = M1_napaka;

M2_napaka = B2_pwm - M2_hitrost;
M2_integral = M2_integral+M2_napaka*t4;
M2_diferencial = (M2_napaka - M2_napaka_prej)/t4;
M2_integral = constrain(M2_integral, -5L, 5L);
M2_prop = 1.1 * M2_napaka;
M2_int = 0.6 * M2_integral;
M2_dif = 0.01 * M2_diferencial;
M2_pwm = (M2_prop + M2_int + M2_dif);
M2_napaka_prej = M2_napaka;


//--------------------------------------------------------------------

double min_pwm = 1;
double max_pwm = 255;
double pwm2 = constrain(M1_pwm,-255, 255);
double pwm1 = constrain(M2_pwm,-255, 255);


if ((B_kot>55.0) || (B_kot<-55.0)){
  pwm1 = 0;
  pwm2 = 0;
  }

if(M2_pwm >= 0){
  if(M2_pwm < 1){a = 0;}
    else if(M2_pwm >= 1){a = map(pwm1, 1, 255, min_pwm, max_pwm);} 
    digitalWrite(set4, LOW);
    analogWrite(set3, a);
  } 
  else if (M2_pwm < 0){
    if(M2_pwm > -1){a = 0;}
    else if(M2_pwm <= -1){a = map(pwm1, -1, -255, min_pwm, max_pwm);}
    analogWrite(set4, a); 
    digitalWrite(set3, LOW);
  }


if(M1_pwm >= 0){
  if(M1_pwm < 1){a = 0;}
    else if(M1_pwm >= 1){b = map(pwm2, 1, 255, min_pwm, max_pwm);} 
    digitalWrite(set2, LOW); 
    analogWrite(set1, b);  
}
  else if (M1_pwm < 0){
    if(M1_pwm > -1){a = 0;}
    else if(M1_pwm <= -1){b = map(pwm2, -1, -255, min_pwm, max_pwm);}
       analogWrite(set2, b); 
       digitalWrite(set1, LOW);    
  }

T2++;
delay(3);

}
}
}
