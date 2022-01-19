/*    Universidade Federal de Uberlandia
       Faculdade de Engenharia Mecanica
          Engenharia Mecatrônica        
          Realizado em 02/12/2019
/*          Realizar o ensaio de forca e velocidade do motor brushless        */
//----------------------------------
//    BIBLIOTECAS
//----------------------------------

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>
#include "HX711.h"
HX711 scale;

//----------------------------------
//    OBJETOS
//----------------------------------

Servo Motor1; 

//----------------------------------
//    VARIAVEIS GLOBAIS
//---------------------------------- 

float t_ks,t_k, t_km1,now,newnow;
float N1=0.2066;
float N2=0.4131;
float N3=0.2066;
      
float C1=1.0000;
float C2=-0.3695;
float C3=0.1958;
float deltaT;
float Ta =0.012;//[segundos]
int j=0;
int count=0;
int count2=0;
//----------------------------------
//   VARIAVEIS DOS MOTORES
//----------------------------------
float U_0=1000;
float U=1000;
float pwm=1000;
float u_k=0;
float u_km1=0;

float force=0.00;
float f_k=0.00;
float f_km1=0.00;
float f_km2=0.00;
float ff_km1=0.00;
float ff_km2=0.00;
int rndSeed=8;
// HX711 circuit wiring
#define LOADCELL_DOUT_PIN D5
#define LOADCELL_SCK_PIN D6
bool start;
int command;
//----------------------------------
//    VOID SETUP
//----------------------------------

void setup() {
  
  Serial.begin(115200); 

//Initialising the HX711 SCALE
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  scale.set_scale(871000.45);
  scale.tare(); 
  
  //======================
  //  MOTOR
  //======================
  //GPIO4 = D2
  Motor1.attach (4); //attach the motor
  Motor1.writeMicroseconds(1); //DJI MOTOR Initialization

  randomSeed(rndSeed);
  t_ks=millis();
  newnow=millis();
  delay(250);   
}


//----------------------------------
//    VOID LOOP
//----------------------------------

void loop()
{
   t_k=millis();
   if(Serial.available()>0)
   {
      command=Serial.read();
      if(command==101)//letra e
      {
      pwm=1000;
      start=true;
      saturacao();
      Motor1.writeMicroseconds (pwm);
      }
      if(command==32||command==113)// space or q
        {
          Motor1.writeMicroseconds (1); //full reverse. The ESC will automatically brake the motor.
          pwm=1000;
        }   
   }

   if((start==true))
   {
      if((count%350==0)){
          if((count2 < 14)){
            pwm=pwm+50;
            saturacao();
            Motor1.writeMicroseconds(pwm);
            count2=count2+1;
          }
          else if((count2 < 26)){
              pwm=pwm-50;
              saturacao();
              Motor1.writeMicroseconds(pwm);
              count2=count2+1;
          }
          else{ 
            start=false;
            count=0;
            count2=0;
            pwm=1000;
            saturacao();
            Motor1.writeMicroseconds(pwm);
          }
        }
      count=count+1;
   }
   readSensor();
   while( (millis()- t_k)/1000 < Ta)
        {
          //Wait for the next amostragem 
        }
   printInfo();
      
}

//----------------------------------
//    SUBROTINAS
//----------------------------------

void saturacao(){
  
  /*Saturaçao da variavel de comando entre 1000us e 2000us*/
  //MOTOR 1 PRETO
  if(pwm < 1000)
  {
    pwm = 1000;
  }
  if(pwm > 1700)
  {
   pwm=1700;
  }
}

void printInfo(){
  //Serial.print((millis()- t_k)/1000,4);
  Serial.print((millis()/1000.00),5);
  Serial.print("\t ");  
  Serial.print(pwm);
  Serial.print("\t ");
  Serial.print(force);
  Serial.print("\t ");
  Serial.print(count);
  Serial.print("\t ");
  Serial.print(count2);
  Serial.print("\t ");
  Serial.println(((millis()-t_k)/1000.00),5);
}

void readSensor(){
  f_k= scale.get_units(1);
  force = ( (N1)* f_k + (N2) * f_km1 + (N3) * f_km2 - C2 * ff_km1 - C3 * ff_km2 )/(C1);
  f_km2=f_km1;
  f_km1=f_k;
  ff_km2=ff_km1;
  ff_km1=force;
}
