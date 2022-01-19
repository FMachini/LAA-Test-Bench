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
#include <math.h>
#include "HX711.h"
//----------------------------------
//    OBJETOS
//----------------------------------


HX711 scale;


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
   
   readSensor();
   while( (millis()- t_k)/1000 < Ta)
        {
          
        }
   printInfo();
      
}

//----------------------------------
//    SUBROTINAS
//----------------------------------

void printInfo(){
  Serial.print((millis()/1000.00),5);
  Serial.print("\t ");
  Serial.print(force);
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
