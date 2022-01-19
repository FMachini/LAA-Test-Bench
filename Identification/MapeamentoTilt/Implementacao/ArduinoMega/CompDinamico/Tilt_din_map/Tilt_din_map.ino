/*    Universidade Federal de Uberlandia
       Faculdade de Engenharia Mecanica
          Engenharia Mecatrônica        
          Realizado em 02/12/2019
/*          Realizar o mapeamento estatico do mecanismo tilt */
//----------------------------------
//    BIBLIOTECAS
//----------------------------------

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

//----------------------------------
//    OBJETOS
//----------------------------------

Servo Tilt1;
Servo Motor1;


//----------------------------------
//    VARIAVEIS GLOBAIS
//---------------------------------- 

float t_k;
float Ta =0.02;//[segundos]
int potPin = 0;

//----------------------------------
//   VARIAVEL DE COMANDO
//----------------------------------
float pwm=1650;
int i=0;

//----------------------------------
//   VARIAVEL DE ESTADO
//----------------------------------
int tilt_out=0;
float theta_pot=0;
float theta_tilt1=0;
float theta_tilt2=0;

// potenciometro
float coef_ang= 0.2586;
float coef_lin= 47.2534 ;

// tilt 1
// Relacao thetatilt1 thetapot
// Relacao thetatilt1 CAD
// Relacao thetatilt1 PWM

// tilt 2
// Relacao thetatilt2 thetapot
// Relacao thetatilt2 CAD
// Relacao thetatilt2 PWM

float a=0.0000396248925;
float b=-0.2146384032536;
float c=280.8235824934245;
//----------------------------------
//    VOID SETUP
//----------------------------------


void setup() {
  
  
  Serial.begin(115200);
  
  Tilt1.attach(8); //attach the motor
  Motor1.attach(2); //attach the motor

  Tilt1.writeMicroseconds(1650); // Tilt Initialization
  Motor1.writeMicroseconds(1); // Motor Initialization
  delay(2000);
 
  t_k=millis();
  Motor1.writeMicroseconds(1150); // Motor Initialization
  delay(2000);
  Motor1.writeMicroseconds(1350); // Motor Initialization
  delay(2000);
}

//----------------------------------
//    VOID LOOP
//----------------------------------


void loop() {
   t_k=millis();
   
   if(i==200){
   pwm=1800;
   saturacaoTilt();
   Tilt1.writeMicroseconds (pwm);
   }
   else if(i==400){
    pwm=1650;
    saturacaoTilt();
    Tilt1.writeMicroseconds (pwm);
    }
   else if(i==600){
    pwm=2350;
    saturacaoTilt();
    Tilt1.writeMicroseconds (pwm);
    }
    else if(i==800){
    pwm=1650;
    saturacaoTilt();
    Tilt1.writeMicroseconds (pwm);
    Motor1.writeMicroseconds (1);
    }

      readSensor();
      printInfo();

   while( (millis()- t_k)/1000 < Ta)
        {
          //Wait for the next amostragem 
        }
      i++;
}

//----------------------------------
//    SUBROTINAS
//----------------------------------

void saturacaoTilt(){
  if(pwm > 2200)
  {
    pwm = 2200;
  }
  if(pwm < 1650)
  {
   pwm=1650;
  }
}




void printInfo(){
  Serial.print((millis()/1000.00),5);
  Serial.print("\t ");  
  Serial.print(pwm);
  Serial.print("\t ");
  Serial.print(tilt_out);
  Serial.print("\t ");
  Serial.print(theta_pot);
  Serial.print("\t ");
  Serial.print(theta_tilt2);
  Serial.print("\t ");
  Serial.println(   ( (millis()-t_k)/1000.00 )  ,4);
  
}

void readSensor(){
  tilt_out = analogRead(potPin); // read the value from potentiometer
  theta_pot = tilt_out*coef_ang+coef_lin;

  
  //theta_tilt1 = pwm2angle(pwm);
  
  theta_tilt2 = pwm2angle(pwm);
  
}

float pwm2angle(int pwm){
  return(a*pwm*pwm+b*pwm+c);
}

//
//float cad2angle(float cad){
//  return(coef1_ang*cad+coef1_lin);
//}
//
//float thetaPot2angle(float thetapot){
//  return(coef1_ang*thetapot+coef1_lin);
//}

/*
  if(i==200){
   flag=1;
   pwm=1800;
   saturacao();
   Tilt1.writeMicroseconds (pwm);
   }
   else if(i==400){
    flag=-1;
    pwm=1650;
    saturacao();
    Tilt1.writeMicroseconds (pwm);
    }
   else if(i==600){
    flag=-1;
    pwm=2350;
    saturacao();
    Tilt1.writeMicroseconds (pwm);
    }
    else if(i==800){
    flag=-1;
    pwm=1650;
    saturacao();
    Tilt1.writeMicroseconds (pwm);
    Motor1.writeMicroseconds (1);
    }
    */

    /*

   
  if(i<700){
   flag=1;
   pwm=pwm+1;
   saturacao();
   Tilt1.writeMicroseconds (pwm);
   }
   else if(i==1400){
    flag=-1;
    pwm=1650;
    saturacao();
    Tilt1.writeMicroseconds (pwm);
    Motor1.writeMicroseconds(1);
    }
   else if(i>700){
    flag=-1;
    pwm=pwm-1;
    saturacao();
    Tilt1.writeMicroseconds (pwm);
    }
     */
