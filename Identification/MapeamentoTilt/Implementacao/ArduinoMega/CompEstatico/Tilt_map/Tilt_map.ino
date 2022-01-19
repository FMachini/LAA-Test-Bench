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

//----------------------------------
//    VARIAVEIS GLOBAIS
//---------------------------------- 

float t_k;

float Ta =0.01;//[segundos]
int numValues = 250;


int i=0;
int potPin = 0;

//----------------------------------
//   VARIAVEL DE COMANDO
//----------------------------------

float pwm=1500;

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
float a1=0.0000221518341;
float b1=-0.1495658268261;
float c1=228.6938894893316;

// tilt 2
// Relacao thetatilt2 thetapot
// Relacao thetatilt2 CAD

// Relacao thetatilt2 PWM

float a2=0.0000396248925;
float b2=-0.2146384032536;
float c2=280.8235824934245;

//----------------------------------
//    VOID SETUP
//----------------------------------


void setup() {
  
  
  Serial.begin(115200);
  
  Tilt1.attach (8); //attach the motor
  Tilt1.writeMicroseconds(1650); // Tilt Initialization

  delay(250);
}

//----------------------------------
//    VOID LOOP
//----------------------------------


void loop() {
   t_k=millis();
   
   if(((i%numValues)==0)&&(i<17*numValues)){
    pwm=pwm+50;
    saturacao();
    Tilt1.writeMicroseconds(pwm); // Tilt Initialization
   }
   else if(((i%numValues)==0)&&(i<35*numValues)){
    pwm=pwm-50;
    saturacao();
    Tilt1.writeMicroseconds(pwm); // Tilt Initialization
   }

//   pwm=2350;
//   saturacao();
//   Tilt1.writeMicroseconds(pwm);
 
   readSensor();
  
   while( ((millis()- t_k)/1000) < Ta )
        {
          //Wait for the next amostragem 
        }
   printInfo();
   i++;
}

//----------------------------------
//    SUBROTINAS
//----------------------------------

void saturacao(){
  
  /*Saturaçao da variavel de comando entre 1650us e 2350us*/
 
  if(pwm > 2350)
  {
    pwm = 2350;
  }
  if(pwm < 1500)
  {
   pwm=1500;
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
  Serial.print(theta_tilt1);
  Serial.print("\t ");
  Serial.print(theta_tilt2);
  Serial.print("\t ");
  Serial.println(   ( (millis()-t_k)/1000.00 )  ,4);
  
}

void readSensor(){
  tilt_out = analogRead(potPin); // read the value from potentiometer
  theta_pot = tilt_out*coef_ang+coef_lin;
  theta_tilt1 = pwm2angleAlpha1(pwm);
  theta_tilt2 = pwm2angleAlpha2(pwm);
  
}


float pwm2angleAlpha1(int pwm){
  return(a1*pwm*pwm+b1*pwm+c1);
}

float pwm2angleAlpha2(int pwm){
  return(a2*pwm*pwm+b2*pwm+c2);
}

//float cad2angle(float cad){
//  return(coef1_ang*cad+coef1_lin);
//}
//
//float thetaPot2angle(float thetapot){
//  return(coef1_ang*thetapot+coef1_lin);
//}
