#include "SPI.h"
#include <Wire.h>
#include <stdlib.h>

char str[3];
volatile byte i;
volatile bool pin;

//Magnetic sensor things
int magnetStatus = 0; //value of the status register (MD, ML, MH)

int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle 
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])

float startAngle = 0; //starting angle

//Pines puente H
int PWM1 = 2;
int R1 = 3;
int L1 = 4;

float SetPoint = 180;  //Es el valor del ángulo que queremos (Lo vamos a leer por SPI)
float SetPointAnterior = 180;
float PosPrevia = 0;  //La posición que tenía el encoder (sirve para guardar la posición anterior, no la actual)

float Proporcional = 7;  //Ganancia proporcional 1 ////////////////////////////////////////3-0.99-20
float Integral = 1.5;  //Ganancia integral 0.9
float Derivativa = 0;  //Ganancia derivativa 1.5
float AccionControl = 0;  //Variable de proceso (PWM que se alimentará al motor)

float tActual = 0;  //Tiempo que se tiene en este preciso momento
float tPrevio = 0;  //Tiempo anterior para calcular el delta-t
float dt = 0;  //La delta-t (diferencia entre el tiempo actual y el pasado)
float error = 0;  //Literalmente eso, el error xD (diferencia entre el SetPoint y el valor actual)
float ePrevio = 0;  //Para poder calcular la pendente del derivativo
float eIntegral = 0;  //Error intefral que se ha acumulado a lo largo del tiempo
float eDerivativo = 0;  //La pendiente del error para la acción derivativa

void setup()
 {
  Serial.begin (115200);    // set baud rate to 115200 for usart
  Serial.println("Hello I'm SPI UNO_SLAVE");
  pinMode(MISO, OUTPUT);   // have to send on Master in so it set as output
  pinMode(R1, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  SPCR |= _BV(SPE);        // turn on SPI in slave mode
  i = 0; // buffer empty
  pin = false;
  SPI.attachInterrupt();     // turn on interrupt
  Wire.begin();
  Wire.setClock(800000L); //fast clock

  checkMagnetPresence(); //check the magnet (blocks until magnet is found)

  ReadRawAngle(); //make a reading so the degAngle gets updated
  
  startAngle = degAngle; //update startAngle with degAngle - for taring
}

void loop(){
  if (pin)
  {
    pin = false;   //reset the pin
    //Serial.print(" : ");
    if(atoi(str) > 270 || atoi(str) <90){
      SetPoint = SetPointAnterior;
    }
    else{
      SetPoint = atoi(str);
    }
    SetPointAnterior = SetPoint;
    //ReadRawAngle(); //ask the value from the sensor
    Serial.print("SetPoint:  ");
    Serial.print(SetPoint); //print the array on serial monitor
    Serial.print("  Ángulo:  ");
    Serial.print(degAngle);
    Serial.print("  Error:  ");
    Serial.print(error);
    Serial.print("  Acumulado:  ");
    Serial.print(eIntegral);
    Serial.print("  Acción:  ");
    Serial.println(AccionControl);
    //delay(1000);
    i= 0; //reset button to zero
  }
  PID();
}

// Interrupt function
ISR(SPI_STC_vect) 
{
  char c = SPDR;        // read byte from SPI Data Register
  if (c != " ") {
    //Serial.print(c);
    str [i++] = c; // save data in the next index in the array buff
  }
  else if ( (c == '\r') || (c == '\n') || (c=='\0') || (c==' ') ){ //check for the end of the word
    pin = true;
  }
}

void CW(int Potencia)
{
  analogWrite(PWM1, Potencia);
  digitalWrite(R1, LOW);
  digitalWrite(L1, HIGH);
}
void CCW(int Potencia)
{
  analogWrite(PWM1, Potencia);
  digitalWrite(L1, LOW);
  digitalWrite(R1, HIGH);
}

void motor_off()
{
  analogWrite(PWM1, 0);
  digitalWrite(R1, LOW);
  digitalWrite(L1, LOW);
}

void PID(){
  tActual = micros();  //Guardamos el tiempo del temporizador micros como el tiempo actual (esta en micro segundos)
  dt = (tActual - tPrevio)/1000000.0;  //La diferencia de tiempos, pero entre 1000000.0 para que esté en segundos
  tPrevio = tActual;  //Actualizamos el tiemo que teniamos anteriormente con el nuevo
  ReadRawAngle(); //ask the value from the sensor
  error = SetPoint - degAngle;
  //if(error < 2 && error > -2){eIntegral = 0;}  //BORRAR
  eIntegral = eIntegral + (error*dt);  //Al acumulado del error, le sumamos el error de esta nueva iteracion
  eDerivativo = (error - ePrevio)/dt;  //Con eso se calcula la pendiente, es (y2-y1)/(x2-x1), las y's son posiciones, las x's los tiempos

  AccionControl = (Proporcional*error) + (Integral*eIntegral) + (Derivativa*eDerivativo);  //La acción a realizar es la suma del P el I y el D
  if(AccionControl > 0){
      if(AccionControl >= 255){
        AccionControl = 255;
      }
      AccionControl = map(AccionControl, 0, 255, 110, 200);  //De 100 a 255 originalmente
      motor_off();
      CW(AccionControl);
    }
    else if(AccionControl < 0){
      AccionControl = abs(AccionControl);
      if(AccionControl >= 255){
        AccionControl = 255;
      }
      AccionControl = map(AccionControl, 0, 255, 110, 200);
      motor_off();
      CCW(AccionControl);
    }
  ePrevio = error;  //Actualizamos el tiempo anterior para que sea el nuevo tiempo
}

void checkMagnetPresence()
{  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

    //Serial.print("Magnet status: ");
    //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }      
  
  //Status register output: 0 0 MD ML MH 0 0 0  
  //MH: Too strong magnet - 100111 - DEC: 39 
  //ML: Too weak magnet - 10111 - DEC: 23     
  //MD: OK magnet - 110111 - DEC: 55

  //Serial.println("Magnet found!"); 
}

void ReadRawAngle()
{ 
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
  
  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625;


  if(degAngle > 0 && degAngle <= 135)
  {
    degAngle = degAngle + 225;
  }
  else if (degAngle > 135 && degAngle <= 360)
  {
    degAngle = degAngle - 136; //Angle correction
  }
  //Serial.print("Deg angle: ");
  //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
  
}
