#include "SPI.h"
#include <Servo.h>
#include <stdlib.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "SparkFun_ENS160.h"

#define PI 3.1415926535897932384626433832795
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

Adafruit_BNO055 bno = Adafruit_BNO055(55);  //Giroscopio
SparkFun_ENS160 myENS;  //Sensor de CO2

int ensStatus; 

//Salidas de PPM para drivers de llantas:
const uint8_t VescOutputFront = 9;
const uint8_t VescOutputBack = 10;

Servo Driver12;
Servo Driver34;

//Canales del módulo FS-IA6B
#define N 6
#define CH1 3
#define CH2 4
#define CH3 5
#define CH4 6
#define CH5 7
#define CH6 8
int ch[N];
int Vi;
int Ve;

//Pines para los esclavos
const int esclavo1 = 42;  //Pin que ses usa para decir cuando queremos que el esclavo 1 reciba info
const int esclavo2 = 44;
const int esclavo3 = 46;
const int esclavo4 = 48;

unsigned int SP1;
unsigned int SP2;
unsigned int SP3;
unsigned int SP4;
char str1[4]="350";
char str2[4]="350";
char str3[4]="350";
char str4[4]="350";

//Variables para Ackerman
float x = 0.220;
float y = 0.264;


int alpha_grad = 0;
int alpha_abs;
float alpha_rad;
int beta_grad;
float beta_rad;

//0 es izquierda 1 es derecha 2 es al centro
int direccion = 2;

void setup()
{
  pinMode(esclavo1, OUTPUT);
  pinMode(esclavo2, OUTPUT);
  pinMode(esclavo3, OUTPUT);
  pinMode(esclavo4, OUTPUT);
  //Declaramos como entrada los pines receptores del modulo FS-IA6B
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
  //Salidas PPM para llantas brushless
  Driver12.attach(VescOutputFront);
  Driver12.writeMicroseconds(1500);
  Driver34.attach(VescOutputBack);
  Driver34.writeMicroseconds(1500);
  //Inicializamos comunicación SPI
  SPI.begin();
  Wire.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8); //divide the clock by 8
  Serial.println("Hello I'm SPI Mega_Master");
  digitalWrite(esclavo1, HIGH);
  digitalWrite(esclavo2, HIGH);
  digitalWrite(esclavo3, HIGH);
  digitalWrite(esclavo4, HIGH);
  
  /* Inicializar giroscopio */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Problema giroscopio");
    while(1);
  }
  
  delay(100);
    
  bno.setExtCrystalUse(true);

  /* Inicializar CO2 */
  if( !myENS.begin() )
	{
		Serial.println("Trikitrakatelas, no se pudo comunicar con CO2 :(");
		while(1);
	}

	// Reset the indoor air quality sensor's settings.
	if( myENS.setOperatingMode(SFE_ENS160_RESET) )
		Serial.println("Ready.");
  delay(100);

	// Device needs to be set to idle to apply any settings.
	// myENS.setOperatingMode(SFE_ENS160_IDLE);

	// Set to standard operation
	// Others include SFE_ENS160_DEEP_SLEEP and SFE_ENS160_IDLE
	myENS.setOperatingMode(SFE_ENS160_STANDARD);

	// There are four values here: 
	// 0 - Operating ok: Standard Operation
	// 1 - Warm-up: occurs for 3 minutes after power-on.
	// 2 - Initial Start-up: Occurs for the first hour of operation.
  //												and only once in sensor's lifetime.
	// 3 - No Valid Output
	ensStatus = myENS.getFlags();


  // set baud rate to 115200 for UART
  Serial.begin(115200);
}

void loop (void)
{
  //Leemos el sensor de CO2
  if( myENS.checkDataStatus() )
	{
		Serial.print("Concentración de CO2: ");
		Serial.print(myENS.getECO2());
		Serial.println("ppm");
	}

  /* Esto hace una lectura del giroscopio */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Imprimimos cada una de sus posiciónes */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");

  ch[1] = pulseIn(CH2, HIGH);   //Brushless
  ch[3] = pulseIn(CH4, HIGH);   //Dirección
  
  while(ch[3] > 950 && ch[3] < 1100)                    //Giro derecha
  {
    ch[3] = pulseIn(CH4, HIGH);
    if(alpha_grad >= 89)
    {
      //NADA
    }
    else if(alpha_grad < 0)
    {
      alpha_grad = alpha_grad+3;
      alpha_abs = abs(alpha_grad);
      Serial.print("Alpha: ");
      Serial.println(alpha_grad);
      alpha_rad = alpha_abs * DEG_TO_RAD;
      beta_rad = atan((y*tan(alpha_rad))/(y+(2*x*tan(alpha_rad))));
      beta_grad = beta_rad * RAD_TO_DEG;
      
      SP1 = 180+beta_grad;
      SP2 = 180+alpha_abs;
      SP3 = 180-alpha_abs;
      SP4 = 180-beta_grad;
      
      itoa(SP1, str1, 10);
      itoa(SP2, str2, 10);
      itoa(SP3, str3, 10);
      itoa(SP4, str4, 10);
      enviarPosicion(esclavo1, str1);
      delay(10);
      enviarPosicion(esclavo2, str2);
      delay(10);
      enviarPosicion(esclavo3, str3);
      delay(10);
      enviarPosicion(esclavo4, str4);
      delay(10);
    }
    else
    {
      alpha_grad = alpha_grad+3;
      Serial.print("Alpha: ");
      Serial.println(alpha_grad);
      alpha_rad = alpha_grad*DEG_TO_RAD;
      beta_rad = atan((y*tan(alpha_rad))/(y+(2*x*tan(alpha_rad))));
      beta_grad = beta_rad * RAD_TO_DEG;
      SP1 = 180-alpha_grad;
      SP2 = 180-beta_grad;
      SP3 = 180+beta_grad;
      SP4 = 180+alpha_grad;
      itoa(SP1, str1, 10);
      itoa(SP2, str2, 10);
      itoa(SP3, str3, 10);
      itoa(SP4, str4, 10);
      enviarPosicion(esclavo1, str1);
      delay(10);
      enviarPosicion(esclavo2, str2);
      delay(10);
      enviarPosicion(esclavo3, str3);
      delay(10);
      enviarPosicion(esclavo4, str4);
      delay(10);
    }
    if(SP1 < 135)
    {
      direccion = 1;
    }
    else
    {
      direccion = 2;
    }
  }
  while(ch[3] > 1850 && ch[3] < 2000)                      //Giro izquierda
  {
    ch[3] = pulseIn(CH4, HIGH);
    if(alpha_grad <= -89)
    {
      //NADA
    }
    else if(alpha_grad > 0)
    {
      alpha_grad = alpha_grad-3;
      Serial.print("Alpha: ");
      Serial.println(alpha_grad);
      alpha_rad = alpha_grad*DEG_TO_RAD;
      beta_rad = atan((y*tan(alpha_rad))/(y+(2*x*tan(alpha_rad))));
      beta_grad = beta_rad * RAD_TO_DEG;
      SP1 = 180-alpha_grad;
      SP2 = 180-beta_grad;
      SP3 = 180+beta_grad;
      SP4 = 180+alpha_grad;
      itoa(SP1, str1, 10);
      itoa(SP2, str2, 10);
      itoa(SP3, str3, 10);
      itoa(SP4, str4, 10);
      enviarPosicion(esclavo1, str1);
      delay(10);
      enviarPosicion(esclavo2, str2);
      delay(10);
      enviarPosicion(esclavo3, str3);
      delay(10);
      enviarPosicion(esclavo4, str4);
      delay(10);
    }
    else
    {
      alpha_grad = alpha_grad-3;
      alpha_abs = abs(alpha_grad);
      Serial.print("Alpha: ");
      Serial.println(alpha_grad);
      alpha_rad = alpha_abs * DEG_TO_RAD;
      beta_rad = atan((y*tan(alpha_rad))/(y+(2*x*tan(alpha_rad))));
      beta_grad = beta_rad * RAD_TO_DEG;

      SP1 = 180+beta_grad;
      SP2 = 180+alpha_abs;
      SP3 = 180-alpha_abs;
      SP4 = 180-beta_grad;
      
      itoa(SP1, str1, 10);
      itoa(SP2, str2, 10);
      itoa(SP3, str3, 10);
      itoa(SP4, str4, 10);
      enviarPosicion(esclavo1, str1);
      delay(10);
      enviarPosicion(esclavo2, str2);
      delay(10);
      enviarPosicion(esclavo3, str3);
      delay(10);
      enviarPosicion(esclavo4, str4);
      delay(10);
    }
  }
  if(alpha_grad < 0)                  //Izquierda
  {
    Ve = ch[1]-1500;
    Vi = (sin(beta_rad)/sin(alpha_rad))*Ve;
    Vi = Vi+1500;
    Driver12.writeMicroseconds(ch[1]);
    Driver34.writeMicroseconds(Vi);
  }
  else if(alpha_grad > 0)             //Derecha
  {
    Ve = ch[1]-1500;
    Vi = (sin(beta_rad)/sin(alpha_rad))*Ve;
    Vi = Vi+1500;
    Driver12.writeMicroseconds(Vi);
    Driver34.writeMicroseconds(ch[1]);
  }
  else                                //Al centro
  {
    Driver12.writeMicroseconds(ch[1]);
    Driver34.writeMicroseconds(ch[1]);
  }
}

//Función para enviar la posición que queremos (ya en grados) a un esclavo específico. Le metemos como argumentos el esclavo y el ángulo
void enviarPosicion(int direccionEsclavo, String posicion)
{
  digitalWrite(direccionEsclavo, LOW); // enable Slave Select
  // send test string
  Serial.print("Valor enviado esclavo: ");
  Serial.println(direccionEsclavo);
  for(int i=0; i< sizeof(posicion)-3; i++)
  {
    Serial.println(posicion[i]);
    SPI.transfer(posicion[i]);
  }
  digitalWrite(direccionEsclavo, HIGH); // disable Slave Select
}
