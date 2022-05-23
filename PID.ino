#include <string.h>
#include <LiquidCrystal.h>
#include <Arduino.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LedControl.h>
//------------------------------------------------------
int ContFan; 
int Calc;
int NumRpm = 3; 
float valor = 0;
int SensorTemp = A7;
int pwm = 11;
//Leds control temperatura
int Led_Rot = 31;
int Led_Gruen = 35;
int Led_Blau = 39;
//Leds control bateria
int Led_Bat_R = 45;
int Led_Bat_G = 49;
int Led_Bat_B = 53;
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
unsigned long lastTime;
double Input, Output;
double Setpoint = 30;
double ITerm, lastInput;
double kp, ki, kd;
double error;
double dInput;
int SampleTime = 1000; //1 sec
double previousValor = 0;
float Bat_Lectura;
float volt;
// Umbrales de control de voltaje de la bateria.
// > 8.5 nueva
// entre 6.5 y 8.4 media carga
// < 6.4 descargada
float maximo = 8.4;
float medio = 6.5;
//Configurar una instancia de oneWire para comunicarse con cualquier dispositivo OneWire 
OneWire oneWire(SensorTemp);  
//Pasar la referencia oneWire a la biblioteca DallasTemperature 
DallasTemperature sensors(&oneWire);

// rpm -----------------------------------------------
typedef struct{

char fantype;
unsigned int fandiv; }fanspec;
 
void rpm(){ 
ContFan++; 
}
//----------------------------------------------------
//----------------------------------------------------
void Compute()
{
   //Cuánto tiempo desde la última vez que calculamos
   unsigned long now = millis();

   int timeChange = (now - lastTime);
      if(timeChange>=SampleTime)
      {
  
      //Calcular todas las variables de error de trabajo
      error = Setpoint - Input;
      ITerm += (ki * error);
      dInput = (Input - lastInput);
  
      //Calcular salida PID
      Output = kp * error + ITerm - kd * dInput;
  
      //Recuerda algunas variables para la próxima
      lastInput = Input;
      lastTime = now;
      }
}
//----------------------------------------------------
void SetTunings(double Kp, double Ki, double Kd)
{
   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
//-----------------------------------------------------
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
//-----------------------------------------------------
void setup(){
    pinMode(pwm, OUTPUT);
    pinMode(NumRpm, INPUT);
    Serial.begin(9600);
    attachInterrupt(0, rpm, RISING); 
    SetTunings(4.2,0.01,0);
    sensors.begin();  //Pon en marcha la biblioteca 
    lcd.begin(16, 2); //Configurar la pantalla LCD
    lcd.setCursor(0, 0); 
    //Configurar las salidas de los LEDs de control y la entrada del control de batería
    pinMode(Led_Rot, OUTPUT); 
    pinMode(Led_Gruen, OUTPUT);
    pinMode(Led_Blau, OUTPUT); 
    pinMode(A6,INPUT);
    pinMode(Led_Bat_R, OUTPUT);
    pinMode(Led_Bat_G, OUTPUT);
    pinMode(Led_Bat_B, OUTPUT);
}

String leStringSerial(){
  String conteudo = "";
  char caractere;
  // Mientras recibes algo por serial 
  while(Serial.available() > 0) {
    caractere = Serial.read();  // leer byte de serie 
    if (caractere != '\n'){  // Ignorar el carácter de salto de línea 
      conteudo.concat(caractere);  
    }
    // Espere a que el búfer en serie lea el siguiente carácter 
    delay(10);
  }
    
  Serial.print("Recebi: ");
  Serial.println(conteudo);
    
  return conteudo;
}
  //Control de voltaje de la batería con histéresis. 
  void bat() {
    Bat_Lectura = analogRead(A6);
    volt = Bat_Lectura /1023 * 5.0; // Convertir el valor de ADC en voltaje 
    Serial.print("Baterry:");
    Serial.println(volt);
    // Dependiendo del voltaje encender el Led?
    if (volt >= maximo){
      digitalWrite(Led_Bat_R, LOW);
      digitalWrite(Led_Bat_G, HIGH);
      digitalWrite(Led_Bat_B, LOW);
    }else if (volt < maximo && volt >= medio){
      digitalWrite(Led_Bat_R, LOW);
      digitalWrite(Led_Bat_G, LOW);
      digitalWrite(Led_Bat_B, HIGH);
    }else if (volt < medio){
      digitalWrite(Led_Bat_R, HIGH);
      digitalWrite(Led_Bat_G, LOW);
      digitalWrite(Led_Bat_B, LOW);
    }
  }

void loop(){
  ContFan = 0;

  sei();
  delay (1000);
  cli();
  // Se inicia la medición de temperatura ...
  sensors.requestTemperatures(); 
  float leitura = sensors.getTempCByIndex(0); 
  Serial.print("Temp :  ");
  Serial.println(leitura);

  valor = leitura;

  if (Serial.available() > 0){
    String recebido = leStringSerial();
    valor = recebido.toFloat();
    Serial.println(valor);
  }

  Input = valor;

  Compute();

  Serial.print("error:");
  Serial.println(error);
  
  Serial.print("ITerm:");
  Serial.println(ITerm);

  Serial.print("dInput:");
  Serial.println(dInput);

  Serial.print("Output:");
  Serial.println(Output);
 
  //On/off con histéresis.
  if (valor != previousValor){ 
    // Valor por encima del umbral máximo 
    if ((( (sensors.getTempCByIndex(0)) > ( 55 ) ))){
        digitalWrite(Led_Rot, HIGH);
        digitalWrite(Led_Gruen, LOW);
        digitalWrite(Led_Blau, LOW);
      }else{
        digitalWrite(Led_Rot, LOW);
        digitalWrite(Led_Gruen, LOW);
        digitalWrite(Led_Blau, LOW);
      }
    // Valor entre dos umbrales (región media) 
    if (( ( (sensors.getTempCByIndex(0)) <= ( 55 ) ) && ( (sensors.getTempCByIndex(0)) >= ( 30 ) ) )){       
        digitalWrite(Led_Rot, LOW);
        digitalWrite(Led_Gruen, HIGH);
        digitalWrite(Led_Blau, LOW);
        }else{
        digitalWrite(Led_Rot, LOW);
        digitalWrite(Led_Gruen, LOW);
        digitalWrite(Led_Blau, LOW);
        }
    // Valor por debajo del umbral mínimo 
    if (( (sensors.getTempCByIndex(0)) < ( 30 ) )){
        Output = 0; 
        digitalWrite(Led_Rot, LOW);
        digitalWrite(Led_Gruen, LOW);
        digitalWrite(Led_Blau, HIGH);
        }else{
        digitalWrite(Led_Rot, LOW);
        digitalWrite(Led_Gruen, LOW);
        digitalWrite(Led_Blau, LOW);
        }
  }
  // Guarda el estado actual como último estado, para la próxima vez a través del bucle
  previousValor = valor; 

  analogWrite(pwm,Output);
  // Llamar a la función de control de la batería
  bat();
  // Imprimir en el serial y en la pantalla LCD
  Calc = ((ContFan * 60)/2);
  Serial.print (Calc, DEC);
  Serial.print (" rpm\r\n"); 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(leitura);
  lcd.setCursor(6, 0);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print(Calc);
  lcd.setCursor(6, 1);
  lcd.print("rpm");
  lcd.setCursor(11, 1);
  lcd.print(volt);
  lcd.setCursor(15, 1);
  lcd.print("V");
}
