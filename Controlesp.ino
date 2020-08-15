#include <WiFi.h>
#include <Wire.h>
#include <sys/time.h>
#include "tabela.h"

const char* ssid       = "Nome da rede wifi";
const char* password   = "Senha da rede wifi";

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

#define MCP_ADDRESS 0x20    //endereço I2C do MCP23017
#define GPA         0x12    // DATA PORT REGISTER A
#define GPB         0x13    // DATA PORT REGISTER B 
#define IODIRA      0x00    // I/O DIRECTION REGISTER A
#define IODIRB      0x01    // I/O DIRECTION REGISTER B

#define CLOCK_ADDRESS 0x68

RTC_DATA_ATTR time_t now;
RTC_DATA_ATTR struct tm * timeinfo;

uint8_t hora, minuto, segundo, dia, mes, semana;
uint16_t ano;

const long  gmtOffset_sec = -3 * 3600;
const int   daylightOffset_sec = 0;

volatile bool btVermelhoPress = 0;
volatile bool btAmareloPress = 0;
volatile bool btVerdePress = 0;
volatile bool btAzulPress = 0;
void btVermelho() {btVermelhoPress = true;}
void btAmarelo()  {btAmareloPress  = true;}
void btVerde()    {btVerdePress    = true;}
void btAzul()     {btAzulPress     = true;}

void setup() {
  Serial.begin(115200);
  setupPins();
  leDS3231();
  if (iniciaWifi()) configTime(gmtOffset_sec, daylightOffset_sec, "south-america.pool.ntp.org", "pool.ntp.org");
  //gravaDS3231();
  delay(100);
  attachInterrupt(digitalPinToInterrupt(34), btAzul,     FALLING); //subrotina de acionamento do botão azul
  attachInterrupt(digitalPinToInterrupt(35), btVerde,    FALLING); //subrotina de acionamento do botão verde
  attachInterrupt(digitalPinToInterrupt(36), btAmarelo,  FALLING); //subrotina de acionamento do botão amarelo
  attachInterrupt(digitalPinToInterrupt(39), btVermelho, FALLING); //subrotina de acionamento do botão vermelho

}

void loop() {
  int x = 1;
  for (int i = 0; i > -1;  i += x) {
    if (!btAzulPress && !btVerdePress && !btAmareloPress && !btVermelhoPress) return;
    if (btAzulPress)      ledcWrite(0, gama[i]);
    if (btVerdePress)     ledcWrite(1, gama[i]);
    if (btAmareloPress)   ledcWrite(2, gama[i]);
    if (btVermelhoPress)  ledcWrite(3, gama[i]);
    delayMicroseconds(500);
    if (i == 1023) x = -1;
  }
  printLocalTime();
  Serial.println(getTemperature());
  //btVermelhoPress = false;
  //btAmareloPress  = false;
  //btVerdePress    = false;
  //btAzulPress     = false;
  return;
  
  for (uint8_t i = 0; i < 3; i++){
    digitalWriteMCP (i, HIGH);
    delay (1000);
  }
  for (uint8_t i = 0; i < 3; i++){
    digitalWriteMCP (i, LOW);
    delay (1000);
  }
  delay(1000);
}

bool iniciaWifi(){
  uint8_t wifiCounter;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++wifiCounter > 30) return (false); // restart after 15 s
  }
  return (true);
}

void printLocalTime() {
  time(&now);
  timeinfo = localtime (&now);
  
  segundo = timeinfo->tm_sec;
  minuto  = timeinfo->tm_min;
  hora    = timeinfo->tm_hour;
  dia     = timeinfo->tm_mday;
  mes     = timeinfo->tm_mon;
  ano     = timeinfo->tm_year + 1900;
  semana  = timeinfo->tm_wday;

  Serial.printf ("%s\n", asctime(timeinfo));
}


static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }

void leDS3231(){
  I2Cone.beginTransmission(CLOCK_ADDRESS);
  I2Cone.write(0);  // This is the first register address (Seconds)
        // We'll read from here on for 7 bytes: secs reg, minutes reg, hours, days, months and years.
  I2Cone.endTransmission();
  
  I2Cone.requestFrom(CLOCK_ADDRESS, 7);
  uint8_t ss = bcd2bin(I2Cone.read() & 0x7F);
  uint8_t mm = bcd2bin(I2Cone.read());
  uint8_t hh = bcd2bin(I2Cone.read());
  I2Cone.read();
  uint8_t d = bcd2bin(I2Cone.read());
  uint8_t m = bcd2bin(I2Cone.read());
  uint16_t y = bcd2bin(I2Cone.read()) + 2000;

  timeinfo = localtime ( &now );
  timeinfo->tm_sec = ss;
  timeinfo->tm_min = mm;
  timeinfo->tm_hour = hh;
  timeinfo->tm_mday = d;;
  timeinfo->tm_mon = m - 1;
  timeinfo->tm_year = y - 1900;
  //timeinfo->tm_wday;
  time_t t = mktime (timeinfo);
  Serial.printf ("%s\n", asctime(timeinfo));
  
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);
}

uint8_t decToBcd (uint8_t val) {  // Convert normal decimal numbers to binary coded decimal
  return ( (val/10*16) + (val%10) );
}

void gravaDS3231(){

  for (int i = 0; i < 5; i++){ 
    time(&now);
    timeinfo = localtime (&now);
    Serial.printf ("%s\n", asctime(timeinfo));
    delay(1000);
  }

Serial.println(timeinfo->tm_sec);
Serial.println(timeinfo->tm_min);
Serial.println(timeinfo->tm_hour);
Serial.println(timeinfo->tm_wday);
Serial.println(timeinfo->tm_mday);
Serial.println(timeinfo->tm_mon);
Serial.println(timeinfo->tm_year - 100);

  
  I2Cone.beginTransmission(CLOCK_ADDRESS);
  I2Cone.write(0x00);
  I2Cone.write(decToBcd(timeinfo->tm_sec)); //segundo
  I2Cone.write(decToBcd(timeinfo->tm_min)); //minuto
  I2Cone.write(decToBcd(timeinfo->tm_hour)); //hora
  I2Cone.write(decToBcd(timeinfo->tm_wday + 1)); //DoW (domingo = 1; segunda = 2)
  I2Cone.write(decToBcd(timeinfo->tm_mday)); //dia
  I2Cone.write(decToBcd(timeinfo->tm_mon + 1)); //mes
  I2Cone.write(decToBcd(timeinfo->tm_year - 100)); //ano
  I2Cone.endTransmission();
}

float getTemperature() {
  // Checks the internal thermometer on the DS3231 and returns the 
  // temperature as a floating-point value.

  // Updated / modified a tiny bit from "Coding Badly" and "Tri-Again"
  // http://forum.arduino.cc/index.php/topic,22301.0.html
  
  byte tMSB, tLSB;
  float temp3231;
  
  // temp registers (11h-12h) get updated automatically every 64s
  I2Cone.beginTransmission(CLOCK_ADDRESS);
  I2Cone.write(0x11);
  I2Cone.endTransmission();
  I2Cone.requestFrom(CLOCK_ADDRESS, 2);

  // Should I do more "if available" checks here?
  if(I2Cone.available()) {
    tMSB = I2Cone.read(); //2's complement int portion
    tLSB = I2Cone.read(); //fraction portion

    temp3231 = ((((short)tMSB << 8) | (short)tLSB) >> 6) / 4.0;
  }
  else {
    temp3231 = -9999; // Some obvious error value
  }
   
  return temp3231;
}

//---------------------------------------------------------------------------------

void setupPins(){
  //Inicializa o Wire nos pinos SDA e SCL padrões do ESP32
  I2Cone.begin(32,33,200000); // SDA pin 21, SCL pin 22, 200kHz frequency
  configurePort(IODIRA, OUTPUT);//Configura todos os pinos das duas portas do MCP23017 como saída
  configurePort(IODIRB, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  pinMode(36, INPUT);
  pinMode(39, INPUT);
  ledcAttachPin(12, 0); //azul
  ledcAttachPin(13, 1);
  ledcAttachPin(14, 2);
  ledcAttachPin(15, 3);
  ledcSetup(0, 1000, 10);
  ledcSetup(1, 1000, 10);
  ledcSetup(2, 1000, 10);
  ledcSetup(3, 1000, 10);
}

void configurePort(uint8_t port, uint8_t type) {
  if(type == INPUT) writeBlockData(port, 0xFF);
  else if(type == OUTPUT) writeBlockData(port, 0x00);
}

//muda o estado de um pino desejado
void digitalWriteMCP(int pin, int value) {
  uint8_t port;
  static uint8_t v;
 
  //de 0 a 7 porta A, de 8 a 15 porta B
  if(pin < 8){
    port = GPA;
  }else{
    port = GPB;
    pin -= 8;
  }
     
  if (value == LOW) v &= ~(B00000001 << (pin)); // muda o pino para LOW
  else if (value == HIGH) v |= (B00000001 << (pin)); // muda o pino para HIGH
 
  //Salva os valores dos bits da porta correspondente
 
  //envia os dados para o MCP
  writeBlockData(port, v);
}

//envia dados para o MCP23017 através do barramento i2c
void writeBlockData(uint8_t port, uint8_t data) {
  I2Cone.beginTransmission(MCP_ADDRESS);
  I2Cone.write(port);
  I2Cone.write(data);
  I2Cone.endTransmission();
  delay(10);
}
