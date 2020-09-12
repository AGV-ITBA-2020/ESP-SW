#include <ESP8266WiFi.h>

#define UART_BAUDRATE 9600
//#define FIXED_AGV_NUM 1


/* Variables para comunicación wifi*/
#define HB_TIME 5000 //Delay de milisegundos entre señales de Heartbeat
#define BUFSIZE 1024
const char* ssid = "TeleCentro-2df3";
const char* password = "QGNJDZYWEWMX";
WiFiClient client;
const uint16_t port = 12345;
const char * host = "192.168.0.102"; //192.168.98.1
String agvHeader= "AGV 1\n";



/*Variables de la máquina*/
enum ESPStates{
  IDLE,
  REC_CIAA_MSG,
  CONNECT_TO_SERVER,
  SEND_WIFI_MSG,
  REC_WIFI_MSG
};
byte uartBuffer[BUFSIZE],wifiBuffer[BUFSIZE];
unsigned int uartBytesCount=0;
unsigned int wifiBytesCount=0;
enum ESPStates ESPState;
long lastMillis=0;

void runStateMachine(void);
void blockingWifiConnect(void);
bool recSerialMsg(unsigned char * buf, unsigned int * msgLen);
bool recWifiMsg(unsigned char * buf, unsigned int * msgLen);
void resetVariables(void );
void ignoreUART(void );
void initRoutine(void);

void setup() {
  Serial.begin(UART_BAUDRATE); //UART con la que se comunica con el filtro
  Serial.setTimeout(10000);
  pinMode(5, OUTPUT); //Led para hacer pruebas
  WiFi.begin(ssid, password); //Conexión a la red
  blockingWifiConnect();
  lastMillis = millis();
  ESPState= IDLE;
  initRoutine();
}

void loop() {
  
  while(WiFi.status() == WL_CONNECTED)
      runStateMachine();
  Serial.write("Disconnected");
  blockingWifiConnect();
  resetVariables();

  
}

void initRoutine(void)
{
  #ifndef FIXED_AGV_NUM
    while(!Serial.available()); //Espera que serial le pase el número de AGV
    size_t len=Serial.readBytesUntil('\n', uartBuffer, BUFSIZE);
    uartBuffer[len]='\n';
    uartBuffer[len+1]=0;
    agvHeader= String((char *)uartBuffer);
  #endif
  String msg= agvHeader + "Online";
  client.connect(host, port);
  client.print(msg);
  client.stop();
}
void blockingWifiConnect(void)
{
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    digitalWrite(5, !digitalRead(5)); //Parpadea el led hasta que se conecta 
  }
  digitalWrite(5, 1);
  Serial.write('c');
}

void runStateMachine(void)
{
  String msg;
  switch(ESPState)
      {
        case IDLE:
          if (Serial.available() > 0 )
            ESPState=REC_CIAA_MSG;
          else if (millis() - lastMillis > HB_TIME)
            ESPState=CONNECT_TO_SERVER;
          break;
        case REC_CIAA_MSG:
          if (recSerialMsg(uartBuffer,&uartBytesCount))
            ESPState=CONNECT_TO_SERVER;
          break;
        case CONNECT_TO_SERVER:
          ignoreUART();
          if (client.connect(host, port))
            ESPState=SEND_WIFI_MSG;
          break;
        case SEND_WIFI_MSG:
          //String msg = uartBytesCount==0 ? String(agvHeader + "HB") : String(agvHeader + String(uartBuffer)); //Elige que mensaje enviar
          if (uartBytesCount==0)
            msg=agvHeader + "HB";
          else
            msg=agvHeader + String((char *)uartBuffer);
          ignoreUART();
          client.print(msg);
          ESPState=REC_WIFI_MSG;
          break;
        case REC_WIFI_MSG:
          ignoreUART();
          if (recWifiMsg(wifiBuffer,&wifiBytesCount))
          {
              Serial.write((char *)wifiBuffer,wifiBytesCount);
              Serial.flush();
              Serial.write(0);//Terminador
              Serial.flush();
          }
          resetVariables();
          break;
      }
}
bool recSerialMsg(unsigned char * buf, unsigned int * msgLen)
{
  bool finished=false;
  if (Serial.available() > 0 ) 
  { 
    byte c = Serial.read();
    buf[*msgLen]=c;
    (*msgLen)++;
    if (c == 0x00)
       finished=1;
    /*
    if (c == '\n')
    {
      buf[*msgLen]=0;
      finished=1;
    }*/
  }
  return finished;
}
bool recWifiMsg(unsigned char * buf, unsigned int * msgLen)
{
  bool recieved_anything=false;
  while(client.connected())
  {
    while(client.available() > 0 ) 
    { 
      recieved_anything=true;
      byte c = static_cast<char>(client.read());
      buf[*msgLen]=c;
      (*msgLen)++;
    }
  }
  return recieved_anything;
}
void resetVariables(void )
{
  ESPState=IDLE;
  lastMillis = millis();
  wifiBytesCount = 0;
  uartBytesCount = 0;
}
void ignoreUART(void )
{
  if (Serial.available() > 0 ) //Si llegan cosas de uart las va leyendo
  { 
     byte c = Serial.read();
     if (c == 0x00) //Cuando llega todo el mensaje le dice que está ocupado.
      Serial.write("Busy");
  }
}
