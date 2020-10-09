
#include <ESP8266WiFi.h>
#include "ArduinoQueue.h"

#define REC_QUEUE_MAX_SIZE 10

// Queue creation:
ArduinoQueue<String> recQueue(REC_QUEUE_MAX_SIZE);


#define UART_BAUDRATE 9600
//#define FIXED_AGV_NUM 1


/* Variables para comunicación wifi*/
#define HB_TIME 250 //Delay de milisegundos entre señales de Heartbeat
#define BUFSIZE 1500
const char* ssid = "Flia BD Network";//"TeleCentro-2df3";
const char* password = "muratureadrog";//"QGNJDZYWEWMX";
WiFiClient client;
const uint16_t port = 12345;
const char * host = "192.168.1.51"; //"192.168.0.102"
String agvHeader= "AGV 1\n";



/*Variables de la máquina*/
enum ESPStates{
  IDLE,
  CONNECT_TO_SERVER,
  SEND_WIFI_MSG,
  REC_WIFI_MSG,
  SEND_WIFI_MSG_UART
};
byte uartBuffer[BUFSIZE],wifiBuffer[BUFSIZE];
unsigned int uartBytesCount=0;
unsigned int wifiBytesCount=0;
unsigned int wifiBytesSent=0;
enum ESPStates ESPState;
long lastMillis=0;

void handshake(void);
void blockingWifiConnect(void);
void recUARTMsgs(unsigned char * buf, unsigned int * msgLen);
void runStateMachine(void);
bool recWifiMsg(unsigned char * buf, unsigned int * msgLen);
void resetVariables(void );
void initRoutine(void);
void printStatus(void);

void setup() {
  Serial.begin(UART_BAUDRATE); //UART con la que se comunica con el filtro
  Serial.setTimeout(60000);
  byte incomingByte;
  
  pinMode(5, OUTPUT); //Led para hacer pruebas
  //Serial.println("Conectando");
  while(Serial.available()<6);//Espera que Llegue "Reset\0"
  while(Serial.available())
     incomingByte = Serial.read();
  handshake();
  //Serial.println("Me conecté");
  
}

void loop() {
  
  while(WiFi.status() == WL_CONNECTED)
  {
      recUARTMsgs(uartBuffer, &uartBytesCount);
      runStateMachine();
  }
  Serial.write("Disconnected");
  blockingWifiConnect();
  
}
void handshake(void)
{
  blockingWifiConnect();
  resetVariables();
  WiFi.begin(ssid, password); //Conexión a la red
  blockingWifiConnect();
  lastMillis = millis();
  ESPState= IDLE;
  initRoutine();
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
  int status = WiFi.status();
  while(status != WL_CONNECTED)
  {
    delay(100);
    status = WiFi.status();
    //printStatus();
    digitalWrite(5, !digitalRead(5)); //Parpadea el led hasta que se conecta 
    //Serial.println("Intento conectar de vuelta");
    if(status == WL_CONNECT_FAILED)
       WiFi.begin(ssid, password); //Conexión a la red de vuelta
    
  }
  digitalWrite(5, 1);
  Serial.write('c');
}

void runStateMachine(void)
{
  String msg,aux;
  int sentCount;
  switch(ESPState)
      {
        case IDLE:
          if ((millis() - lastMillis > HB_TIME) || !recQueue.isEmpty())
            ESPState=CONNECT_TO_SERVER;
          break;
        case CONNECT_TO_SERVER:
          if (client.connect(host, port))
            ESPState=SEND_WIFI_MSG;
          break;
        case SEND_WIFI_MSG:
          if (recQueue.isEmpty())
            msg=agvHeader + "HB";
          else
          {
            aux=recQueue.dequeue();
            if(aux=="Reset")
              handshake();
            else
            {
              msg=agvHeader + aux;
              client.print(msg);
              ESPState=REC_WIFI_MSG;
            }
          }
            
          break;
        case REC_WIFI_MSG:
          //ignoreUART();
          recWifiMsg(wifiBuffer,&wifiBytesCount);
          ESPState=SEND_WIFI_MSG_UART;
          break;
        case SEND_WIFI_MSG_UART:
          if(wifiBytesSent<wifiBytesCount)
          {
            sentCount = Serial.write((char *)(wifiBuffer+wifiBytesSent),wifiBytesCount-wifiBytesSent);
            wifiBytesSent += sentCount;
          }
          else
          {  
            while(!Serial.availableForWrite());
            Serial.write(0);
            //Serial.flush();
          }
          resetVariables();
          break;
      }
}
void recUARTMsgs(unsigned char * buf, unsigned int * msgLen)
{
  while(Serial.available() > 0 ) 
  { 
    byte c = Serial.read();
    buf[*msgLen]=c;
    (*msgLen)++;
    if (c == 0x00)
    {
      recQueue.enqueue(String((char *)uartBuffer));
      *msgLen=0;
    }
  }
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
  wifiBytesSent = 0;
}
void printStatus() 
{
  int status=WiFi.status();
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connected");
  else if (WiFi.status() == WL_DISCONNECTED)
    Serial.println("Disconnected");
  else if (WiFi.status() == WL_CONNECT_FAILED)
    Serial.println("connect failed");
  else if (WiFi.status() == WL_CONNECTION_LOST)
    Serial.println("Connection lost");
  else if (WiFi.status() == WL_IDLE_STATUS)
    Serial.println("Idle");
}


void listNetworks() {
  // scan for nearby networks:
  Serial.println("** Scan Networks **");
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1)
  { 
    Serial.println("Couldn't get a wifi connection");
    while(true);
  } 

  // print the list of networks seen:
  Serial.print("number of available networks:");
  Serial.println(numSsid);

  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet<numSsid; thisNet++) {
    Serial.print(thisNet);
    Serial.print(") ");
    Serial.print(WiFi.SSID(thisNet));
    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm");
  }
}
