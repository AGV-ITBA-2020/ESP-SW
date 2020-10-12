#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "ArduinoQueue.h"

#define QUEUE_MAX_SIZE 10
#define AUX_BUF_SIZE 1024
#define HEADER_MAX_SIZE 30
#define HB_TIME 3000 //Delay de milisegundos entre señales de Heartbeat

#define TERMINATOR 0 // \n para el debug con arduino, 0 para la ciaa


char auxRecievedArrayMQTT[AUX_BUF_SIZE],fromCIAABuffer[AUX_BUF_SIZE],toCIAABuffer[AUX_BUF_SIZE]; //Arrays auxiliares
ArduinoQueue<String> toCIAAQueue(QUEUE_MAX_SIZE); //Queue de mensajes recibidos
String agvHeader,currMsgToCIAA; //Header para todos los mensajes enviados
unsigned int fromCIAABytesCount=0,toCIAABytesCount=0; //Counts para arrays auxiliares
long lastMillis=0;
bool sendingToCIAA=0;
/********************Datos de conexión de internet**************************/
const char* ssid = "TeleCentro-a01d";        // Poner aquí la SSID de la WiFi
const char* password = "ZWMXGZM2JZ2D";  // Poner aquí el passwd de la WiFi
const char* mqtt_server = "192.168.0.52";
WiFiClient espClient;
PubSubClient client(espClient);

/********************Funciones internas**************************/
void setup_wifi(void);
void callback(char* topic, byte* payload, unsigned int length);
void reconnect(void); 
void handshake(void);

void sendToCIAAA(void);
void recieveFromCIAA(void);
void HBLogic(void);

/********************Setup y loop**************************/
void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  digitalWrite(BUILTIN_LED, HIGH); //Usamos el LED para indicar que se conectó a internet
  Serial.setTimeout(60000);
  setup_wifi();
  client.setServer(mqtt_server, 1883); //Conección al mqtt server
  client.setCallback(callback); 
  handshake();  //Handshake en el que la CIAA le dice el nro de AGV que es
  lastMillis = millis();
}


void loop() { //Loop principal 
  if (!client.connected()) { //Si se desconectó, se conecta de vuelta
    digitalWrite(BUILTIN_LED, HIGH);
    reconnect();
  }
  client.loop();  //Checkea nuevos msgs recibidos
  sendToCIAAA();  //Si tiene mensajes para la CIAA, llena el buffer de serial tx con el mensaje (no bloqueante)
  recieveFromCIAA(); //Lee los bytes en la cola del serial Rx (No bloqueante)
  HBLogic();      //Si pasó tanto tiempo, publica el heartbeat
}

/********************Funciones de conección**************************/
void setup_wifi() { //Conexión al Wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
    delay(500);
  digitalWrite(BUILTIN_LED, LOW);
}

void callback(char* topic, byte* payload, unsigned int length) { //Al recibir un msj del topic del AGV
  for (int i = 0; i < length; i++) 
    auxRecievedArrayMQTT[i]=payload[i];
  auxRecievedArrayMQTT[length]=0;
  toCIAAQueue.enqueue(String(auxRecievedArrayMQTT)); //Encola el mensaje recibido
}

void reconnect() {
  while (!client.connected()) {
    String clientId = "ESP8266Client-"; // Create a random client ID
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())){
      client.subscribe(agvHeader.c_str());// ... and resubscribe
    }
    else 
      delay(1000); // Wait 1 second before retrying
  }
}

void handshake(void)
{
  char headerArray[HEADER_MAX_SIZE];
  Serial.write('c'); //Le indica a la CIAA que está conectado, de manera que la ciaa le responda con el header que tiene que usar
  while(!Serial.available()); //Espera que serial le pase el número de AGV
  size_t len=Serial.readBytesUntil(TERMINATOR, headerArray, HEADER_MAX_SIZE); // 00 para la ciaa, \t para debuggear con el serial de arduino
  headerArray[len]=0;
  agvHeader = String((char *)headerArray); // Garda en el string global el header que le mandó la CIAA
  Serial.println("Llegué al header: " + agvHeader);
  String clientId = "ESP8266Client-"; // Create a random client ID
  clientId += String(random(0xffff), HEX);
  client.connect(clientId.c_str());
  client.subscribe(agvHeader.c_str());
  client.publish("Houston", (agvHeader + "\nOnline").c_str()); // Comunica a Houston que se encuentra en linea
}

/********************Funciones de rutina**************************/
void recieveFromCIAA(void)
{
  String auxStr;
  while(Serial.available() > 0 ) //Si hay bytes en la fifo del serial, los lee y los guarda
  { 
    byte c = Serial.read();
    fromCIAABuffer[fromCIAABytesCount++]=c; //Guarda el caracter recibido en un buffer
    if (c == TERMINATOR) //En el caso que termina un string
    {
      //fromCIAABuffer[fromCIAABytesCount-1]=0; //Terminador de string para ver el string recibido. Solo es necesario cuando se debuggea con el serial
      auxStr = String((char *)fromCIAABuffer);
      if(auxStr == "Reset"){ //Si es un reset, vuelve a la rutina del principio
        while(Serial.available())
          Serial.read(); //Vacía el buffer si había otras cosas
        handshake(); //Comienza el handshake del principio.
      }
      else
        client.publish("Houston", (agvHeader + "\n" +auxStr).c_str()); //Sino, envía a houston el mensaje que recibió
      fromCIAABytesCount=0;
      lastMillis = millis();
      break; //Sale del while
    }
  }
}
void sendToCIAAA(void)
{
  if(sendingToCIAA) //Si estaba mandando un mensaje a la CIAA
  {
    int freeBuf = Serial.availableForWrite(); //Espacio libre en la fifo tx de serial
    int bytesLeft = currMsgToCIAA.length() - toCIAABytesCount; //Bytes que todavía tiene que mandar de este mensaje
    if(bytesLeft > 0)
    {
      int bytesToSend = (bytesLeft >= freeBuf) ? freeBuf : bytesLeft; //Si tiene que mandar más de lo que puede poner en el buffer, lo limita para que no sea bloqueante
      Serial.write(currMsgToCIAA.substring(toCIAABytesCount).c_str(),bytesToSend); //Llena el FIFO
      toCIAABytesCount+=bytesToSend;
    }
    else if(bytesLeft == 0) //Si terminó de escribir todo el mensaje
    {
      while(!Serial.availableForWrite()); //Cuando haya espacio en la FIFO
      Serial.write(TERMINATOR); //Pone el terminador
      sendingToCIAA=0;
    }
    
      
  }
  else if(!toCIAAQueue.isEmpty()) //Si hay mensajes para mandar
  {
    currMsgToCIAA=toCIAAQueue.dequeue(); //Pullea el mensaje más viejo y resetea todo para enviarlo
    toCIAABytesCount=0;
    sendingToCIAA=true;
  }
    
}
void HBLogic(void)
{
  if((millis() - lastMillis) > HB_TIME)//Si pasó tanto tiempo desde el último mensaje, manda un heartbeat
  {
    client.publish("Houston", (agvHeader + "\nHB").c_str());
    lastMillis = millis(); //Actualiza el tiempo en el que se envió el último paquete
  }
}
