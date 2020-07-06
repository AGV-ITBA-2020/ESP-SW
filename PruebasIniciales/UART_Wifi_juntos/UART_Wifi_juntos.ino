#include <ESP8266WiFi.h>

/* Variables para comunicación wifi*/
const char* ssid = "TeleCentro-2df3";
const char* password = "QGNJDZYWEWMX";
WiFiClient client1;
const uint16_t port = 12345;
const char * host = "192.168.0.7";


/*  */
#define HB_TIME 2000 //Delay de milisegundos entre señales de Heartbeat
#define BUFSIZE 1024
long lastMillis = 0;
byte recBuffer[BUFSIZE];
unsigned int recBytesCount=0;
byte wifiBuffer[BUFSIZE];
unsigned int wifiBytesCount=0;
boolean msg_rcv=0;
String agvHeader= "AGV 1\n";

void setup() {
  Serial.begin(9600); //UART con la que se comunica con el filtro
  pinMode(5, OUTPUT); //Led para hacer pruebas

  WiFi.begin(ssid, password); //Conexión a la red
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(200);
    digitalWrite(5, !digitalRead(5)); //Parpadea el led hasta que se conecta 
  }
  //Asumimos en este código que la conexión con el wifi no se cae.
  lastMillis = millis();
}

void loop() {
  long currentMillis = millis();
  unsigned int dt = currentMillis - lastMillis;
  WiFiClient client;
  if(dt > HB_TIME) //Cada tanto tiempo
  {
    Serial.write('?');//Le pregunta al micro si tiene algo para mandar
    do{
      if (Serial.available() > 0 ) { //Con prioridad lee si llegó algo por uart y lo guarda
        byte c = Serial.read();
        recBuffer[recBytesCount]=c;
        recBytesCount++;
        if (c == 0x00){ 
          msg_rcv=1;
        }
      }
    }while(msg_rcv==0);

    while (client.connect(host, port)); //Se conecta con houston
    
    if(recBytesCount==1)
      client.println(agvHeader + "HB");       //Si el micro no tenía nada, manda un hearbeat
    else
      client.println(agvHeader + (char*)recBuffer); //Sino, manda mensaje del micro
     
    while (client.available() == 0); //Espera hasta que llegue la respuesta
    while (client.available()) {
        byte c = static_cast<char>(client.read());
        wifiBuffer[wifiBytesCount]=c;
        wifiBytesCount++;
    }
    Serial.write((char*) wifiBuffer); //Manda al micro lo que se recibió por wifi
    msg_rcv=0;
    recBytesCount=0;
    do{
      if (Serial.available() > 0 ) { //Con prioridad lee si llegó algo por uart y lo guarda
        byte c = Serial.read();
        recBuffer[recBytesCount]=c;
        recBytesCount++;
        if (c == 0x00){ 
          msg_rcv=1;
        }
      }
    }while(msg_rcv==0);

    client.println(agvHeader + (char*)recBuffer); //Sino, manda mensaje del micro
    client.stop();
    
    recBytesCount=0;
    msg_rcv=0;
  }
}
