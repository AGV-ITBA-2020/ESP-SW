#include <ESP8266WiFi.h>

const char* ssid = "TeleCentro-2df3";
const char* password = "QGNJDZYWEWMX";

WiFiClient client1;
const uint16_t port = 12345;
const char * host = "192.168.0.7";

void setup() {
  Serial.begin(9600);
  delay(10);

  //Configuración  del GPIO2
  pinMode(5, OUTPUT);
  digitalWrite(5,LOW);
  
  
  Serial.println();
  Serial.println();
  Serial.print("Conectandose a red : ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password); //Conexión a la red
  
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado");
  
  Serial.println("Esta es mi IP:");
  Serial.println(WiFi.localIP()); //Obtenemos la IP
}

void loop() {
    WiFiClient client;
    if (!client.connect(host, port)) {
        Serial.println("Connection to host failed");
        delay(1000);
        return;
    }
    Serial.println("Connected to server successful!");
    client.print("Hello from ESP32!");
    Serial.println("Disconnecting...");
    client.stop();
 
    delay(10000);
}
