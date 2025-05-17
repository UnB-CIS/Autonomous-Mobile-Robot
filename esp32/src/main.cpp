#include <WiFi.h>
#include <WiFiUdp.h>
#include <env.h>

const char* ssid = WIFI_SSID; // Nome da sua rede Wi-Fi
const char* password = WIFI_PASSWORD;  // Senha da sua rede

// IP e porta do CoppeliaSim (sua máquina)
const char* coppeliaIP = COPPELIA_IP;  // Substitua pelo IP da máquina com CoppeliaSim
const int coppeliaPort = COPPELIA_PORT;

// Porta local para receber a posição do robô
const int localPort = 20000;

WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  // Aguarda a conexão Wi-Fi
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");

  // Inicia o socket UDP para escutar na porta local (para receber posição)
  udp.begin(localPort);
  Serial.print("Escutando na porta UDP ");
  Serial.println(localPort);

  Serial.println("\nWiFi conectado!");
  Serial.print("IP do ESP32: ");
  Serial.println(WiFi.localIP());  // Exibe o IP do ESP32 no monitor serial
}

void loop() {
  // Enviando comando de velocidade para o CoppeliaSim
  String command = "1.0,2.0";  // Exemplo: velocidade fixa para ambas as rodas
  udp.beginPacket(coppeliaIP, coppeliaPort);
  udp.print(command);
  udp.endPacket();

  // Verifica se recebeu dados de posição
  int packetSize = udp.parsePacket();
  if(packetSize) {
    char packetBuffer[255];  // Buffer para armazenar os dados recebidos
    int len = udp.read(packetBuffer, 255);
    if(len > 0) {
      packetBuffer[len] = 0;  // Finaliza a string
    }
    Serial.print("Posição do robô: ");
    Serial.println(packetBuffer);
  }

  delay(100);  // Aguarda um pouco antes da próxima iteração
}
