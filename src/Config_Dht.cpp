/************************************************************************************************************

                                      configurações Esp32
************************************************************************************************************/

                                   /* Arquivos de Bibliotecas */

 // ========================================================================================================
// --- Bibliotecas Auxiliares --- //

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#include "Config_mcu.h"


#define myAdress 0x07             // ESP32

//Declaracao das variaveis dos pinos dos sensores e botao

const int pinoLED   = 13;
const int pinoBotao =  4;
const int RelePin1  = 33;



const int pinoDHT   = 12;



//Criacao da instancia DHT, em funcao do pino do sensor e do tipo do DHT
DHT dht(pinoDHT, DHT11);





  float temperatura = 0;
  float umidade = 0;

// Pino onde está conectado o sensor DHT11/DHT22.
#define DHTPIN 12
// Tipo de módulo DHT em uso.
#define DHTTYPE DHT11
// De quanto em quanto tempo (ms) deve ser feita a leitura do sensor.
#define READ_INTERVAL 2000


// Pino onde está conectado o LED de status.
#define LED_PIN 2

// UUID do serviço que contém as características de temperatura e umidade.
#define SERVICE_UUID "c467ea89-16b0-4314-a6e7-9ef52dfcb489"
// UUID da característica de temperatura.
#define TEMPERATURE_UUID "c467ea89-16b0-4314-a6e7-9ef52dfcb499"
// UUID da característica de umidade.
#define HUMIDITY_UUID "c467ea89-16b0-4314-a6e7-9ef52dfcb589"

// O servidor BLE, ou seja, o periférico que estamos fornecendo (configurado em setup()).
BLEServer *server = NULL;
// Característica de temperatura (configurada em setup()).
BLECharacteristic *temperatureChar = NULL;
// Característica de umidade (configurada em setup()).
BLECharacteristic *humidityChar = NULL;


// ========================================================================================================
// --- variaveis 

//Declaracao das variaveis dos pinos dos sensores e botao
const int pinoLDR   = 15;

//Declaracao das variaveis que receberao a leitura do botao e sensor de luz
int estadoBotao = 0;
int valorLDR = 0;

//Cria a variavel que salvara o estado atual do botao
byte flag = 0;




// Número de centrais atualmente contectadas ao periférico.
// É utilizado para piscar o LED de status quando não houver nenhuma central conectada
// e para otimizar o envio de notificações via BLE, apenas quando há alguma central conectada.
int devicesConnected = 0;

// Valor de millis() na última vez em que o estado do LED de status foi alterado.
int blinkMillis = 0;
// Valor de millis() na última vez em que foi feita a leitura do sensor DHT.
int readMillis = 0;

// Callbacks para conexão e desconexão de centrais.
class ServerCallbacks: public BLEServerCallbacks {
  
  // Chamado quando uma central se conecta ao periférico.
  void onConnect(BLEServer *s) {
    Serial.println("[+] Device connected");

    // Incrementa o número de dispositivos conectados.
    devicesConnected++;
	
    // Reinicia o advertising para que mais centrais possam descobrir o periférico e se conectar a ele.
	// Do contrário, apenas uma central conseguiria se conectar ao periférico.
    BLEDevice::startAdvertising();
  }

  // Chamado quando uma central se desconecta do periférico.
  void onDisconnect(BLEServer *s) {
    Serial.println("[-] Device disconnected");

    // Decrementa o número de dispositivos conectados.
    devicesConnected--;
  }
};


// ========================================================================================================
// --- variaveis da temperatura
byte byte1, byte2, byte3, byte4;
unsigned int aux_temp;


// ========================================================================================================
// --- variaveis da umidade 
byte byte5, byte6, byte7, byte8;
unsigned int aux_umid;


void requestEvent_Dht()
{


  //envia o valor de temperatura
     
      Wire.write(byte1);                    // Envia os bytes do número antes da vírgua e depois da vírgula
      Wire.write(byte2);                      // temperatura
      Wire.write(byte3);
      Wire.write(byte4); 
      Wire.write(byte5);                    // Envia os bytes do número antes da vírgua e depois da vírgula
      Wire.write(byte6);                    // umidade
      Wire.write(byte7);
      Wire.write(byte8);  



}





void Config_dht ()
{

 

  //Abrimos uma comunicacao serial para imprimir dados no Monitor Serial
  Serial.begin(115200);

  
  //Inicializamos nosso sensor DHT11
  dht.begin();

Serial.println("Initializing...");

 
  // Configura o LED de status como saída.
  pinMode(LED_PIN, OUTPUT);

 

  // Inicializa o dispositivo Bluetooth Low Energy.
  // IMPORTANTE: O nome passado aqui não pode ser maior do que estes 5 caracteres, do contrário podem haver
  // problemas com o advertising do SERVICE_UUID, impedindo a conexão da central.
  // Este nome pode ser passado em branco, mas é útil termos um nome para debug utilizando o LightBlue ou nRF Connect.
  BLEDevice::init("ESP32");

  // Cria um server BLE (periférico).
  server = BLEDevice::createServer();
  // Configura o periférico para utilizar a classe de callbacks que definimos acima.
  server->setCallbacks(new ServerCallbacks());

  // Cria o service que irá encapsular nossas características de temperatura e umidade.
  BLEService *service = server->createService(SERVICE_UUID);

  // Adiciona a característica de temperatura.
  temperatureChar = service->createCharacteristic(
    TEMPERATURE_UUID, // UUID que nós definimos para a característica de temperatura.
    BLECharacteristic::PROPERTY_READ | // Indica que esta característica pode ser lida pela central.
    BLECharacteristic::PROPERTY_NOTIFY // Indica que a central pode assinar notificações sobre esta característica.
  );

  // Adiciona a característica de umidade.
  humidityChar = service->createCharacteristic(
    HUMIDITY_UUID, // UUID que nós definimos para a característica de umidade.
    BLECharacteristic::PROPERTY_READ | // Indica que esta característica pode ser lida pela central.
    BLECharacteristic::PROPERTY_NOTIFY // Indica que a central pode assinar notificações sobre esta característica.
  );

  // Configura as características para que sigam o padrão BLE2902.
  // Isto não é obrigatório, mas torna nosso periférico mais compatível.
  temperatureChar->addDescriptor(new BLE2902());
  humidityChar->addDescriptor(new BLE2902());

  // Torna nosso serviço disponível no periférico.
  service->start();

  // Obtém um advertising, que utilizaremos para divulgar nosso periférico e seu serviço para centrais próximas.
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  // Incluí o UUID do nosso serviço no advertisement, para que centrais possam filtrar seu scan com base nele.
  advertising->addServiceUUID(SERVICE_UUID);
  // Indica que não suportamos scan response.
  advertising->setScanResponse(false);
  // Configura o intervalo mínimo de advertisement, o valor 0x06 é compatível com a maioria dos dispositivos.
  advertising->setMinPreferred(0x06);

  // Começa o advertising do periférico, de modo que centrais próximas possam encontrá-lo.
  BLEDevice::startAdvertising();

  Serial.println("Advertising");



  


                 pinMode(33, OUTPUT);                // configura digital 33 como saida
          digitalWrite(33,   HIGH);                // pino configurado nivel baixo


  
  // configura o pino do botao como entrada com resistor de pullup interno
  pinMode(pinoBotao, INPUT_PULLUP);
      digitalWrite(pinoBotao, LOW);
 
  

  //Informa que o pino do LED eh saida e o pino botao eh entrada
        pinMode(pinoLED,   OUTPUT);
        digitalWrite(pinoLED, LOW);

}


// Valor mais recente de temperatura.
float lastTemperature = -999;
// Valor mais recente de umidade.
float lastHumidity = -999;

// Realiza a leitura do sensor DHT, valida as informações e transmite através do Bluetooth Low Energy
// caso os valores tenham mudado desde a última leitura e caso haja alguma central conectada ao periférico.
void sense()
{
  // Leitura do sensor DHT.
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Validação do valor de umidade (returna NaN em caso de falha na leitura).
  if (isnan(humidity)) {
    Serial.println("Humidity reading failed!");
    return;
  }

  // Validação do valor de temperatura (returna NaN em caso de falha na leitura).
  if (isnan(temperature)) {
    Serial.println("Temperature reading failed!");
    return;
  }

  Serial.printf("Humidity = %f | Temperature: %f C \n", humidity, temperature);

  // Se houver uma central conectada e se a leitura de temperatura for diferente da leitura anterior.
  if (devicesConnected && (temperature != lastTemperature)) {
    Serial.println("Notifying temperature");

	// Armazena a nova leitura.
    lastTemperature = temperature;

    // Atualiza a característica de temperatura com o novo valor.
    temperatureChar->setValue(temperature);
	// Notifica as centrais conectadas de que o valor da característica de temperatura foi alterado.
    temperatureChar->notify();

    delay(3);
  }

  // Se houver uma central conectada e se a leitura de umidade for diferente da leitura anterior.
  if (devicesConnected && (humidity != lastHumidity)) {
    Serial.println("Notifying humidity");

	// Armazena a nova leitura.
    lastHumidity = humidity;
	
    // Atualiza a característica de umidade com o novo valor.
    humidityChar->setValue(humidity);
	// Notifica as centrais conectadas de que o valor da característica de umidade foi alterado.
    humidityChar->notify();

    delay(3);
  }
}





void Comunicacao_dht ()
{


     	//Criamos duas variaveis locais para armazenar a temperatura e a umidade lidas
    float temperatura = dht.readTemperature();
    float umidade = dht.readHumidity();


    
  // Chama a função sense() para efetuar a leitura do sensor a cada READ_INTERVAL (2 segundos por padrão).
  if (readMillis == 0 || (millis() - readMillis) >= READ_INTERVAL) {
    sense();
    readMillis = millis();
  }

  // Pisca o LED de status se não houver nenhuma central conectada.
  if (!devicesConnected) {
    if (blinkMillis == 0 || (millis() - blinkMillis) >= 1000) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      blinkMillis = millis();
    }
  } else {
    // Mantém o LED de status sempre ligado quando há uma um mais centrais conectadas.
    digitalWrite(LED_PIN, HIGH);
  }

  aux_temp = (unsigned int) temperatura;  // aux = 46689, Pega somente a parte inteira da variável float (0 - 65536)
  byte2 = aux_temp;                          // byte2 = 0B01100001, pega apenas os primeros 8 bits
  byte1 = (aux_temp>>8);                     // byte1 = 0B10110110, pega os 8 ultimos bits
  
  // Ajustando o número depois da vírgula
  temperatura -= aux_temp;                // Deixa apenas o número depois da vírgula
  temperatura *= 10000;              // Multiplica por 10k para pegar 4 dígitos após a vírgula
  aux_temp = (unsigned int) temperatura;  // Pega somente o valor antes da vírgula
  byte4 = aux_temp;                          // byte2 = 0B00101110, pega apenas os primeros 8 bits
  byte3 = (aux_temp>>8);                     // byte1 = 0B00100010, pega os 8 ultimos bits

 



  aux_umid = (unsigned int) umidade;  // aux = 46689, Pega somente a parte inteira da variável float (0 - 65536)
  byte6 = aux_umid;                          // byte2 = 0B01100001, pega apenas os primeros 8 bits
  byte5 = (aux_umid>>8);                     // byte1 = 0B10110110, pega os 8 ultimos bits
  
  // Ajustando o número depois da vírgula
  umidade -= aux_umid;                     // Deixa apenas o número depois da vírgula
  umidade *= 10000;                         // Multiplica por 10k para pegar 4 dígitos após a vírgula
  aux_umid = (unsigned int) umidade;  // Pega somente o valor antes da vírgula
  byte8 = aux_umid;                          // byte2 = 0B00101110, pega apenas os primeros 8 bits
  byte7 = (aux_umid>>8);                     // byte1 = 0B00100010, pega os 8 ultimos bits

Wire.onRequest(requestEvent_Dht);
Wire.endTransmission();                          // pare de transmitir 

}