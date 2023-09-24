/************************************************************************************************************

                                      configurações Esp32
************************************************************************************************************/

                                   /* Arquivos de Bibliotecas */

 // ========================================================================================================
// --- Bibliotecas Auxiliares --- //

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#include "Config_mcu.h"


#define myAdress 0x07             // ESP32

//Declaracao das variaveis dos pinos dos sensores e botao
const int pinoDHT   = 12;

//Criacao da instancia DHT, em funcao do pino do sensor e do tipo do DHT
DHT dht(pinoDHT, DHT11);

  float temperatura = 0;
  float umidade = 0;

// Instância do DHT para comunicação com o sensor.
//DHT dht(DHTPIN, DHTTYPE);

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


}


void Comunicacao_dht ()
{


     	//Criamos duas variaveis locais para armazenar a temperatura e a umidade lidas
    float temperatura = dht.readTemperature();
    float umidade = dht.readHumidity();


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