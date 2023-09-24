/************************************************************************************************************

                                      configurações Esp32
************************************************************************************************************/

                                   /* Arquivos de Bibliotecas */

 // ========================================================================================================
// --- Bibliotecas Auxiliares --- //

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "wire.h"



#include "Config_mcu.h"
#include "Config_Dht.h"

/**********************************************************************************************************/
                                        /* Arquivos de inclusão */




#define myAdress 0x07             // ESP32


const int LED_ESP32 = 2;

#define ReleledPin 32 // numero do pino onde o Rele esta conectado


// funcao executada sempre que algum dado e recebido no barramento I2C
// vide "void setup()"
void receiveEvent(int howMany) {
  // verifica se existem dados para serem lidos no barramento I2C
  if (Wire.available()) {
    // le o byte recebido
    char received = Wire.read();

    // se o byte recebido for igual a 0, apaga o LED
    if (received == 0) {
      digitalWrite(ReleledPin, LOW);
    }

    // se o byte recebido for igual a 1 acende o LED
    if (received == 1) {
      digitalWrite(ReleledPin, HIGH);
    }
  }
}


void Config_mcu() {
  


 
  
 
  Serial.begin(115200);


 

  Wire.begin((uint8_t)myAdress);
  // ingressa ao barramento I2C com o endereço definido no myAdress (0x07)


  Config_dht();

  pinMode (LED_ESP32, OUTPUT);        // configura o pino do LED 2 como saida
  digitalWrite(LED_ESP32,LOW);

  pinMode(ReleledPin, OUTPUT);  // configura o pino do LED como saida
  digitalWrite(ReleledPin,HIGH);

  //Registra um evento para ser chamado quando chegar algum dado via I2C
  Wire.onReceive(receiveEvent);


}

void Supervisionamento() {

  //Acende o LED durante 1 segundo
  digitalWrite(LED_ESP32, HIGH);
  delay(1000);

  //Apaga o LED durante 1 segundo
  digitalWrite(LED_ESP32, LOW);
  delay(1000);

}