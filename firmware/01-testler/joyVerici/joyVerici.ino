#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);                  // CE, CSN

const uint8_t ADRES[6] = "LED01";   // alici ile AYNI
const uint8_t KANAL    = 108;       // alici ile AYNI

uint8_t ledDurum = 0;
unsigned long gonderilen = 0, basarili = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { }
  Serial.println(F("\n=== ADIM 3 VERICI ==="));

  if (!radio.begin()) {
    Serial.println(F("HATA: nRF24 cevap vermiyor."));
    Serial.println(F("CE=D9 CSN=D10 SCK=D13 MOSI=D11 MISO=D12, VCC=3.3V, GND ortak?"));
    while (1) { }
  }
  Serial.println(F("Modul bulundu."));

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(KANAL);

  radio.openWritingPipe(ADRES);
  radio.stopListening();

  Serial.print(F("Kanal: "));   Serial.println(radio.getChannel());
  Serial.println(F("Gonderiyorum..."));
}

void loop() {
  ledDurum = !ledDurum;

  bool ack = radio.write(&ledDurum, sizeof(ledDurum));

  gonderilen++;
  if (ack) basarili++;

  Serial.print(F("gonderilen=")); Serial.print(ledDurum);
  Serial.print(F("  ack="));      Serial.print(ack ? "OK " : "YOK");
  Serial.print(F("  basari="));   Serial.print(basarili);
  Serial.print(F("/"));           Serial.println(gonderilen);

  delay(500);
}