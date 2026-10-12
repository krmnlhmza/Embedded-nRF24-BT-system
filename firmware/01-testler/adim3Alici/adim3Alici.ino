// ================== ADIM 3: HAVADAN BLINK - ALICI =======================
// Gelen 1 byte'i okur, D4'teki LED'i surer.
//
// BAGLANTI: GND->GND  VCC->3.3V  CE->D9  CSN->D10  SCK->D13  MOSI->D11  MISO->D12
//           LED: D4 -> 220 ohm -> LED UZUN bacak,  KISA bacak -> GND
// Serial Monitor: 115200
// ======================================================================

#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);                  // CE, CSN

const uint8_t ADRES[6] = "LED01";   // verici ile AYNI
const uint8_t KANAL    = 108;       // verici ile AYNI

const int LED_PIN = 4;

uint8_t ledDurum = 0;
unsigned long sayac = 0;
unsigned long sonHaber = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { }
  Serial.println(F("\n=== ADIM 3 ALICI ==="));

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  if (!radio.begin()) {
    Serial.println(F("HATA: nRF24 cevap vermiyor."));
    Serial.println(F("CE=D9 CSN=D10 SCK=D13 MOSI=D11 MISO=D12, VCC=3.3V, GND ortak?"));
    while (1) { }
  }
  Serial.println(F("Modul bulundu."));

  // --- VERICI ILE BIREBIR AYNI UC SATIR ---
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(KANAL);

  radio.openReadingPipe(1, ADRES);   // 0 degil 1
  radio.startListening();            // en sonda: alici moduna gec

  Serial.print(F("Kanal: "));   Serial.println(radio.getChannel());
  Serial.println(F("Dinliyorum..."));
}

void loop() {
  if (radio.available()) {
    radio.read(&ledDurum, sizeof(ledDurum));
    digitalWrite(LED_PIN, ledDurum ? HIGH : LOW);

    sayac++;
    Serial.print(F("geldi -> LED "));  Serial.print(ledDurum);
    Serial.print(F("   toplam="));     Serial.println(sayac);
    sonHaber = millis();
  }

  // Paket gelmiyorsa da hayatta oldugunu goster: ekran bos kalmasin.
  if (millis() - sonHaber >= 2000) {
    sonHaber = millis();
    Serial.println(F("(paket yok, dinlemeye devam)"));
  }
}
