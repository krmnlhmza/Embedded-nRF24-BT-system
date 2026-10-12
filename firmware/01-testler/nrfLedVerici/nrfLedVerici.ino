// ================= ADIM 1: VERICI (LED KOMUTU GONDEREN) =================
// Bu kart "kumanda"nin en basit hali. Iki yoldan komut alir:
//   (a) Serial Monitor'a  1 / 0 / t  yazarsin   (1=yak, 0=sondur, t=degistir)
//   (b) D2'ye takili butona basarsin            (opsiyonel, yoksa da calisir)
// Komutu nRF24 ile karsi karta yollar, karsi kartin cevabini ekrana basar.
//
// NEDEN BOYLE OGRENIYORUZ: arabadaki vericiNrf.ino ile birebir ayni iskelet
// - ayni adres/kanal mantigi, ayni struct paket, ayni ACK payload cevabi.
// Sadece "joystick + motor hizi" yerine "buton + LED" var. Burada calisan
// mantigi arabaya tasimak sadece struct icerigini degistirmek demek.
//
// GEREKEN KUTUPHANE: "RF24" by TMRh20
//
// BAGLANTILAR (Uno):
//   nRF24: GND->GND  VCC->3.3V  CE->D9  CSN->D10  SCK->D13  MOSI->D11  MISO->D12
//   Buton (opsiyonel): bir bacak D2, diger bacak GND. Direnc gerekmez
//                      (dahili pull-up kullaniyoruz).
//   Durum LED'i (opsiyonel): D4 -> 220 ohm -> LED anot(uzun bacak),
//                            LED katot(kisa bacak) -> GND
//                            YANIK = karsi kart cevap veriyor (link var)
//
// DIKKAT: Uno'nun kart uzerindeki LED'i D13'te ve D13 = SCK. SPI trafigi
// yuzunden surekli titrer, bu NORMAL. Gostergeler icin D13 kullanma.
// ======================================================================

#include <SPI.h>
#include <RF24.h>

// --------------------------- RADYO AYARLARI ---------------------------
const uint8_t CE_PIN  = 9;
const uint8_t CSN_PIN = 10;
RF24 radio(CE_PIN, CSN_PIN);

// Adres: 5 byte. Alici ile BIREBIR AYNI olmali. Iki nRF24 birbirini
// ancak ayni adres + ayni kanal + ayni hizda bulur.
const uint8_t ADRES[6] = "LED01";

// Kanal 0-125, her biri 1 MHz. 108 -> 2508 MHz, WiFi bandinin uzerinde.
const uint8_t KANAL = 108;

// --- Gonderilen paket: alici ile BIREBIR AYNI struct olmali ---
// __attribute__((packed)): derleyici araya bosluk byte'i koymasin diye.
// Iki kartta yapi ayni olmazsa sayilar birbirine karisir.
struct __attribute__((packed)) Paket {
  uint8_t led;   // 0 = sondur, 1 = yak
  uint8_t seq;   // sira numarasi, sadece "paket akiyor mu" gormek icin
};

// --- Alicidan ACK'in ICINDE gelen cevap: alici ile BIREBIR AYNI ---
struct __attribute__((packed)) Cevap {
  uint8_t  ledDurum;   // alici LED'i gercekte hangi durumda
  uint16_t sayac;      // alici acildigindan beri kac paket aldi
};

// --- Pinler ---
const int BUTON_PIN = 2;
const int LINK_LED  = 4;

// --- Durum ---
uint8_t ledIstek = 0;      // gondermek istedigimiz LED durumu
uint8_t seqNo    = 0;
Cevap   sonCevap = {0, 0};

bool          butonOnceki  = HIGH;
unsigned long butonZaman   = 0;
const unsigned long DEBOUNCE = 50;

unsigned long sonGonderim = 0;
const unsigned long GONDER_PERIYOT = 100;   // saniyede 10 paket

unsigned long sonAck = 0;
bool baglantiVar = false;
const unsigned long ACK_TIMEOUT = 500;

// ----------------------------------------------------------------------
// Paketi gonder. Donus degeri cok kiymetli:
//   true  = karsi kart paketi ALDI ve donanim ACK'i yolladi
//   false = 5 denemede de cevap yok (kart kapali / menzil disi / parazit)
// Yani "baglanti var mi" sorusunu her pakette aninda cevapliyoruz.
// ----------------------------------------------------------------------
bool gonder(uint8_t durum) {
  Paket p;
  p.led = durum;
  p.seq = seqNo++;

  bool ok = radio.write(&p, sizeof(p));

  // Alici ACK'in icine veri koyduysa burada okunur. Ayri bir gonderim yok:
  // tek el sikismada hem komut gitti hem cevap geldi.
  if (ok && radio.isAckPayloadAvailable()) {
    radio.read(&sonCevap, sizeof(sonCevap));
  }
  return ok;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  pinMode(BUTON_PIN, INPUT_PULLUP);   // buton yoksa pin HIGH kalir, sorun olmaz
  pinMode(LINK_LED, OUTPUT);
  digitalWrite(LINK_LED, LOW);

  Serial.println(F("\n=== VERICI ==="));

  if (!radio.begin()) {
    Serial.println(F("nRF24 BULUNAMADI. Once nrfKontrol sketch'ini calistir."));
    while (1) { }
  }

  // --------- AYARLARIN HEPSI ALICI ILE AYNI OLMAK ZORUNDA ---------
  radio.setPALevel(RF24_PA_LOW);       // masa ustu testi: DUSUK GUC.
                                       // PA+LNA modulde MAX ~115 mA ceker ve
                                       // Uno'nun 3.3V regulatoru (50 mA) coker.
                                       // Menzil testine gecince MAX yaparsin.
  radio.setDataRate(RF24_250KBPS);     // dusuk hiz = alicinin duyarliligi yuksek
                                       // = uzun menzil. Verimiz zaten 2 byte.
  radio.setChannel(KANAL);
  radio.setCRCLength(RF24_CRC_16);     // 2 byte donanim CRC: bozuk paket bize
                                       // hic ulasmaz, elle CRC yazmaya gerek yok
  radio.setRetries(5, 5);              // cevap gelmezse 5 kez tekrar dene,
                                       // aralarinda (5+1)*250us = 1500 us bekle

  radio.enableDynamicPayloads();       // ACK payload icin on sart
  radio.enableAckPayload();            // alici cevabini ACK'in icine koyabilsin
  radio.openWritingPipe(ADRES);
  radio.stopListening();               // biz gondericiyiz (PTX modu)

  Serial.println(F("Hazir. Serial Monitor'a yaz:  1=yak  0=sondur  t=degistir"));
  Serial.println(F("(Satir sonu ayari onemsiz, tek karakter yeter.)"));
}

void loop() {
  unsigned long simdi = millis();

  // --- 1) Serial'den komut ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '1') ledIstek = 1;
    else if (c == '0') ledIstek = 0;
    else if (c == 't' || c == 'T') ledIstek = !ledIstek;
  }

  // --- 2) Butondan komut (debounce'li) ---
  bool butonSimdi = digitalRead(BUTON_PIN);
  if (butonSimdi != butonOnceki && (simdi - butonZaman) > DEBOUNCE) {
    butonZaman  = simdi;
    butonOnceki = butonSimdi;
    if (butonSimdi == LOW) ledIstek = !ledIstek;   // basildigi ANDA degistir
  }

  // --- 3) Duzenli gonderim ---
  // Sadece durum degisince gondermek YETMEZ: paket kaybolursa alici
  // sonsuza kadar yanlis durumda kalir. Surekli gonderirsek en fazla
  // 100 ms sonra kendini duzeltir. Arabada da mantik ayni.
  if (simdi - sonGonderim >= GONDER_PERIYOT) {
    sonGonderim = simdi;

    bool ack = gonder(ledIstek);
    if (ack) sonAck = simdi;

    bool durum = (simdi - sonAck <= ACK_TIMEOUT) && (sonAck != 0);
    if (durum != baglantiVar) {
      baglantiVar = durum;
      Serial.println(baglantiVar ? F(">>> BAGLANDI") : F(">>> KOPTU"));
    }
    digitalWrite(LINK_LED, baglantiVar ? HIGH : LOW);

    Serial.print(F("gonderilen="));  Serial.print(ledIstek);
    Serial.print(F("  ack="));       Serial.print(ack ? "OK " : "YOK");
    Serial.print(F("  alicidaLED=")); Serial.print(sonCevap.ledDurum);
    Serial.print(F("  alicisayac=")); Serial.println(sonCevap.sayac);
  }
}
