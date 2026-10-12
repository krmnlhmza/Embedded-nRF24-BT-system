// ================== ADIM 1: ALICI (LED'I YAKAN KART) ====================
// Karsi karttan gelen paketi okur, LED'i ona gore surer ve kendi durumunu
// ACK payload ile geri yollar. Bu kart arabadaki aliciNrf.ino'nun iskeleti:
// tek fark motor/radar yerine tek bir LED olmasi.
//
// GEREKEN KUTUPHANE: "RF24" by TMRh20
//
// BAGLANTILAR (Uno):
//   nRF24: GND->GND  VCC->3.3V  CE->D9  CSN->D10  SCK->D13  MOSI->D11  MISO->D12
//   LED  : D4 -> 220 ohm direnc -> LED uzun bacak (anot)
//          LED kisa bacak (katot) -> GND
//
// DIKKAT: D13 = SCK oldugu icin kart uzerindeki LED SPI trafiginde titrer.
// Gostergeyi D13'e BAGLAMA, D4 kullan.
// ======================================================================

#include <SPI.h>
#include <RF24.h>

// --------------------------- RADYO AYARLARI ---------------------------
const uint8_t CE_PIN  = 9;
const uint8_t CSN_PIN = 10;
RF24 radio(CE_PIN, CSN_PIN);

const uint8_t ADRES[6] = "LED01";   // verici ile BIREBIR AYNI
const uint8_t KANAL    = 108;       // verici ile BIREBIR AYNI

// --- Gelen paket: verici ile BIREBIR AYNI struct ---
struct __attribute__((packed)) Paket {
  uint8_t led;
  uint8_t seq;
};

// --- ACK icinde geri gonderdigimiz cevap: verici ile BIREBIR AYNI ---
struct __attribute__((packed)) Cevap {
  uint8_t  ledDurum;
  uint16_t sayac;
};

const int LED_PIN = 4;

uint8_t  ledDurum = 0;
uint16_t sayac    = 0;

// --- Baglanti kopma korumasi ---
// Arabada bu satirlar hayat kurtariyor: kumanda giderse motorlar durmali.
// Burada sadece LED'i sonduruyoruz ama mantik birebir ayni.
unsigned long sonPaket = 0;
bool linkAktif = false;
const unsigned long TIMEOUT = 500;

unsigned long sonDebug = 0;

// ----------------------------------------------------------------------
// Cevabi ACK kuyruguna koy. Bu veri BIR SONRAKI gelen paketin donanim
// ACK'i ile birlikte gider. Yani ayri bir gonderim/zamanlama yok.
// ----------------------------------------------------------------------
void cevapHazirla() {
  Cevap c;
  c.ledDurum = ledDurum;
  c.sayac    = sayac;
  radio.writeAckPayload(1, &c, sizeof(c));   // 1 = okuma borusu numarasi
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println(F("\n=== ALICI ==="));

  if (!radio.begin()) {
    Serial.println(F("nRF24 BULUNAMADI. Once nrfKontrol sketch'ini calistir."));
    while (1) { }
  }

  // --------- AYARLAR VERICI ILE BIREBIR AYNI OLMAK ZORUNDA ---------
  // Bunlardan biri bile farkli olursa paket asla ulasmaz ve hicbir hata
  // mesaji almazsin. Sessiz basarisizlik: en cok zaman yakan hata tipi.
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(KANAL);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(5, 5);

  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openReadingPipe(1, ADRES);   // 1 numarali boruyu bu adrese ac
  radio.startListening();            // biz aliciyiz (PRX modu)

  cevapHazirla();                    // ilk ACK'e konacak veri hazir olsun

  Serial.println(F("Dinlemede..."));
}

void loop() {
  unsigned long simdi = millis();

  // --- Gelen paketleri oku ---
  // while ile FIFO'yu TAMAMEN bosaltiyoruz. Donanimda 3 paketlik kuyruk var;
  // bir sebeple gec kalirsak elimizde EN TAZE komut kalsin istiyoruz.
  // (Arabada bu satir "eski komutla surme" problemini tek basina cozuyor.)
  Paket p;
  bool yeni = false;
  while (radio.available()) {
    radio.read(&p, sizeof(p));
    yeni = true;
  }

  if (yeni) {
    ledDurum = (p.led != 0) ? 1 : 0;
    digitalWrite(LED_PIN, ledDurum);

    sayac++;
    sonPaket  = simdi;
    if (!linkAktif) { linkAktif = true; Serial.println(F(">>> BAGLANDI")); }

    cevapHazirla();      // bir sonraki ACK icin taze veri yukle

    Serial.print(F("paket seq="));  Serial.print(p.seq);
    Serial.print(F("  led="));      Serial.print(ledDurum);
    Serial.print(F("  toplam="));   Serial.println(sayac);
  }

  // --- Sessizlik = kumanda gitti: guvenli duruma dus ---
  if (linkAktif && (simdi - sonPaket > TIMEOUT)) {
    linkAktif = false;
    ledDurum = 0;
    digitalWrite(LED_PIN, LOW);
    cevapHazirla();
    Serial.println(F(">>> KOPTU - guvenli duruma gecildi"));
  }

  // --- Sessizken de hayatta oldugunu goster ---
  if (!linkAktif && (simdi - sonDebug >= 2000)) {
    sonDebug = simdi;
    Serial.println(F("(paket yok, dinlemeye devam)"));
  }
}
