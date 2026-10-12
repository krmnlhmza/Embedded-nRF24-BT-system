// ==================== VERICI (KUMANDA) - nRF24L01+PA+LNA ====================
// HC-05 SURUMUNUN nRF24 KARSILIGI. Joystick'i okur, sol/sag motor hizlarini
// hesaplar, nRF24 uzerinden gonderir. Arabanin telemetrisini (Vcc, minVcc,
// mesafe) ACK PAYLOAD icinde geri alir. 16x2 I2C LCD ayni sekilde calisir.
//
// GEREKEN KUTUPHANE: "RF24" by TMRh20  (Arduino IDE > Kutuphane Yoneticisi)
//
// ---------------------- HC-05'TEN NE DEGISTI -------------------------
// [1] Serial hatti ARTIK BOS -> Serial Monitor ile debug geri geldi.
//     Kod yuklerken modulu sokmene de gerek yok.
// [2] '<' ... '>' cerceveleme ve elle yazilan CRC-8 KALKTI.
//     nRF24'un kendi donanim CRC'si (2 byte) ayni isi daha iyi yapiyor;
//     bozuk paketi bize hic vermiyor.
// [3] Heartbeat paketi KALKTI. Artik radio.write() bize donus degeri veriyor:
//     true  = araba paketi aldi ve ACK yolladi -> BAGLI
//     false = 5 denemede de cevap yok          -> KOPUK adayi
//     Yani "baglanti var mi" sorusu paket basina, aninda cevaplaniyor.
// [4] Telemetri ACK PAYLOAD ile geliyor: araba cevap ACK'inin icine
//     6 byte veri koyuyor. Ayri bir gonderim yok, ayri bir zamanlama yok.
//
// ------------------------- BAGLANTILAR -------------------------------
//   Joystick : X -> A0, Y -> A1                 (DEGISMEDI)
//   LCD (I2C): SDA -> A4, SCL -> A5             (DEGISMEDI)
//
//   nRF24L01+PA+LNA (modul ust taraftan bakildiginda, 2x4 header):
//     GND  -> GND
//     VCC  -> *** 3.3V *** (asagidaki BESLEME UYARISI'ni oku)
//     CE   -> D9
//     CSN  -> D10
//     SCK  -> D13
//     MOSI -> D11
//     MISO -> D12
//     IRQ  -> bos birak
//
// ------------------------ BESLEME UYARISI ----------------------------
// PA+LNA modul gonderirken 100-115 mA cekiyor. Arduino Uno'nun uzerindeki
// 3.3V regulator en fazla 50 mA verir -> modul acilir gibi yapar, sonra
// rastgele kopar. Cozum ikisinden biri:
//   (a) Ayri bir AMS1117-3.3 regulator: girisi Arduino 5V, cikisi modul VCC
//   (b) Hazir "nRF24 adaptor karti" (uzerinde regulator + kondansator var)
// HER IKI DURUMDA DA modulun VCC-GND ucuna 10-100 uF elektrolit kondansator
// lehimle/tak. Bu kondansator olmadan calisan PA+LNA modul neredeyse yoktur.
//
// VERI PINLERI 5V TOLERANSLI: CE/CSN/SCK/MOSI/MISO'yu Arduino'ya dogrudan
// baglayabilirsin, level shifter gerekmez. SADECE VCC 3.3V olacak.
//
// ANTENI TAKMADAN GUC VERME. Anteni olmayan PA katini yansiyan guc bozar.
// ======================================================================

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);   // I2C adresi: calismazsa 0x3F dene

// --------------------------- RADYO AYARLARI ---------------------------
const uint8_t CE_PIN  = 9;
const uint8_t CSN_PIN = 10;
RF24 radio(CE_PIN, CSN_PIN);

// Adres: alici ile BIREBIR AYNI olmali. 5 karakter + sonlandirici.
const uint8_t ADRES[6] = "ARB01";

// Kanal: 0-125 arasi, her kanal 1 MHz. 2.4 GHz WiFi kanallari 1-11 arasi
// 2400-2462 MHz'i doldurur; 108 -> 2508 MHz, WiFi'nin TAMAMEN uzerinde.
// Ortamda cok modul varsa 76, 100, 108, 120 gibi degerleri dene.
const uint8_t KANAL = 108;

// --- Paket yapisi: alici ile BIREBIR AYNI olmali ---
// CRC ve cerceve byte'lari yok: onlari nRF24'un kendisi hallediyor.
struct __attribute__((packed)) CommandData {
  int16_t sol;   // sol motor grubu hizi: -255 (tam geri) .. +255 (tam ileri)
  int16_t sag;   // sag motor grubu hizi: -255 .. +255
  uint8_t seq;   // sira numarasi (sadece teshis icin; tekrar elemeyi donanim yapar)
};

// --- Arabadan ACK icinde gelen telemetri: alici ile BIREBIR AYNI ---
struct __attribute__((packed)) Telemetri {
  uint16_t vcc;      // arabanin anlik besleme gerilimi (mV)
  uint16_t minVcc;   // gordugu en dusuk gerilim (mV)
  uint16_t mesafe;   // radar mesafesi (cm), 999 = yanki yok
};

uint8_t seqNo = 0;

// --- Pinler ve ayarlar (DEGISMEDI) ---
const int X_PIN = A0;
const int Y_PIN = A1;
const int SAFE_ZONE = 40;

// --- Joystick kalibrasyonu (DEGISMEDI) ---
int merkezX = 512, merkezY = 512;
int olcekX  = 470,  olcekY = 470;

// --- Radar esikleri: ALICI ile AYNI olmali (sadece gosterim icin) ---
const unsigned int MESAFE_YOK = 999;
const unsigned int DUR_MESAFE = 25;

// --- Baglanti takibi ---
// Tek bir basarisiz write "koptu" demek degil (bir paket parazite yenilebilir).
// Bu sure boyunca HIC ACK alamazsak koptu deriz.
const unsigned long ACK_TIMEOUT = 700;
bool baglantiVar = false;
unsigned long sonAck = 0;
bool hicAckGeldi = false;

// --- Arabadan gelen degerler ---
unsigned int aracVcc    = 0;
unsigned int aracMinVcc = 0;
unsigned int aracMesafe = MESAFE_YOK;

// --- LCD guncelleme zamanlayicilari (DEGISMEDI) ---
unsigned long sonUstYazim  = 0;
unsigned long sonAltYazim  = 0;
unsigned long sonModDegis  = 0;
uint8_t altMod = 0;
int gosterilenSol = 9999;
int gosterilenSag = 9999;
unsigned int gosterilenMesafe = 0;
bool gosterilenDurum = false;

// ----------------------------------------------------------------------
// Komutu gonder, arabadan gelen ACK payload'u varsa oku.
// Donus: araba paketi aldi mi (donanim ACK'i geldi mi).
// ----------------------------------------------------------------------
bool komutGonder(int16_t sol, int16_t sag) {
  CommandData c;
  c.sol = sol;
  c.sag = sag;
  c.seq = seqNo++;

  bool ok = radio.write(&c, sizeof(c));   // ACK bekler, gelmezse tekrar dener

  if (ok && radio.isAckPayloadAvailable()) {
    Telemetri t;
    radio.read(&t, sizeof(t));
    aracVcc    = t.vcc;
    aracMinVcc = t.minVcc;
    aracMesafe = t.mesafe;
  }
  return ok;
}

// ----------------------------------------------------------------------
// LCD yazim fonksiyonlari (HC-05 surumuyle BIREBIR AYNI)
// ----------------------------------------------------------------------
void lcdUstYaz(bool bagli, unsigned int mes) {
  char buf[17];
  if (!bagli)
    snprintf(buf, sizeof(buf), "KOPUK   M:---cm ");
  else if (mes == 0 || mes >= MESAFE_YOK)
    snprintf(buf, sizeof(buf), "BAGLI   M:---cm ");
  else
    snprintf(buf, sizeof(buf), "BAGLI  %cM:%3ucm ",
             (mes <= DUR_MESAFE) ? '!' : ' ', mes);
  lcd.setCursor(0, 0);
  lcd.print(buf);
}

void lcdHizYaz(int sol, int sag) {
  char buf[17];
  snprintf(buf, sizeof(buf), "S:%-6dG:%-6d", sol, sag);
  lcd.setCursor(0, 1);
  lcd.print(buf);
}

void lcdGerilimYaz(unsigned int vcc, unsigned int mn) {
  char buf[17], v[6], m[6];
  if (vcc >= 2000) snprintf(v, sizeof(v), "%4u", vcc); else strcpy(v, "----");
  if (mn  >= 2000) snprintf(m, sizeof(m), "%4u", mn);  else strcpy(m, "----");
  snprintf(buf, sizeof(buf), "V:%s  MIN:%s", v, m);
  lcd.setCursor(0, 1);
  lcd.print(buf);
}

// --- Acilista joystick merkezini olc (DEGISMEDI) ---
void joystickKalibreEt() {
  long toplamX = 0, toplamY = 0;
  for (int i = 0; i < 50; i++) {
    toplamX += analogRead(X_PIN);
    toplamY += analogRead(Y_PIN);
    delay(4);
  }
  merkezX = toplamX / 50;
  merkezY = toplamY / 50;

  olcekX = min(merkezX, 1023 - merkezX);
  olcekY = min(merkezY, 1023 - merkezY);

  if (olcekX < 150) { merkezX = 512; olcekX = 470; }
  if (olcekY < 150) { merkezY = 512; olcekY = 470; }
}

int eksenOku(int pin, int merkez, int olcek) {
  int sapma = analogRead(pin) - merkez;
  long deger = (long)sapma * 255L / (long)olcek;
  if (deger >  255) deger =  255;
  if (deger < -255) deger = -255;
  return (int)deger;
}

void setup() {
  Serial.begin(115200);            // ARTIK SERBEST: debug icin kullanabilirsin

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("KALIBRASYON...  ");
  lcd.setCursor(0, 1);
  lcd.print("JOYSTICK SERBEST");
  joystickKalibreEt();

  // ------------------------- RADYOYU BASLAT -------------------------
  if (!radio.begin()) {
    // Buraya dusuyorsa SPI kablolari veya besleme yanlis.
    lcd.setCursor(0, 0); lcd.print("NRF24 BULUNAMADI");
    lcd.setCursor(0, 1); lcd.print("SPI/3V3 KONTROL ");
    Serial.println(F("nRF24 yok: CE/CSN/SCK/MOSI/MISO ve 3.3V besleme kontrol"));
    while (1) { }                  // devam etmenin anlami yok
  }

  radio.setPALevel(RF24_PA_MAX);       // PA+LNA modulde tam guc
  radio.setDataRate(RF24_250KBPS);     // en dusuk hiz = en uzun menzil
  radio.setChannel(KANAL);
  radio.setCRCLength(RF24_CRC_16);     // 2 byte donanim CRC (bizim crc8 yerine)
  radio.setRetries(5, 5);              // 5 tekrar, aralarinda (5+1)*250 = 1500 us
  // Neden 250KBPS: ayni PA gucunde bit hizini dusurmek alicinin duyarliligini
  // artirir. 2MBPS ile 250KBPS arasinda pratikte 2-3 kat menzil farki var.
  // Bizim veri hacmimiz cok kucuk (5 byte / 20 ms), hiza ihtiyacimiz yok.

  radio.enableDynamicPayloads();       // ACK payload icin sart
  radio.enableAckPayload();            // araba cevabin icine telemetri koyacak
  radio.openWritingPipe(ADRES);
  radio.stopListening();               // biz gondericiyiz (PTX)

  lcdUstYaz(false, MESAFE_YOK);
  lcdHizYaz(0, 0);
  gosterilenSol = 0;
  gosterilenSag = 0;
  gosterilenDurum = false;
  gosterilenMesafe = MESAFE_YOK;
  sonModDegis = millis();
}

void loop() {
  unsigned long simdi = millis();

  // 1) Joystick'i kalibrasyonlu oku (DEGISMEDI)
  int ileri = eksenOku(Y_PIN, merkezY, olcekY);
  int donus = eksenOku(X_PIN, merkezX, olcekX);

  // 2) Safe zone (DEGISMEDI)
  if (abs(ileri) < SAFE_ZONE) ileri = 0;
  if (abs(donus) < SAFE_ZONE) donus = 0;

  int sol = 0, sag = 0;

  if (ileri == 0 && donus == 0) {
    sol = 0;
    sag = 0;
  } else if (ileri == 0) {
    sol =  donus;
    sag = -donus;
  } else if (ileri > 0) {
    sol = constrain(ileri + donus, 0, 255);
    sag = constrain(ileri - donus, 0, 255);
  } else {
    sol = constrain(ileri + donus, -255, 0);
    sag = constrain(ileri - donus, -255, 0);
  }

  // 3) HER ZAMAN gonder. HC-05'teki "kopukken gonderme" kurali burada GEREKSIZ:
  //    nRF24'te tampon birikmesi yok, gonderilmeyen paket 5 denemede olur ve
  //    atilir. Aksine surekli gondermek zorundayiz, cunku baglantinin geri
  //    geldigini ancak gondererek anlariz.
  bool ack = komutGonder(sol, sag);

  if (ack) {
    sonAck = simdi;
    hicAckGeldi = true;
  }

  bool simdikiDurum = hicAckGeldi && (simdi - sonAck <= ACK_TIMEOUT);

  if (simdikiDurum != baglantiVar) {
    baglantiVar = simdikiDurum;
    if (!baglantiVar) {
      hicAckGeldi = false;         // yeni ACK gelene kadar "yok"
      aracVcc = 0;
      aracMesafe = MESAFE_YOK;     // ekranda bayat deger asili kalmasin
    }
  }

  if (!baglantiVar) { sol = 0; sag = 0; }   // ekranda da 0 gorunsun

  // 4) --- LCD UST SATIR: durum + mesafe (DEGISMEDI) ---
  bool ustGerekli = (baglantiVar != gosterilenDurum);
  if (!ustGerekli && baglantiVar &&
      (aracMesafe != gosterilenMesafe) &&
      (simdi - sonUstYazim >= 250)) {
    ustGerekli = true;
  }
  if (ustGerekli) {
    lcdUstYaz(baglantiVar, aracMesafe);
    gosterilenDurum  = baglantiVar;
    gosterilenMesafe = aracMesafe;
    sonUstYazim = simdi;
  }

  // 5) --- LCD ALT SATIR: 2 sn'de bir hizlar <-> gerilim (DEGISMEDI) ---
  if (simdi - sonModDegis >= 2000) {
    sonModDegis = simdi;
    altMod = 1 - altMod;
    gosterilenSol = 9999;
  }

  if (altMod == 0) {
    if ((sol != gosterilenSol || sag != gosterilenSag) &&
        (simdi - sonAltYazim >= 250)) {
      lcdHizYaz(sol, sag);
      gosterilenSol = sol;
      gosterilenSag = sag;
      sonAltYazim = simdi;
    }
  } else {
    if (simdi - sonAltYazim >= 500) {
      lcdGerilimYaz(aracVcc, aracMinVcc);
      sonAltYazim = simdi;
    }
  }

  delay(20);   // ~50 paket/sn. radio.write baglantiliyken ~1 ms suruyor,
               // kopukken 5 tekrar yuzunden en fazla ~9 ms.
}
