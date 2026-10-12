// =====================================================================
//  AKTARICI (ROLE / MENZIL UZATICI)
//  Arduino Nano + nRF24L01+PA+LNA (antenli)
//
//  KUTUPHANE : "RF24" by TMRh20  (surum 1.4.0 veya ustu)
//  SERI PORT : 115200 baud   <-- Serial Monitor'u 115200'e AYARLA
// =====================================================================
//
//  ################  SISTEM  ################
//
//    KUMANDA  --(AKT01)-->  AKTARICI  --(ARB02)-->  ARABA
//
//  Ne yapiyor:
//   1) Surekli "AKT01" adresini dinler (kumanda buraya yollar).
//   2) Kumandadan paket gelir gelmez, HIC BEKLEMEDEN ve HIC
//      DEGISTIRMEDEN ayni 13 bayti "ARB02" adresine (arabaya) yollar.
//   3) Iletim biter bitmez tekrar dinlemeye doner.
//  Kodda hic delay() yok; loop sadece telsizi yokluyor.
//
//  Telemetri (araba -> kumanda) de geri tasiniyor:
//   - Araba, telemetriyi aktariciya verdigi ACK'in icine koyar.
//   - Aktarici bunu saklar ve kumandanin BIR SONRAKI paketine verecegi
//     ACK'in icine koyar. Boylece kumandanin LCD'si calismaya devam eder.
//   - Araba cevap vermiyorsa aktarici bayat telemetri YOLLAMAZ; kumanda
//     1 sn sonra "KOPUK" gosterir. (Kumanda ACK'i aktaricidan alir, ama
//     telemetri gelmedigi icin arabanin kopuk oldugunu anlar.)
//
//  ################  ONEMLI: TEK TELSIZ, YARI CIFT YON  ################
//
//  nRF24 ayni anda hem dinleyip hem gonderemez. Aktarici arabaya
//  gonderdigi ~2-3 ms boyunca kumandayi duyamaz. Sorun degil:
//   - Kumanda 20 ms'de bir yolluyor, iletim o araliga rahat sigiyor.
//   - Nadiren cakisirsa kumandanin donanimsal tekrar gonderimi
//     (15 deneme x 1.5 ms) paketi aktarici dinlemeye donunce ulastirir.
//  Arabaya giden hattin tekrar sayisini BILEREK kisa tuttuk (5 deneme):
//  araba kapaliyken bile aktarici en fazla ~12 ms mesgul kalir, sonra
//  hemen kumandayi dinlemeye doner.
//
//  ################  GUVENLIK  ################
//
//  Aktarici kendi basina hicbir komut uretmez. Kumanda susarsa aktarici
//  da susar; araba 400 ms sonra kendi timeout'u ile durur.
//  Aktarici kapanirsa da araba ayni sekilde durur.
//
//  ################  BAGLANTILAR  ################
//
//   nRF24L01:  CE  -> D9
//              CSN -> D10
//              MOSI-> D11
//              MISO-> D12
//              SCK -> D13
//              VCC -> 3.3V   (5V VERME, MODUL YANAR)
//              GND -> GND
//              VCC-GND arasina 10uF (mumkunse 100uF) kondansator
//
//  PA+LNA modul tepe akimda ~115 mA ceker. Nano'nun 3.3V pini bunu
//  kaldirmaz: ayri bir 3.3V regulator ya da nRF24 adaptor karti kullan.
//
//  ################  YERLESIM  ################
//   - Kumanda ile arabanin ARASINA, mumkun oldugunca YUKSEGE koy.
//   - Antenler dik ve birbirine paralel dursun.
//   - Masada test ederken modulleri en az 2-3 metre ayir (PA_MAX'ta
//     yakindaki alici doyar).
// =====================================================================

#include <SPI.h>
#include <RF24.h>

// ======================= AYAR ANAHTARLARI =============================
#define DEBUG   1       // 1: saniyede bir ozet basar. Sahada 0 yapabilirsin.
// ======================================================================

// ========================= TELSIZ =====================================
#define CE_PIN    9
#define CSN_PIN   10

// Kumandanin gonderdigi adres (kumandadaki ADRES ile AYNI)
const uint8_t ADRES_KUMANDA[6] = "AKT01";
// Arabanin dinledigi adres (arabadaki ADRES ile AYNI)
const uint8_t ADRES_ARABA[6]   = "ARB02";

// Uc cihazda da AYNI kanal ve hiz. Iki hat, farkli ADRESLER sayesinde
// birbirine karismiyor.
const uint8_t KANAL = 108;
#define VERI_HIZI   RF24_250KBPS
#define RF_GUC      RF24_PA_MAX

// Arabaya giden hat icin tekrar ayari:
//   gecikme = (3+1) x 250 us = 1000 us  -> 250KBPS'te 8 baytlik ACK
//                                          payload icin yeterli (min 750)
//   deneme  = 5                          -> en kotu durumda ~12 ms
#define ARABA_TEKRAR_GECIKME  3
#define ARABA_TEKRAR_SAYI     5

RF24 radio(CE_PIN, CSN_PIN);
// ======================================================================

// ========================= PAKET YAPILARI =============================
// KUMANDA ve ARABA ile BIREBIR AYNI. Tek bir alan bile degismemeli.
const uint8_t KOMUT_IMZA = 0xA5;
const uint8_t TLM_IMZA   = 0x5A;

struct __attribute__((packed)) KomutPaketi {   // kumanda -> araba, 13 bayt
  uint8_t  imza;
  uint8_t  sayac;
  uint16_t joyX, joyY;
  uint16_t pot;
  uint8_t  butonlar;
  int16_t  sol, sag;
};

struct __attribute__((packed)) TelemetriPaketi { // araba -> kumanda, 8 bayt
  uint8_t  imza;
  uint16_t vcc;
  uint16_t minVcc;
  uint16_t mesafe;
  uint8_t  durum;
};

// Boyut bekcisi: biri struct'i yanlislikla degistirirse derleme durur.
static_assert(sizeof(KomutPaketi)     == 13, "KomutPaketi 13 bayt olmali");
static_assert(sizeof(TelemetriPaketi) == 8,  "TelemetriPaketi 8 bayt olmali");
// ======================================================================

// --- Telemetri deposu ---
TelemetriPaketi tlm;              // arabadan gelen son telemetri
bool tlmVar = false;
unsigned long sonTlm = 0;
bool ackYuklu = false;            // kumandaya verilecek ACK'e telemetri kondu mu
const unsigned long TLM_BAYAT = 300;   // ms: bundan eskisini kumandaya yollama

// --- Istatistikler (DEBUG ozeti icin) ---
unsigned long gelen = 0;          // kumandadan alinan gecerli paket
unsigned long iletilen = 0;       // arabanin ACK verdigi paket
unsigned long iletilemeyen = 0;   // arabanin cevap vermedigi paket
unsigned long cop = 0;            // boyu/imzasi tutmayan paket
unsigned long sonOzet = 0;
unsigned long ozetGelen = 0, ozetIletilen = 0, ozetIletilemeyen = 0;

// =====================================================================
//  YARDIMCI FONKSIYONLAR
// =====================================================================

void cizgi() {
  Serial.println(F("---------------------------------------------"));
}

// mV -> "4.95V"  (gecersizse "-")
void voltYaz(unsigned int mv) {
  if (mv < 2000) { Serial.print('-'); return; }
  unsigned int yuzde = (mv % 1000) / 10;
  Serial.print(mv / 1000); Serial.print('.');
  if (yuzde < 10) Serial.print('0');
  Serial.print(yuzde);     Serial.print('V');
}

// FIFO'daki istenmeyen paketi okuyup at
void copAt(uint8_t boy) {
  uint8_t tampon[32];
  radio.read(tampon, (boy > 32) ? 32 : boy);
  cop++;
}

// ----------------------------------------------------------------------
// Kumandanin paketini arabaya ilet ve hemen dinlemeye don.
// ----------------------------------------------------------------------
void arabayaIlet(const KomutPaketi &p) {
  // --- Dinlemeden gondermeye gec ---
  radio.stopListening();
  // TX FIFO'da kumanda icin bekleyen ACK payload varsa SIL. Silmezsek
  // write() onu arabaya normal paket gibi yollar. (RF24 1.4 bunu kendisi
  // de yapiyor, surum farkina karsi garanti olsun diye elle de yapiyoruz.)
  radio.flush_tx();
  ackYuklu = false;

  // --- Paketi AYNEN gonder ---
  bool ok = radio.write(&p, sizeof(p));

  if (ok) {
    iletilen++;
    ozetIletilen++;

    // Arabanin ACK'inin icinde telemetri var mi? (pipe 0'a gelir)
    uint8_t boru;
    while (radio.available(&boru)) {
      uint8_t boy = radio.getDynamicPayloadSize();
      if (boru == 0 && boy == sizeof(TelemetriPaketi)) {
        TelemetriPaketi t;
        radio.read(&t, sizeof(t));
        if (t.imza == TLM_IMZA) {
          tlm = t;
          tlmVar = true;
          sonTlm = millis();
        } else {
          cop++;
        }
      } else {
        // Nadiren: gonderime gecerken kumandadan yeni bir paket yetismis
        // olabilir. 20 ms sonra tazesi gelecek, bunu atiyoruz.
        copAt(boy);
      }
    }
  } else {
    iletilemeyen++;
    ozetIletilemeyen++;
  }

  // --- Hemen tekrar kumandayi dinle ---
  radio.startListening();

  // Kumandanin bir sonraki paketine verilecek ACK'e telemetriyi koy.
  // Sadece TAZE ise: araba cevap vermiyorsa kumanda bunu gormeli.
  if (tlmVar && (millis() - sonTlm <= TLM_BAYAT)) {
    radio.writeAckPayload(1, &tlm, sizeof(tlm));
    ackYuklu = true;
  }
}

// ----------------------------------------------------------------------
// Telsiz kurulumu. Basarisizsa sonsuz donguye girer.
// ----------------------------------------------------------------------
void radyoKur() {
  Serial.println(F("[2/3] nRF24L01 baslatiliyor..."));

  if (!radio.begin()) {
    cizgi();
    Serial.println(F("HATA 1: nRF24 modulu hic yanit vermiyor."));
    Serial.println(F("  - CE=D9, CSN=D10, MOSI=D11, MISO=D12, SCK=D13"));
    Serial.println(F("  - VCC kesinlikle 3.3V olacak (5V modulu yakar)"));
    Serial.println(F("  - VCC-GND arasina 10uF kondansator tak"));
    cizgi();
    while (1) delay(1000);
  }

  if (!radio.isChipConnected()) {
    cizgi();
    Serial.println(F("HATA 2: SPI hatti kararsiz, cip okunamiyor."));
    Serial.println(F("  - MISO(D12) ve MOSI(D11) ters olabilir"));
    Serial.println(F("  - Kablolari kisalt"));
    cizgi();
    while (1) delay(1000);
  }

  radio.setChannel(KANAL);
  radio.setDataRate(VERI_HIZI);
  radio.setPALevel(RF_GUC);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);

  // Iki yonde de ACK payload kullaniyoruz (kumandaya verirken ve
  // arabadan alirken). Dinamik payload sart.
  radio.enableDynamicPayloads();
  radio.enableAckPayload();

  radio.setRetries(ARABA_TEKRAR_GECIKME, ARABA_TEKRAR_SAYI);

  if (radio.getChannel() != KANAL) {
    cizgi();
    Serial.println(F("HATA 3: Ayarlar module yazilamiyor."));
    Serial.println(F("  Besleme yetersiz olabilir. 3.3V + 10uF sart."));
    cizgi();
    while (1) delay(1000);
  }

  // Gonderme adresi: araba. (RF24 bunu pipe 0'a da yazar; arabanin
  // ACK'i pipe 0'dan gelir.)
  radio.openWritingPipe(ADRES_ARABA);
  // Dinleme adresi: kumanda. MUTLAKA pipe 1 (pipe 0 gonderime ayrildi).
  radio.openReadingPipe(1, ADRES_KUMANDA);
  radio.startListening();

  Serial.println(F("      SONUC: telsiz OK"));
}

void radyoBilgisiYaz() {
  cizgi();
  Serial.println(F("TELSIZ AYARLARI (kumanda ve araba ile AYNI olmali):"));
  Serial.print(F("  Kanal        : ")); Serial.println(radio.getChannel());
  Serial.print(F("  Veri hizi    : "));
  switch (radio.getDataRate()) {
    case RF24_250KBPS: Serial.println(F("250KBPS")); break;
    case RF24_1MBPS:   Serial.println(F("1MBPS"));   break;
    case RF24_2MBPS:   Serial.println(F("2MBPS"));   break;
  }
  Serial.print(F("  Dinlenen     : ")); Serial.print((const char*)ADRES_KUMANDA);
  Serial.println(F("  (kumanda)"));
  Serial.print(F("  Gonderilen   : ")); Serial.print((const char*)ADRES_ARABA);
  Serial.println(F("  (araba)"));
  Serial.print(F("  Komut paketi : ")); Serial.print(sizeof(KomutPaketi));
  Serial.println(F(" bayt"));
  Serial.print(F("  Telemetri    : ")); Serial.print(sizeof(TelemetriPaketi));
  Serial.println(F(" bayt"));
  Serial.print(F("  Modul tipi   : "));
  if (radio.isPVariant()) {
    Serial.println(F("nRF24L01+ (gercek)"));
  } else {
    Serial.println(F("nRF24L01 veya KLON"));
    Serial.println(F("  >> !! Bu modulde RF24_250KBPS CALISMAZ !!"));
    Serial.println(F("  >> VERI_HIZI'ni RF24_1MBPS yap (UC cihazda da)."));
  }
  cizgi();
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { }

  delay(300);
  cizgi();
  Serial.println(F("  AKTARICI : KUMANDA -> [AKTARICI] -> ARABA"));
  Serial.println(F("  Serial Monitor 115200 baud olmali"));
  cizgi();
  Serial.println(F("[1/3] Kart uyandi."));

  radyoKur();
  radyoBilgisiYaz();

  Serial.println(F("[3/3] Hazir. Kumandayi ve arabayi ac."));
  cizgi();
}

// =====================================================================
//  LOOP  -- delay() YOK, sadece telsizi yokluyoruz
// =====================================================================
void loop() {

  // ---------------- 1) Kumandayi dinle ----------------
  // FIFO'da birden fazla paket biriktiyse EN SONUNCUSUNU ilet
  // (bayat komutu arabaya tasimanin anlami yok).
  bool yeni = false;
  KomutPaketi p;
  uint8_t boru;

  while (radio.available(&boru)) {
    uint8_t boy = radio.getDynamicPayloadSize();

    if (boru != 1 || boy != sizeof(KomutPaketi)) {
      copAt(boy);
      continue;
    }

    KomutPaketi gecici;
    radio.read(&gecici, sizeof(gecici));

    if (gecici.imza != KOMUT_IMZA) {
      cop++;
      continue;
    }

    p = gecici;
    yeni = true;
  }

  // ---------------- 2) Geldiyse ANINDA arabaya ilet ----------------
  if (yeni) {
    gelen++;
    ozetGelen++;
    arabayaIlet(p);          // bitince zaten tekrar dinlemede
  }

  unsigned long simdi = millis();

  // ---------------- 3) Bayat telemetriyi kuyruktan at ----------------
  // Kumanda uzun sure sustuysa ACK kuyrugunda eski telemetri kalmasin;
  // kumanda geri geldiginde ilk cevapta eski veri gormesin.
  if (ackYuklu && (simdi - sonTlm > TLM_BAYAT)) {
    radio.flush_tx();
    ackYuklu = false;
  }

  // ---------------- 4) Saniyede bir ozet ----------------
#if DEBUG
  if (simdi - sonOzet >= 1000) {
    sonOzet = simdi;

    // Ornek:
    // Kumanda 50 paket/sn | Arabaya 50 ulasti, 0 ulasmadi | Araba 4.95V | Engel yok
    if (ozetGelen == 0) {
      Serial.println(F("Kumanda YOK"));
    } else {
      Serial.print(F("Kumanda "));        Serial.print(ozetGelen);
      Serial.print(F(" paket/sn | Arabaya ")); Serial.print(ozetIletilen);
      Serial.print(F(" ulasti, "));       Serial.print(ozetIletilemeyen);
      Serial.print(F(" ulasmadi"));

      if (tlmVar && (simdi - sonTlm <= 1000)) {
        Serial.print(F(" | Araba "));     voltYaz(tlm.vcc);
        Serial.print(F(" | Engel "));
        if (tlm.mesafe >= 999) Serial.print(F("yok"));
        else { Serial.print(tlm.mesafe); Serial.print(F(" cm")); }
        if (tlm.durum & 0x01) Serial.print(F(" DUR"));   // engel biti
      } else {
        Serial.print(F(" | Araba YOK"));
      }
      if (cop) { Serial.print(F(" | Hatali paket (toplam) ")); Serial.print(cop); }
      Serial.println();
    }

    ozetGelen = 0; ozetIletilen = 0; ozetIletilemeyen = 0;
  }
#endif
}
