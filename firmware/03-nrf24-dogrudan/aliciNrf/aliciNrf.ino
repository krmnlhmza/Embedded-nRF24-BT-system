// ===================== ALICI (ARABA) - nRF24L01+PA+LNA =====================
// HC-05 SURUMUNUN nRF24 KARSILIGI. Kumandadan komut paketi alir, L298N ile
// 4 motoru surer, HC-SR04 ile mesafe olcer ve telemetriyi ACK PAYLOAD icinde
// kumandaya geri yollar.
//
// GEREKEN KUTUPHANE: "RF24" by TMRh20
//
// ---------------------- HC-05'TEN NE DEGISTI -------------------------
// [1] Serial hatti ARTIK BOS. #define DEBUG 1 yapip Serial Monitor'den
//     her seyi izleyebilirsin -- HC-05 surumunde bu imkansizdi.
// [2] readCommand() icindeki cerceve arama / cop byte atma / CRC dogrulama
//     KALKTI. nRF24 bize sadece CRC'si tutan tam paketleri veriyor.
// [3] RESENKRONIZASYON KALKTI. HC-05'te gerekiyordu cunku kopukken paketler
//     BT modulunun tamponunda birikip link donunce toplu dusuyordu.
//     nRF24'te boyle bir birikme yok: RX FIFO 3 paket derinliginde ve
//     her dongude tamamen bosaltiyoruz, elimizde hep EN TAZE komut kaliyor.
// [4] Ayri heartbeat gonderimi KALKTI. Telemetriyi writeAckPayload ile
//     donanim ACK'inin icine koyuyoruz; kumanda paketi gonderdigi anda
//     cevabi da almis oluyor.
// [5] crc8() fonksiyonu KALKTI (donanim CRC-16 devrede).
//
// ------------------------- BAGLANTILAR -------------------------------
//   L298N  : ENA->5, IN1->2, IN2->3, ENB->6, IN3->4, IN4->7   (DEGISMEDI)
//   HC-SR04: TRIG -> 9, ECHO -> 8                             (DEGISMEDI)
//
//   nRF24L01+PA+LNA:
//     GND  -> GND
//     VCC  -> *** 3.3V ***  (ayri AMS1117-3.3 regulator + 10-100uF kondansator)
//     CE   -> A0      <-- D9 radar TRIG'de oldugu icin analog pini digital
//                         olarak kullaniyoruz. A0 = digital 14, sorunsuz.
//     CSN  -> D10
//     SCK  -> D13
//     MOSI -> D11
//     MISO -> D12
//     IRQ  -> bos
//
//   Pin 0, 1 ve A1-A5 artik BOS.
//
// ------------------------ BESLEME UYARISI ----------------------------
// ARABADA BU DAHA KRITIK: motorlar akim cektiginde besleme cokuyor (zaten
// minVcc'yi bu yuzden izliyorsun). nRF24 3.3V'u 2.9V'un altina inerse
// modul susar. Regulatoru MOTOR BESLEMESINDEN DEGIL, mantik 5V hattindan
// besle ve modulun ucune 100 uF kondansator koy.
// Anteni takmadan guc verme.
//
// ------------------------- DAVRANIS (DEGISMEDI) ----------------------
//   mesafe > 60 cm        -> serbest
//   25 < mesafe <= 60 cm  -> ileri hiz 130 PWM ile sinirli
//   mesafe <= 25 cm       -> ileri gidis engellenir
//   Geri gitmek ve yerinde donmek her zaman serbest.
// ======================================================================

#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>

// ======================= AYAR ANAHTARLARI =============================
#define DEBUG       0     // ARTIK GUVENLE 1 YAPABILIRSIN: Serial bos.
                          // Sadece surekli print loop'u yavaslatir, unutma.
#define FREN_AKTIF  1     // 1: kopunca aktif fren, 0: serbest birak

const int MAX_PWM = 255;

const bool SOL_TERS = false;
const bool SAG_TERS = false;

const int MIN_PWM = 0;
const int GUC_YUZDE = 100;
// ======================================================================

// --------------------------- RADYO AYARLARI ---------------------------
const uint8_t CE_PIN  = A0;   // D9 radar TRIG'de: CE'yi A0'a aldik
const uint8_t CSN_PIN = 10;
RF24 radio(CE_PIN, CSN_PIN);

const uint8_t ADRES[6] = "ARB01";   // verici ile BIREBIR AYNI
const uint8_t KANAL    = 108;       // verici ile BIREBIR AYNI

// --- Paket yapisi: verici ile BIREBIR AYNI olmali ---
struct __attribute__((packed)) CommandData {
  int16_t sol;
  int16_t sag;
  uint8_t seq;
};

// --- ACK icinde kumandaya giden telemetri: verici ile BIREBIR AYNI ---
struct __attribute__((packed)) Telemetri {
  uint16_t vcc;
  uint16_t minVcc;
  uint16_t mesafe;
};

// ========================= RADAR (HC-SR04) ============================
const int TRIG_PIN = 9;
const int ECHO_PIN = 8;

const unsigned long OLCUM_PERIYOT = 80;
const unsigned long ECHO_TIMEOUT  = 12000;   // ~206 cm
const unsigned int  MESAFE_YOK    = 999;

const unsigned int DUR_MESAFE     = 25;
const unsigned int YAVASLA_MESAFE = 60;
const int          YAVAS_PWM      = 130;

unsigned int mesafe = MESAFE_YOK;
unsigned long sonOlcum = 0;
bool engelVar = false;
uint8_t yakinSayac = 0;
uint8_t uzakSayac  = 0;
// pulseIn en fazla 12 ms bloklar. nRF24 bu sirada bir sey kaybetmez:
// RX FIFO 3 paket tutar, verici 20 ms'de bir yolluyor -> 12 ms'de en fazla
// 1 paket birikir, loop donunce okunur. HC-05/SoftwareSerial'deki sorunun
// nRF24'te karsiligi YOK, cunku alma isini modul kendi basina yapiyor.
// ======================================================================

// --- EEPROM ---
const int EEPROM_MIN_ADRES = 0;
unsigned int oncekiMinVcc = 0;

// --- L298N pin tanimlari (DEGISMEDI) ---
const int ENA = 5;
const int IN1 = 2;
const int IN2 = 3;
const int ENB = 6;
const int IN3 = 4;
const int IN4 = 7;

// --- Baglanti kopma korumasi ---
unsigned long sonMesaj = 0;
const unsigned long TIMEOUT = 400;
bool linkAktif = false;

// --- PWM rampasi ve yon degisimi korumasi (DEGISMEDI) ---
int komutSol = 0, komutSag = 0;
int hedefSol = 0, hedefSag = 0;
int simdikiSol = 0, simdikiSag = 0;
unsigned long yonBekleSol = 0, yonBekleSag = 0;
unsigned long sonRampa = 0;
const unsigned long RAMPA_PERIYOT = 20;
const int RAMPA_ADIM = 30;
const unsigned long YON_BEKLEME = 70;

// --- Fren durumu ---
unsigned long frenBitis = 0;
const unsigned long FREN_SURE = 250;

// --- Vcc izleme ---
unsigned int anlikVcc = 0;
unsigned int minVcc = 9999;
unsigned long sonVccOlcum = 0;
const unsigned long VCC_PERIYOT = 2;
unsigned long sonDebug = 0;

// ----------------------------------------------------------------------
// Arduino'nun kendi besleme gerilimini olcer (DEGISMEDI)
// ----------------------------------------------------------------------
unsigned int okuVcc() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delayMicroseconds(300);
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC));
  unsigned int sonuc = ADC;
  if (sonuc == 0) return 0;
  return (unsigned int)(1125300UL / sonuc);
}

void minVccKaydet(unsigned int deger) {
  EEPROM.put(EEPROM_MIN_ADRES, deger);
}

// ----------------------------------------------------------------------
// Kumandaya gidecek telemetriyi ACK kuyruguna koy.
// Bu veri, kumandadan gelen BIR SONRAKI paketin cevabiyla birlikte gider.
// Verici 20 ms'de bir gonderdigi icin telemetri en fazla 20 ms bayat olur.
// ----------------------------------------------------------------------
void telemetriHazirla() {
  Telemetri t;
  t.vcc    = anlikVcc;
  t.minVcc = (minVcc == 9999) ? 0 : minVcc;
  t.mesafe = mesafe;
  radio.writeAckPayload(1, &t, sizeof(t));
}

// ----------------------------------------------------------------------
// Radar olcumu (DEGISMEDI)
// ----------------------------------------------------------------------
void mesafeOlc() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long sure = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT);

  if (sure == 0) {
    mesafe = MESAFE_YOK;
  } else {
    unsigned int cm = (unsigned int)(sure / 58UL);
    if (cm < 2) cm = 2;
    mesafe = cm;
  }

  if (mesafe <= DUR_MESAFE) {
    if (yakinSayac < 10) yakinSayac++;
    uzakSayac = 0;
  } else if (mesafe > DUR_MESAFE + 5) {
    if (uzakSayac < 10) uzakSayac++;
    yakinSayac = 0;
  }

  if (yakinSayac >= 2) engelVar = true;
  if (uzakSayac  >= 3) engelVar = false;
}

// ----------------------------------------------------------------------
// Komutu radara gore sinirla (DEGISMEDI)
// ----------------------------------------------------------------------
void radarSinirla(int &sol, int &sag) {
  bool yerindeDonus = ((sol > 0 && sag < 0) || (sol < 0 && sag > 0));
  if (yerindeDonus) return;

  if (engelVar) {
    if (sol > 0) sol = 0;
    if (sag > 0) sag = 0;
  } else if (mesafe <= YAVASLA_MESAFE) {
    if (sol > YAVAS_PWM) sol = YAVAS_PWM;
    if (sag > YAVAS_PWM) sag = YAVAS_PWM;
  }
}

// ----------------------------------------------------------------------
// PWM olcekleme ve motor surme (DEGISMEDI)
// ----------------------------------------------------------------------
int pwmOlcekle(int hiz) {
  if (hiz <= 0) return 0;
  if (hiz > 255) hiz = 255;

  long tavan = (long)MAX_PWM * GUC_YUZDE / 100;
  if (tavan < MIN_PWM) tavan = MIN_PWM;

  return (int)(MIN_PWM + ((long)hiz * (tavan - MIN_PWM)) / 255);
}

void solMotor(int hiz) {
  bool ileriYon = (hiz >= 0);
  if (hiz < 0) hiz = -hiz;

  if (ileriYon != SOL_TERS) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else                      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }

  analogWrite(ENA, pwmOlcekle(hiz));
}

void sagMotor(int hiz) {
  bool ileriYon = (hiz >= 0);
  if (hiz < 0) hiz = -hiz;

  if (ileriYon != SAG_TERS) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else                      { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }

  analogWrite(ENB, pwmOlcekle(hiz));
}

void dur() {
  analogWrite(ENA, 0);     analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
}

void fren() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);   analogWrite(ENB, 255);
}

int rampa(int simdiki, int hedef, unsigned long &bekle) {
  unsigned long simdi = millis();

  bool yonDegisimi = ((long)hedef * (long)simdiki < 0);
  if (yonDegisimi) hedef = 0;

  if (simdiki == 0) {
    if (bekle != 0) {
      if (simdi < bekle) return 0;
      bekle = 0;
    }
  }

  int yeni = simdiki;
  if (hedef > yeni)      { yeni += RAMPA_ADIM; if (yeni > hedef) yeni = hedef; }
  else if (hedef < yeni) { yeni -= RAMPA_ADIM; if (yeni < hedef) yeni = hedef; }

  if (yonDegisimi && yeni == 0 && simdiki != 0) bekle = simdi + YON_BEKLEME;

  return yeni;
}

void motorlariGuncelle() {
  unsigned long simdi = millis();

  if (frenBitis != 0) {
    if (simdi < frenBitis) return;
    frenBitis = 0;
    dur();
    return;
  }

  if (simdi - sonRampa < RAMPA_PERIYOT) return;
  sonRampa = simdi;

  simdikiSol = rampa(simdikiSol, hedefSol, yonBekleSol);
  simdikiSag = rampa(simdikiSag, hedefSag, yonBekleSag);

  solMotor(simdikiSol);
  sagMotor(simdikiSag);
}

// Baglanti koptu: her seyi sifirla, frenle.
// HC-05 surumundeki resenkron modu YOK: tekrar paket gelir gelmez devam.
void linkKoptu() {
  linkAktif  = false;
  komutSol   = 0;  komutSag   = 0;
  hedefSol   = 0;  hedefSag   = 0;
  simdikiSol = 0;  simdikiSag = 0;
  yonBekleSol = 0; yonBekleSag = 0;

#if FREN_AKTIF
  fren();
  frenBitis = millis() + FREN_SURE;
#else
  dur();
  frenBitis = 0;
#endif
}

void setup() {
#if DEBUG
  Serial.begin(115200);
#endif

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  dur();                   // acilista motorlar kesin dursun
  linkAktif = false;

  // ------------------------- RADYOYU BASLAT -------------------------
  if (!radio.begin()) {
#if DEBUG
    Serial.println(F("nRF24 yok: SPI kablolari / 3.3V besleme kontrol"));
#endif
    while (1) { dur(); }   // radyo yoksa araba asla hareket etmemeli
  }

  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);     // verici ile AYNI olmali
  radio.setChannel(KANAL);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(5, 5);

  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openReadingPipe(1, ADRES);
  radio.startListening();              // biz aliciyiz (PRX)

  telemetriHazirla();                  // ilk ACK'e konacak veri hazir olsun

  // --- Onceki surusten kalan dip degeri oku ---
  EEPROM.get(EEPROM_MIN_ADRES, oncekiMinVcc);
  if (oncekiMinVcc < 2000 || oncekiMinVcc > 6000) oncekiMinVcc = 0;
#if DEBUG
  Serial.println();
  Serial.print(F("=== ONCEKI SURUSTE GORULEN EN DUSUK Vcc: "));
  if (oncekiMinVcc == 0) Serial.print(F("kayit yok"));
  else { Serial.print(oncekiMinVcc); Serial.print(F(" mV")); }
  Serial.println(F(" ==="));
#endif

  minVccKaydet(0xFFFF);
  minVcc = 9999;
}

void loop() {
  unsigned long simdi = millis();

  // --- Gelen komutlari oku ---
  // FIFO'yu TAMAMEN bosaltiyoruz; elimizde en son gelen (en taze) komut kalir.
  // HC-05'teki "bayat paket" problemi burada bu tek satirla cozuluyor.
  bool yeniPaket = false;
  CommandData cmd;
  while (radio.available()) {
    radio.read(&cmd, sizeof(cmd));
    yeniPaket = true;
  }

  if (yeniPaket) {
    if (cmd.sol >= -255 && cmd.sol <= 255 && cmd.sag >= -255 && cmd.sag <= 255) {
      komutSol = constrain(cmd.sol, -MAX_PWM, MAX_PWM);
      komutSag = constrain(cmd.sag, -MAX_PWM, MAX_PWM);
      sonMesaj = simdi;
      linkAktif = true;
    }
    telemetriHazirla();          // bir sonraki ACK icin taze veri yukle
  }

  if (linkAktif && (simdi - sonMesaj > TIMEOUT)) {
    linkKoptu();                 // sessizlik = kumanda gitti
  }

  // --- Radar ---
  if (simdi - sonOlcum >= OLCUM_PERIYOT) {
    sonOlcum = simdi;
    mesafeOlc();
  }

  // --- Ham komutu radara gore sinirla ---
  hedefSol = komutSol;
  hedefSag = komutSag;
  radarSinirla(hedefSol, hedefSag);

  // --- Besleme gerilimini ornekle ---
  if (simdi - sonVccOlcum >= VCC_PERIYOT) {
    sonVccOlcum = simdi;
    unsigned int v = okuVcc();
    if (v > 2000) {
      anlikVcc = v;
      if (v < minVcc) {
        minVcc = v;
        minVccKaydet(minVcc);
      }
    }
  }

  // --- Rampa + motor guncelleme ---
  motorlariGuncelle();

#if DEBUG
  if (simdi - sonDebug >= 500) {
    sonDebug = simdi;
    Serial.print(F("link="));   Serial.print(linkAktif ? 1 : 0);
    Serial.print(F(" sol="));   Serial.print(simdikiSol);
    Serial.print(F(" sag="));   Serial.print(simdikiSag);
    Serial.print(F(" Vcc="));   Serial.print(anlikVcc);
    Serial.print(F(" min="));   Serial.print(minVcc);
    Serial.print(F(" mes="));   Serial.print(mesafe);
    Serial.print(F(" engel=")); Serial.println(engelVar ? 1 : 0);
  }
#endif
}
