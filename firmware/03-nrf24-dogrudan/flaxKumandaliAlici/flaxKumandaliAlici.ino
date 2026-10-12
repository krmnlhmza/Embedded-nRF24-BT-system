// =====================================================================
//  ALICI (ARABA)
//  nRF24L01 + L298N (4 motor) + HC-SR04 radar
//
//  KUTUPHANE : "RF24" by TMRh20  (surum 1.4.0 veya ustu)
//  SERI PORT : 115200 baud   <-- Serial Monitor'u 115200'e AYARLA
// =====================================================================
//
//  ################  HC-05'TEN nRF24'E GECIS  ################
//
//  NE GITTI:
//   - SoftwareSerial ve HC-05 tamamen kaldirildi.
//   - Elle yazilmis '<' ... '>' cerceveleme, CRC-8 ve resenkronizasyon
//     kodu da kaldirildi. Gerek kalmadi: nRF24 donanimda CRC-16,
//     otomatik ACK ve otomatik tekrar gonderim yapiyor. Bozuk paket
//     zaten bize hic ulasmiyor.
//   - HC-05'in seri hattini isgal etmesi bitti; artik Serial tamamen
//     bizim, 115200 baud'da rahat rahat log basiyoruz.
//
//  NE KALDI (dokunulmadi):
//   - L298N motor surme, PWM rampasi, yon degisimi olu zamani, fren
//   - HC-SR04 radar ve histerezisli engel karari
//   - Vcc izleme + EEPROM'a kalici min Vcc kaydi
//   - 400 ms timeout: kumanda susarsa araba durur
//
//  ################  PIN DEGISIKLIGI  ################
//
//  nRF24 SPI kullaniyor: MOSI=D11, MISO=D12, SCK=D13 SABITTIR.
//  Eski kodda HC-05 D10/D11'deydi, o pinler artik bosaldi.
//  D11 SPI'a gitti, D12 ve D13 zaten bostu.
//
//  CE ve CSN icin standart pinleri koruyoruz: CE = D9, CSN = D10.
//  Bunun bedeli radari tasimak oldu: eskiden TRIG = D9, ECHO = D8'di,
//  simdi TRIG = A0, ECHO = A1. Sebebi, nRF24 ornek kodlarinin ve
//  soketli kartlarin neredeyse tamaminda CE/CSN'in D9/D10 olmasi;
//  boylece ileride modulu baska bir karta takarken pin degistirmen
//  gerekmiyor. Radar iki telli basit bir sensor, tasimasi kolay.
//
//  D8 ARTIK BOS. Ileride korna, far, ikinci sensor icin kullanabilirsin.
//
//  BAGLANTILAR:
//   nRF24L01:  CE  -> D9
//              CSN -> D10       <-- eskiden HC-05 RX'ti, bosaldi
//              MOSI-> D11
//              MISO-> D12
//              SCK -> D13
//              VCC -> 3.3V   (5V VERME, MODUL YANAR)
//              GND -> GND
//              VCC-GND arasina 10uF kondansator (uzun bacak = VCC)
//
//   L298N:     ENA->5, IN1->2, IN2->3, ENB->6, IN3->4, IN4->7
//              ENA/ENB jumper'lari CIKARILMIS olmali
//
//   HC-SR04:   TRIG->A0, ECHO->A1, VCC->5V, GND->GND     <-- YENI
//              (A0/A1 dijital giris-cikis olarak sorunsuz calisir;
//               pulseIn analog pinlerde de aynen calisir)
//
//  ANTENLI (PA+LNA) MODUL KULLANIYORSAN: modul tepe akimda ~115 mA
//  cekiyor. 3.3V'u Arduino'nun kendi regulatorunden aliyorsan
//  motor akim tepesinde brownout yasarsin. Ayri bir 3.3V regulator
//  + 10uF (mumkunse 100uF) kondansator kullan.
//
//  ################  RADAR DAVRANISI (DEGISMEDI)  ################
//   mesafe > 60 cm        -> serbest
//   25 cm < mesafe <= 60  -> ileri hiz 130 PWM ile sinirli
//   mesafe <= 25 cm       -> ILERI GIDIS ENGELLENIR
//  Geri gitmek ve yerinde donmek HER ZAMAN serbest; yoksa araba
//  engele yaklasinca kurtulamaz.
// =====================================================================

#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>

// ======================= AYAR ANAHTARLARI =============================
#define DEBUG       1     // 1: 500 ms'de bir ozet basar. SAHADA 0 YAP.
#define FREN_AKTIF  1     // 1: kopunca aktif fren, 0: serbest birak

const int MAX_PWM = 255;  // tepe hiz siniri (255 = tam guc)
// Regulator takildiktan sonra mantik beslemesi motorlardan ayrildi.
// min Vcc 4600 mV altina inerse buradan 230'a cekip dengele.
// ======================================================================

// ========================= TELSIZ =====================================
#define CE_PIN    9       // standart nRF24 pini (radar A0/A1'e tasindi)
#define CSN_PIN   10      // HC-05 gidince bosaldi

// VERICI ile BIREBIR AYNI olmali
const uint8_t ADRES[6] = "ARB01";
const uint8_t KANAL    = 108;
#define VERI_HIZI   RF24_250KBPS
#define RF_GUC      RF24_PA_MAX

RF24 radio(CE_PIN, CSN_PIN);
// ======================================================================

// ========================= PAKET YAPILARI =============================
// VERICI ile BIREBIR AYNI olmali. Tek bir alanini bile degistirirsen
// iki taraf anlasamaz.
const uint8_t KOMUT_IMZA = 0xA5;
const uint8_t TLM_IMZA   = 0x5A;

struct __attribute__((packed)) KomutPaketi {   // kumanda -> araba, 13 bayt
  uint8_t  imza;
  uint8_t  sayac;
  uint16_t joyX, joyY;    // ham ADC 0..1023 (kumandadaki tek joystick)
  uint16_t pot;           // ham ADC 0..1023 (kullanilmiyorsa 0)
  uint8_t  butonlar;      // bit0=Buton1, bit1=Buton2, bit2=Buton4
  int16_t  sol, sag;      // kumandada hesaplanmis hizlar -255..+255
};

#define BIT_BUTON1   0x01
#define BIT_BUTON2   0x02
#define BIT_BUTON4   0x04

struct __attribute__((packed)) TelemetriPaketi { // araba -> kumanda, 8 bayt
  uint8_t  imza;
  uint16_t vcc;
  uint16_t minVcc;
  uint16_t mesafe;
  uint8_t  durum;         // bit0 = engel var
};

#define TLM_BIT_ENGEL  0x01

TelemetriPaketi tlm;      // ACK'in icine konacak guncel telemetri
// ======================================================================

// ========================= RADAR (HC-SR04) ============================
const int TRIG_PIN = A0;   // YENI: eskiden D9'du, CE oraya gecti
const int ECHO_PIN = A1;   // YENI: eskiden D8'di, D8 artik bos

const unsigned long OLCUM_PERIYOT = 60;      // ms: saniyede ~16 olcum
const unsigned long ECHO_TIMEOUT  = 4500;    // us: ~77 cm otesini beklemeyiz
const unsigned int  MESAFE_YOK    = 999;     // yanki gelmedi = onu bos say

const unsigned int DUR_MESAFE     = 25;      // cm: ileri kapanir
const unsigned int YAVASLA_MESAFE = 60;      // cm: yavaslamaya basla
const int          YAVAS_PWM      = 130;     // yaklasma bolgesi ust siniri

unsigned int mesafe = MESAFE_YOK;
unsigned long sonOlcum = 0;
bool engelVar = false;
uint8_t yakinSayac = 0;
uint8_t uzakSayac  = 0;
// ======================================================================

// --- EEPROM: kalici min Vcc kaydi ---
const int EEPROM_MIN_ADRES = 0;
unsigned int oncekiMinVcc = 0;

// --- L298N pin tanimlari (DEGISMEDI) ---
const int ENA = 5;   // sol grup hiz (PWM)
const int IN1 = 2;   // sol grup yon ucu 1
const int IN2 = 3;   // sol grup yon ucu 2
const int ENB = 6;   // sag grup hiz (PWM)
const int IN3 = 4;   // sag grup yon ucu 1
const int IN4 = 7;   // sag grup yon ucu 2

// --- Baglanti kopma korumasi ---
unsigned long sonMesaj = 0;
const unsigned long TIMEOUT = 400;   // bu kadar sessizlik olursa DUR
bool linkAktif = false;

// --- Paket istatistikleri ---
unsigned long toplamPaket = 0, bozukPaket = 0, kayipPaket = 0;
uint8_t sonSayac = 0;
bool    ilkPaket = true;
unsigned long sayimBaslangic = 0, sayimPaket = 0;

// --- Son gelen komutun buton durumu (simdilik sadece raporlaniyor) ---
uint8_t sonButonlar = 0;

// --- PWM rampasi ve yon degisimi korumasi ---
int komutSol = 0, komutSag = 0;            // kumandadan gelen HAM komut
int hedefSol = 0, hedefSag = 0;            // radar sinirlamasindan SONRAKI hedef
int simdikiSol = 0, simdikiSag = 0;        // motorlara fiilen uygulanan hiz
unsigned long yonBekleSol = 0, yonBekleSag = 0;
unsigned long sonRampa = 0;
const unsigned long RAMPA_PERIYOT = 20;    // ms
const int RAMPA_ADIM = 30;                 // 0'dan 255'e ~170 ms
const unsigned long YON_BEKLEME = 70;      // ms

// --- Fren durumu ---
unsigned long frenBitis = 0;
const unsigned long FREN_SURE = 250;

// --- Vcc izleme ---
unsigned int anlikVcc = 0;
unsigned int minVcc = 9999;
unsigned long sonVccOlcum = 0;
const unsigned long VCC_PERIYOT = 2;       // ms
unsigned long sonDebug = 0;

// =====================================================================
//  YARDIMCI FONKSIYONLAR
// =====================================================================

void cizgi() {
  Serial.println(F("---------------------------------------------"));
}

// ----------------------------------------------------------------------
// Arduino'nun kendi besleme gerilimini olcer (ATmega328P).
// Dahili 1.1V bandgap referansini Vcc'ye gore okur -> ekstra donanim yok.
// ----------------------------------------------------------------------
unsigned int okuVcc() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // 1.1V bandgap
  delayMicroseconds(300);              // referansin oturmasi icin
  ADCSRA |= _BV(ADSC);                 // donusumu baslat
  while (ADCSRA & _BV(ADSC));          // bitmesini bekle
  unsigned int sonuc = ADC;
  if (sonuc == 0) return 0;
  return (unsigned int)(1125300UL / sonuc);   // 1.1 * 1023 * 1000 / ADC
}

// Yeni bir dip deger goruldugunde EEPROM'a yaz.
// Sadece DUSTUGUNDE yaziyoruz -> bir suruste 10-20 yazma olur.
// EEPROM omru ~100.000 cevrim, sorun degil.
void minVccKaydet(unsigned int deger) {
  EEPROM.put(EEPROM_MIN_ADRES, deger);
}

// ----------------------------------------------------------------------
// Telemetriyi guncelle ve ACK kuyruguna koy.
// nRF24'te ACK payload'i ONCEDEN yuklenir: kumandadan bir sonraki
// paket geldiginde donanim bu veriyi cevabin icine koyup yollar.
// Bu yuzden her paketi isledikten hemen sonra tazeliyoruz.
// ----------------------------------------------------------------------
void telemetriyiHazirla() {
  tlm.imza   = TLM_IMZA;
  tlm.vcc    = anlikVcc;
  tlm.minVcc = (minVcc == 9999) ? 0 : minVcc;
  tlm.mesafe = mesafe;
  tlm.durum  = engelVar ? TLM_BIT_ENGEL : 0;

  radio.writeAckPayload(1, &tlm, sizeof(tlm));
}

// ----------------------------------------------------------------------
// HC-SR04 ile mesafe olc ve engel kararini guncelle.
// pulseIn en fazla ECHO_TIMEOUT kadar bloklar (4.5 ms).
// ----------------------------------------------------------------------
void mesafeOlc() {
  digitalWrite(TRIG_PIN, LOW);        // temiz bir baslangic
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);       // 10 us'lik tetik darbesi
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long sure = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT);

  if (sure == 0) {
    mesafe = MESAFE_YOK;              // yanki yok: menzilde engel yok say
  } else {
    unsigned int cm = (unsigned int)(sure / 58UL);   // 58 us = 1 cm gidis-donus
    if (cm < 2) cm = 2;               // sensorun olu bolgesi
    mesafe = cm;
  }

  // Histerezis: tek bir sacma olcum arabayi durdurmasin
  if (mesafe <= DUR_MESAFE) {
    if (yakinSayac < 10) yakinSayac++;
    uzakSayac = 0;
  } else if (mesafe > DUR_MESAFE + 5) {     // +5 cm bosluk: sinirda zipzip olmasin
    if (uzakSayac < 10) uzakSayac++;
    yakinSayac = 0;
  }

  if (yakinSayac >= 2) engelVar = true;     // 2 yakin olcum -> engel var
  if (uzakSayac  >= 3) engelVar = false;    // 3 uzak olcum  -> yol acildi
}

// ----------------------------------------------------------------------
// Komutu radara gore sinirla.
// Kural: SADECE ileri giden tekerler kisitlanir.
//   - Geri gitmek her zaman serbest (engelden kurtulmak icin sart)
//   - Yerinde donus her zaman serbest
// ----------------------------------------------------------------------
void radarSinirla(int &sol, int &sag) {
  bool yerindeDonus = ((sol > 0 && sag < 0) || (sol < 0 && sag > 0));
  if (yerindeDonus) return;

  if (engelVar) {                           // DUR bolgesi: ileri kapali
    if (sol > 0) sol = 0;
    if (sag > 0) sag = 0;
  } else if (mesafe <= YAVASLA_MESAFE) {    // YAVASLA bolgesi
    if (sol > YAVAS_PWM) sol = YAVAS_PWM;
    if (sag > YAVAS_PWM) sag = YAVAS_PWM;
  }
}

// ----------------------------------------------------------------------
// Motor surme yardimcilari: +hiz ileri, -hiz geri, 0 dur
// TEKERLER TERS DONERSE: ilgili fonksiyondaki HIGH ve LOW'u degistir.
// ----------------------------------------------------------------------
void solMotor(int hiz) {
  if (hiz >= 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else          { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); hiz = -hiz; }
  analogWrite(ENA, hiz);
}

void sagMotor(int hiz) {
  if (hiz >= 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else          { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); hiz = -hiz; }
  analogWrite(ENB, hiz);
}

// Iki grubu birden serbest birak (coast): motor bosa doner
void dur() {
  analogWrite(ENA, 0);     analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
}

// Aktif fren: her iki yon ucu ayni seviyede -> motor sargisi kisa devre
void fren() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);   analogWrite(ENB, 255);
}

// ----------------------------------------------------------------------
// Rampa: simdiki hizi hedefe kucuk adimlarla yaklastirir.
// Yon degisimi istenirse once 0'a iner, YON_BEKLEME kadar bekler.
// ----------------------------------------------------------------------
int rampa(int simdiki, int hedef, unsigned long &bekle) {
  unsigned long simdi = millis();

  bool yonDegisimi = ((long)hedef * (long)simdiki < 0);   // isaret degisiyor mu
  if (yonDegisimi) hedef = 0;             // once mutlaka 0'a in

  if (simdiki == 0) {
    if (bekle != 0) {
      if (simdi < bekle) return 0;        // olu zaman dolmadi, kalkma
      bekle = 0;
    }
  }

  int yeni = simdiki;
  if (hedef > yeni)      { yeni += RAMPA_ADIM; if (yeni > hedef) yeni = hedef; }
  else if (hedef < yeni) { yeni -= RAMPA_ADIM; if (yeni < hedef) yeni = hedef; }

  if (yonDegisimi && yeni == 0 && simdiki != 0) bekle = simdi + YON_BEKLEME;

  return yeni;
}

// Rampayi isle ve motorlara uygula (fren surerken dokunma)
void motorlariGuncelle() {
  unsigned long simdi = millis();

  if (frenBitis != 0) {                 // fren devrede mi
    if (simdi < frenBitis) return;      // suresi dolmadi
    frenBitis = 0;
    dur();                              // freni birak, serbest kal
    return;
  }

  if (simdi - sonRampa < RAMPA_PERIYOT) return;
  sonRampa = simdi;

  simdikiSol = rampa(simdikiSol, hedefSol, yonBekleSol);
  simdikiSag = rampa(simdikiSag, hedefSag, yonBekleSag);

  solMotor(simdikiSol);
  sagMotor(simdikiSag);
}

// Baglanti koptu: her seyi sifirla, frenle
void linkKoptu() {
  linkAktif  = false;
  ilkPaket   = true;
  komutSol   = 0;  komutSag   = 0;
  hedefSol   = 0;  hedefSag   = 0;
  simdikiSol = 0;  simdikiSag = 0;      // rampayi bekleme, ANINDA sifirla
  yonBekleSol = 0; yonBekleSag = 0;

#if FREN_AKTIF
  fren();
  frenBitis = millis() + FREN_SURE;
#else
  dur();
  frenBitis = 0;
#endif

#if DEBUG
  Serial.println(F("!!! KUMANDA KOPTU -> DURDURULDU"));
#endif
}

// ----------------------------------------------------------------------
// Telsiz kurulumu. Basarisizsa sonsuz donguye girer (araba hareket etmez).
// ----------------------------------------------------------------------
void radyoKur() {
  Serial.println(F("[2/4] nRF24L01 baslatiliyor..."));

  if (!radio.begin()) {
    cizgi();
    Serial.println(F("HATA 1: nRF24 modulu hic yanit vermiyor."));
    Serial.println(F("  - CE=D9, CSN=D10, MOSI=D11, MISO=D12, SCK=D13"));
    Serial.println(F("  - VCC kesinlikle 3.3V olacak (5V modulu yakar)"));
    Serial.println(F("  - VCC-GND arasina 10uF kondansator tak"));
    Serial.println(F("  - Jumper'lari tek tek oynat, temassizlik cok yaygin"));
    cizgi();
    while (1) { dur(); delay(1000); }     // motorlar kesin dursun
  }

  if (!radio.isChipConnected()) {
    cizgi();
    Serial.println(F("HATA 2: SPI hatti kararsiz, cip okunamiyor."));
    Serial.println(F("  - MISO(D12) ve MOSI(D11) ters olabilir"));
    Serial.println(F("  - Kablolari kisalt"));
    cizgi();
    while (1) { dur(); delay(1000); }
  }

  radio.setChannel(KANAL);
  radio.setDataRate(VERI_HIZI);
  radio.setPALevel(RF_GUC);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);

  // ACK payload (telemetri) icin dinamik payload sart
  radio.enableDynamicPayloads();
  radio.enableAckPayload();

  if (radio.getChannel() != KANAL) {
    cizgi();
    Serial.println(F("HATA 3: Ayarlar module yazilamiyor."));
    Serial.println(F("  Besleme yetersiz olabilir. 3.3V + 10uF sart."));
    cizgi();
    while (1) { dur(); delay(1000); }
  }

  radio.openReadingPipe(1, ADRES);
  radio.startListening();               // biz dinliyoruz (PRX)

  Serial.println(F("      SONUC: telsiz OK"));
}

void radyoBilgisiYaz() {
  cizgi();
  Serial.println(F("TELSIZ AYARLARI (verici ile AYNI olmali):"));
  Serial.print(F("  Kanal        : ")); Serial.println(radio.getChannel());
  Serial.print(F("  Veri hizi    : "));
  switch (radio.getDataRate()) {
    case RF24_250KBPS: Serial.println(F("250KBPS")); break;
    case RF24_1MBPS:   Serial.println(F("1MBPS"));   break;
    case RF24_2MBPS:   Serial.println(F("2MBPS"));   break;
  }
  Serial.print(F("  Adres        : ")); Serial.println((const char*)ADRES);
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
    Serial.println(F("  >> VERI_HIZI'ni RF24_1MBPS yap (iki tarafta da)."));
  }
  cizgi();
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);                 // HC-05 gitti, hat tamamen bizim
  while (!Serial && millis() < 2000) { }

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  dur();                   // acilista motorlar KESIN dursun
  linkAktif = false;       // ilk gecerli pakete kadar hicbir komut uygulanmaz

  delay(300);
  cizgi();
  Serial.println(F("  ALICI : ARABA + nRF24L01 + L298N + RADAR"));
  Serial.println(F("  Serial Monitor 115200 baud olmali"));
  cizgi();
  Serial.println(F("[1/4] Kart uyandi, motorlar durduruldu."));

  // --- Onceki surusten kalan dip degeri oku ve raporla ---
  EEPROM.get(EEPROM_MIN_ADRES, oncekiMinVcc);
  if (oncekiMinVcc < 2000 || oncekiMinVcc > 6000) oncekiMinVcc = 0;  // bos/bozuk
  Serial.print(F("      ONCEKI SURUSTE EN DUSUK Vcc: "));
  if (oncekiMinVcc == 0) Serial.println(F("kayit yok"));
  else { Serial.print(oncekiMinVcc); Serial.println(F(" mV")); }
  Serial.println(F("      (4600 mV alti = besleme cokuyor demektir)"));

  radyoKur();
  radyoBilgisiYaz();

  // Ilk ACK payload'i simdiden kuyruga koy ki ilk pakete de cevap
  // verirken telemetri gitsin.
  anlikVcc = okuVcc();
  telemetriyiHazirla();

  Serial.println(F("[3/4] Kumandayi ac ve joystick'i oynat."));
  Serial.println(F("[4/4] Hazir."));
  cizgi();

  // Yeni surus icin kaydi temizle
  minVccKaydet(0xFFFF);
  minVcc = 9999;

  sayimBaslangic = millis();
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
  unsigned long simdi = millis();

  // ---------------- 1) Gelen paketleri oku ----------------
  // Tamponda birikmis TUM paketleri oku, sadece EN SONUNCUSUNU kullan.
  // Tek tek islersek bayat komutla is gormus oluruz.
  bool yeni = false;
  KomutPaketi p;

  while (radio.available()) {
    uint8_t boy = radio.getDynamicPayloadSize();

    if (boy != sizeof(KomutPaketi)) {
      // Beklenmedik boy: baska bir cihazin paketi ya da bozuk veri.
      uint8_t cop[32];
      radio.read(cop, (boy > 32) ? 32 : boy);
      bozukPaket++;
      continue;
    }

    KomutPaketi gecici;
    radio.read(&gecici, sizeof(gecici));
    toplamPaket++;

    if (gecici.imza != KOMUT_IMZA) {   // imza tutmuyorsa cop
      bozukPaket++;
      continue;
    }

    // Kayip paket sayimi (sayac atlamasindan)
    if (!ilkPaket) {
      uint8_t fark = (uint8_t)(gecici.sayac - sonSayac);
      if (fark > 1) kayipPaket += (fark - 1);
    }
    sonSayac = gecici.sayac;
    ilkPaket = false;

    p = gecici;
    yeni = true;
    sayimPaket++;
  }

  // ---------------- 2) Yeni komut geldiyse uygula ----------------
  if (yeni) {
    sonMesaj = simdi;

    if (!linkAktif) {
      linkAktif = true;
      Serial.println(F("*** KUMANDA BAGLANDI ***"));
    }

    // Aralik disi deger gelirse (bozuk veri) komutu uygulama
    if (p.sol >= -255 && p.sol <= 255 && p.sag >= -255 && p.sag <= 255) {
      komutSol = constrain(p.sol, -MAX_PWM, MAX_PWM);
      komutSag = constrain(p.sag, -MAX_PWM, MAX_PWM);
    }

    sonButonlar = p.butonlar;
    // NOT: butonlar su an sadece raporlaniyor, bir gorevi yok.
    // Ornegin B1'i korna, B2'yi far yapmak istersen burada islersin.

    // Cevap kuyrugundaki telemetriyi tazele
    telemetriyiHazirla();
  }

  // ---------------- 3) Sessizlik kontrolu ----------------
  if (linkAktif && (simdi - sonMesaj > TIMEOUT)) {
    linkKoptu();
  }

  // ---------------- 4) Radar: mesafeyi olc (60 ms'de bir) ----------------
  if (simdi - sonOlcum >= OLCUM_PERIYOT) {
    sonOlcum = simdi;
    mesafeOlc();
  }

  // ---------------- 5) Ham komutu radara gore sinirla ----------------
  // Her dongude yeniden hesapliyoruz: joystick sabit dursa bile onumuze
  // bir engel cikarsa araba hemen tepki vermeli.
  hedefSol = komutSol;
  hedefSag = komutSag;
  radarSinirla(hedefSol, hedefSag);

  // ---------------- 6) Besleme gerilimini ornekle (2 ms'de bir) ------
  // Motor akim tepeleri onlarca ms surer, 2 ms ornekleme rahat yakalar.
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

  // ---------------- 7) Rampa + motor guncelleme ----------------
  motorlariGuncelle();

  // ---------------- 8) Saniyede kac paket geliyor ----------------
  if (simdi - sayimBaslangic >= 3000) {
#if DEBUG
    if (linkAktif) {
      Serial.print(F("[link] saniyede ~"));
      Serial.print(sayimPaket / 3);
      Serial.print(F(" paket (beklenen ~50)  kayip="));
      Serial.print(kayipPaket);
      if (bozukPaket) { Serial.print(F("  bozuk=")); Serial.print(bozukPaket); }
      Serial.println();
    } else {
      Serial.println(F("[link] paket gelmiyor. Kumanda acik mi?"));
      Serial.println(F("       Kanal/adres/veri hizi iki tarafta ayni mi?"));
    }
#endif
    sayimBaslangic = simdi;
    sayimPaket = 0;
  }

  // ---------------- 9) Debug ozeti ----------------
#if DEBUG
  if (simdi - sonDebug >= 500) {
    sonDebug = simdi;
    Serial.print(F("link="));   Serial.print(linkAktif ? 1 : 0);
    Serial.print(F(" sol="));   Serial.print(simdikiSol);
    Serial.print(F(" sag="));   Serial.print(simdikiSag);
    Serial.print(F(" Vcc="));   Serial.print(anlikVcc);
    Serial.print(F(" min="));   Serial.print(minVcc);
    Serial.print(F(" mes="));   Serial.print(mesafe);
    Serial.print(F(" engel=")); Serial.print(engelVar ? 1 : 0);
    Serial.print(F(" B1="));    Serial.print((sonButonlar & BIT_BUTON1) ? 1 : 0);
    Serial.print(F(" B2="));    Serial.println((sonButonlar & BIT_BUTON2) ? 1 : 0);
  }
#endif
}
