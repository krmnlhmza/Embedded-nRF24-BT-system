// =========================== ALICI (ARABA) ===========================
// HC-05'ten binary paket okur, CRC dogrular, L298N ile 4 motoru surer.
//   Sol 2 motor paralel -> OUT1/OUT2
//   Sag 2 motor paralel -> OUT3/OUT4
//
// BAGLANTILAR:
//   HC-05: TXD -> pin 0 (RX), RXD -> pin 1 (TX), VCC -> 5V, GND -> GND
//          *** DEGISTI: eskiden pin 10/11 idi. Gerilim bolucuyu de pin 1'e tasi.
//          *** KOD YUKLERKEN HC-05'I CIKAR (kumandada zaten boyle yapiyorsun).
//   L298N: ENA->5, IN1->2, IN2->3, ENB->6, IN3->4, IN4->7
//   ENA/ENB jumper'lari CIKARILMIS olmali (yoksa hiz kontrolu calismaz)
//   HC-SR04: TRIG -> pin 9, ECHO -> pin 8
//   Pin 10 ve 11 artik BOS.
//
// ---------------------- ONCEKI SURUMDE YAPILANLAR --------------------
// [1] BT.setTimeout(20): readBytes'in 1 saniyelik blokajini onler
// [2] DEBUG anahtari: her pakette Serial.print bloklamasin diye
// [3] RESENKRONIZASYON: yeniden baglaninca bayat paketleri cope atar
// [4] CRC-8 + sira numarasi: bozuk/tekrarlanan paket eleme
// [5] PWM rampasi + yon degisimi korumasi: akim tepesini dusurur
// [6] MAX_PWM: tepe akim siniri (artik 255 = tam guc)
// [7] Aktif fren
// [8] Vcc izleme
//
// ============ BU SURUMDE YENI: RADAR ARTIK GERCEKTEN OKUNUYOR ==========
// SORUN NEYDI:
//   Radar tek basina calisiyordu ama projede kumandanin ekraninda surekli
//   "M:---cm" yaziyordu. Iki ayri sebebi vardi:
//
//   (1) ECHO_TIMEOUT 4500 us idi -> 4500/58 = sadece 77 cm. Bundan uzagi
//       pulseIn beklemeden birakiyor, mesafe 999 (=yok) oluyordu. Arabanin
//       onunde 77 cm'den yakin bir sey olmadikca ekran hep "---" gosterir.
//
//   (2) ASIL SEBEP: SoftwareSerial bir byte alirken KESMELERI KAPATIP byte
//       bitene kadar ISR icinde bekliyor (9600 baud'da ~1.04 ms/byte).
//       Verici 20 ms'de bir 8 byte yolluyordu -> her 20 ms'nin ~8.3 ms'sinde
//       CPU tamamen kilitli. pulseIn o sirada donuyor: yanki darbesi o
//       araliga denk gelirse HIC GORULMUYOR (-> 999), yarisina denk gelirse
//       KISA olculuyor (-> ortada engel yokken "engel var" karari).
//       Olcumlerin kabaca ucte ikisi bu yuzden bozuluyordu.
//
// COZUM: SoftwareSerial tamamen kaldirildi, HC-05 donanim seri porta
//   (pin 0/1) tasindi. Donanim UART'in kesmesi byte basina birkac
//   MIKROsaniye surer ve 64 byte'lik tamponu vardir; pulseIn'i bloklamaz,
//   pulseIn de veri kaybettirmez. Radar artik dogru olcuyor.
//
//   Bedeli: Serial hatti HC-05'e ait oldugu icin USB Serial Monitor ile
//   debug YAPILAMAZ. Zaten Vcc/minVcc/mesafe kumandanin LCD'sinde canli
//   goruluyor, ihtiyac kalmadi.
//
// HC-SR04 baglantisi: TRIG -> pin 9, ECHO -> pin 8, VCC -> 5V, GND -> GND
//
// DAVRANIS:
//   mesafe > 60 cm        -> serbest, hicbir sinirlama yok
//   25 cm < mesafe <= 60  -> ileri hiz 130 PWM ile sinirlanir (yaklasma)
//   mesafe <= 25 cm       -> ILERI GIDIS ENGELLENIR
// Geri gitmek ve yerinde donmek HER ZAMAN serbesttir; yoksa araba engele
// yaklastiginda kurtulamaz, kilitlenip kalirdi.
//
// YANLIS ALARMA KARSI: HC-SR04 arada bir sacma deger dondurur. Bu yuzden
// engel karari tek olcume degil, ust uste 2 yakin olcume dayaniyor; birakmak
// icin de 3 uzak olcum gerekiyor (histerezis).
//
// NEDEN pulseIn yeterli: donanim UART'a gecince pulseIn'in blokladigi sure
// (en fazla 12 ms) hicbir sey kaybettirmiyor. UART'in 64 byte'lik donanim
// tamponu var; 12 ms'de gelen ~5 byte tamponda bekler, loop donunce okunur.
//
// ------------------- ONCEKI SURUMDE YENI OLAN (TELEMETRI) --------------
// Sorun: arabayi pille suruyorsun, USB takili degil -> Serial Monitor'deki
// "min=" satirini goremiyorsun. USB'yi sonradan taksan da Serial Monitor
// acilirken Arduino RESET atiyor ve minVcc siliniyor.
//
// Iki cozum birden eklendi:
//   (A) EEPROM: gorulen en dusuk Vcc kalici olarak saklanir. Bir sonraki
//       acilista Serial'e "ONCEKI SURUS" satiri olarak basilir. Reset
//       silmez, pil bitse de kaybolmaz.
//   (B) TELEMETRI: heartbeat paketi artik Vcc ve minVcc degerlerini de
//       tasiyor. Kumandanin LCD'sinde CANLI gorunuyor. Hangi manevrada
//       gerilimin coktugunu aninda gorursun.
//
// Heartbeat paketi (araba -> kumanda), 7 byte:
//   0xA5 0x5A [vccL] [vccH] [minL] [minH] [crc8]
// ======================================================================

#include <EEPROM.h>                 // kalici min Vcc kaydi icin

// [DEGISTI] SoftwareSerial kaldirildi. HC-05 artik donanim UART'ta.
// Kodun geri kalanindaki tum "BT." cagrilari aynen calissin diye takma ad:
HardwareSerial &BT = Serial;   // HC-05: TXD -> pin 0, RXD -> pin 1

// ======================= AYAR ANAHTARLARI =============================
#define DEBUG       0     // [DEGISTI] ARTIK 1 YAPMA! Serial hatti HC-05'e ait;
                          // debug ciktisi kumandaya cop veri olarak gider.
                          // Izlemek istedigin her sey zaten LCD'de.
#define FREN_AKTIF  1     // 1: kopunca aktif fren, 0: serbest birak (coast)

const int MAX_PWM = 255;  // tepe PWM siniri (255 = tam guc)
// Regulator takildiktan sonra mantik beslemesi motorlardan ayrildi, artik
// akim tepesi HC-05'i resetlemiyor. Bu yuzden tepe hiz sinirini kaldirdik.
// LCD'deki MIN Vcc 4600 mV altina inerse buradan 230'a cekip dengele.

// ------------------- [YENI] MOTOR YON ANAHTARLARI ---------------------
// Bir teker grubu ters donuyorsa KABLOYU SOKME, buradan cevir.
//
// NASIL BULUNUR: arabayi kutuya koy, tekerler havada serbest donsun.
// Kumandayi bagla, joystick'i DUZ ILERI it (LCD'de "S:255  G:255" yazmali):
//    sol iki teker GERI donuyorsa -> SOL_TERS'i true yap
//    sag iki teker GERI donuyorsa -> SAG_TERS'i true yap
//    dordu de geri donuyorsa      -> ikisini de true yap
//
// DIKKAT: ayni grubu hem buradan hem kablodan cevirme, basa donersin.
// Sadece bir yontem sec.
const bool SOL_TERS = false;
const bool SAG_TERS = false;

// --------------------- [YENI] HIZ HISSI AYARLARI ----------------------
// MIN_PWM: motorun DONMEYE BASLADIGI esik. Bunun altindaki PWM'de motor
//   donmez, sadece zorlanip titrer -- joystick'i hafif ittiginde arabanin
//   "gidi gidi" yapip sonra birden firlamasinin sebebi budur. Sifirdan
//   buyuk verirsen kod 0..255 arasini MIN_PWM..tavan araligina yayar,
//   yani en kucuk joystick hareketi bile duzgun bir donuse karsilik gelir.
//
//   VARSAYILAN 0 = KAPALI, davranis eskisiyle birebir ayni. Denemek icin:
//   arabayi yere koy, joystick'i safe zone'u gececek kadar HAFIFCE it.
//     - tekerlek donmuyor / titriyorsa -> MIN_PWM'i 5'er artir
//     - en hafif itiste firliyorsa     -> 5'er azalt
//   Tipik oturma araligi 60-90'dir, motoruna ve piline gore degisir.
const int MIN_PWM = 0;

// GUC_YUZDE: genel guc siniri. 100 = tam guc, 60 = belirgin yavas mod.
//   Kapali alanda denerken 50-60 arabayi cok daha kontrol edilebilir kilar.
//   MIN_PWM tabani korunur, sadece tavan asagi ceker.
const int GUC_YUZDE = 100;
// ======================================================================

// ========================= RADAR (HC-SR04) ============================
const int TRIG_PIN = 9;    // tetik cikisi  (bos pin)
const int ECHO_PIN = 8;    // yanki girisi  (bos pin)

const unsigned long OLCUM_PERIYOT = 80;      // [DEGISTI] 60 -> 80 ms: saniyede ~12 olcum
const unsigned long ECHO_TIMEOUT  = 12000;   // [DEGISTI] 4500 -> 12000 us
// 12000/58 = ~206 cm. Artik ekranda gercek mesafe okunuyor, sadece 77 cm'lik
// dar bir bant degil. Yanki yoksa pulseIn bu kadar bloklar; 80 ms periyotta
// en kotu ihtimalle zamanin %15'i. UART tamponu bunu sorunsuz karsiliyor.
const unsigned int  MESAFE_YOK    = 999;     // yanki gelmedi = onu bos say

const unsigned int DUR_MESAFE     = 25;      // cm: bu mesafede ileri kapanir
const unsigned int YAVASLA_MESAFE = 60;      // cm: bu mesafede yavaslamaya basla
const int          YAVAS_PWM      = 130;     // yaklasma bolgesindeki ust sinir

unsigned int mesafe = MESAFE_YOK;   // son gecerli olcum (cm)
unsigned long sonOlcum = 0;         // en son ne zaman olctuk
bool engelVar = false;              // ileri gidis engellendi mi
uint8_t yakinSayac = 0;             // ust uste kac yakin olcum geldi
uint8_t uzakSayac  = 0;             // ust uste kac uzak olcum geldi
// ======================================================================

// --- [YENI] EEPROM adresleri ---
const int EEPROM_MIN_ADRES = 0;     // uint16_t: gorulen en dusuk Vcc (mV)
unsigned int oncekiMinVcc = 0;      // bir onceki surusten kalan deger

// --- Paket yapisi: verici ile BIREBIR AYNI olmali ---
struct __attribute__((packed)) CommandData {
  int16_t sol;   // sol motor grubu hizi: -255 (geri) .. +255 (ileri)
  int16_t sag;   // sag motor grubu hizi: -255 .. +255
  uint8_t seq;   // sira numarasi
};
// Kablodaki paket: '<' + 5 byte veri + 1 byte CRC + '>'  (toplam 8 byte)

// --- CRC-8 (poly 0x31) : verici ile BIREBIR AYNI olmali ---
uint8_t crc8(const uint8_t *veri, uint8_t n) {
  uint8_t crc = 0xFF;
  while (n--) {
    crc ^= *veri++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31)
                         : (uint8_t)(crc << 1);
  }
  return crc;
}

// --- L298N pin tanimlari (DEGISMEDI) ---
const int ENA = 5;   // sol grup hiz (PWM olmali - D5 uygun)
const int IN1 = 2;   // sol grup yon ucu 1
const int IN2 = 3;   // sol grup yon ucu 2
const int ENB = 6;   // sag grup hiz (PWM olmali - D6 uygun)
const int IN3 = 4;   // sag grup yon ucu 1
const int IN4 = 7;   // sag grup yon ucu 2

// --- Baglanti kopma korumasi ---
unsigned long sonMesaj = 0;          // son gecerli paketin geldigi an (ms)
const unsigned long TIMEOUT = 400;   // bu sureden uzun sessizlik olursa DUR

// --- Link durumu ve resenkronizasyon ---
bool linkAktif = false;                    // gecerli paket akisi var mi
unsigned long resenkronBasladi = 0;        // cope atmaya ne zaman basladik
unsigned long sonResenkronByte = 0;        // resenkron sirasinda son byte ani
const unsigned long RESENKRON_SURE = 300;  // bu kadar sure gelen her sey cope

// --- Sira numarasi takibi ---
uint8_t sonSeq = 0;         // en son kabul edilen paketin sira numarasi
bool seqGecerli = false;    // henuz referans seq'imiz var mi

// --- Heartbeat / telemetri: kumandaya "buradayim + gerilimim bu" ---
const uint8_t HB1 = 0xA5;                      // verici ile AYNI olmali
const uint8_t HB2 = 0x5A;
const unsigned long HEARTBEAT_ARALIK = 200;    // ms
// [NOT] Eskiden bu deger SoftwareSerial yuzunden uzun tutulmustu: yazarken
// RX kor kaliyordu. Donanim UART tam cift yonlu calisir, gonderirken alim
// hic durmaz. 200 ms'yi yine de degistirmedik: kumandanin LCD'si zaten
// 250 ms'den sik yenilenmiyor, daha sik yollamanin faydasi yok.
unsigned long sonHeartbeat = 0;                // en son ne zaman yolladik

// --- PWM rampasi ve yon degisimi korumasi ---
int komutSol = 0, komutSag = 0;            // [YENI] kumandadan gelen HAM komut
int hedefSol = 0, hedefSag = 0;            // radar sinirlamasindan SONRAKI hedef
int simdikiSol = 0, simdikiSag = 0;        // motorlara fiilen uygulanan hizlar
unsigned long yonBekleSol = 0, yonBekleSag = 0;  // yon degisimi olu zamani
unsigned long sonRampa = 0;                // rampayi en son ne zaman isledik
const unsigned long RAMPA_PERIYOT = 20;    // ms: rampa guncelleme araligi
const int RAMPA_ADIM = 30;                 // [DEGISTI] 12 -> 30: hizli kalkis
// 30/20ms -> 0'dan 255'e ~170 ms. Kullanicinin fark edemeyecegi kadar hizli
// ama PWM'i tek hamlede 0'dan 255'e sicratmiyor: pil sag'i kontrol altinda
// kaliyor. Daha da sert isteme, hiz kazanci yok, akim tepesi buyuk.
const unsigned long YON_BEKLEME = 70;      // [DEGISTI] 100 -> 70 ms

// --- Fren durumu ---
unsigned long frenBitis = 0;               // 0 degilse fren su ana kadar surer
const unsigned long FREN_SURE = 250;       // aktif fren suresi (ms)

// --- Vcc izleme ---
unsigned int anlikVcc = 0;                 // en son olculen besleme (mV)
unsigned int minVcc = 9999;                // gorulen en dusuk besleme (mV)
unsigned long sonVccOlcum = 0;             // en son ne zaman olctuk
const unsigned long VCC_PERIYOT = 2;       // ms: 2 ms'de bir ornekle
unsigned long sonDebug = 0;                // debug ciktisi zamanlayici

// ----------------------------------------------------------------------
// Arduino'nun kendi besleme gerilimini olcer (ATmega328P).
// Dahili 1.1V bandgap referansini Vcc'ye gore okur -> ekstra donanim yok.
// ----------------------------------------------------------------------
unsigned int okuVcc() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // 1.1V bandgap sec
  delayMicroseconds(300);              // referansin oturmasi icin bekle
  ADCSRA |= _BV(ADSC);                 // donusumu baslat
  while (ADCSRA & _BV(ADSC));          // bitmesini bekle
  unsigned int sonuc = ADC;
  if (sonuc == 0) return 0;
  return (unsigned int)(1125300UL / sonuc);   // 1.1 * 1023 * 1000 / ADC
}

// [YENI] Yeni bir dip deger goruldugunde EEPROM'a yaz.
// Sadece DUSTUGUNDE yaziyoruz -> bir suruste 10-20 yazma olur.
// EEPROM omru ~100.000 cevrim, sorun degil.
void minVccKaydet(unsigned int deger) {
  EEPROM.put(EEPROM_MIN_ADRES, deger);
}

// ----------------------------------------------------------------------
// Seri tampondan paket okur. Ozellikleri:
//  - BLOKLAMAZ, cop byte gelirse '<' bulana kadar atar
//  - CRC tutmayan paketi reddeder
//  - Sira numarasi tekrarli veya cok atlamis paketi reddeder
//  - Tamponda birikmis birden fazla paket varsa EN SONUNCUYU kullanir
// ----------------------------------------------------------------------
bool readCommand(Stream &s, CommandData &out) {
  bool got = false;                                  // gecerli paket bulduk mu
  while (s.available() > 0) {                        // tamponda byte varken
    if (s.peek() == '<') {                           // paket baslangici mi?
      if (s.available() >= (int)sizeof(CommandData) + 3) {  // paketin tamami geldi mi?
        s.read();                                    // '<' isaretini atla
        CommandData tmp;                             // gecici okuma alani
        s.readBytes((char *)&tmp, sizeof(CommandData)); // 5 byte veriyi al
        uint8_t chk   = s.read();                    // gelen CRC
        uint8_t bitis = s.read();                    // paket sonu isareti olmali

        if (bitis == '>' && chk == crc8((uint8_t *)&tmp, sizeof(tmp))) {
          uint8_t fark = (uint8_t)(tmp.seq - sonSeq);
          if (!seqGecerli || (fark != 0 && fark <= 20)) {
            out = tmp;                               // komutu disari ver
            got = true;
            sonSeq = tmp.seq;
            seqGecerli = true;
          }
        }
      } else {
        break;                                       // paket henuz tam gelmemis
      }
    } else {
      s.read();                                      // '<' degil: cop byte, at
    }
  }
  return got;
}

// ----------------------------------------------------------------------
// [YENI] HC-SR04 ile mesafe olc ve engel kararini guncelle.
// pulseIn en fazla ECHO_TIMEOUT kadar bloklar (4.5 ms).
// ----------------------------------------------------------------------
void mesafeOlc() {
  digitalWrite(TRIG_PIN, LOW);        // temiz bir baslangic
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);       // 10 us'lik tetik darbesi
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long sure = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT);  // yanki suresi (us)

  if (sure == 0) {
    mesafe = MESAFE_YOK;              // yanki yok: menzilde engel yok say
  } else {
    unsigned int cm = (unsigned int)(sure / 58UL);   // ses hizi: 58 us = 1 cm gidis-donus
    if (cm < 2) cm = 2;               // sensorun olu bolgesi
    mesafe = cm;
  }

  // --- Histerezis: tek bir sacma olcum arabayi durdurmasin ---
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
// [YENI] Komutu radara gore sinirla.
// Kural: SADECE ileri giden tekerler kisitlanir.
//   - Geri gitmek her zaman serbest (engelden kurtulmak icin sart)
//   - Yerinde donus (bir taraf ileri, diger taraf geri) her zaman serbest
// ----------------------------------------------------------------------
void radarSinirla(int &sol, int &sag) {
  bool yerindeDonus = ((sol > 0 && sag < 0) || (sol < 0 && sag > 0));
  if (yerindeDonus) return;                 // donmesine karisma

  if (engelVar) {                           // DUR bolgesi: ileri kapali
    if (sol > 0) sol = 0;
    if (sag > 0) sag = 0;
  } else if (mesafe <= YAVASLA_MESAFE) {    // YAVASLA bolgesi: ileri sinirli
    if (sol > YAVAS_PWM) sol = YAVAS_PWM;
    if (sag > YAVAS_PWM) sag = YAVAS_PWM;
  }
}

// ----------------------------------------------------------------------
// [YENI] Telemetri paketi yolla (9 byte):
//   0xA5 0x5A vccL vccH minL minH mesL mesH crc
// Kumanda bunu hem "canliyim" sinyali, hem gerilim, hem de mesafe gostergesi
// olarak kullanir. [DEGISTI] Radar icin 2 byte daha eklendi.
// ----------------------------------------------------------------------
void telemetriGonder(unsigned int vcc, unsigned int mn, unsigned int mes) {
  uint8_t d[6];
  d[0] = (uint8_t)(vcc & 0xFF);      // Vcc dusuk byte
  d[1] = (uint8_t)(vcc >> 8);        // Vcc yuksek byte
  d[2] = (uint8_t)(mn & 0xFF);       // minVcc dusuk byte
  d[3] = (uint8_t)(mn >> 8);         // minVcc yuksek byte
  d[4] = (uint8_t)(mes & 0xFF);      // mesafe dusuk byte
  d[5] = (uint8_t)(mes >> 8);        // mesafe yuksek byte

  BT.write(HB1);
  BT.write(HB2);
  BT.write(d, 6);
  BT.write(crc8(d, 6));
}

// ----------------------------------------------------------------------
// [YENI] Komut degerini gercek PWM'e cevirir.
//   - MIN_PWM 0 ise: hicbir sey yapmaz, deger aynen gecer (varsayilan)
//   - MIN_PWM > 0 ise: 1..255 araligini MIN_PWM..tavan araligina yayar,
//     boylece motorun donmedigi olu bolge atlanmis olur
//   - GUC_YUZDE tavani asagi ceker (yavas mod)
// Girdi 0..255 arasi MUTLAK deger olmali; yonu cagiran fonksiyon ayarlar.
// ----------------------------------------------------------------------
int pwmOlcekle(int hiz) {
  if (hiz <= 0) return 0;                    // dur, taban uygulanmaz
  if (hiz > 255) hiz = 255;

  long tavan = (long)MAX_PWM * GUC_YUZDE / 100;
  if (tavan < MIN_PWM) tavan = MIN_PWM;      // sacma ayarda bile tutarli kal

  return (int)(MIN_PWM + ((long)hiz * (tavan - MIN_PWM)) / 255);
}

// ----------------------------------------------------------------------
// Motor surme yardimcilari: +hiz ileri, -hiz geri, 0 dur
// [DEGISTI] Yon artik SOL_TERS / SAG_TERS anahtarlarindan ayarlaniyor;
// bu fonksiyonlarin icindeki HIGH/LOW'a dokunmana gerek yok.
// ----------------------------------------------------------------------
void solMotor(int hiz) {
  bool ileriYon = (hiz >= 0);                // istenen yon
  if (hiz < 0) hiz = -hiz;                   // buyuklugu ayir

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

// Iki grubu birden serbest birak (coast): motor bosa doner
void dur() {
  analogWrite(ENA, 0);     analogWrite(ENB, 0);     // PWM'leri kes
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  // yon pinlerini notr yap
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
// Yon degisimi istenirse once 0'a iner, YON_BEKLEME kadar bekler, sonra kalkar.
// ----------------------------------------------------------------------
int rampa(int simdiki, int hedef, unsigned long &bekle) {
  unsigned long simdi = millis();

  bool yonDegisimi = ((long)hedef * (long)simdiki < 0);   // isaret degisiyor mu
  if (yonDegisimi) hedef = 0;             // once mutlaka 0'a in

  if (simdiki == 0) {
    if (bekle != 0) {
      if (simdi < bekle) return 0;        // olu zaman dolmadi, kalkma
      bekle = 0;                          // olu zaman bitti
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
    if (simdi < frenBitis) return;      // suresi dolmadi, motorlara karisma
    frenBitis = 0;
    dur();                              // freni birak, serbest kal
    return;
  }

  if (simdi - sonRampa < RAMPA_PERIYOT) return;   // henuz sirasi degil
  sonRampa = simdi;

  simdikiSol = rampa(simdikiSol, hedefSol, yonBekleSol);
  simdikiSag = rampa(simdikiSag, hedefSag, yonBekleSag);

  solMotor(simdikiSol);
  sagMotor(simdikiSag);
}

// Baglanti koptu: her seyi sifirla, frenle, resenkron moduna gec
void linkKoptu() {
  linkAktif  = false;
  seqGecerli = false;
  komutSol   = 0;  komutSag   = 0;      // [YENI] ham komutu da sifirla
  hedefSol   = 0;  hedefSag   = 0;
  simdikiSol = 0;  simdikiSag = 0;      // rampayi bekleme, ANINDA sifirla
  yonBekleSol = 0; yonBekleSag = 0;
  resenkronBasladi = 0;

#if FREN_AKTIF
  fren();
  frenBitis = millis() + FREN_SURE;
#else
  dur();
  frenBitis = 0;
#endif
}

// Resenkronizasyon: yeniden baglandiktan sonra bayat veriyi temizle
void resenkronYap() {
  if (BT.available() > 0) {
    if (resenkronBasladi == 0) resenkronBasladi = millis();  // ilk byte geldi
    sonResenkronByte = millis();
    while (BT.available() > 0) BT.read();                    // gelen her seyi at

    if (millis() - resenkronBasladi >= RESENKRON_SURE) {
      linkAktif  = true;                // artik gelen veri taze sayilir
      sonMesaj   = millis();            // timeout sayacini sifirla
      seqGecerli = false;               // ilk paket referans olsun
      resenkronBasladi = 0;
    }
  } else if (resenkronBasladi != 0 && millis() - sonResenkronByte > 200) {
    resenkronBasladi = 0;               // akis kesildi, sayaci bastan baslat
  }
}

void setup() {
  BT.begin(9600);          // = Serial.begin: HC-05 data modu hizi
  BT.setTimeout(20);       // [KRITIK] readBytes'in 1 saniyelik blokajini onler

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);       // tum surucu pinleri cikis
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);                        // [YENI] radar tetik cikisi
  pinMode(ECHO_PIN, INPUT);                         // [YENI] radar yanki girisi
  digitalWrite(TRIG_PIN, LOW);

  dur();                   // acilista motorlar kesin dursun
  linkAktif = false;       // ilk gecerli akisa kadar hicbir komut uygulanmaz

  // --- [YENI] Onceki surusten kalan dip degeri oku ve raporla ---
  EEPROM.get(EEPROM_MIN_ADRES, oncekiMinVcc);
  if (oncekiMinVcc < 2000 || oncekiMinVcc > 6000) oncekiMinVcc = 0;  // bos/bozuk
#if DEBUG
  Serial.println();
  Serial.print(F("=== ONCEKI SURUSTE GORULEN EN DUSUK Vcc: "));
  if (oncekiMinVcc == 0) Serial.print(F("kayit yok"));
  else { Serial.print(oncekiMinVcc); Serial.print(F(" mV")); }
  Serial.println(F(" ==="));
  Serial.println(F("(4600 mV altiysa HC-05 brownout yasiyor demektir)"));
#endif

  // Yeni surus icin kaydi temizle: bundan sonraki dipler buraya yazilacak
  minVccKaydet(0xFFFF);
  minVcc = 9999;
}

void loop() {
  unsigned long simdi = millis();

  if (!linkAktif) {
    // --- Link yok: gelen her seyi cope at, temiz bir baslangic bekle ---
    resenkronYap();
  } else {
    // --- Link var: normal paket isleme ---
    CommandData cmd;                       // gelen komut buraya yazilacak
    if (readCommand(BT, cmd)) {            // yeni gecerli paket var mi?
      if (cmd.sol >= -255 && cmd.sol <= 255 && cmd.sag >= -255 && cmd.sag <= 255) {
        komutSol = constrain(cmd.sol, -MAX_PWM, MAX_PWM);   // [DEGISTI] ham komut
        komutSag = constrain(cmd.sag, -MAX_PWM, MAX_PWM);   // radar sonra sinirlar
        sonMesaj = simdi;                  // "en son bu an veri geldi" diye kaydet
      }
    }

    if (simdi - sonMesaj > TIMEOUT) {      // sessizlik = baglanti gitti
      linkKoptu();
    }
  }

  // --- [YENI] Radar: mesafeyi olc (60 ms'de bir) ---
  if (simdi - sonOlcum >= OLCUM_PERIYOT) {
    sonOlcum = simdi;
    mesafeOlc();
  }

  // --- [YENI] Ham komutu radara gore sinirla ---
  // Her dongude yeniden hesapliyoruz: joystick sabit dursa bile onumuze
  // bir engel cikarsa araba hemen tepki vermeli.
  hedefSol = komutSol;
  hedefSag = komutSag;
  radarSinirla(hedefSol, hedefSag);

  // --- Besleme gerilimini ornekle (2 ms'de bir) ---
  // Motor akim tepeleri onlarca ms surer, 2 ms ornekleme rahat yakalar.
  if (simdi - sonVccOlcum >= VCC_PERIYOT) {
    sonVccOlcum = simdi;
    unsigned int v = okuVcc();
    if (v > 2000) {                        // olcum makul mu
      anlikVcc = v;
      if (v < minVcc) {
        minVcc = v;
        minVccKaydet(minVcc);              // [YENI] kalici olarak sakla
      }
    }
  }

  // --- Kumandaya telemetri + heartbeat yolla ---
  // [NOT] Donanim UART tam cift yonlu: gonderim alimi artik hic aksatmiyor,
  // bu cagrinin loop icindeki yeri kritik degil.
  if (simdi - sonHeartbeat >= HEARTBEAT_ARALIK) {
    telemetriGonder(anlikVcc, (minVcc == 9999) ? 0 : minVcc, mesafe);
    sonHeartbeat = simdi;
  }

  // --- Rampa + motor guncelleme (fren suresince dokunmaz) ---
  motorlariGuncelle();

#if DEBUG
  // ARTIK KULLANILAMAZ: bu satirlar HC-05'in hattina yazar, kumandaya cop
  // gider. Sadece HC-05 sokulu DEGILKEN, tezgah testi icin acilabilir.
  if (simdi - sonDebug >= 500) {
    sonDebug = simdi;
    Serial.print(F("link="));  Serial.print(linkAktif ? 1 : 0);
    Serial.print(F(" sol="));  Serial.print(simdikiSol);
    Serial.print(F(" sag="));  Serial.print(simdikiSag);
    Serial.print(F(" Vcc="));  Serial.print(anlikVcc);
    Serial.print(F(" min="));  Serial.print(minVcc);
    Serial.print(F(" mes="));  Serial.print(mesafe);          // [YENI]
    Serial.print(F(" engel=")); Serial.println(engelVar ? 1 : 0);
  }
#endif
}
