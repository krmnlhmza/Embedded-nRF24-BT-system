// ========================== VERICI (KUMANDA) ==========================
// Joystick'i okur, sol/sag motor hizlarini hesaplar, HC-05 uzerinden
// binary paket olarak gonderir. 16x2 I2C LCD'de baglanti durumunu,
// arabanin CANLI besleme gerilimini ve hizlari gosterir.
//
// HC-05 hardware Serial'de (pin 0/1). KOD YUKLERKEN HC-05'I CIKAR!
//
// BAGLANTILAR (DEGISMEDI):
//   Joystick : X -> A0, Y -> A1
//   LCD (I2C): SDA -> A4, SCL -> A5, VCC -> 5V, GND -> GND
//   HC-05    : TXD -> pin 0 (RX), RXD -> pin 1 (TX), VCC -> 5V, GND -> GND
//
// ---------------------- ONCEKI SURUMDE YAPILANLAR --------------------
// [1] Surekli akis (safe zone'da da paket gider)
// [2] Baglanti yokken gonderim yok (HC-05 tamponunda bayat paket birikmesin)
// [3] Sira numarasi (seq)
// [4] CRC-8
// [5] 2 byte'lik heartbeat imzasi
// [6] Joystick merkez kalibrasyonu
//
// ------------------- BU SURUMDE YENI OLAN (TELEMETRI) -----------------
// Araba artik heartbeat'in icinde besleme gerilimini de yolluyor.
// Amac: arabayi PILLE surerken (USB takili degilken) gerilim cokusunu
// canli gorebilmek. Serial Monitor'e ihtiyac kalmiyor.
//
// ----------------------- BU SURUMDE YENI: RADAR -----------------------
// Araba HC-SR04 mesafe sensoru tasiyor ve olctugu mesafeyi telemetri
// paketinin icinde yolluyor. Kumanda bunu ust satirda gosteriyor.
// Telemetri paketi 7 byte'tan 9 byte'a cikti (mesafe icin 2 byte).
//
// LCD DUZENI:
//   Ust satir : BAGLI/KOPUK + arabanin onundeki mesafe (CANLI)
//               "BAGLI   M:120cm "  -> yol acik
//               "BAGLI  !M: 18cm "  -> unlem = ileri gidis engellendi
//               "KOPUK   M:---cm "  -> baglanti yok
//   Alt satir : 2 saniyede bir donusumlu
//               "S:180  G:180    "  (gonderilen hizlar)
//               "V:4820  MIN:4580"  (anlik ve en dusuk besleme, mV)
//
// NASIL YORUMLANIR:
//   MIN Vcc > 4900 mV -> besleme saglam, kopmalarin sebebi baska
//   4600 - 4900 mV    -> sinirdasin, buck regulator isi bitirir
//   MIN Vcc < 4600 mV -> HC-05 brownout yasiyor, kopmalarin sebebi bu
// =======================================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);   // I2C adresi: calismazsa 0x3F dene

// --- Paket yapisi: alici ile BIREBIR AYNI olmali ---
struct __attribute__((packed)) CommandData {
  int16_t sol;   // sol motor grubu hizi: -255 (tam geri) .. +255 (tam ileri)
  int16_t sag;   // sag motor grubu hizi: -255 .. +255
  uint8_t seq;   // sira numarasi, her pakette 1 artar (255'ten sonra 0)
};
// Kablodaki paket: '<' + 5 byte veri + 1 byte CRC + '>'  (toplam 8 byte)

// --- CRC-8 (poly 0x31) : alici ile BIREBIR AYNI olmali ---
uint8_t crc8(const uint8_t *veri, uint8_t n) {
  uint8_t crc = 0xFF;                            // baslangic degeri
  while (n--) {
    crc ^= *veri++;                              // siradaki byte'i karistir
    for (uint8_t i = 0; i < 8; i++)              // 8 bit boyunca kaydir
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31)
                         : (uint8_t)(crc << 1);
  }
  return crc;
}

uint8_t seqNo = 0;             // gonderilen paketin sira numarasi

// Bir komutu paketleyip seri porta (HC-05'e) yazar
void sendCommand(Stream &s, int16_t sol, int16_t sag) {
  CommandData c;                 // gonderilecek veri yapisi
  c.sol = sol;                   // sol hizi yerlestir
  c.sag = sag;                   // sag hizi yerlestir
  c.seq = seqNo++;               // sira numarasini koy ve bir artir

  uint8_t *p = (uint8_t *)&c;    // struct'a byte byte erisim icin pointer
  uint8_t chk = crc8(p, sizeof(c));

  s.write('<');                  // paket baslangic isareti
  s.write(p, sizeof(c));         // 5 byte'lik veri
  s.write(chk);                  // CRC (alici dogrulayacak)
  s.write('>');                  // paket bitis isareti
}

// --- Pinler ve ayarlar ---
const int X_PIN = A0;        // joystick X ekseni (sag/sol)
const int Y_PIN = A1;        // joystick Y ekseni (ileri/geri)
const int SAFE_ZONE = 40;    // merkez guvenli bolge (-255..255 olceginde)

// --- Joystick kalibrasyonu ---
int merkezX = 512, merkezY = 512;   // acilista olculecek gercek merkez
int olcekX  = 470,  olcekY = 470;   // merkezden uca kalan ham ADC mesafesi

// --- [YENI] Radar esikleri: ALICI ile AYNI olmali (sadece gosterim icin) ---
const unsigned int MESAFE_YOK = 999;   // yanki yok / menzil disi
const unsigned int DUR_MESAFE = 25;    // cm: bu mesafede ekranda '!' cikar

// --- Heartbeat / telemetri ayarlari: alici ile AYNI olmali ---
// Paket: 0xA5 0x5A [vccL] [vccH] [minL] [minH] [mesL] [mesH] [crc8] -> 9 byte
const uint8_t HB1 = 0xA5;
const uint8_t HB2 = 0x5A;
const unsigned long HEARTBEAT_TIMEOUT = 700;  // bu sure hic gelmezse baglanti yok
// Araba her 200 ms'de bir yolluyor -> 700 ms, ust uste 3 tanesinin kaybina
// tahammul eder. Daha kisa yaparsan tek parazitte bosuna "KOPUK" yazar.

bool baglantiVar = false;            // LCD'de su an yazan durum
unsigned long sonHeartbeat = 0;      // son gecerli telemetriyi ne zaman aldik
bool heartbeatAlindi = false;        // hic telemetri geldi mi (acilista false)

// --- [YENI] Telemetri cozumleme durum makinesi ---
uint8_t hbDurum = 0;                 // 0:0xA5 ara  1:0x5A ara  2:veri  3:crc
uint8_t hbBuf[6];                    // [DEGISTI] vccL vccH minL minH mesL mesH
uint8_t hbIndex = 0;                 // veri icinde kacinci byte'tayiz

// --- [YENI] Arabadan gelen degerler ---
unsigned int aracVcc    = 0;         // arabanin anlik besleme gerilimi (mV)
unsigned int aracMinVcc = 0;         // arabanin gordugu en dusuk gerilim (mV)
unsigned int aracMesafe = MESAFE_YOK;   // [YENI] radar mesafesi (cm)

// --- LCD guncelleme zamanlayicilari ---
unsigned long sonUstYazim  = 0;      // ust satiri en son ne zaman yazdik
unsigned long sonAltYazim  = 0;      // alt satiri en son ne zaman yazdik
unsigned long sonModDegis  = 0;      // alt satir modunu en son ne zaman degistirdik
uint8_t altMod = 0;                  // 0: hizlar   1: min Vcc
int gosterilenSol = 9999;            // LCD'de su an yazan hizlar
int gosterilenSag = 9999;
unsigned int gosterilenMesafe = 0;   // [DEGISTI] ust satirda su an yazan mesafe
bool gosterilenDurum = false;        // ust satirda su an yazan durum

// ----------------------------------------------------------------------
// Seri tampona bakip telemetri paketi ayikla ve baglanti var mi karar ver.
// Gelen HER byte tuketilir (tampon dolup tasmasin diye bosaltmak sart).
// ----------------------------------------------------------------------
bool baglantiOku() {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();

    switch (hbDurum) {
      case 0:                                  // imzanin ilk yarisini ariyoruz
        if (b == HB1) hbDurum = 1;
        break;

      case 1:                                  // ikinci yarisi gelmeli
        if (b == HB2) { hbDurum = 2; hbIndex = 0; }
        else          hbDurum = (b == HB1) ? 1 : 0;   // kaydi, bastan basla
        break;

      case 2:                                  // [DEGISTI] 6 byte veri topluyoruz
        hbBuf[hbIndex++] = b;
        if (hbIndex >= 6) hbDurum = 3;
        break;

      case 3:                                  // son byte CRC olmali
        if (b == crc8(hbBuf, 6)) {             // dogruysa degerleri al
          aracVcc    = (unsigned int)hbBuf[0] | ((unsigned int)hbBuf[1] << 8);
          aracMinVcc = (unsigned int)hbBuf[2] | ((unsigned int)hbBuf[3] << 8);
          aracMesafe = (unsigned int)hbBuf[4] | ((unsigned int)hbBuf[5] << 8);  // [YENI]
          sonHeartbeat = millis();             // araba yasiyor
          heartbeatAlindi = true;
        }
        hbDurum = 0;                           // her halukarda bastan
        break;
    }
  }

  if (!heartbeatAlindi) return false;              // henuz hic sinyal gelmedi

  if (millis() - sonHeartbeat > HEARTBEAT_TIMEOUT) {
    heartbeatAlindi = false;         // koptu: yeni sinyal gelene kadar "yok"
    aracVcc = 0;                     // ekranda eski deger asili kalmasin
    aracMesafe = MESAFE_YOK;         // [YENI] eski mesafe asili kalmasin
    return false;
  }
  return true;
}

// ----------------------------------------------------------------------
// [DEGISTI] Ust satir: durum + arabanin onundeki mesafe
//   "BAGLI   M:120cm "  yol acik
//   "BAGLI  !M: 18cm "  unlem = ileri gidis engellendi
//   "KOPUK   M:---cm "  baglanti yok
// Mesafeyi surerken gormek gerilimden daha ise yarar; gerilim alt satira indi.
// ----------------------------------------------------------------------
void lcdUstYaz(bool bagli, unsigned int mes) {
  char buf[17];
  if (!bagli)
    snprintf(buf, sizeof(buf), "KOPUK   M:---cm ");
  else if (mes == 0 || mes >= MESAFE_YOK)
    snprintf(buf, sizeof(buf), "BAGLI   M:---cm ");   // menzilde engel yok
  else
    snprintf(buf, sizeof(buf), "BAGLI  %cM:%3ucm ",
             (mes <= DUR_MESAFE) ? '!' : ' ', mes);
  lcd.setCursor(0, 0);
  lcd.print(buf);
}

// Alt satir modu 0: gonderilen hizlar
void lcdHizYaz(int sol, int sag) {
  char buf[17];                                        // 16 karakter + sonlandirici
  snprintf(buf, sizeof(buf), "S:%-6dG:%-6d", sol, sag); // sola dayali, kalani bosluk
  lcd.setCursor(0, 1);
  lcd.print(buf);
}

// [DEGISTI] Alt satir modu 1: anlik VE en dusuk besleme birlikte
//   "V:4820  MIN:4580"
void lcdGerilimYaz(unsigned int vcc, unsigned int mn) {
  char buf[17], v[6], m[6];
  if (vcc >= 2000) snprintf(v, sizeof(v), "%4u", vcc); else strcpy(v, "----");
  if (mn  >= 2000) snprintf(m, sizeof(m), "%4u", mn);  else strcpy(m, "----");
  snprintf(buf, sizeof(buf), "V:%s  MIN:%s", v, m);
  lcd.setCursor(0, 1);
  lcd.print(buf);
}

// --- Acilista joystick merkezini olc ---
// ONEMLI: acilirken joystick'e DOKUNMA, serbest birak.
void joystickKalibreEt() {
  long toplamX = 0, toplamY = 0;
  for (int i = 0; i < 50; i++) {        // 50 orneklem al, ortalamasini kullan
    toplamX += analogRead(X_PIN);
    toplamY += analogRead(Y_PIN);
    delay(4);
  }
  merkezX = toplamX / 50;
  merkezY = toplamY / 50;

  // Merkezden uca kalan mesafe (hangi taraf kisaysa o belirler)
  olcekX = min(merkezX, 1023 - merkezX);
  olcekY = min(merkezY, 1023 - merkezY);

  // Sacma bir olcum ciktiysa (joystick tutuluyordu vb.) makul degere don
  if (olcekX < 150) { merkezX = 512; olcekX = 470; }
  if (olcekY < 150) { merkezY = 512; olcekY = 470; }
}

// --- Ham ADC degerini -255..+255 araligina cevirir (kalibrasyonlu) ---
int eksenOku(int pin, int merkez, int olcek) {
  int sapma = analogRead(pin) - merkez;                 // merkezden ne kadar uzakta
  long deger = (long)sapma * 255L / (long)olcek;        // olcekle
  if (deger >  255) deger =  255;
  if (deger < -255) deger = -255;
  return (int)deger;
}

void setup() {
  Serial.begin(9600);              // HC-05 ile ayni hiz (data modu 9600)

  lcd.init();                      // ekrani baslat
  lcd.backlight();                 // arka isik acik

  lcd.setCursor(0, 0);
  lcd.print("KALIBRASYON...  ");   // kullaniciya "elini cek" mesaji
  lcd.setCursor(0, 1);
  lcd.print("JOYSTICK SERBEST");
  joystickKalibreEt();

  lcdUstYaz(false, MESAFE_YOK);    // acilista bagli degiliz
  lcdHizYaz(0, 0);
  gosterilenSol = 0;
  gosterilenSag = 0;
  gosterilenDurum = false;
  gosterilenMesafe = MESAFE_YOK;
  sonModDegis = millis();
}

void loop() {
  unsigned long simdi = millis();

  // 0) Arabadan telemetri geldi mi bak
  bool simdikiDurum = baglantiOku();
  if (simdikiDurum != baglantiVar) {
    baglantiVar = simdikiDurum;
    if (!baglantiVar) seqNo = 0;   // kopunca sira numarasini sifirla
  }

  // 1) Joystick'i kalibrasyonlu oku, dogrudan -255..+255 olceginde
  int ileri = eksenOku(Y_PIN, merkezY, olcekY);   // + ileri, - geri
  int donus = eksenOku(X_PIN, merkezX, olcekX);   // + sag,   - sol

  // 2) Safe zone: merkeze yakin sallantilari 0 kabul et
  if (abs(ileri) < SAFE_ZONE) ileri = 0;
  if (abs(donus) < SAFE_ZONE) donus = 0;

  int sol = 0, sag = 0;  // hesaplanacak motor hizlari

  if (ileri == 0 && donus == 0) {
    // --- JOYSTICK ORTADA (safe zone) --- 0,0 paketi de duzenli gider
    sol = 0;
    sag = 0;
  } else if (ileri == 0) {
    // --- SADECE TAM SAG veya TAM SOL: YERINDE DON ---
    sol =  donus;                    // saga itildiyse sol grup ileri
    sag = -donus;                    //               sag grup geri
  } else if (ileri > 0) {
    // --- ILERI GIDIS (duz veya capraz) ---
    sol = constrain(ileri + donus, 0, 255);   // saga donuste sol (dis) hizli
    sag = constrain(ileri - donus, 0, 255);   //              sag (ic) yavas
  } else {
    // --- GERI GIDIS (duz veya capraz) ---
    sol = constrain(ileri + donus, -255, 0);
    sag = constrain(ileri - donus, -255, 0);
  }

  // 3) SADECE BAGLIYKEN gonder. Kopukken yazarsak paketler HC-05 tamponunda
  //    birikir ve link geri gelince toplu duser -> araba eski komutu uygular.
  if (baglantiVar) {
    sendCommand(Serial, sol, sag);
  } else {
    sol = 0;                         // ekranda da 0 gorunsun
    sag = 0;
  }

  // 4) --- LCD UST SATIR: durum + mesafe ---
  //    Durum degistiyse HEMEN, mesafe degistiyse en fazla 4 kez/sn yaz.
  //    (Mesafe hizli degisir; I2C'yi bogmamak icin kisitliyoruz.)
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

  // 5) --- LCD ALT SATIR: 2 saniyede bir hizlar <-> min Vcc donusumlu ---
  if (simdi - sonModDegis >= 2000) {
    sonModDegis = simdi;
    altMod = 1 - altMod;             // modu degistir
    gosterilenSol = 9999;            // zorla yeniden yazdir
  }

  if (altMod == 0) {
    // Hizlari en fazla 4 kez/sn ve sadece deger degistiyse yaz (I2C bogulmasin)
    if ((sol != gosterilenSol || sag != gosterilenSag) &&
        (simdi - sonAltYazim >= 250)) {
      lcdHizYaz(sol, sag);
      gosterilenSol = sol;
      gosterilenSag = sag;
      sonAltYazim = simdi;
    }
  } else {
    // Gerilimler: saniyede 2 kez yeter, zaten yavas degisir
    if (simdi - sonAltYazim >= 500) {
      lcdGerilimYaz(aracVcc, aracMinVcc);
      sonAltYazim = simdi;
    }
  }

  delay(20);  // [DEGISTI] 30 -> 20 ms. ~50 paket/sn: joystick hareketi
              // motora daha cabuk yansir, kontrol daha tepkili hissettirir.
              // 50 x 8 byte = 400 B/s; 9600 baud'un (~960 B/s) yarisi, hat rahat.
}
