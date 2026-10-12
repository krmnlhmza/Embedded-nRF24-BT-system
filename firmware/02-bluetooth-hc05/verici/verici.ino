// ========================== VERICI (KUMANDA) ==========================
// Joystick'i okur, sol/sag motor hizlarini hesaplar, HC-05 uzerinden
// binary paket olarak gonderir. 16x2 I2C LCD'de baglanti durumunu gosterir.
//
// HC-05 hardware Serial'de (pin 0/1). KOD YUKLERKEN HC-05'I CIKAR!
//
// BAGLANTILAR (EK KABLO YOK, hepsi zaten mevcut):
//   Joystick : X -> A0, Y -> A1
//   LCD (I2C): SDA -> A4, SCL -> A5, VCC -> 5V, GND -> GND
//   HC-05    : TXD -> pin 0 (RX), RXD -> pin 1 (TX), VCC -> 5V, GND -> GND
//              (TXD -> pin 0 kablosu SART: heartbeat oradan geliyor)
//
// BAGLANTI TESPITI (STATE pini yok, tamamen yazilimla):
//   Araba surekli geriye "canliyim" byte'i yolluyor. Belirlenen sure icinde
//   hic gelmezse baglanti kopmus demektir -> ekranda "BAGLANTI YOK".
//   Bu yontem sadece BT eslesmesini degil, arabanin acik ve calisir
//   durumda oldugunu da dogrular.
//
// HAREKET MANTIGI:
//   - Joystick ortada (safe zone)  -> VERI GONDERILMEZ (birkac dur paketi haric)
//   - Sadece ileri/geri            -> duz gider
//   - Sadece tam sag / tam sol     -> YERINDE DONER (paletli tank gibi)
//   - Capraz (ileri+sag gibi)      -> bir teker hizli, digeri yavas (kavisli donus)
// =======================================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);   // I2C adresi: calismazsa 0x3F dene

// --- Paket yapisi: alici ile BIREBIR AYNI olmali ---
struct __attribute__((packed)) CommandData {
  int16_t sol;   // sol motor grubu hizi: -255 (tam geri) .. +255 (tam ileri)
  int16_t sag;   // sag motor grubu hizi: -255 .. +255
};
// Kablodaki paket: '<' + 4 byte veri + 1 byte checksum + '>'  (toplam 7 byte)

// Bir komutu paketleyip seri porta (HC-05'e) yazar
void sendCommand(Stream &s, int16_t sol, int16_t sag) {
  CommandData c;                 // gonderilecek veri yapisi
  c.sol = sol;                   // sol hizi yerlestir
  c.sag = sag;                   // sag hizi yerlestir

  uint8_t *p = (uint8_t *)&c;    // struct'a byte byte erisim icin pointer
  uint8_t chk = 0;               // checksum burada birikecek
  for (uint8_t i = 0; i < sizeof(c); i++) chk ^= p[i];  // tum byte'larin XOR'u

  s.write('<');                  // paket baslangic isareti
  s.write(p, sizeof(c));         // 4 byte'lik veri
  s.write(chk);                  // checksum (alici dogrulayacak)
  s.write('>');                  // paket bitis isareti
}

// --- Pinler ve ayarlar ---
const int X_PIN = A0;        // joystick X ekseni (sag/sol)
const int Y_PIN = A1;        // joystick Y ekseni (ileri/geri)
const int SAFE_ZONE = 40;    // merkez guvenli bolge: eksen degeri bunun altindaysa 0 sayilir

// --- Heartbeat (canliyim sinyali) ayarlari: alici ile AYNI olmali ---
const uint8_t HEARTBEAT_BYTE = 'A';           // arabanin yolladigi isaret
const unsigned long HEARTBEAT_TIMEOUT = 800;  // bu sure hic gelmezse baglanti yok
// Araba her 200 ms'de bir yolluyor -> 800 ms, ust uste 3 tanesinin kaybolmasina
// tahammul eder. Daha kisa yaparsan tek bir parazitte bosuna "YOK" yazar.

int durPaketSayaci = 0;              // safe zone'a girince kac dur paketi gonderdik

bool baglantiVar = false;            // LCD'de su an yazan durum
unsigned long sonHeartbeat = 0;      // son "canliyim" byte'ini ne zaman aldik
bool heartbeatAlindi = false;        // hic heartbeat geldi mi (acilista false)
unsigned long sonHizYazimi = 0;      // alt satiri en son ne zaman guncelledik
int gosterilenSol = 9999;            // LCD'de su an yazan hizlar (9999 = "hic yazilmadi")
int gosterilenSag = 9999;

// Seri tampona bakip baglanti var mi diye karar verir.
// Gelen HER byte'i tuketir: heartbeat ise saati gunceller, cop ise atar
// (tampon dolup tasmasin diye bosaltmak sart).
bool baglantiOku() {
  while (Serial.available() > 0) {
    if ((uint8_t)Serial.read() == HEARTBEAT_BYTE) {
      sonHeartbeat = millis();       // araba yasiyor
      heartbeatAlindi = true;
    }
  }

  if (!heartbeatAlindi) return false;              // henuz hic sinyal gelmedi

  if (millis() - sonHeartbeat > HEARTBEAT_TIMEOUT) {
    heartbeatAlindi = false;         // koptu: yeni sinyal gelene kadar "yok"
    return false;
  }
  return true;
}

// Ust satira baglanti durumunu yazar (16 karaktere tamamlanmis, eski yazi silinsin)
void lcdDurumYaz(bool bagli) {
  lcd.setCursor(0, 0);
  lcd.print(bagli ? "BAGLANTI VAR    " : "BAGLANTI YOK    ");
}

// Alt satira gonderilen hizlari yazar
void lcdHizYaz(int sol, int sag) {
  char buf[17];                                        // 16 karakter + sonlandirici
  snprintf(buf, sizeof(buf), "S:%-6dG:%-6d", sol, sag); // sola dayali, kalani bosluk
  lcd.setCursor(0, 1);
  lcd.print(buf);
}

void setup() {
  Serial.begin(9600);              // HC-05 ile ayni hiz (data modu 9600)

  lcd.init();                      // ekrani baslat
  lcd.backlight();                 // arka isik acik

  lcdDurumYaz(false);              // acilista bagli degiliz (heartbeat gelmedi)
  lcdHizYaz(0, 0);
  gosterilenSol = 0;
  gosterilenSag = 0;
}

void loop() {
  // 0) Arabadan heartbeat geldi mi bak; durum SADECE degistiginde LCD'ye yaz.
  //    (I2C yazmak yavas, her dongude yazarsak kumanda tepkisi bozulur.)
  bool simdikiDurum = baglantiOku();
  if (simdikiDurum != baglantiVar) {
    baglantiVar = simdikiDurum;
    lcdDurumYaz(baglantiVar);
  }

  // 1) Joystick'in ham degerlerini oku (0..1023 arasi)
  int xRaw = analogRead(X_PIN);
  int yRaw = analogRead(Y_PIN);

  // 2) Ham degerleri -255..+255 araligina cevir
  int ileri = map(yRaw, 0, 1023, -255, 255);   // + ileri, - geri
  int donus = map(xRaw, 0, 1023, -255, 255);   // + sag,   - sol

  // 3) Safe zone: merkeze yakin sallantilari 0 kabul et
  if (abs(ileri) < SAFE_ZONE) ileri = 0;
  if (abs(donus) < SAFE_ZONE) donus = 0;

  int sol = 0, sag = 0;  // hesaplanacak motor hizlari

  if (ileri == 0 && donus == 0) {
    // --- JOYSTICK ORTADA (safe zone) ---
    // Surekli veri GONDERME. Sadece ilk giriste birkac "dur" paketi yolla ki
    // araba hemen dursun; sonra sus. (Alicida ayrica 500ms timeout var, yedek.)
    if (durPaketSayaci < 5) {          // toplam 5 dur paketi yeter
      sendCommand(Serial, 0, 0);       // dur komutu
      durPaketSayaci++;                // sayaci artir
    }
    // 5 paketten sonra hicbir sey gonderilmez -> hat bos kalir
  } else {
    durPaketSayaci = 0;                // hareket basladi, sayaci sifirla

    if (ileri == 0) {
      // --- SADECE TAM SAG veya TAM SOL: YERINDE DON ---
      // Bir taraf ileri, diger taraf ayni hizda geri doner.
      sol =  donus;                    // saga itildiyse sol grup ileri
      sag = -donus;                    //               sag grup geri (tersine doner)
    } else if (ileri > 0) {
      // --- ILERI GIDIS (duz veya capraz) ---
      // Capraza itilirse: donus, IC tekeri yavaslatir, DIS tekeri hizlandirir.
      // 0'in altina inmesine izin YOK -> ileri giderken hicbir teker geri donmez.
      sol = constrain(ileri + donus, 0, 255);   // saga donuste sol (dis) hizli
      sag = constrain(ileri - donus, 0, 255);   //              sag (ic) yavas
    } else {
      // --- GERI GIDIS (duz veya capraz) ---
      // Ayni mantik, negatif tarafta: 0'in ustune cikamaz -> teker ileri donmez.
      sol = constrain(ileri + donus, -255, 0);
      sag = constrain(ileri - donus, -255, 0);
    }

    sendCommand(Serial, sol, sag);     // hesaplanan hizlari HC-05'ten gonder
  }

  // 4) Alt satirdaki hizlari guncelle: en fazla 4 kez/sn ve sadece deger degistiyse.
  //    (Yine I2C'yi bogmamak icin; ekran zaten daha hizlisini gosteremez.)
  if ((sol != gosterilenSol || sag != gosterilenSag) &&
      (millis() - sonHizYazimi >= 250)) {
    lcdHizYaz(sol, sag);
    gosterilenSol = sol;
    gosterilenSag = sag;
    sonHizYazimi = millis();
  }

  delay(30);  // ~33 paket/sn: akici kontrol icin yeterli, hatti bogmaz
}
