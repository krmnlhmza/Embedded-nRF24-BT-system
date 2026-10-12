// ===================== ADIM 0: DONANIM KONTROLU =========================
// Bu sketch RADYO HABERLESMESI YAPMAZ. Sadece sunu sorar:
// "Arduino, SPI uzerinden nRF24 yongasiyla konusabiliyor mu?"
//
// IKI KARTA DA SIRAYLA YUKLE. Ikisinde de temiz cikti almadan
// verici/alici sketch'lerine GECME. Haberlesme sorunlarinin %90'i
// bu adimda yakalanir (yanlis pin, soguk lehim, yetersiz 3.3V).
//
// GEREKEN KUTUPHANE: "RF24" by TMRh20
//   Arduino IDE > Sketch > Include Library > Manage Libraries > "RF24" ara
//
// BAGLANTI (Uno):
//   nRF24 pin      Uno pin     Not
//   ---------      -------     ---
//   1 GND          GND         kare pad olan kose = pin 1
//   2 VCC          3.3V        *** 5V VERIRSEN MODUL OLUR ***
//   3 CE           D9
//   4 CSN          D10
//   5 SCK          D13
//   6 MOSI         D11
//   7 MISO         D12
//   8 IRQ          bos
//
// VCC-GND ucuna 10-100 uF elektrolit kondansator SART (eksi bacak GND'ye).
// ANTENI TAKMADAN GUC VERME.
// ======================================================================

#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);   // CE, CSN

void setup() {
  Serial.begin(115200);
  while (!Serial) { }
  Serial.println(F("\n=== nRF24 DONANIM KONTROLU ==="));

  // radio.begin() SPI'yi baslatir ve yongadan bir kayit okuyup dogrular.
  if (!radio.begin()) {
    Serial.println(F("SONUC: BASARISIZ - yonga cevap vermiyor."));
    Serial.println(F("Kontrol et: CE=D9 CSN=D10 SCK=D13 MOSI=D11 MISO=D12,"));
    Serial.println(F("VCC 3.3V mi (5V DEGIL), GND ortak mi, kondansator takili mi."));
    while (1) { }
  }

  Serial.println(F("SONUC: SPI baglantisi TAMAM."));

  // --- Yazma/okuma testi -------------------------------------------------
  // Yongaya bir deger yazip geri okuyoruz. begin() bazen bos kabloda bile
  // gecebiliyor; bu test kesin sonuc verir.
  radio.setChannel(108);
  uint8_t okunan = radio.getChannel();
  Serial.print(F("Kanal yazildi: 108, geri okunan: "));
  Serial.println(okunan);
  if (okunan != 108) {
    Serial.println(F("UYARI: yazilan deger geri okunamiyor -> kablo/besleme suphesi."));
  }

  Serial.print(F("Yonga bagli mi (isChipConnected): "));
  Serial.println(radio.isChipConnected() ? F("EVET") : F("HAYIR"));

  // --- Tum kayit dokumu --------------------------------------------------
  // Asagidaki listede HER SATIR 0x00 veya 0xFF cikiyorsa kablo problemi var.
  // Saglikli modulde ornegin: EN_AA=0x3F, SETUP_AW=0x03, RF_CH=0x6C
  Serial.println(F("\n--- KAYIT DOKUMU ---"));
  radio.printPrettyDetails();   // eski kutuphane surumunde: printDetails()
  Serial.println(F("--------------------"));
  Serial.println(F("Buraya kadar sorunsuzsa modul saglam. Diger karti da test et."));
}

void loop() { }
