// ======================= HC-05 STATE PIN TESHIS =======================
// Amac: pin 4'teki STATE sinyalinin GERCEKTEN ne yaptigini gormek.
// Motor/joystick yok, sadece olcum yapar. HC-05'in takili kalmasi sorun degil
// (bu kod Serial kullanmiyor) ama KOD YUKLERKEN yine de HC-05'i cikar.
//
// BAGLANTI: HC-05 STATE -> pin 4,  LCD -> A4/A5
//
// EKRANDA NE GORECEKSIN:
//   Ust satir : pinin su anki hali (HIGH / LOW)
//   Alt satir : %..  -> son 1 saniyenin yuzde kaci HIGH gecti
//               D:.. -> son 1 saniyede kac kez seviye degistirdi (yanip sonme)
//
// YORUMLAMA:
//   %0    D:0   -> pin sabit LOW  : eslesme yok. DOGRU davranis.
//   %100  D:0   -> pin sabit HIGH : ya gercekten bagli, ya da kablo bos (havada).
//                  Ayirt etmek icin: pin 4 ile GND arasina 10k direnc tak.
//                  Direncle %0 oluyorsa -> kablo/STATE calismiyor.
//                  Direncle %100 kaliyorsa -> HC-05 gercekten HIGH suruyor.
//   %50 civari, D buyuk (10+) -> pin yanip soniyor : eslesme ARANIYOR, bagli degil.
// ======================================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int BT_STATE_PIN = 4;

unsigned long pencereBasi = 0;   // olcum penceresinin baslangic ani
unsigned int  ornekSayisi = 0;   // bu pencerede kac kez okuduk
unsigned int  highSayisi  = 0;   // bunlarin kaci HIGH'di
unsigned int  degisimSayisi = 0; // kac kez seviye degisti
int oncekiSeviye = -1;           // bir onceki okuma (-1 = henuz yok)

void setup() {
  pinMode(BT_STATE_PIN, INPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("STATE olcumu...");

  pencereBasi = millis();
}

void loop() {
  int seviye = digitalRead(BT_STATE_PIN);   // pini oku

  ornekSayisi++;
  if (seviye == HIGH) highSayisi++;
  if (oncekiSeviye != -1 && seviye != oncekiSeviye) degisimSayisi++;
  oncekiSeviye = seviye;

  // Her saniye bir kez ekrani tazele
  if (millis() - pencereBasi >= 1000) {
    int yuzde = (int)((highSayisi * 100UL) / ornekSayisi);

    lcd.setCursor(0, 0);
    lcd.print(seviye == HIGH ? "PIN4: HIGH      " : "PIN4: LOW       ");

    char buf[17];
    snprintf(buf, sizeof(buf), "%%%-4dD:%-8u", yuzde, degisimSayisi);
    lcd.setCursor(0, 1);
    lcd.print(buf);

    // sayaclari sifirla, yeni pencere basla
    pencereBasi   = millis();
    ornekSayisi   = 0;
    highSayisi    = 0;
    degisimSayisi = 0;
  }

  delay(2);   // saniyede ~500 ornek: yanip sonmeyi yakalamaya fazlasiyla yeter
}
