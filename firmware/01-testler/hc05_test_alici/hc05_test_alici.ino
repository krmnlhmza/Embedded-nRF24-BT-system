// HC-05 TEST - ALICI (HAM VERI MONITOR)  MOTOR YOK.
// Amac: kumandadan (verici) HC-05 uzerinden veri GELIYOR MU gormek.
// Protokolden bagimsiz: gelen her byte'i oldugu gibi yazar.
// Kumandada hangi kod olursa olsun (checksum'li/text/binary) fark etmez.
//
// Baglanti:
//   HC-05 TXD -> Arduino pin 10
//   HC-05 RXD -> Arduino pin 11
//   HC-05 VCC -> 5V, GND -> GND
// Serial Monitor: 9600 baud

#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11);  // RX=10 (HC-05 TXD), TX=11 (HC-05 RXD)

unsigned long sonOzet = 0;
long byteSayac = 0;

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  Serial.println("=== HC-05 HAM VERI MONITOR ===");
  Serial.println("HC-05'ten gelen her byte asagida yazilir.");
  Serial.println("Byte geliyorsa link CALISIYOR. Her saniye toplam byte sayisi yazilir.");
  Serial.println("--------------------------------------------------");
}

void loop() {
  while (BT.available()) {
    int b = BT.read();
    byteSayac++;

    // Byte'i hem sayi hem (yazilabilirse) karakter olarak goster
    Serial.print(b);
    if (b >= 32 && b <= 126) {       // yazilabilir karakter
      Serial.print("('");
      Serial.print((char)b);
      Serial.print("')");
    }
    Serial.print(' ');
  }

  // Her saniye ozet
  if (millis() - sonOzet > 1000) {
    sonOzet = millis();
    Serial.println();
    Serial.print(">>> son 1sn toplam byte: ");
    Serial.print(byteSayac);
    Serial.println(byteSayac == 0 ? "   ***HIC VERI YOK***" : "   (veri geliyor)");
    Serial.println("--------------------------------------------------");
    byteSayac = 0;
  }
}
