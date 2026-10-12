// HC-05 AT ayar sketch'i - hem master hem slave icin ayni kod
// Bagantilar:
//   HC-05 VCC -> Uno 5V
//   HC-05 GND -> Uno GND
//   HC-05 TXD -> Uno pin 10
//   HC-05 RXD -> Uno pin 11
//   EN pini bosta (butona basip guc verecegiz)
//
// Serial Monitor: 9600 baud, Both NL & CR

#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11);  // RX, TX

void setup() {
  Serial.begin(9600);
  BT.begin(38400);   // AT modunda HC-05 38400 baud
  Serial.println("=== HC-05 AT ayar ===");
  Serial.println("Butona basip guc ver, LED yavas yanmali");
  Serial.println("Sonra AT yaz, OK gelmeli");
}

void loop() {
  if (Serial.available()) BT.write(Serial.read());
  if (BT.available())     Serial.write(BT.read());
}
