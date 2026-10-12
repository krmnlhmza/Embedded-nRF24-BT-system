// HC-05 AT komut test sketch'i
// Bagantilar: HC-05 TXD -> pin 10, HC-05 RXD -> pin 11
// Serial Monitor: 9600 baud, Both NL & CR

#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11);  // RX, TX

void setup() {
  Serial.begin(9600);
  BT.begin(38400);            // HC-05 AT modunda 38400
  Serial.println("AT komutlarini yazabilirsin");
}

void loop() {
  if (Serial.available()) BT.write(Serial.read());
  if (BT.available())     Serial.write(BT.read());
}
