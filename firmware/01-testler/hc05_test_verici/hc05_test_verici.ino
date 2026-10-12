// HC-05 TEST - VERICI  (SADECE joystick oku -> HC-05'ten gonder. MOTOR YOK.)
// HC-05 hardware Serial (pin 0/1) header'inda.
// ONEMLI: Upload ederken HC-05'i pin 0/1'den CIKAR, yukleyince geri tak.
//
// Baglanti (joystick):
//   VRx -> A0, VRy -> A1, VCC -> 5V, GND -> GND

struct __attribute__((packed)) CommandData {
  int16_t sol;
  int16_t sag;
};

void sendCommand(Stream &s, int16_t sol, int16_t sag) {
  CommandData c;
  c.sol = sol;
  c.sag = sag;
  uint8_t *p = (uint8_t *)&c;
  uint8_t chk = 0;
  for (uint8_t i = 0; i < sizeof(c); i++) chk ^= p[i];  // XOR checksum
  s.write('<');
  s.write(p, sizeof(c));
  s.write(chk);
  s.write('>');
}

const int X_PIN = A0;
const int Y_PIN = A1;
const int DEAD_ZONE = 40;

void setup() {
  Serial.begin(9600);  // ayni hat HC-05'e gidiyor
}

void loop() {
  int xRaw = analogRead(X_PIN);
  int yRaw = analogRead(Y_PIN);

  int ileri = map(yRaw, 0, 1023, -255, 255);
  int donus = map(xRaw, 0, 1023, -255, 255);

  if (abs(ileri) < DEAD_ZONE) ileri = 0;
  if (abs(donus) < DEAD_ZONE) donus = 0;

  int sol = constrain(ileri + donus, -255, 255);
  int sag = constrain(ileri - donus, -255, 255);

  sendCommand(Serial, sol, sag);

  delay(50);  // 20 paket/sn
}
