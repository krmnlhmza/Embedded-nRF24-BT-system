// TEST_ARABA - Bluetooth YOK, joystick KABLO ile ayni Arduino'ya bagli
// Amac: motor + surucu + joystick saglam mi diye test etmek.
// Calisirsa, ayni sol/sag mantigini HC-05 uzerinden calistiracagiz.
//
// --- BAGLANTILAR ---
// Joystick (kablo ile, shield'i ustune TAKMADAN jumper ile):
//   Joystick VCC -> Arduino 5V
//   Joystick GND -> Arduino GND
//   Joystick VRx -> A0
//   Joystick VRy -> A1
//
// L298N motor surucu:
//   ENA -> 5   (sol grup hiz / PWM)
//   IN1 -> 2   (sol yon)
//   IN2 -> 3   (sol yon)
//   ENB -> 6   (sag grup hiz / PWM)
//   IN3 -> 4   (sag yon)
//   IN4 -> 7   (sag yon)
//   OUT1/OUT2 -> sol 2 motor (paralel)
//   OUT3/OUT4 -> sag 2 motor (paralel)
//
// --- GUC (onemli) ---
//   Arduino: PC USB'den beslenir.
//   Motorlar: adaptor/prizden L298N'in MOTOR giris terminaline (12V/VMS) baglanir.
//   L298N GND  <->  Arduino GND  <->  adaptor GND   HEPSI ORTAK OLMALI.
//   Adaptorun (+) ucu SADECE L298N motor girisine gider, Arduino 5V'ye ASLA verilmez.

const int X_PIN = A0;
const int Y_PIN = A1;
const int DEAD_ZONE = 40;

// L298N pinleri
const int ENA = 5;
const int IN1 = 2;
const int IN2 = 3;
const int ENB = 6;
const int IN3 = 4;
const int IN4 = 7;

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  dur();
  Serial.println("TEST_ARABA hazir - joystik oynatinca sol,sag degerleri gelecek");
}

void loop() {
  int xRaw = analogRead(X_PIN);
  int yRaw = analogRead(Y_PIN);

  int ileri = map(yRaw, 0, 1023, -255, 255);
  int donus = map(xRaw, 0, 1023, -255, 255);

  if (abs(ileri) < DEAD_ZONE) ileri = 0;
  if (abs(donus) < DEAD_ZONE) donus = 0;

  // Diferansiyel mix (verici.ino ile ayni)
  int sol = ileri + donus;
  int sag = ileri - donus;

  sol = constrain(sol, -255, 255);
  sag = constrain(sag, -255, 255);

  motorlariSur(sol, sag);

  // Seri ekrandan takip: joystik bosta 0,0 gelmeli
  Serial.print(sol); Serial.print(','); Serial.println(sag);

  delay(50);
}

void motorlariSur(int sol, int sag) {
  // SOL grup
  if (sol >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    sol = -sol;
  }
  analogWrite(ENA, sol);

  // SAG grup
  if (sag >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    sag = -sag;
  }
  analogWrite(ENB, sag);
}

void dur() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
