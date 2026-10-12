// =========================== ALICI (ARABA) ===========================
// HC-05'ten binary paket okur, checksum dogrular, L298N ile 4 motoru surer.
//   Sol 2 motor paralel -> OUT1/OUT2
//   Sag 2 motor paralel -> OUT3/OUT4
//
// BAGLANTILAR:
//   HC-05: TXD -> pin 10, RXD -> pin 11, VCC -> 5V, GND -> GND
//   L298N: ENA->5, IN1->2, IN2->3, ENB->6, IN3->4, IN4->7
//   ENA/ENB jumper'lari CIKARILMIS olmali (yoksa hiz kontrolu calismaz)
//
// GUVENLIK:
//   - Checksum tutmayan paket COPE atilir (motor asla bozuk veriyle donmez)
//   - 500 ms paket gelmezse motorlar durur (baglanti koptu demektir)
// ======================================================================

#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11);  // RX=10 (HC-05 TXD'ye), TX=11 (HC-05 RXD'ye)

// --- Paket yapisi: verici ile BIREBIR AYNI olmali ---
struct __attribute__((packed)) CommandData {
  int16_t sol;   // sol motor grubu hizi: -255 (geri) .. +255 (ileri)
  int16_t sag;   // sag motor grubu hizi: -255 .. +255
};
// Kablodaki paket: '<' + 4 byte veri + 1 byte checksum + '>'  (toplam 7 byte)

// --- L298N pin tanimlari ---
const int ENA = 5;   // sol grup hiz (PWM olmali - D5 uygun)
const int IN1 = 2;   // sol grup yon ucu 1
const int IN2 = 3;   // sol grup yon ucu 2
const int ENB = 6;   // sag grup hiz (PWM olmali - D6 uygun)
const int IN3 = 4;   // sag grup yon ucu 1
const int IN4 = 7;   // sag grup yon ucu 2

// --- Baglanti kopma korumasi ---
unsigned long sonMesaj = 0;          // son gecerli paketin geldigi an (ms)
const unsigned long TIMEOUT = 500;   // bu sureden uzun sessizlik olursa DUR

// --- Heartbeat: kumandaya "ben buradayim" sinyali ---
// Kumandanin STATE pini yok, baglantiyi baska turlu anlayamiyor. Bu yuzden
// duzenli araliklarla geriye tek byte yolluyoruz. Kumanda 800 ms boyunca
// bunu goremezse ekranda "BAGLANTI YOK" yazar.
const uint8_t HEARTBEAT_BYTE = 'A';            // verici ile AYNI olmali
const unsigned long HEARTBEAT_ARALIK = 200;    // ms (saniyede 5 kez)
unsigned long sonHeartbeat = 0;                // en son ne zaman yolladik

// ----------------------------------------------------------------------
// Seri tampondan paket okur. Ozellikleri:
//  - BLOKLAMAZ: paket tam gelmediyse beklemez, sonraki loop'ta devam eder
//  - Cop byte gelirse '<' bulana kadar atar (kendini toparlar)
//  - Checksum tutmayan paketi reddeder
//  - Tamponda birikmis birden fazla paket varsa EN SONUNCUYU kullanir
// Donus: true = 'out' icinde gecerli yeni komut var
// ----------------------------------------------------------------------
bool readCommand(Stream &s, CommandData &out) {
  bool got = false;                                  // gecerli paket bulduk mu
  while (s.available() > 0) {                        // tamponda byte varken
    if (s.peek() == '<') {                           // paket baslangici mi?
      if (s.available() >= (int)sizeof(CommandData) + 3) {  // paketin tamami geldi mi? (veri+chk+'>')
        s.read();                                    // '<' isaretini atla
        CommandData tmp;                             // gecici okuma alani
        s.readBytes((char *)&tmp, sizeof(CommandData)); // 4 byte veriyi al
        uint8_t chk   = s.read();                    // gelen checksum
        uint8_t bitis = s.read();                    // paket sonu isareti olmali

        uint8_t *p = (uint8_t *)&tmp;                // checksum'i yeniden hesapla
        uint8_t hesap = 0;
        for (uint8_t i = 0; i < sizeof(tmp); i++) hesap ^= p[i];  // XOR toplami

        if (bitis == '>' && chk == hesap) {          // cerceve VE checksum dogruysa
          out = tmp;                                 // komutu disari ver
          got = true;                                // (dongude kalip daha yenisi var mi bakariz)
        }
        // dogru degilse: paket bozuk, cope; dongu '<' arayarak devam eder
      } else {
        break;                                       // paket henuz tam gelmemis: bekleme, cik
      }
    } else {
      s.read();                                      // '<' degil: cop byte, at
    }
  }
  return got;
}

// ----------------------------------------------------------------------
// Motor surme yardimcilari: +hiz ileri, -hiz geri, 0 dur
// TEKERLER TERS DONERSE: ilgili fonksiyondaki HIGH ve LOW'u yer degistir.
// ----------------------------------------------------------------------
void solMotor(int hiz) {
  if (hiz >= 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }        // ileri yon
  else          { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); hiz = -hiz; }  // geri yon
  analogWrite(ENA, hiz);   // hiz: 0-255 arasi PWM
}

void sagMotor(int hiz) {
  if (hiz >= 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }        // ileri yon
  else          { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); hiz = -hiz; }  // geri yon
  analogWrite(ENB, hiz);   // hiz: 0-255 arasi PWM
}

// Iki grubu birden durdur
void dur() {
  analogWrite(ENA, 0);     analogWrite(ENB, 0);     // PWM'leri kes
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  // yon pinlerini notr yap
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
}

void setup() {
  Serial.begin(9600);      // USB uzerinden debug cikti icin
  BT.begin(9600);          // HC-05 data modu hizi

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);       // tum surucu pinleri cikis
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  dur();                   // acilista motorlar kesin dursun
}

void loop() {
  CommandData cmd;                       // gelen komut buraya yazilacak
  if (readCommand(BT, cmd)) {            // yeni gecerli paket var mi?
    // Ek guvenlik: degerler beklenen aralikta mi? (bozuk paket motoru oynatmasin)
    if (cmd.sol >= -255 && cmd.sol <= 255 && cmd.sag >= -255 && cmd.sag <= 255) {
      solMotor(cmd.sol);                 // sol grubu gelen hizla sur
      sagMotor(cmd.sag);                 // sag grubu gelen hizla sur
      sonMesaj = millis();               // "en son bu an veri geldi" diye kaydet
      Serial.print(cmd.sol);             // debug: Serial Monitor'de izlemek icin
      Serial.print(',');
      Serial.println(cmd.sag);
    }
  }

  // Kumandaya heartbeat yolla. Komutlar okunduktan HEMEN SONRA yapiyoruz:
  // SoftwareSerial ayni anda hem gonderip hem alamaz, tampon yeni bosalmisken
  // yazmak paket kaybetme ihtimalini en aza indirir. (Kaybolsa bile checksum
  // bozuk paketi eler ve saniyede ~33 paket geliyor, biri dusse fark etmez.)
  if (millis() - sonHeartbeat >= HEARTBEAT_ARALIK) {
    BT.write(HEARTBEAT_BYTE);
    sonHeartbeat = millis();
  }

  // Baglanti kopma korumasi: TIMEOUT boyunca paket gelmediyse aracbayi durdur.
  // NOT: Verici safe zone'da bilerek veri gondermiyor; o durumda zaten
  // son gelen komut 0,0 idi (dur paketleri), yani araba durmus olur.
  if (millis() - sonMesaj > TIMEOUT) {
    dur();
  }
}
