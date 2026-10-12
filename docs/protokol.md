# Haberleşme protokolü

Motor hızlarını **kumanda** hesaplar (joystick merkez kalibrasyonu oradadır); araba yalnızca `sol` / `sag` değerlerini uygular. Değer aralığı her sürümde −255 … +255'tir, çok baytlı alanlar little-endian gider.

## nRF24 (son sürüm)

Radyo: kanal 108, 250 kbps, `RF24_PA_MAX`, CRC-16, otomatik ACK, dinamik payload + ACK payload.

**Komut paketi** — kumanda → araba, 13 bayt, 20 ms'de bir:

| Alan | Tip | Açıklama |
|---|---|---|
| `imza` | uint8 | Her zaman `0xA5` |
| `sayac` | uint8 | 0–255 döngüsel; atlamalardan kayıp paket sayılır |
| `joyX`, `joyY` | uint16 | Ham ADC (0–1023) |
| `pot` | uint16 | Ham ADC, kullanılmıyorsa 0 |
| `butonlar` | uint8 | bit0 = Buton 1, bit1 = Buton 2, bit2 = Buton 4 |
| `sol`, `sag` | int16 | Motor hızları |

**Telemetri paketi** — araba → kumanda, 8 bayt, ACK payload içinde:

| Alan | Tip | Açıklama |
|---|---|---|
| `imza` | uint8 | Her zaman `0x5A` |
| `vcc` | uint16 | Arabanın anlık besleme gerilimi (mV) |
| `minVcc` | uint16 | Sürüşte görülen en düşük gerilim (mV) |
| `mesafe` | uint16 | Radar mesafesi (cm), 999 = engel yok |
| `durum` | uint8 | bit0 = engel var, ileri kapalı |

Her iki `struct` `__attribute__((packed))` ile tanımlıdır; aktarıcılı sürümde boyutları `static_assert` ile sabitlenmiştir.

**Adresler**

| Kurulum | Kumanda gönderir | Araba dinler |
|---|---|---|
| Doğrudan (`03-nrf24-dogrudan`) | `ARB01` | `ARB01` |
| Aktarıcılı (`04-nrf24-aktaricili`) | `AKT01` | `ARB02` |

Aktarıcı `AKT01`'i dinler, gelen 13 baytı değiştirmeden `ARB02`'ye yollar. Arabanın ACK'indeki telemetriyi saklar ve kumandanın bir sonraki paketine vereceği ACK'e koyar; 300 ms'den eski telemetriyi iletmez, böylece araba kapalıyken kumanda "KOPUK" gösterir. Arabaya giden hatta tekrar sayısı kısa tutulmuştur (5 deneme × 1 ms) ki aktarıcı kumandayı dinlemeye çabuk dönsün.

**Zamanlamalar**

| Olay | Süre |
|---|---|
| Komut gönderimi | 20 ms (≈50 paket/sn) |
| Araba failsafe | 400 ms sessizlik → fren |
| Telemetri bayatlama (kumanda) | 1000 ms → "KOPUK" |
| Radar ölçümü | 60 ms |
| PWM rampası | 20 ms'de 30 adım (0 → 255 ≈ 170 ms) |
| Yön değişimi ölü zamanı | 70 ms |

## Bluetooth (HC-05)

9600 baud seri hat. nRF24'ün donanımda yaptığı çerçeveleme, CRC ve bağlantı tespiti burada elle yazılmıştır.

| Sürüm | Komut çerçevesi | Boy |
|---|---|---|
| v1 (`verici` / `alici`) | `<` + `sol` + `sag` + XOR checksum + `>` | 7 bayt |
| Radarlı (`vericiRadarli` / `aliciRadarli`) | `<` + `sol` + `sag` + `seq` + CRC-8 + `>` | 8 bayt |

- CRC-8: polinom `0x31`, başlangıç `0xFF`.
- Kumandanın HC-05'inde STATE pini kullanılamadığı için bağlantı durumu yazılımla bulunur: araba 200 ms'de bir geri bildirim yollar. v1'de tek bayt (`'A'`), radarlı sürümde telemetri çerçevesi: `0xA5 0x5A` + `vcc` + `minVcc` + `mesafe` + CRC-8.
- Yeniden bağlanınca HC-05 tamponunda birikmiş bayat paketler atılır (resenkronizasyon).
