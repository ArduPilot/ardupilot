# 🚀 Indoor Altitude Hold - Hızlı Başlangıç Kılavuzu

**Hedef:** 15 dakikada firmware kurulumu + ilk test uçuşu

---

## ⚡ Hızlı Kurulum (3 Adım)

### 1️⃣ Firmware İndirme & Build (5 dakika)

```bash
# Clone repo
git clone https://github.com/yasincildir/ardupilot.git
cd ardupilot
git checkout claude/ardupilot-indoor-altitude-hold-01MdpREHPEf2iRDqtmsRDV4h
git submodule update --init --recursive

# Build (örnek: Pixhawk 4)
./waf configure --board=Pixhawk4
./waf copter

# Firmware: build/Pixhawk4/bin/arducopter.apj
```

**Diğer board'lar:**
- Cube Orange: `--board=CubeOrange`
- MatekH743: `--board=MatekH743`
- Liste: `./waf list_boards`

### 2️⃣ Firmware Yükleme (2 dakika)

**Mission Planner:**
1. **Initial Setup → Install Firmware**
2. **Load custom firmware**
3. `arducopter.apj` dosyasını seç
4. Upload tamamlanana kadar bekle

### 3️⃣ Parametreler (5 dakika)

**Minimum Konfigürasyon (Holybro H-Flow için):**

```ini
# Rangefinder
RNGFND1_TYPE = 31
RNGFND1_MAX = 4.00
RNGFND1_ORIENT = 25

# CAN + Flow
CAN_P1_DRIVER = 1
CAN_D1_PROTOCOL = 1
FLOW_TYPE = 6

# EKF3
EK3_SRC1_POSZ = 2
EK3_SRC1_VELXY = 6
EK3_RNG_M_NSE = 0.15
EK3_RNG_I_GATE = 250
EK3_ALT_M_NSE = 5.0
EK3_FLOW_USE = 1

# Position Controller
PSC_POSZ_P = 1.5
PSC_ACCZ_P = 0.50
PILOT_ACCEL_Z = 100
PILOT_SPEED_UP = 100

# Logging
LOG_DISARMED = 1
```

**Yükleme:**
- Mission Planner → **CONFIG → Full Parameter List**
- Parametreleri gir veya `.param` dosyasından yükle
- **Write Params** → **Reboot**

---

## ✅ Test Checklist (5 dakika)

### Pre-Flight:

```bash
☐ Rangefinder test (Mission Planner → Status → rangefinder)
  └─ Drone'u kaldır/indir → mesafe değişmeli

☐ Optical Flow test (Status → opt_m_x, opt_m_y)
  └─ Drone'u hareket ettir → flow rate değişmeli

☐ EKF Health (Status → ekf_flags)
  └─ Hepsi "OK" olmalı

☐ ARM test
  └─ ARM et → "Pre-Arm: Check" yoksa OK
```

### İlk Uçuş (Outdoor - Güvenli Alan):

```bash
1. STABILIZE modda takeoff
2. 1m yüksekliğe çık
3. ALTHOLD moduna geç
4. Throttle stick bırak
5. 30 saniye hover
6. Altitude stable mı? (±10-15cm OK)
```

### Indoor Test (Obje ile):

```bash
1. 30-40cm obje yerleştir (kutu, kitap)
2. ALTHOLD'da 1.5m yükseklikten yaklaş
3. Yavaşça obje üzerinden geç (max 0.5 m/s)
4. Altitude stable kalmalı (<10cm varyasyon)
```

---

## 🔧 Sorun Giderme (Hızlı Çözümler)

| Problem | Hızlı Çözüm |
|---------|-------------|
| **"Pre-Arm: EKF Variance"** | `EK3_RNG_I_GATE = 400` (was 250) |
| **Hala zıplama var** | Kod'da `OBSTACLE_JUMP_THRESHOLD_M 0.5f` (rebuild) |
| **Altitude drift** | Optical flow ekle veya `EK3_ALT_M_NSE = 2.0` |
| **Flow çalışmıyor** | Zeminde texture var mı? Aydınlatma yeterli mi? |
| **Log'da SURF.FH yok** | Firmware versiyonu `ce53306` veya sonrası mı? |

---

## 📊 Başarı Kriterleri

| Metrik | Hedef |
|--------|-------|
| **Hover Altitude Varyasyon** | <5cm (max 15cm kabul edilebilir) |
| **Obje Geçişinde Zıplama** | <10cm (max 20cm kabul edilebilir) |
| **Tilt Response** | <100ms |

---

## 📚 Detaylı Dokümantasyon

- **Tam Manual:** [INDOOR_ALTITUDE_HOLD_MANUAL_TR.md](./INDOOR_ALTITUDE_HOLD_MANUAL_TR.md)
- **GitHub:** https://github.com/yasincildir/ardupilot/tree/claude/ardupilot-indoor-altitude-hold-01MdpREHPEf2iRDqtmsRDV4h

---

## 🎯 Sonraki Adımlar

1. ✅ Firmware kuruldu
2. ✅ İlk test yapıldı
3. ⏭️ **Log analizi** (Mission Planner → Dataflash Logs)
   - `SURF.D` (raw), `SURF.FH` (floor), `SURF.OC` (obstacle counter) plotla
4. ⏭️ **Fine-tuning** (ihtiyaç duyarsan)
   - Parametreleri ayarla veya kod'daki threshold'ları değiştir

---

**İyi Uçuşlar! 🚁✨**

*15 dakikada kurulum tamamlandı. Sorular için: [INDOOR_ALTITUDE_HOLD_MANUAL_TR.md](./INDOOR_ALTITUDE_HOLD_MANUAL_TR.md)*
