# MUS 2.0 (Minded Universal Speedster 2) — Autonomiczny Robot Line Follower

[![Platform](https://img.shields.io/badge/Platform-ESP32--S3-orange.svg)](https://www.espressif.com/)
[![Framework](https://img.shields.io/badge/Framework-Arduino%20%2F%20ESP--IDF-blue.svg)](https://docs.platformio.org/)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green.svg)](https://www.freertos.org/)
[![UI](https://img.shields.io/badge/WebUI-WebSockets%20%2B%20HTML5-red.svg)](#interfejs-webowy-web-dashboard)
[![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)](LICENSE)

---

## 📌 Spis treści
1. [O projekcie](#-o-projekcie)
2. [Główne cechy systemu](#-główne-cechy-systemu)
3. [Architektura sprzętowa](#-architektura-sprzętowa)
4. [Struktura repozytorium](#-struktura-repozytorium)
5. [Algorytmy i teoria sterowania](#-algorytmy-i-teoria-sterowania)
   - [Fuzja sensorów i odometria (RK2)](#1-fuzja-sensorów-i-odometria-rk2)
   - [Kompensacja dryfu żyroskopu (Online Bias Correction)](#2-kompensacja-dryfu-żyroskopu-online-bias-correction)
   - [Odczyt linii i filtracja (Listwa 14x QTR + MUX)](#3-odczyt-linii-i-filtracja-listwa-14x-qtr--mux)
   - [Obsługa przerw w trasie (Gap Bridge)](#4-obsługa-przerw-w-trasie-gap-bridge)
   - [Mapowanie i odtwarzanie trasy (Map & DriveOnMap)](#5-mapowanie-i-odtwarzanie-trasy-map--driveonmap)
6. [Architektura oprogramowania (FreeRTOS Dual-Core)](#-architektura-oprogramowania-freertos-dual-core)
7. [Interfejs webowy (Web Dashboard)](#-interfejs-webowy-web-dashboard)
8. [Narzędzia analityczne i optymalizacyjne (Python)](#-narzędzia-analityczne-i-optymalizacyjne-python)
   - [MapOptimizer (Profilowanie prędkości)](#1-mapoptimizer-profilowanie-prędkości)
   - [Route Analyzer (Analizator tras z GUI)](#2-route-analyzer-analizator-tras-z-gui)
9. [Uruchomienie i konfiguracja](#-uruchomienie-i-konfiguracja)
   - [Wymagania](#wymagania)
   - [Kompilacja i wgrywanie firmware](#kompilacja-i-wgrywanie-firmware)
   - [Procedura kalibracji](#procedura-kalibracji)
10. [Podziękowania i bibliografia](#-podziękowania-i-bibliografia)

---

## 📖 O projekcie

**MUS 2.0 (Minded Universal Speedster 2)** to zaawansowany robot wyścigowy klasy **Line Follower**, zaprojektowany z myślą o rywalizacji sportowej oraz precyzyjnym mapowaniu toru. Projekt łączy klasyczne, szybkie śledzenie linii przy użyciu regulatora PD z zaawansowaną odometrią fuzjonującą odczyty enkoderów i żyroskopu (IMU), zapisem trajektorii w czasie rzeczywistym oraz odtwarzaniem zoptymalizowanej ścieżki z dynamicznym profilem prędkości (*Fast Trajectory Playback*).

Projekt bazuje na sprawdzonych rozwiązaniach z platformy **GRUZIK 4.0** (pierwotnie rozwijanej na mikrokontrolery STM32), przenosząc je i udoskonalając pod kątem dwurdzeniowego układu **ESP32-S3** (240 MHz, 16 MB Flash, PSRAM). Dzięki temu robot zyskał bezprzewodową łączność **Wi-Fi**, wbudowany serwer **WebSockets**, nowoczesny pulpit telemetryczny działający w przeglądarce oraz bezkolizyjną rejestrację telemetrii w pamięci PSRAM.

Matematyczne fundamenty rekonstrukcji toru i optymalizacji profilu prędkości bazują na pracy magisterskiej:  
> *Rafał Cymiński, „Optymalizacja toru ruchu robota klasy linefollower”, Politechnika Wrocławska, 2018 (promotor: dr inż. Robert Muszyński).*

---

## 🚀 Główne cechy systemu

- **Wysoka częstotliwość pętli sterowania (1 kHz):** Dedykowane zadanie FreeRTOS przypięte na stałe do rdzenia 1 wykonuje odczyty enkoderów, pobiera dane z IMU, liczy odometrię i steruje silnikami w deterministycznym takcie 1 ms.
- **Fuzja sensoryczna (IMU + Enkodery):** Połączenie enkoderów o wysokiej rozdzielczości z żyroskopem trójosiowym (oś Z) w algorytmie całkowania Rungego-Kutty 2. rzędu (RK2 Midpoint) z wagą żyroskopu na poziomie 98%.
- **Zero-drift Gyro Z:** Automatyczna, nieblokująca kalibracja biasu żyroskopu przy starcie oraz adaptacyjny mechanizm douczania zera (*Online Bias Learning Rate*) w momentach postoju robota.
- **Odczyt 14 czujników linii:** Listwa optyczna zintegrowana z 16-kanałowym multiplekserem analogowym **CD74HC4067**, oferująca subpikselową detekcję środka taśmy.
- **Gap Bridge (Pokonywanie przerw):** Inteligentny algorytm podtrzymania trajektorii i komend silników w przypadku zaniku linii (do 4 sekund), zapobiegający gwałtownemu wypadnięciu z trasy.
- **Bezkolizyjna telemetria w PSRAM (512 KB):** Brak zacięć magistrali SPI – dane z pętli 1 kHz trafiają bezpośrednio do szybkiej pamięci RAM, a zrzut na kartę microSD następuje asynchronicznie po zatrzymaniu robota.
- **Wbudowany Web Dashboard:** Panel sterowania z wizualizacją parametrów w czasie rzeczywistym (20 Hz przez WebSockets), strojeniem nastaw regulatora PID oraz podglądem mapy.
- **Pakiet narzędzi Python:**
  - `MapOptimizer`: Generowanie zoptymalizowanego pliku `map.txt` z dynamicznym wyprzedzającym hamowaniem przed łukami (*Anticipatory Braking*).
  - `route_analyzer`: Interaktywne środowisko GUI (Tkinter) do badania torów jazdy lewego/prawego koła, kompensacji wysunięcia czujnika (128 mm) oraz nanoszenia heatmap prędkości.

---

## 🛠 Architektura sprzętowa

| Podzespół | Specyfikacja / Model | Rola w systemie |
| :--- | :--- | :--- |
| **MCU** | **ESP32-S3-WROOM-1 / DevKitC-1** (Dual-Core 240 MHz, 16 MB Flash, PSRAM) | Główna jednostka obliczeniowa, obsługa pętli RTOS, serwera Wi-Fi i odometrii. |
| **Listwa czujników** | 14x fototranzystor / czujnik odbiciowy (układ analogowy) | Pomiar pozycji linii pod przednią częścią robota. |
| **Multiplekser** | **CD74HC4067** (16-kanałowy analogowy MUX) | Przełączanie 14 kanałów czujników linii na 1 wejście analogowe ADC MCU. |
| **Czujnik IMU** | **Bosch BMI160** (SPI / HSPI) | Precyzyjny pomiar prędkości kątowej wokół osi pionowej $\omega_z$ (Gyro Z). |
| **Enkodery** | 2x magnetyczne enkodery Halla (N20) / opcjonalnie **AS5600** (I2C) | Pomiar obrotów kół, przebytej drogi i prędkości liniowej obu stron robota. |
| **Sterownik silników** | **DRV8835** (Dual H-Bridge, tryb IN/IN) | Sterowanie silnikami lewym i prawym za pomocą sygnałów PWM. |
| **Napęd** | 2x silniki mikropololu DC **N20** (przełożenie 10:1, koła $\varnothing$ 26 mm) | Układ jezdny robota w konfiguracji różnicowej. |
| **Pamięć trasy** | Karta MicroSD (magistrala FSPI) | Archiwizacja surowych map `GRUZIK.txt` i odczyt zoptymalizowanych `map.txt`. |
| **Sygnalizacja** | **WS2812B** (RGB LED) | Wizualny wskaźnik stanu robota, kalibracji i błędów. |

### Schemat połączeń pinów (ESP32-S3)
```text
           ┌────────────────────────────────────────┐
           │              ESP32-S3 MCU              │
           ├──────────────────┬─────────────────────┤
           │ GPIO 1           │ MUX Signal (ADC)    │
           │ GPIO 2 .. 5      │ MUX S0 .. S3        │
           │ GPIO 6, 7        │ Enkoder Lewy (A, B) │
           │ GPIO 8, 9        │ Enkoder Prawy (A, B)│
           │ GPIO 10          │ Dioda LED WS2812B   │
           │ GPIO 11, 12, 13  │ SD Card SPI (MOSI/SCK/MISO)
           │ GPIO 20          │ SD Card CS          │
           │ GPIO 15, 16      │ Silnik Lewy (BIN2, BIN1)
           │ GPIO 17, 18      │ Silnik Prawy (AIN2, AIN1)
           │ GPIO 21          │ DRV8835 MODE        │
           │ GPIO 39, 40, 41  │ IMU SPI (MOSI/MISO/SCK)
           │ GPIO 42          │ IMU CS              │
           └──────────────────┴─────────────────────┘
```

---

## 📂 Struktura repozytorium

```
MUS2/
├── platformio.ini              # Konfiguracja projektu PlatformIO (płytka, biblioteki, partycje)
├── MUS_2.0_gruzik_port/        # Główny kod źródłowy firmware robota (C++ / Arduino / FreeRTOS)
│   ├── MUS_2.0_gruzik_port.ino # Główna pętla, definicje zadań FreeRTOS, setup i zarządca pamięci
│   ├── robot_config.h          # Stałe fizyczne, rozstaw kół, piny GPIO, wagi odometrii
│   ├── motor.cpp / .h          # Sterowanie silnikami DRV8835, regulatory PI prędkości kół
│   ├── encoder.cpp / .h        # Obsługa przerwań enkoderów kwadraturowych (2x CHANGE ISR)
│   ├── imu.cpp / .h            # Obsługa BMI160 po SPI, kalibracja i dynamiczna kompensacja biasu
│   ├── odometry.cpp / .h       # Fuzja danych enkoderów i żyroskopu, estymacja pozycji (x, y, yaw)
│   ├── line_follower.cpp / .h  # Odczyt czujników linii (MUX), kalibracja, regulator PD
│   ├── map.cpp / .h            # Rejestracja punktów trasy, zamknięcie pętli, algorytm DriveOnMap
│   ├── web_server.cpp / .h     # Asynchroniczny serwer HTTP i WebSockets (AsyncWebServer)
│   ├── web_ui.h                # Kompletny interfejs HTML/CSS/JS zapisany w pamięci Flash (PROGMEM)
│   └── nvs_params.cpp / .h     # Zapis i odczyt parametrów konfiguracyjnych w pamięci NVS
├── MapOptimizer/               # Narzędzie optymalizacji mapy (Python)
│   ├── optimize_map.py         # Skrypt generujący map.txt (krzywizna, anticipatory braking)
│   └── README.md               # Dokumentacja obsługi optymalizatora
├── route_analyzer/             # Desktopowy analizator tras z graficznym interfejsem (GUI)
│   ├── app.py                  # Aplikacja okienkowa Tkinter
│   ├── canvas_view.py          # Renderowanie trajektorii, kompensacja listwy sensorów, tory kół
│   ├── file_io.py              # Parser plików GRUZIK.txt i obsługa formatów tras
│   ├── transform.py            # Narzędzia transformacji afinicznych (translacja, rotacja, skala)
│   └── reference_route.json    # Przykładowa trasa referencyjna do testów i kalibracji
├── GRUZIK4.0-main/             # Oryginalne źródła bazowe robota GRUZIK 4.0 (STM32 + Android App)
│   └── Android App/            # Aplikacja na platformę Android do komunikacji Bluetooth
└── MUS2_bugreport_chatbot.md   # Dziennik analizy i usuniętych błędów odometrii i żyroskopu
```

---

## 🧠 Algorytmy i teoria sterowania

### 1. Fuzja sensorów i odometria (RK2)
Śledzenie pozycji robota $(x, y, \theta)$ odbywa się w oparciu o fuzję enkoderów i żyroskopu. Z enkoderów wyznaczany jest przyrost drogi lewego i prawego koła ($\Delta s_L, \Delta s_R$), co pozwala obliczyć średni przemieszczenie liniowe:
$$\Delta s = \frac{\Delta s_R + \Delta s_L}{2}$$

Przyrost kąta obrotu szacowany jest równolegle z dwóch niezależnych źródeł:
- **Z enkoderów:** $\Delta \theta_{enc} = \frac{\Delta s_R - \Delta s_L}{B}$ (gdzie $B$ to efektywny rozstaw kół, `ROBOT_WHEEL_BASE_M`).
- **Z żyroskopu:** $\Delta \theta_{gyro} = \omega_z \cdot \Delta t$

Kąty są fuzjonowane z wagą $w = 0.98$:
$$\Delta \theta = w \cdot \Delta \theta_{gyro} + (1 - w) \cdot \Delta \theta_{enc}$$

Pozycja jest aktualizowana metodą punktu pośredniego (Runge-Kutta 2. rzędu):
$$\theta_{mid} = \theta_k + \frac{\Delta \theta}{2}$$
$$x_{k+1} = x_k + \Delta s \cdot \cos(\theta_{mid})$$
$$y_{k+1} = y_k + \Delta s \cdot \sin(\theta_{mid})$$
$$\theta_{k+1} = \theta_k + \Delta \theta$$

### 2. Kompensacja dryfu żyroskopu (Online Bias Correction)
Żyroskop BMI160 ma tendencję do powolnego dryfu temperaturowego. Poza 500-milisekundową kalibracją przy starcie, robot monitoruje stan zatrzymania:
```cpp
if (GRUZIK.PowerMode != Start &&
    fabsf(Motor_L.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M &&
    fabsf(Motor_R.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M &&
    fabsf(IMU.Gyro_Z) < ROBOT_GYRO_STATIONARY_DPS)
{
    IMU_UpdateGyroZBias(&IMU, ROBOT_GYRO_BIAS_LEARN_RATE);
}
```
Gdy robot stoi na starcie, bias żyroskopu jest adaptacyjnie korygowany, co zapobiega fałszywemu "zakrzywianiu" prostych odcinków toru.

### 3. Odczyt linii i filtracja (Listwa 14x QTR + MUX)
Listwa składa się z 14 sensorów optycznych. Poziomy napięć poddawane są normalizacji na podstawie zmapowanych minimów i maksimów z kalibracji:
$$V_{norm} = \text{clamp}\left(\frac{V_{raw} - V_{min}}{V_{max} - V_{min}} \cdot 1000, 0, 1000\right)$$
Pozycja linii wyliczana jest jako średnia ważona aktywnych czujników. Środek listwy odpowiada wartości `6500`. Błąd regulacji wynosi:
$$e_P = \frac{\text{Pozycja} - 6500}{1000}$$
Sterowanie silnikami realizuje szybki regulator PD z filtracją składowej różniczkowej:
$$u = K_p \cdot e_P + K_d \cdot (e_P - e_{P, prev})$$

### 4. Obsługa przerw w trasie (Gap Bridge)
Jeśli robot widział stabilną linię blisko środka listwy, a w kolejnym cyklu nastąpił nagły brak odczytu na wszystkich 14 czujnikach, aktywowany jest tryb mostkowania przerw (`Gap Bridge`). Robot przez maksymalnie 4 sekundy kontynuuje jazdę z zachowaniem ostatnich poprawnych prędkości kół, unikając gwałtownego obrotu w miejscu. Po ponownym wykryciu linii płynnie wraca do standardowego algorytmu PD.

### 5. Mapowanie i odtwarzanie trasy (Map & DriveOnMap)
1. **Tryb mapowania:** Robot pokonuje tor po linii, rejestrując co 40 cykli (40 ms / ~25 Hz) rekordy w postaci:
   $$\text{Format zapisu: } [x_{ri}, \quad y_{ri}, \quad x_{pc}, \quad y_{pc}, \quad v, \quad e_P, \quad e_D]$$
   gdzie $(x_{ri}, y_{ri})$ to środek osi kół, a $(x_{pc}, y_{pc})$ to pozycja przedniej listwy sensorów.
2. **Automatyczne domknięcie pętli:** Po pokonaniu minimalnego dystansu (0.45 m) i zbliżeniu się do punktu początkowego na odległość poniżej 3 cm (`ROBOT_MAP_CLOSE_RADIUS_M = 0.030f`), rejestracja zostaje automatycznie zakończona, a dane zbuforowane w PSRAM lądują w pliku `/GRUZIK.txt` na karcie SD.
3. **Odtwarzanie (DriveOnMap):** Robot podąża wzdłuż zdefiniowanych segmentów ścieżki, stosując zmienny lookahead uzależniony od prędkości jazdy oraz hybrydową korektę łączącą odometrię ze stopniowym zaufaniem do czujnika optycznego (*Line Confidence*).

---

## ⚡ Architektura oprogramowania (FreeRTOS Dual-Core)

Projekt w pełni wykorzystuje wielordzeniowość mikrokontrolera ESP32-S3:

```
                  ┌────────────────────────────────────────────────────────┐
                  │                 ESP32-S3 FreeRTOS                      │
                  └───────────────────────────┬────────────────────────────┘
                                              │
              ┌───────────────────────────────┴───────────────────────────────┐
              ▼                                                               ▼
  ┌───────────────────────┐                                       ┌───────────────────────┐
  │   CORE 1 (Real-Time)  │                                       │   CORE 0 (Background) │
  ├───────────────────────┤                                       ├───────────────────────┤
  │ ControlLoopTask       │                                       │ WebTelemetryTask      │
  │ • Częstotliwość: 1 kHz│                                       │ • Częstotliwość: 20 Hz│
  │ • Enkodery & IMU (SPI)│                                       │ • WebSockets Broadcast│
  │ • Odometria RK2       │                                       │ • Serwer HTTP         │
  │ • Pętla PD linii      │                                       ├───────────────────────┤
  │ • Regulatory PI kół   │                                       │ sdWriteTask           │
  │ • Buforowanie w PSRAM │                                       │ • Asynchroniczny zrzut│
  │   (512 KB no-wait)    │                                       │   danych z PSRAM na SD│
  └───────────────────────┘                                       │ • Dostęp przez semafor│
                                                                  └───────────────────────┘
```

### Arbitraż magistrali SPI
Podczas jazdy żyroskop IMU korzysta z szybkiego odczytu SPI w przerwaniu pętli 1 kHz. Zapis na kartę SD w trakcie jazdy zablokowałby pętlę na kilkadziesiąt milisekund (SD write latency). Z tego względu dane trafiają do bufora **PSRAM**, a zadanie `sdWriteTask` (Core 0) wykonuje flush na SD dopiero po zatrzymaniu robota, z pełną synchronizacją przez semafor binarny `spiSem`.

---

## 🌐 Interfejs webowy (Web Dashboard)

Robot po włączeniu uruchamia punkt dostępowy Wi-Fi (SoftAP):
- **SSID:** `LFMUS`
- **Hasło:** `rabarbar`
- **Adres panelu:** `http://192.168.4.1/`

Dashboard został zrealizowany jako aplikacja Single-Page (SPA) w pliku `web_ui.h` i udostępnia 4 główne moduły:

1. **📊 Telemetry (Główny pulpit):**
   - Aktualna prędkość lewego i prawego koła [m/s].
   - Zliczenia enkoderów oraz błąd linii.
   - Odczyt żyroskopu Z (surowy, bias oraz po korekcie).
   - Aktualny stan automatu skończonego (Idle, Following Line, Mapping, Playback).
2. **🎮 Control (Sterowanie i nastawy):**
   - Przyciski trybów: `START`, `STOP`, `Force Forward` (jazda testowa w przód na żądanie).
   - Kalibracja czujników linii (`Calibrate Line`) oraz zapis parametrów do NVS (`Save Params`).
   - Uruchamianie procedur: `Record Map`, `Play Map`, `Fast Play Map`.
   - Suwaki w czasie rzeczywistym do strojenia prędkości bazowej oraz wzmocnień $K_p$, $K_d$ i ich mnożników.
3. **🔬 Debug (Diagnostyka sprzętowa):**
   - Statusy sprzętowe modułów: BMI160, Lewy enkoder (GPIO 6/7), Prawy enkoder (GPIO 8/9).
   - Wskaźnik postępu kalibracji żyroskopu.
   - Odometria na żywo: współrzędne $X$ [m], $Y$ [m] oraz kąt $\text{Yaw}$ [°].
4. **🗺️ Map (Wizualizacja toru):**
   - Podgląd surowej (`/map_raw`) oraz zoptymalizowanej (`/map_opt`) mapy narysowanej na elemencie HTML5 Canvas.

---

## 🐍 Narzędzia analityczne i optymalizacyjne (Python)

### 1. MapOptimizer (Profilowanie prędkości)
Katalog `MapOptimizer/` zawiera skrypt `optimize_map.py`, który przetwarza surowy plik telemetrii `GRUZIK.txt` na gotową mapę `map.txt`:
- Usuwa powtarzające się punkty z przestojów odometrii.
- Analizuje lokalne promienie krzywizny toru $R$.
- Nadaje wysoką prędkość na długich prostych, a obniża prędkość w ciasnych zakrętach w zależności od współczynnika `gain`.
- **Anticipatory Braking:** Implementuje wyprzedzające hamowanie – robot zaczyna zwalniać z wyprzedzeniem przed wejściem w łuk, biorąc pod uwagę maksymalne dopuszczalne przeciążenie poprzeczne.

**Uruchomienie:**
```bash
cd MapOptimizer
python optimize_map.py --input GRUZIK.txt --output map.txt --max_speed 250 --min_speed 80
```

---

### 2. Route Analyzer (Analizator tras z GUI)
Katalog `route_analyzer/` to dedykowana desktopowa aplikacja diagnostyczna napisana w Pythonie (Tkinter):
- Wizualizuje zrekonstruowaną trasę robota w układzie współrzędnych świata.
- **Kompensacja listwy:** Przelicza pozycję środka obrotu kół $(x_{ri}, y_{ri})$ na realny ślad linii pod czujnikami z uwzględnieniem odsunięcia 128 mm i odchyłki $e_P$.
- **Wizualizacja torów kół:** Pokazuje ślady lewego i prawego koła z uwzględnieniem rozstawu 140 mm, ułatwiając detekcję ścięcia zakrętów lub poślizgu.
- **Heatmapa prędkości:** Koloruje trasę w zależności od chwilowej prędkości liniowej.
- **Dopasowanie do trasy wzorcowej:** Umożliwia manualną lub automatyczną transformację (przesunięcie, obrót, skala) i porównanie z referencyjnym torem zawodów (`reference_route.json`).

**Uruchomienie:**
```bash
cd route_analyzer
python app.py test_route.txt
```

---

## 💻 Uruchomienie i konfiguracja

### Wymagania
- [VS Code](https://code.visualstudio.com/) z rozszerzeniem [PlatformIO IDE](https://platformio.org/)
- Przewód USB-C do programowania ESP32-S3
- Python 3.9+ (dla skryptów optymalizacji i analizy)

### Kompilacja i wgrywanie firmware
1. Otwórz folder projektu w VS Code / PlatformIO.
2. Podłącz płytkę ESP32-S3 do portu USB komputera.
3. W terminalu PlatformIO wykonaj:
```bash
# Kompilacja projektu
pio run

# Wgranie firmware na mikrokontroler
pio run -t upload

# Otwarcie monitora portu szeregowego (115200 baud)
pio run -t monitor
```

---

### Procedura kalibracji

#### 1. Kalibracja czujników linii
1. Postaw robota nad białą i czarną powierzchnią toru.
2. W panelu webowym w zakładce **Control** kliknij `Calibrate Line`.
3. Przesuwaj ręcznie przodem robota w poprzek linii, aby każdy z 14 czujników zarejestrował skrajne wartości bieli i czerni.
4. Kliknij `Save Params`, aby zapisać nastawy w pamięci NVS.

#### 2. Kalibracja efektywnej bazy kół (Rozstawu osi)
Fizyczny rozstaw kół wynosi 140 mm (`ROBOT_WHEEL_BASE_M = 0.140f`). Ze względu na odkształcenia opon i poślizg mikroskopijny, efektywna baza kół wymaga kalibracji:
1. W pliku `robot_config.h` ustaw tymczasowo wagę żyroskopu na 0:
   ```cpp
   #define ROBOT_ODOMETRY_GYRO_WEIGHT 0.0f
   ```
2. Postaw robota na torze i wykonaj precyzyjny obrót o 360° w miejscu.
3. Odczytaj kąt z enkoderów w panelu debugowania (`dbg-odomYaw`).
4. Skoryguj bazę kół według wzoru:
   $$B_{nowa} = B_{stara} \cdot \left(\frac{\theta_{zmierzony}}{360^\circ}\right)$$
5. Przywróć wagę żyroskopu:
   ```cpp
   #define ROBOT_ODOMETRY_GYRO_WEIGHT 0.98f
   ```

---

## 📚 Podziękowania i bibliografia

- **Rafał Cymiński** – autor pracy magisterskiej *"Optymalizacja toru ruchu robota klasy linefollower"*, Politechnika Wrocławska (2018), która posłużyła za wzór matematyczny odometrii i algorytmu segmentowej jazdy po mapie.
- **Projekt GRUZIK 4.0** – baza architektoniczna, algorytmy sterowania silnikami i logika mapowania trasy.
- Społeczność **PlatformIO** i **Espressif Systems** za biblioteki wspierające dwurdzeniowy system FreeRTOS na układach ESP32-S3.

---

<p align="center">
  <b>MUS 2.0 Team</b> • Projekt rozwijany w ramach zaawansowanych konstrukcji robotów wyścigowych
</p>
