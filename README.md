# MUS 2.0 (Minded Universal Speedster)

Projekt szybkiego robota klasy Line Follower z funkcją mapowania toru i późniejszego odtwarzania zoptymalizowanej trasy (*speed playback*). 

Jest to port i ewolucja wcześniejszego projektu **GRUZIK 4.0** (który działał na STM32) przeniesiona na **ESP32-S3**. Zmiana platformy pozwoliła na wrzucenie całości na FreeRTOS (dwa rdzenie), dodanie łączności Wi-Fi z panelem webowym do telemetrii i strojenia parametrów w locie oraz buforowanie logów w pamięci PSRAM.

---

## Jak to działa w skrócie

1. **Zwykła jazda po linii (PID):** Robot zbiera odczyty z listwy 14 czujników analogowych (przez multiplekser) i jedzie po trasie regulatorem PD.
2. **Mapowanie trasy:** W trybie mapowania robot jedzie po linii i co określony interwał zapisuje punkty odometrii (fuzja enkoderów + żyroskop BMI160) do bufora w pamięci PSRAM. Gdy wróci na start, pętla zamyka się automatycznie i całość zrzucana jest do pliku `GRUZIK.txt` na karcie SD.
3. **Optymalizacja trasy:** Skrypt w Pythonie (`MapOptimizer`) analizuje krzywizny toru, wyznacza proste i zakręty, po czym generuje plik `map.txt` z dynamicznym profilem prędkości (wyprzedzające dohamowywanie przed zakrętami).
4. **Odtwarzanie trasy (DriveOnMap):** Robot jedzie po wyznaczonych segmentach mapy z wyższą prędkością. Nawigacja opiera się głównie na odometrii i kącie z IMU, a czujniki linii wprowadzają jedynie delikatną korektę poprzeczną (zależną od poziomu pewności odczytu).

---

## Hardware

- **MCU:** ESP32-S3 DevKitC-1 (16 MB Flash, PSRAM, taktowanie 240 MHz)
- **Czujniki linii:** Listwa 14 fototranzystorów / czujników odbiciowych spięta z 16-kanałowym multiplekserem analogowym **CD74HC4067** (1 wejście ADC w ESP32)
- **IMU:** Bosch **BMI160** po magistrali SPI (odczyt żyroskopu $\omega_z$)
- **Enkodery:** Halla na silnikach N20 (oprogramowane po przerwaniach GPIO, 2x CHANGE na kanale A, 140 impulsów na obrót koła)
- **Napęd:** 2x silniki DC N20 z przekładnią 10:1, koła o średnicy 26 mm, rozstaw kół nominalnie 140 mm
- **Sterownik silników:** **DRV8835** w trybie IN/IN
- **Pamięć trasy:** Slot kart microSD (SPI)
- **Sygnalizacja:** Adresowalna dioda WS2812B (RGB)

### Rozpiska pinów ESP32-S3

| Sygnał / Peryferium | Pin GPIO | Uwagi |
|---|---|---|
| MUX SIG (Analog In) | GPIO 1 | Sygnał analogowy z listwy czujników |
| MUX S0, S1, S2, S3 | GPIO 2, 3, 4, 5 | Adresowanie kanałów multipleksera |
| Enkoder Lewy (A, B) | GPIO 6, 7 | Kanał A na przerwaniu, B jako kierunek |
| Enkoder Prawy (A, B) | GPIO 8, 9 | Kanał A na przerwaniu, B jako kierunek |
| Dioda WS2812B RGB | GPIO 10 | Status i błędy |
| SD Card SPI (MOSI, SCK, MISO, CS) | GPIO 11, 12, 13, 20 | Główna magistrala SPI (FSPI) |
| Silnik Lewy (BIN2, BIN1) | GPIO 15, 16 | PWM / kierunek |
| Silnik Prawy (AIN2, AIN1) | GPIO 17, 18 | PWM / kierunek |
| DRV8835 MODE | GPIO 21 | Stan niski (tryb wejść IN/IN) |
| IMU SPI (MOSI, MISO, SCK, CS) | GPIO 39, 40, 41, 42 | Dedykowany interfejs SPI (HSPI) |

---

## Architektura oprogramowania (FreeRTOS)

ESP32-S3 ma dwa rdzenie i kod jest pod to ściśle podzielony:

### Core 1 — Deterministyczna pętla sterowania (`ControlLoopTask`, 1 kHz)
- Odczyt enkoderów i pobranie próbek żyroskopu po SPI.
- Odometria metodą Rungego-Kutty (RK2 midpoint) łącząca enkodery z żyroskopem (waga 0.98 dla IMU).
- Aktualizacja stanu regulatora linii (PD) lub algorytmu śledzenia mapy (`DriveOnMap`).
- Niezależne pętle regulatorów PI prędkości kół (zadawanie wypełnienia PWM dla mostka H).
- Zapis punktów trasy do bufora w pamięci PSRAM.

### Core 0 — Zadania w tle i komunikacja
- **`WebTelemetryTask` (20 Hz):** Rozsyła ramki JSON przez WebSockets do podłączonych przeglądarek (aktualna prędkość, odometria, zliczenia enkoderów, błąd linii).
- **`sdWriteTask`:** Obsługuje zapis na kartę SD. Zapis na SD potrafi przyblokować magistralę na kilkadziesiąt milisekund, co rozwaliłoby pętlę 1 kHz. Dlatego podczas jazdy logi lądują w 512 KB buforze w PSRAM, a to zadanie zrzuca całość na kartę dopiero po zatrzymaniu robota. Dostęp do magistrali chroniony jest semaforem FreeRTOS.

---

## Kluczowe elementy algorytmów

### 1. Fuzja odometrii i auto-bias żyroskopu
Odometria liczy drogę kół z enkoderów, a kąt obrotu pobiera w 98% z żyroskopu i w 2% z różnicy drogi kół. 

BMI160 ma naturalny dryf z temperaturą. Rozwiązano to dwuetapowo:
- Przy starcie robot zbiera 500 próbek i zeruje bias (robot musi wtedy stać nieruchomo).
- W trakcie pracy, jeśli robot stoi (prędkość z enkoderów bliska zeru i brak obrotu), pętla w sposób ciągły doucza bias żyroskopu z małą wagą (`ROBOT_GYRO_BIAS_LEARN_RATE = 0.0005f`). Dzięki temu proste odcinki nie zakrzywiają się na mapie z upływem czasu.

### 2. Gap Bridge (przejazd przez przerwy w linii)
Gdy robot jedzie prosto i nagle wszystkie czujniki tracą linię (np. przerwa w trasie na zawodach), nie wchodzi w gwałtowny obrót poszukiwawczy. Zamiast tego utrzymuje ostatnie poprawne wysterowanie silników do 4000 ms. Jeśli linia pojawi się z powrotem, regulator natychmiast wraca do normalnego śledzenia.

### 3. Zaufanie do czujników podczas jazdy z mapy (`DriveOnMap`)
W trybie jazdy po mapie robot wyznacza segment ścieżki i podąża za celem z wyprzedzeniem (*lookahead*). Odczyt z czujników linii jest brany pod uwagę tylko wtedy, gdy czujnik faktycznie widzi linię i jest ona blisko środka. Gdy robot lekko zjedzie z linii lub minie skrzyżowanie (za dużo aktywnych czujników naraz), waga czujnika spada i robot trzyma się wyliczonej trajektorii z odometrii.

---

## Panel webowy (Wi-Fi Dashboard)

Robot po uruchomieniu stawia własną sieć Wi-Fi:
- **SSID:** `LFMUS`
- **Hasło:** `rabarbar`
- **Adres w przeglądarce:** `http://192.168.4.1`

Interfejs zrobiony jest w czystym HTML/JS w pliku `web_ui.h` (siedzi w pamięci flash) i zawiera:
- **Telemetry:** Podgląd na żywo prędkości kół, odczytu żyroskopu, błędu linii i stanu robota.
- **Control:** Przyciski START/STOP, wymuszenie jazdy w przód, uruchamianie mapowania (`Record Map`), odtwarzania trasy oraz suwaki do zmiany w locie nastaw PID ($K_p$, $K_d$, prędkość bazowa).
- **Debug:** Status inicjalizacji BMI160, stan enkoderów, podgląd odometrii $X, Y, \text{Yaw}$.
- **Map:** Prosty podgląd narysowanej mapy na Canvasie (`/map_raw` i `/map_opt`).

---

## Narzędzia w Pythonie

### 1. Optymalizator mapy (`MapOptimizer/optimize_map.py`)
Skrypt bierze plik `GRUZIK.txt` wygenerowany przez robota i tworzy `map.txt`:
- Usuwa powtarzające się punkty z zatrzymań.
- Wylicza promień łuku dla każdego zakrętu.
- Dobiera prędkości: na prostych podbija pod zadaną prędkość maksymalną, w zakrętach redukuje zależnie od promienia.
- Liczy drogę hamowania przed zakrętem, żeby robot zwalniał przed wejściem w łuk, a nie w jego trakcie.

Uruchomienie:
```bash
cd MapOptimizer
python optimize_map.py --input GRUZIK.txt --output map.txt --max_speed 240 --min_speed 90
```

### 2. Analizator tras (`route_analyzer/app.py`)
Aplikacja desktopowa w Tkinterze do analizy zarejestrowanych przejazdów:
- Wczytuje pliki tras i rysuje rzeczywisty ślad robota z uwzględnieniem odsunięcia listwy czujników (128 mm przed osią kół).
- Może pokazać osobne tory dla lewego i prawego koła (rozstaw 140 mm).
- Koloruje trasę prędkością (heatmapa).
- Pozwala porównać przejazd z wzorcową trasą zawodów (`reference_route.json`).

Uruchomienie:
```bash
cd route_analyzer
python app.py test_route.txt
```

---

## Jak skompilować i wgrać

Projekt skonfigurowany jest pod **PlatformIO**:

1. Sklonuj repozytorium i otwórz folder w VS Code z wtyczką PlatformIO.
2. Podłącz ESP32-S3 przez USB.
3. Wyczyść i zbuduj projekt:
   ```bash
   pio run
   ```
4. Wgraj na płytkę:
   ```bash
   pio run -t upload
   ```
5. Podgląd logów na Serialu (115200 baud):
   ```bash
   pio run -t monitor
   ```

---

## Kalibracja

### Kalibracja listwy czujników
1. Połącz się z siecią Wi-Fi robota i wejdź na `http://192.168.4.1`.
2. W zakładce **Control** kliknij `Calibrate Line`.
3. Przeciągnij przodem robota kilkukrotnie w poprzek linii (żeby każdy sensor zobaczył pełną czerń i białe tło).
4. Kliknij `Save Params` – wartości min/max zapiszą się w pamięci nieulotnej NVS.

### Kalibracja efektywnej bazy kół
Baza fizyczna wynosi 140 mm (`ROBOT_WHEEL_BASE_M 0.140f`). Przez poślizg opon efektywny rozstaw na torze zwykle lekko się różni. Jeśli po przejechaniu kwadratu kąty nie mają 90°:
1. W `robot_config.h` ustaw tymczasowo `ROBOT_ODOMETRY_GYRO_WEIGHT 0.0f` (wyłączenie żyroskopu, sama odometria z kół).
2. Wykonaj robotem dokładny obrót o 360° w miejscu.
3. Sprawdź zaraportowany kąt na panelu debugowania.
4. Skoryguj bazę kół:
   $$\text{nowa\_baza} = \text{poprzednia\_baza} \cdot \left(\frac{\text{zliczony\_obrót\_rad}}{2\pi}\right)$$
5. Przywróć `ROBOT_ODOMETRY_GYRO_WEIGHT` na `0.98f`.
