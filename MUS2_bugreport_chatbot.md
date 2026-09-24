# Analiza błędów mapowania — MUS 2.0 (ESP32-S3 line follower)

Poniżej opis problemów znalezionych w repozytorium https://github.com/Naver510/MUS2, folder `MUS_2.0_gruzik_port`. Robot jeździ po linii, nagrywa mapę (odometria enkoder + żyroskop BMI160), a potem odtwarza trasę. Problem: po przejechaniu kwadratu mapa w aplikacji webowej rysuje krzywą niezamkniętą linię zamiast zamkniętego kwadratu.

---

## BUG 1 — KRYTYCZNY — proste odcinki wyglądają jak łuki

**Przyczyna:** brak online korekcji biasu żyroskopu podczas jazdy.

W pliku `imu.cpp` istnieje funkcja `IMU_UpdateGyroZBias()`, ale **nigdy nie jest wywoływana** z pętli sterowania `ControlLoopTask()` w `MUS_2.0_gruzik_port.ino`. Jedyna kalibracja to 500 próbek przy starcie. Bias narasta z temperaturą i wibracjami silników.

Przy `ROBOT_ODOMETRY_GYRO_WEIGHT = 0.96` kurs pochodzi w 96% z żyroskopu — każdy niewykorygowany drift biasu bezpośrednio integruje się w heading i wygina proste odcinki mapy.

**Referencja:** W `GRUZIK4.0/Core/Src/main.c` linia 322 ten blok istnieje i działa:
```c
if (fabsf(Motor_L.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M &&
    fabsf(Motor_R.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M &&
    fabsf(ICM.Gyro_Z) < ROBOT_GYRO_STATIONARY_DPS)
{
    ICM_UpdateGyroZBias(&ICM, ROBOT_GYRO_BIAS_LEARN_RATE);
}
```

**Naprawa:** W `MUS_2.0_gruzik_port.ino`, w `ControlLoopTask()`, zaraz po bloku `Odometry_Update(...)` (linia ~153) dopisać:

```cpp
// Online gyro bias correction — gdy robot stoi
if (fabsf(Motor_L.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M &&
    fabsf(Motor_R.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M &&
    fabsf(IMU.Gyro_Z) < ROBOT_GYRO_STATIONARY_DPS)
{
    IMU_UpdateGyroZBias(&IMU, ROBOT_GYRO_BIAS_LEARN_RATE);
}
```

Wszystkie stałe już są w `robot_config.h` (`ROBOT_ENCODER_STATIONARY_M = 0.00002f`, `ROBOT_GYRO_STATIONARY_DPS = 2.0f`, `ROBOT_GYRO_BIAS_LEARN_RATE = 0.0005f`). Funkcja już jest w `imu.cpp`. To tylko brakujące wywołanie.

---

## BUG 2 — KRYTYCZNY — kwadrat się nie zamyka

**Przyczyna:** `ROBOT_WHEEL_BASE_M = 0.135f` w `robot_config.h` to fizyczny pomiar, a nie skalibrowana efektywna baza kół.

W odometrii kąt skrętu z enkoderów liczy się jako:
```c
delta_yaw_encoder = (right_delta_m - left_delta_m) / ROBOT_WHEEL_BASE_M;
```
Efektywna baza kół zawsze różni się od fizycznej (poślizg, geometria opony). Błąd nawet kilku mm w tej wartości daje narożniki kwadratu które nie mają 90° → kwadrat się nie zamknie.

**Procedura kalibracji:**
1. Tymczasowo ustaw `ROBOT_ODOMETRY_GYRO_WEIGHT = 0.0f` (same enkodery).
2. Obróć robota dokładnie 360° w miejscu (różnicowo w miejscu).
3. Odczytaj `Odometry.encoder_yaw_rad` po zatrzymaniu.
4. Oblicz korektę: `nowa_baza = 0.135f × (wynik_rad / 6.2832f)`.
5. Wpisz nową wartość, powtórz 2–3 razy aż trafi w 360°.
6. Przywróć `ROBOT_ODOMETRY_GYRO_WEIGHT = 0.96f`.

---

## BUG 3 — pomniejszy — błędny komentarz w motor.cpp (nie powoduje błędu runtime)

W `motor.cpp` linia 56:
```cpp
// N20 AB Hall encoder: 7 PPR x 4 (quadrature edges) x 10 (gear ratio) = 280 CPR  ← BŁĄD
```

ISR w `encoder.cpp` to A-only CHANGE = **2× kwadratura**, poprawne CPR = 7×2×10 = **140**, i taka wartość jest w `robot_config.h`. Komentarz jest mylący przy debugowaniu.

Poprawna wersja:
```cpp
// N20 AB Hall encoder: 7 PPR x 2 A-edges (CHANGE ISR, A-only) x 10 (gear ratio) = 140 CPR
```

---

## Kontekst architektury

- Enkodery: software ISR GPIO (A-only, CHANGE) → 140 CPR na koło, ~0.584 mm/impuls
- Odometria: midpoint integration (RK2), fuzja enkoder+żyroskop ze stałą wagą 0.96
- Mapa: zapisuje `x_ri y_ri x_pc y_pc speed error_p error_d` (tab-separated)
- Web UI (`web_ui.h`, funkcja `drawMap()`): rysuje `parts[0]` i `parts[1]` = pozycja środka robota (nie czujnika)
- Format mapy i algorytm `MapUpdate`/`DriveOnMap` są identyczne z GRUZIK 4.0

---

## Priorytety

| # | Bug | Plik | Zmiana |
|---|-----|------|--------|
| 1 | Brak online bias correction żyro | `MUS_2.0_gruzik_port.ino` | Dodać 5 linii po `Odometry_Update()` |
| 2 | Niezskalibrowana baza kół | `robot_config.h` | Procedura empiryczna 360° |
| 3 | Błędny komentarz CPR | `motor.cpp` linia 56 | Poprawić komentarz (280→140) |
