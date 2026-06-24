# GRUZIK Map Optimizer

Ten skrypt w Pythonie optymalizuje zapisaną przez robota trasę z pliku `GRUZIK.txt` do formatu gotowego do odtwarzania `map.txt`.

## Co robi ten skrypt?
1. Usuwa powtarzające się punkty odometrii.
2. Analizuje krzywiznę i promienie zakrętów.
3. Przypisuje wysoką prędkość na odcinkach prostych.
4. Przypisuje niską prędkość w ciasnych zakrętach.
5. Posiada algorytm "wyprzedzającego hamowania" (robot zwalnia odpowiednio wcześnie, ZANIM wjedzie w zakręt).

## Jak uruchomić skrypt?
Aby go użyć, skopiuj plik `GRUZIK.txt` ze swojej karty SD (lub pobierz go z robota) i umieść w tym samym folderze co skrypt. Następnie w terminalu wpisz:

```bash
python optimize_map.py
```

Wygeneruje się nowy plik `map.txt`, który musisz wrzucić na kartę SD, aby robot mógł odtwarzać optymalną ścieżkę.

### Parametry konfiguracyjne
Skrypt pozwala na dostrojenie parametrów maksymalnej i minimalnej prędkości z poziomu linii komend (nie musisz edytować kodu):

```bash
python optimize_map.py --max_speed 250 --min_speed 80 --input mojplik.txt
```

Parametry domyślne to: `--max_speed 220` oraz `--min_speed 110`.
