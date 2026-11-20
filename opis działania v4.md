Arduino UNO
Sterowanie zaworem mieszacza 3-point Afriso ARM 343 czas pełnego ruchu 120s, na podstawie krzywej grzewczej, sterowanie dwoma pompami obiegowymi Afriso APH 160 oraz włączanie palnika kotła gazowego Lumea MPX 24

definicje:
punkty temperaturowe regulacji krzywej grzewczej:
tk1==-20°C, tk2==-10°C, tk3==0°C, tk4==+10°C

dane domyślne krzywej grzewczej:
dla tk1 TempZadana_20 == 40°C, 
dla tk2 TempZadana_10 == 34°C, 
dla tk3 TempZadana_0 == 30°C
dla tk4 TempZadana_plus10 ==28°C

// oblicz pozostałe punkty dla zakresu -20 +10
// algorytm PID

zmienna bTempZewn - bieżąca temperatura zewnętrzna odczytana z cTempZewn
zmienna TempZasilania - temperatura zasilania mieszacza odczytana z cTempZasilania
zmienna TempWyjscWody - temperatura na wyjściu mieszacza odczytana z cTempWyjscWody
zmienna TempPowrotu - temperatura na powrocie ogrzewania odczytana z cTempPowrotu
funkcja ZaworWlewo - zmniejszenie TempWyjscWody 
funkcja ZaworWprawo - zwiększenie TempWyjscWody 
funkcja TermostatObniza - obniżenie temperatury TempWyjscWody o wartość ObnizTemp
zmienna ObnizTemp - wartośc o którą zmienia się TermostatObniza 
funkcja pompaPod - pompa obiegowa ogrzewania podłogowego Afriso APH 160
funkcja pompaKal - pompa obiegowa ogrzewania grzejnikowego Afriso APH 160
funkcja PracaKotla  - włączenie palnika kotła 
TempZadana - żądana TempWyjscWody
zmienna HisterezaReg - histereza funkcji regulacji

Definicja klawiatury 
Przycisk_1: zwiększenie o 1 TempZadana dla wybrango punktu krzywej tk1 tk2 tk3 tk4 ObnizTemp HisterezaReg
Przycisk_2: zmniejszenie o 1 TempZadana dla wybrango punktu krzywej tk1 tk2 tk3 tk4 ObnizTemp HisterezaReg
Przycisk_3: przełączanie edycji TempZadana dla wybrango punktu krzywej tk1 tk2 tk3 tk4 ObnizTemp HisterezaReg
    wybranie danej temperatury do edycji zmienia ją na wyświetlaczu w negatyw
Przycisk_4: przełączanie pomiedzy podprogramami LATO WIOSNA ZIMA ; 
    uruchomienie podprogramu WIOSNA włącza diodę LED1
    uruchomienie podprogramu ZIMA włącza diodę LED2

Definicja wyświetlacza 2x16

Wyświetlacz górny rząd: TempZadana_20, odstęp, TempZadana_10, odstęp, TempZadana_0, odstęp, TempZadana_plus10, symbol stanu ZaworWlewo "<" lub ZaworWprawo ">" lub "0";
Wyświetlacz dolny rząd: bTempZewn (znak i 2 cyfry), odstęp, TermostatObniza (1 cyfra), odstep, TempWyjscWody (2 cyfry), odstęp, różnica TempWyjscWody-TempPowrotu (2 cyfry), odstęp, HisterezaReg (1 cyfra)
 
WEJŚCIE/WYJŚCIA
I2C czujnik DS18820 cTempZewn
I2C czujnik DS18820 cTempZasilania  
I2C czujnik DS18820 cTempWyjscWody
I2C czujnik DS18820 cTempPowrotu
I2C timer DS3231 - co 240h uruchomienie funkcji ANTYSTOP
I2C wyświetlacz LCD 2x16 
1. OUT ZaworWprawo
2. IN D2 SQW timer DS323
3. OUT ZaworWlewo
4. OUT pompaPod
5. OUT pompaKal
6. OUT PracaKotla
7. OUT buzzer 
8. IN TermostatObniza Normalnie Zamknięty
9. OUT dioda LED1 trybu WIOSNA
10. OUT dioda LED2 trybu ZIMA
11. IN przycisk_1 wybór 
12. IN przycisk_2 dodaj 
13. IN przycisk_3 odejmij 
14. IN przycisk_4 


Funkcja regulator_zaworu: 
Funkcja regulator_zaworu dopasowuje algorytmem PID temperaturę TempWyjscWody do zadanej krzywą grzewczą w zależności od bTempZewn zmniejszając ZaworWlewo a zwiększając ZaworWprawo. Pomiar temperatur i regulacja odbywa się co 5 minut, histereza układu regulacji określona zmienną HisterezaReg
Jezeli termostat TERM1 rozewrze wejście obniż TempWyjscWody o TermostatObniza
Jeżeli TempWyjscWody > 49°C: pompaPod OFF, pompaKal OFF, buzzer ON, ZaworkWlewo ON na 120 s, po 20 minutach reset procesora

funkcja ANTYSTOP:
pompaPod i pompaKal ON na 60s, zaworWprawo ON 120s, pauza 2s, zaworWlewo ON 130s, natępnie przejście procesora w tryb POWER DOWN i oczekiwanie na przerwanie timera DS3231

włączenie ZaworWlewo wyświetla symbol "<"  w ostatnim polu górnej linii wyświetlacza
włączenie ZaworWprawo wyświetla symbol ">" w ostatnim polu górnej linii wyświetlacza

// Konieczne zabezpieczenie wykluczające jednoczesne włączenie ZaworWlewo i ZaworWprawo
// Pomiar temperatur i regulacja zaworu raz na 10 minut


Podprogramy trybów pracy:
tryb LATO: regulator_zaworu OFF, PracaKotla OFF, pompaPod OFF, pompaKal OFF, ZaworkWlewo na 120 s, funkcja ANTYSTOP ON, przejście w tryb energooszczędny POWER DOWN;
tryb ZIMA: PracaKotla ON, pompaPod ON, pompaKal ON, regulator_zaworu ON, ANTYSTOP OFF;
tryb WIOSNA-JESIEŃ: PracaKotla ON, pompaPod OFF, pompaKal ON, ZaworWlewo na 120 s, regulator_zaworu OFF, ANTYSTOP OFF;
Domyślny tryb po włączeniu: LATO
