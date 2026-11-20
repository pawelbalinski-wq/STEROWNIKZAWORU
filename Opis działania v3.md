Arduino UNO
Sterowanie zaworem mieszacza na podstawie krzywej grzewczej, sterowanie dwoma pompami obiegowymi oraz włączanie palnika kotła gazowego.

definicje:

tk1==-20°C, tk2==-10°C, tk3==0°C, tk4==+10°C to punkty temperaturowe odniesienia krzywej grzewczej 

dane domyślne krzywej grzewczej:
dla tk1 TempZadana_20 == 40°C, 
dla tk2 TempZadana_10 == 34°C, 
dla tk3 TempZadana_0 == 30°C
dla tk4 TempZadana_plus10 ==28°C

// oblicz pozostałe punkty dla zakresu -20 +10
// algorytm PID

bTempZewn - bieżąca temperatura zewnętrzna odczytana z ctempZewn
TempZasilania - temperatura zasilania mieszacza odczytana z cTempZasilania
TempWyjscWody - temperatura na wyjściu mieszacza odczytana z cTempWyjscWody
TempPowrotu - temperatura na powrocie ogrzewania odczytana z cTempPowrotu
ZaworWlewo - zmniejszenie temperatury na wyjściu zaworu mieszającego 3-point Afriso ARM 343, czas pełnego ruchu 120s,
ZaworWprawo - zwiększenie temperatury na wyjściu zaworu mieszającego 3-point Afriso ARM 343, czas pełnego ruchu 120s,
TermostatObniza - obniżenie temperatury TempWyjscWody na sygnał termostatu pokojowego TERM1 
ObnizTempO - wartośc o którą zmienia się TermostatObniza 
pompaPod - pompa obiegowa ogrzewania podłogowego Afriso APH 160
pompaKal - pompa obiegowa ogrzewania grzejnikowego Afriso APH 160
PracaKotlaON - włączenie palnika kotła Lumea MPX24
TempZadana - żądana TempWyjscWody


Definicja klawiatury 
Przycisk_1: zwiększenie o 1 TempZadana dla wybrango punktu krzywej tk1 tk2 tk3 tk4 lub ObnizTempO 
Przycisk_2: zmniejszenie o 1 TempZadana dla wybrango punktu krzywej tk1 tk2 tk3 tk4 lub ObnizTempO
Przycisk_3: przełączanie edycji TempZadana dla wybrango punktu krzywej tk1 tk2 tk3 tk4 lub ObnizTempO  
    wybranie danej temperatury do edycji zmienia ją na wyświetlaczu w negatyw
Przycisk_4: przełączanie pomiedzy podprogramami LATO WIOSNA ZIMA ; 
    wybranie podprogramu WIOSNA włącza diodę LED1
    wybranie podprogramu ZIMA włącza diodę LED2

Definicja wyświetlacza

Wyświetlacz górny rząd: TempZadana_20, odstęp, TempZadana_10, odstęp, TempZadana_0, odstęp, TempZadana_plus10, symbol stanu ZaworWlewo "<" lub ZaworWprawo ">" lub "0";
Wyświetlacz dolny rząd: bTempZewn (znak i 2 cyfry), odstęp, TermostatObniza (1 cyfra), odstep, TempWyjscWody (dwie cyfry), różnica TempWyjscWody-TempPowrotu (2 cyfry)
 
WEJŚCIE/WYJŚCIA
I2C czujnik DS18820 ctempZewn
I2C czujnik DS18820 cTempZasilania  
I2C czujnik DS18820 cTempWyjscWody
I2C czujnik DS18820 cTempPowrotu
I2C timer DS3231 - co 240h uruchomienie funkcji ANTYSTOP
I2C wyświetlacz LCD 2x16 
1. przekaźnik siłownika ZaworWprawo
D2 SQW przerwanie sprzętowe INT0 timer DS323
2. przekaźnik siłownika ZaworWlewo
3. optotriak MOC3041 + BT136-600E dla pompaPod
4. optotriak MOC3041 + BT136-600E dla pompaKal
5. optotriak MOC3041 przekaźnika PracaKotlaON
6. buzzer sygnalizacji w razie błędu czujników temperatury, wzrostu temperatury na wyjściu powyżej max
7. wejście termostatu TERM1 
8. dioda LED1 trybu WIOSNA
9. dioda LED2 trybu ZIMA
10. przycisk_1
11. przycisk_2
12. przycisk_3
13. przycisk_4


Program pracy: 
Funkcja regulator_zaworu dopasowuje algorytmem PID temperaturę TempWyjscWody do zadanej krzywą grzewczą zmniejszając ją za pomoicąZaworkWlewo a zwiększając za pomocą ZaworWprawo. 
// Konieczne zabezpieczenie wykluczające jednoczesne włączenieZaworkWlewo i ZaworWprawo

tryb LATO: PracaKotlaON OFF, pompaPod OFF, pompaKal OFF,ZaworkWlewo na 120 s, funkcja ANTYSTOP ON, przejście w tryb energooszczędny POWER DOWN;
tryb ZIMA: PracaKotlaON ON, pompaPod ON, pompaKal ON, regulator_zaworu ON, ANTYSTOP OFF;
tryb WIOSNA-JESIEŃ: PracaKotlaON ON, pompaPod OFF, pompaKal ON,ZaworkWlewo na 120 s, regulator_zaworu OFF, ANTYSTOP OFF;

Jezeli termostat TERM1 rozewrze wejście obniż temperaturę wyjścia mieszacza TempWyjscWody o TermostatObniza

Jeżeli różnica pomiędzy wyjściem mieszacza TWM a powrotem TPP jest większa niż 7 stopni - wyświetlaj symbol: ">" a jeżeli mniejsza niż 5 stopni wyświetlaj symbol: "<" w dolnym prawym rogu wyświetlacza
włączenieZaworkWlewo wyświetla symbol "<" i włączenie ZaworWprawo wyświetla symbol ">" w ostatnim polu górnej linii wyświetlacza

funkcja ANTYSTOP: pompaPod i wPKa ON na 1 minutę, wPKPS1 ON 120s, pauza 2s, wPKPS2 ON 120s, natępnie tryb POWER DOWN;

Domyślny tryb po włączeniu: LATO
Temperatura TempWyjscWody > 49°C: pompaPod OFF, pompaKal OFF, buzzer ON,ZaworkWlewo ON na 120 s, po 20 minutach reset procesora

