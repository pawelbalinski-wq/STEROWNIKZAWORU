#include <Arduino.h>                 // Podstawowe funkcje Arduino: pinMode(), digitalWrite(), millis().
#include <Wire.h>                    // Biblioteka magistrali I2C używana przez LCD i RTC.
#include <EEPROM.h>                  // Obsługa trwałego zapisu parametrów w pamięci EEPROM.
#include <avr/sleep.h>               // Obsługa trybów uśpienia mikrokontrolera AVR.
#include <OneWire.h>                 // Obsługa magistrali 1-Wire dla czujników temperatury.
#include <DallasTemperature.h>       // Biblioteka obsługująca czujniki Dallas/Maxim.
#include <LiquidCrystal_I2C.h>       // Obsługa wyświetlacza LCD podłączonego przez I2C.
#include <RTClib.h>                  // Obsługa zegara czasu rzeczywistego DS3231.

// ====================== PINY ======================
// Definicje numerów pinów oraz stałe czasu programu.
constexpr uint8_t PIN_ONEWIRE = 8;       // Pin magistrali OneWire dla czujników temperatury.
constexpr uint8_t PIN_TERMOSTAT = 9;     // Pin wejściowy termostatu.
constexpr uint8_t PIN_SQW = 2;            // Pin wyjścia kwadratu SQW z RTC dla przerwania.

constexpr uint8_t PIN_ZAWOR_PRAWO = 10; // Wyjście sterujące ruchem zaworu w prawo.
constexpr uint8_t PIN_ZAWOR_LEWO = 11;  // Wyjście sterujące ruchem zaworu w lewo.
constexpr uint8_t PIN_POMPA_POD = 12;   // Wyjście sterujące pompą podłogową.
constexpr uint8_t PIN_POMPA_KAL = A0;   // Wyjście sterujące pompą kaloryferową.
constexpr uint8_t PIN_KOCIOL = A1;       // Wyjście sterujące kotłem.
constexpr uint8_t PIN_BUZZER = A2;      // Wyjście sterujące buzzerem alarmowym.

constexpr uint8_t PIN_LED_WIOSNA = A3;  // LED informujący o trybie wiosny.
constexpr uint8_t PIN_LED_ZIMA = A4;    // LED informujący o trybie zimy.

constexpr uint8_t PIN_PRZYCISK_1 = 3;   // Przycisk zwiększania wartości.
constexpr uint8_t PIN_PRZYCISK_2 = 4;   // Przycisk zmniejszania wartości.
constexpr uint8_t PIN_PRZYCISK_3 = 5;   // Przycisk wyboru parametru edycji.
constexpr uint8_t PIN_PRZYCISK_4 = 6;   // Przycisk przełączania trybu pracy.

constexpr unsigned long CZAS_RUCHU_ZAWORU = 120000UL; // Maksymalny czas jednego ruchu zaworu.
constexpr unsigned long INTERWAL_REGULACJI = 600000UL; // Odstęp między uruchomieniami regulatora PID. 600 zekund to 10 minut. Sprawdziłem że lepiej 15-20 minut
constexpr unsigned long ODCZYT_TEMPERATURY_MS = 1000UL; // Częstotliwość pobierania temperatur. 
                                                      //Co 1 s to za często. Temperatura nie zmienia się tak szybko. Wystarczy co 20s
constexpr unsigned long ODCZYT_LCD_MS = 500UL;           // Częstotliwość odświeżania LCD.

// Enum definiuje nazwy liczbowych stanów trybu pracy.
enum Tryb {
  TRYB_LATO,  // Tryb letni z antystopem i bez sterowania kotła.
  TRYB_WIOSNA,// Tryb przejściowy.
  TRYB_ZIMA   // Tryb zimowy z regulacją zaworem.
};

// Enum identyfikuje wartość aktualnie wybraną do edycji.
enum ElementEdycji {
  EDYCJA_20,       // Edycja temperatury dla -20°C.
  EDYCJA_10,       // Edycja temperatury dla -10°C.
  EDYCJA_0,        // Edycja temperatury dla 0°C.
  EDYCJA_PLUS10,   // Edycja temperatury dla +10°C.
  EDYCJA_OBNIZ,    // Edycja obniżenia temperatury termostatem.
  EDYCJA_HISTEREZA // Edycja histerezy regulatora.
};

// ====================== OBIEKTY ======================
OneWire oneWire(PIN_ONEWIRE);          // Tworzy obiekt magistrali OneWire.
DallasTemperature sensors(&oneWire);   // Tworzy sterownik czujników temperatury.
LiquidCrystal_I2C lcd(0x27, 16, 2);    // Tworzy obiekt LCD o adresie I2C 0x27 i rozmiarze 16×2.
RTC_DS3231 rtc;                        // Tworzy obiekt komunikacji z zegarem RTC DS3231.

// ====================== DANE ======================
DeviceAddress adrZewn;      // Adres czujnika temperatury zewnętrznej.
DeviceAddress adrZasilanie; // Adres czujnika temperatury zasilania.
DeviceAddress adrWyjscie;   // Adres czujnika temperatury wyjściowej wody.
DeviceAddress adrPowrot;    // Adres czujnika temperatury powrotu.

Tryb tryb = TRYB_LATO;                 // Początkowy tryb pracy.
ElementEdycji edycja = EDYCJA_20;      // Początkowo wybiera pierwszą temperaturę krzywej.

float TempZewn = 0.0f;                 // Aktualna temperatura zewnętrzna.
float TempZasilania = 0.0f;            // Aktualna temperatura zasilania.
float TempWyjscWody = 0.0f;            // Aktualna temperatura na wyjściu wody.
float TempPowrotu = 0.0f;              // Aktualna temperatura powrotu.

float TempZadana_20 = 40.0f;           // Temperatura zadana dla -20°C.
float TempZadana_10 = 34.0f;           // Temperatura zadana dla -10°C.
float TempZadana_0 = 30.0f;            // Temperatura zadana dla 0°C.
float TempZadana_plus10 = 28.0f;       // Temperatura zadana dla +10°C.
float TempZadana = 28.0f;              // Aktualna temperatura zadana po interpolacji.
float ObnizTemp = 3.0f;                // Ilość obniżenia temperatury zadanej.
float HisterezaReg = 1.0f;             // Martwa strefa regulatora.

float pidIntegral = 0.0f;              // Suma błędów regulatora PID.
float pidPreviousError = 0.0f;         // Poprzedni błąd regulatora PID.

bool zaworWlewoAktywny = false;        // Właściwa informacja o ruchu zaworu w lewo.
bool zaworWprawoAktywny = false;       // Właściwa informacja o ruchu zaworu w prawo.
bool alarmAktywny = false;             // Informuje, czy alarm temperatury jest aktywny.
bool czujnikiOK = false;               // Informuje, czy czujniki działają poprawnie.
bool antystopUruchomiony = false;      // Informuje, czy cykl antystopu jest wykonywany.
bool stanTermostatu = false;           // Informuje o stanie termostatu.

unsigned long czasZaworu = 0;          // Moment rozpoczęcia ruchu zaworu.
unsigned long ostatniPomiar = 0;       // Czas ostatniego pomiaru temperatur.
unsigned long ostatniRegulator = 0;    // Czas ostatniego wykonania regulatora.
unsigned long ostatniLCD = 0;          // Czas ostatniej aktualizacji LCD.
unsigned long czasAlarmu = 0;          // Moment rozpoczęcia alarmu temperatury.
unsigned long czasAntystopu = 0;       // Moment rozpoczęcia bieżącego etapu antystopu.
uint8_t etapAntystopu = 0;             // Numer aktualnego etapu antystopu.
// ====================== EEPROM ======================
// Struktura opisuje dane przechowywane w pamięci EEPROM.
struct ParametryEEPROM {
  uint16_t magic;       // Znacznik poprawności danych EEPROM.
  float temp20;         // Temperatura zadana dla -20°C.
  float temp10;         // Temperatura zadana dla -10°C.
  float temp0;          // Temperatura zadana dla 0°C.
  float tempPlus10;     // Temperatura zadana dla +10°C.
  float obniz;          // Wartość obniżenia temperatury.
  float histereza;      // Histereza działania regulatora.
};

constexpr uint16_t MAGIC = 0x5A3C; // Stała identyfikująca zapisane dane EEPROM.
constexpr int EEPROM_ADRES = 0;    // Początkowy adres bloku danych w EEPROM.

// Zapisuje bieżące parametry użytkownika do EEPROM.
void zapiszEEPROM()
{
  ParametryEEPROM p; // Tworzy strukturę tymczasową.

  p.magic = MAGIC;                    // Ustawia znacznik danych.
  p.temp20 = TempZadana_20;           // Kopiuje ustawienie dla -20°C.
  p.temp10 = TempZadana_10;           // Kopiuje ustawienie dla -10°C.
  p.temp0 = TempZadana_0;             // Kopiuje ustawienie dla 0°C.
  p.tempPlus10 = TempZadana_plus10;   // Kopiuje ustawienie dla +10°C.
  p.obniz = ObnizTemp;                // Kopiuje wartość obniżenia.
  p.histereza = HisterezaReg;         // Kopiuje histerezę.

  EEPROM.put(EEPROM_ADRES, p);        // Zapisuje strukturę pod adresem 0.
}

// Odczytuje parametry EEPROM i sprawdza ich poprawność.
void odczytajEEPROM()
{
  ParametryEEPROM p;          // Tworzy strukturę do odczytu.
  EEPROM.get(EEPROM_ADRES, p); // Pobiera dane z EEPROM.

  if (p.magic != MAGIC) { // Jeśli dane nigdy nie były zapisane lub są niepoprawne.
    zapiszEEPROM();       // Zapisuje bieżące wartości domyślne.
    return;               // Kończy działanie funkcji.
  }

  TempZadana_20 = constrain(p.temp20, 20.0f, 49.0f); // Ogranicza i odczytuje temperaturę dla -20°C.
  TempZadana_10 = constrain(p.temp10, 20.0f, 49.0f); // Ogranicza i odczytuje temperaturę dla -10°C.
  TempZadana_0 = constrain(p.temp0, 20.0f, 49.0f);   // Ogranicza i odczytuje temperaturę dla 0°C.
  TempZadana_plus10 = constrain(p.tempPlus10, 20.0f, 49.0f); // Ogranicza i odczytuje dla +10°C.
  ObnizTemp = constrain(p.obniz, 0.0f, 9.0f);      // Ogranicza obniżenie temperatury.
  HisterezaReg = constrain(p.histereza, 0.5f, 5.0f); // Ogranicza histerezę.
}

// ====================== ZAWÓR ======================
// Natychmiast zatrzymuje zawór wyłączając oba kierunki.
void zatrzymajZawor()
{
  digitalWrite(PIN_ZAWOR_LEWO, LOW);  // Wyłącza silownik ruchu w lewo.
  digitalWrite(PIN_ZAWOR_PRAWO, LOW); // Wyłącza silownik ruchu w prawo.
  zaworWlewoAktywny = false;          // Aktualizuje stan ruchu w lewo.
  zaworWprawoAktywny = false;         // Aktualizuje stan ruchu w prawo.
}

// Uruchamia ruch zaworu w kierunku lewej strony.
void ZaworWlewo()
{
  if (zaworWprawoAktywny) zatrzymajZawor(); // Najpierw zatrzymuje ruch przeciwny.

  if (!zaworWlewoAktywny) { // Jeśli zawór już się nie porusza w lewo.
    digitalWrite(PIN_ZAWOR_LEWO, HIGH); // Włącza wyjście kierunku lewego.
    zaworWlewoAktywny = true;           // Zapisuje aktywny kierunek.
    czasZaworu = millis();              // Zapisuje początek czasu ruchu.
  }
}

// Uruchamia ruch zaworu w kierunku prawej strony.
void ZaworWprawo()
{
  if (zaworWlewoAktywny) zatrzymajZawor(); // Najpierw zatrzymuje ruch przeciwny.

  if (!zaworWprawoAktywny) { // Jeśli zawór aktualnie nie jedzie w prawo.
    digitalWrite(PIN_ZAWOR_PRAWO, HIGH); // Włącza wyjście kierunku prawego.
    zaworWprawoAktywny = true;           // Zapisuje aktywny kierunek.
    czasZaworu = millis();               // Zapisuje początek ruchu.
  }
}

// Zatrzymuje zawór po przekroczeniu maksymalnego czasu ruchu.
void sprawdzCzasRuchuZaworu()
{
  if ((zaworWlewoAktywny || zaworWprawoAktywny) &&
      millis() - czasZaworu >= CZAS_RUCHU_ZAWORU) {
    zatrzymajZawor(); // Zatrzymuje zawór po 120 sekundach.
  }
}

// ====================== CZUJNIKI ======================
// Inicjalizuje czujniki i przypisuje im adresy.
void inicjalizujCzujniki()
{
  sensors.begin(); // Rozpoczyna wykrywanie czujników OneWire.

  if (sensors.getDeviceCount() < 4) { // Wymaga co najmniej czterech czujników.
    czujnikiOK = false;               // Ustawia czujniki jako niedostępne.
    return;                           // Przerywa inicjalizację.
  }

  bool adresyOK = true; // Zakłada, że adresowanie będzie poprawne.

  adresyOK &= sensors.getAddress(adrZewn, 0);      // Pobiera adres pierwszego czujnika.
  adresyOK &= sensors.getAddress(adrZasilanie, 1); // Pobiera adres drugiego czujnika.
  adresyOK &= sensors.getAddress(adrWyjscie, 2);   // Pobiera adres trzeciego czujnika.
  adresyOK &= sensors.getAddress(adrPowrot, 3);    // Pobiera adres czwartego czujnika.

  if (!adresyOK) { // Jeśli pobranie któregokolwiek adresu się nie udało.
    czujnikiOK = false; // Oznacza czujniki jako niedostępne.
    return;             // Przerywa inicjalizację.
  }

  sensors.setResolution(12); // Ustawia maksymalną rozdzielczość 12-bitową.
  sensors.setWaitForConversion(false); // Konwersja nie może blokować programu.
  czujnikiOK = true; // Uznaje czujniki za poprawnie zainicjalizowane.
}

// Uruchamia pomiar i odczytuje temperatury czujników.
void odczytajCzujniki()
{
  if (!czujnikiOK) { // Nie odczytuje temperatur, gdy czujniki są błędne.
    return;
  }

  sensors.requestTemperatures(); // Wysyła polecenie rozpoczęcia pomiaru.

  float z = sensors.getTempC(adrZewn);             // Odczytuje temperaturę zewnętrzną.
  float zasilanie = sensors.getTempC(adrZasilanie);// Odczytuje temperaturę zasilania.
  float wyjscie = sensors.getTempC(adrWyjscie);    // Odczytuje temperaturę wyjściową.
  float powrot = sensors.getTempC(adrPowrot);      // Odczytuje temperaturę powrotu.

  if (z == DEVICE_DISCONNECTED_C || // Jeśli czujnik zewnętrzny nie odpowiada.
      zasilanie == DEVICE_DISCONNECTED_C || // Lub czujnik zasilania.
      wyjscie == DEVICE_DISCONNECTED_C || // Lub czujnik wyjścia.
      powrot == DEVICE_DISCONNECTED_C) { // Lub czujnik powrotu.
    czujnikiOK = false;          // Oznacza zestaw czujników jako wadliwy.
    alarmAktywny = true;         // Włącza alarm czujników.
    zatrzymajZawor();            // Zatrzymuje zawór.
    digitalWrite(PIN_BUZZER, HIGH); // Włącza buzzer.
    return;
  }

  TempZewn = z;                 // Zapisuje temperaturę zewnętrzną.
  TempZasilania = zasilanie;    // Zapisuje temperaturę zasilania.
  TempWyjscWody = wyjscie;      // Zapisuje temperaturę wyjścia.
  TempPowrotu = powrot;         // Zapisuje temperaturę powrotu.
}// ====================== KRZYWA GRZEWCZA ======================
// Oblicza liniową wartość pomiędzy dwoma punktami krzywej grzewczej.
float interpoluj(float x1, float y1, float x2, float y2, float x)
{
  if (x2 == x1) return y1; // Zabezpieczenie przed dzieleniem przez zero.

  // Wzór interpolacji liniowej pomiędzy punktami.
  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

// Wyznacza temperaturę zadaną na podstawie temperatury zewnętrznej.
float obliczTempZadana()
{
  if (TempZewn <= -20.0f) return TempZadana_20; // Poniżej -20°C używa wartości -20°C. Tutaj trzeba zwiększyć wydajność pomp a nie temperaturę podłogi.
  if (TempZewn >= 10.0f) return TempZadana_plus10; // Powyżej 10°C używa wartości +10°C.

  if (TempZewn < -10.0f) { // Przy temperaturach od -20°C do -10°C.
    return interpoluj(-20.0f, TempZadana_20,
                      -10.0f, TempZadana_10,
                      TempZewn); // Interpoluje pomiędzy tymi punktami.
  }

  if (TempZewn < 0.0f) { // Przy temperaturach od -10°C do 0°C.
    return interpoluj(-10.0f, TempZadana_10,
                       0.0f, TempZadana_0,
                       TempZewn); // Interpoluje pomiędzy tymi punktami.
  }

  // Przy temperaturach od 0°C do 10°C.
  return interpoluj(0.0f, TempZadana_0,
                    10.0f, TempZadana_plus10,
                    TempZewn); // Interpoluje pomiędzy tymi punktami.
}

// ====================== REGULATOR PID ======================
// Steruje zaworem w trybie zimowym na podstawie temperatury wyjściowej.
void regulator_zaworu()
{
  if (tryb != TRYB_ZIMA) { // Regulator działa tylko w trybie zimowym.
    zatrzymajZawor();      // Zatrzymuje zawór w pozostałych trybach.
    return;
  }

  TempZadana = obliczTempZadana(); // Oblicza aktualną temperaturę zadaną.

  stanTermostatu =
    (digitalRead(PIN_TERMOSTAT) == LOW); // Odczytuje aktywację termostat. drugi styk do masy, chyba OK, ewentualnie styki termostatu podłączyć do +5v
 
  if (stanTermostatu) { // Jeśli termostat temperatury jest aktywny. 
    TempZadana -= ObnizTemp; // Obniża temperaturę zadaną.
  }

  float blad = TempZadana - TempWyjscWody; // Oblicza różnicę temperatury wymaganej i rzeczywistej.

  pidIntegral += blad; // Dodaje bieżący błąd do składowej całkującej.
  pidIntegral = constrain(pidIntegral, -20.0f, 20.0f); // Ogranicza całkę PID.

  float pochodna = blad - pidPreviousError; // Oblicza zmianę błędu.
  pidPreviousError = blad; // Zapisuje błąd do następnego cyklu.

  float sterowanie =
      1.2f * blad +       // Składnik proporcjonalny.
      0.02f * pidIntegral + // Składnik całkujący.
      0.8f * pochodna;    // Składnik różniczkowy.

  if (fabs(sterowanie) < HisterezaReg) { // Jeśli sterowanie mieści się w martwej strefie.
    zatrzymajZawor(); // Zatrzymuje zawór.
    return;
  }

  if (sterowanie > 0.0f) { // Jeśli temperatura jest za niska.
    ZaworWprawo(); // Kieruje zawór w prawo.
  } else { // Jeśli temperatura jest za wysoka.
    ZaworWlewo(); // Kieruje zawór w lewo.
  }
}

// ====================== TRYBY PRACY ======================
// Zatrzymuje zawór i konfiguruje urządzenia dla aktualnego trybu.
void ustawTryb()
{
  zatrzymajZawor();       // Zatrzymuje zawór przed zmianą trybu.
  pidIntegral = 0.0f;     // Zeruje człon całkujący PID.
  pidPreviousError = 0.0f; // Zeruje poprzedni błąd PID.

  if (tryb == TRYB_LATO) { // Konfiguracja trybu letnego.
    digitalWrite(PIN_KOCIOL, LOW);       // Wyłącza kocioł.
    digitalWrite(PIN_POMPA_POD, LOW);   // Wyłącza pompę podłogową.
    digitalWrite(PIN_POMPA_KAL, LOW);   // Wyłącza pompę kaloryferową.
    digitalWrite(PIN_LED_WIOSNA, LOW);  // Wyłącza LED wiosny.
    digitalWrite(PIN_LED_ZIMA, LOW);    // Wyłącza LED zimy.

    ZaworWlewo();       // Uruchamia ruch zaworu w lewo.
    uruchomAntystop();  // Uruchamia cykl zabezpieczenia przed zacinaniem zaworu.
  }
  else if (tryb == TRYB_WIOSNA) { // Konfiguracja trybu wiosennego.
    digitalWrite(PIN_KOCIOL, HIGH);      // Włącza kocioł.
    digitalWrite(PIN_POMPA_POD, LOW);   // Wyłącza pompę podłogową.
    digitalWrite(PIN_POMPA_KAL, HIGH);  // Włącza pompę kaloryferową.
    digitalWrite(PIN_LED_WIOSNA, HIGH); // Włącza LED wiosny.
    digitalWrite(PIN_LED_ZIMA, LOW);    // Wyłącza LED zimy.

    ZaworWlewo(); // Uruchamia ruch zaworu w lewo.
  }
  else { // Konfiguracja trybu zimowego.
    digitalWrite(PIN_KOCIOL, HIGH);      // Włącza kocioł.
    digitalWrite(PIN_POMPA_POD, HIGH);  // Włącza pompę podłogową.
    digitalWrite(PIN_POMPA_KAL, HIGH);  // Włącza pompę kaloryferową.
    digitalWrite(PIN_LED_WIOSNA, LOW);  // Wyłącza LED wiosny.
    digitalWrite(PIN_LED_ZIMA, HIGH);   // Włącza LED zimy.
  }
}

// Bezpośrednio steruje stanem kotła.
void PracaKotla(bool stan)
{
  digitalWrite(PIN_KOCIOL, stan ? HIGH : LOW); // Włącza lub wyłącza kocioł.
}

// Bezpośrednio steruje pompą podłogową.
void pompaPod(bool stan)
{
  digitalWrite(PIN_POMPA_POD, stan ? HIGH : LOW); // Włącza lub wyłącza pompę podłogową.
}

// Bezpośrednio steruje pompą kaloryferową.
void pompaKal(bool stan)
{
  digitalWrite(PIN_POMPA_KAL, stan ? HIGH : LOW); // Włącza lub wyłącza pompę kaloryferową.
}

// ====================== ANTYSTOP ======================
// Uruchamia procedurę okresowego ruchu zaworu w trybie lato.
void uruchomAntystop()
{
  antystopUruchomiony = true; // Oznacza rozpoczęcie procedury.
  etapAntystopu = 0;          // Resetuje procedurę do pierwszego etapu.
  czasAntystopu = millis();   // Zapisuje moment rozpoczęcia procedury.

  zatrzymajZawor();             // Zatrzymuje zawór.
  digitalWrite(PIN_POMPA_POD, HIGH); // Włącza pompę podłogową.
  digitalWrite(PIN_POMPA_KAL, HIGH); // Włącza pompę kaloryferową.
  digitalWrite(PIN_KOCIOL, LOW);      // Wyłącza kocioł.
}

// Realizuje sekwencję ruchów i wyłączeń antystopu.
void procesAntystop()
{
  if (!antystopUruchomiony) return; // Nie robi nic, gdy procedura jest wyłączona.

  unsigned long uplynelo = millis() - czasAntystopu; // Oblicza czas bieżącego etapu.

  if (etapAntystopu == 0) { // Etap przygotowania.
    lcd.clear(); // Czyści LCD.
    lcd.print("URUCHOMIONY ANTYSTOP"); // Wyświetla komunikat
    digitalWrite(PIN_POMPA_POD, HIGH); // Utrzymuje pompę podłogową włączoną.
    digitalWrite(PIN_POMPA_KAL, HIGH); // Utrzymuje pompę kaloryferową włączoną.
    digitalWrite(PIN_LED_WIOSNA, HIGH);  // Włącza LED wiosny.
    digitalWrite(PIN_LED_ZIMA, HIGH);   // Włącza LED zimy.
    
    if (uplynelo >= 60000UL) { // Po 60 sekundach.
      etapAntystopu = 1;       // Przechodzi do ruchu w prawo.
      czasAntystopu = millis(); // Resetuje czas etapu.
      ZaworWprawo();           // Uruchamia zawór w prawo.
    }
  }
  else if (etapAntystopu == 1) { // Ruch zaworu w prawo.
    ZaworWprawo(); // Utrzymuje wymuszony kierunek.

    if (uplynelo >= 120000UL) { // Po 120 sekundach.
      zatrzymajZawor();         // Zatrzymuje zawór.
      etapAntystopu = 2;        // Przechodzi do przerwy.
      czasAntystopu = millis(); // Resetuje czas przerwy.
    }
  }
  else if (etapAntystopu == 2) { // Dwusekundowa przerwa.
    if (uplynelo >= 2000UL) { // Po dwóch sekundach.
      etapAntystopu = 3;       // Przechodzi do ruchu w lewo.
      czasAntystopu = millis(); // Resetuje czas etapu.
      ZaworWlewo();            // Uruchamia zawór w lewo.
    }
  }
  else if (etapAntystopu == 3) { // Ruch zaworu w lewo.
    ZaworWlewo(); // Utrzymuje kierunek ruchu.

    if (uplynelo >= 130000UL) { // Po 130 sekundach.
      zatrzymajZawor(); // Zatrzymuje zawór.
      digitalWrite(PIN_POMPA_POD, LOW); // Wyłącza pompę podłogową.
      digitalWrite(PIN_POMPA_KAL, LOW); // Wyłącza pompę kaloryferową.
      digitalWrite(PIN_LED_WIOSNA, LOW;  // Wyłącza LED wiosny.
      digitalWrite(PIN_LED_ZIMA, LOW);   // Wyłącza LED zimy.
      antystopUruchomiony = false;      // Zakańcza procedurę antystopu.

      lcd.clear(); // Czyści LCD.
      lcd.print("Tryb LATO"); // Wyświetla tryb letni.
      lcd.setCursor(0, 1); // Przechodzi do drugiego wiersza.
      lcd.print("Power Down"); // Wyświetla komunikat o uśpieniu.

      wejdzPowerDown(); // Przenosi mikrokontroler w tryb głębokiego uśpienia.
    }
  }
}// ====================== ZABEZPIECZENIE ======================
// Sprawdza błąd czujników i przegrzanie wody.
void sprawdzBezpieczenstwo()
{
  if (!czujnikiOK) { // Jeśli jeden lub więcej czujników nie działa.
    zatrzymajZawor(); // Zatrzymuje zawór.
    pompaPod(false); // Wyłącza pompę podłogową.
    pompaKal(false); // Wyłącza pompę kaloryferową.
    PracaKotla(false); // Wyłącza kocioł.
    digitalWrite(PIN_BUZZER, HIGH); // Włącza alarm dźwiękowy.
    lcd.clear(); // Czyści LCD.
    lcd.print("AWARIA CZUJNIKA TEMPERATURY"); // Wyświetla komunikat
    return;
  }

  if (TempWyjscWody > 49.0f) { // Jeśli temperatura wyjściowa przekroczyła 49°C.
    if (!alarmAktywny) { // Jeśli alarm jeszcze nie był aktywny.
      alarmAktywny = true; // Aktywuje alarm.
      czasAlarmu = millis(); // Zapisuje moment rozpoczęcia.
    }

    pompaPod(false); // Wyłącza pompę podłogową.
    pompaKal(false); // Wyłącza pompę kaloryferową.
    zatrzymajZawor(); // Zatrzymuje zawór.
    ZaworWlewo(); // Wymusza ruch zaworu w lewo.
    digitalWrite(PIN_BUZZER, HIGH); // Włącza buzzer.
    lcd.clear(); // Czyści LCD.
    lcd.print("NA WYJSCIU POWYZEJ 49 STOPNI"); // Wyświetla komunikat

    if (millis() - czasAlarmu >= 1200000UL) { // Po 20 minutach alarmu.
      void (*resetuj)(void) = 0; // Tworzy wskaźnik do adresu resetu.
      resetuj(); // Wykonuje reset mikrokontrolera poprzez adres 0.
    }
  }
  else { // Gdy temperatura jest poprawna.
    alarmAktywny = false; // Wyłącza stan alarmu.
  }
}

// ====================== WYŚWIETLACZ ======================
// Formatuje temperaturę zewnętrzną z dodatkowym znakiem.
String formatujZima(float temperatura)
{
  char bufor[8]; // Rezerwuje bufor znakowy.

  if (temperatura >= 0.0f) { // Jeśli temperatura jest dodatnia.
    snprintf(bufor, sizeof(bufor), "+%02d", (int)temperatura); // Dodaje znak plus i dwie cyfry.
  } else { // Jeśli temperatura jest ujemna.
    snprintf(bufor, sizeof(bufor), "-%02d", (int)abs(temperatura)); // Dodaje znak minus i wartość bezwzględną.
  }

  return String(bufor); // Zwraca wynik jako obiekt String.
}

// Wyświetla parametry sterownika i zaznacza pozycję edycji.
void wyswietlLCD()
{
  lcd.clear(); // Czyści zawartość LCD.

  lcd.setCursor(0, 0); // Przechodzi do początku pierwszego wiersza.
  lcd.print((int)TempZadana_20); // Wyświetla temperaturę dla -20°C.
  lcd.print(" "); // Wstawia odstęp.
  lcd.print((int)TempZadana_10); // Wyświetla temperaturę dla -10°C.
  lcd.print(" "); // Wstawia odstęp.
  lcd.print((int)TempZadana_0); // Wyświetla temperaturę dla 0°C.
  lcd.print(" "); // Wstawia odstęp.
  lcd.print((int)TempZadana_plus10); // Wyświetla temperaturę dla +10°C.

  lcd.setCursor(15, 0); // Przechodzi na ostatnią kolumnę pierwszego wiersza.

  if (zaworWlewoAktywny) lcd.print("<"); // Wskazuje ruch zaworu w lewo.
  else if (zaworWprawoAktywny) lcd.print(">"); // Wskazuje ruch zaworu w prawo.
  else lcd.print("0"); // Wskazuje zatrzymany zawór.

  lcd.setCursor(0, 1); // Przechodzi do początku drugiego wiersza.
  lcd.print(formatujZima(TempZewn)); // Wyświetla temperaturę zewnętrzną.

  lcd.setCursor(5, 1); // Ustawia kursor przy obniżeniu temperatury.
  lcd.print((int)ObnizTemp); // Wyświetla obniżenie temperatury.

  lcd.setCursor(7, 1); // Ustawia kursor przy temperaturze wyjściowej.
  lcd.print((int)TempWyjscWody); // Wyświetla temperaturę wyjścia wody.

  lcd.setCursor(10, 1); // Ustawia kursor przy różnicy temperatur.
  lcd.print((int)abs(TempWyjscWody - TempPowrotu)); // Wyświetla różnicę wyjście–powrót.

  lcd.setCursor(13, 1); // Ustawia kursor przy histerezie.
  lcd.print((int)HisterezaReg); // Wyświetla histerezę.

  if (edycja == EDYCJA_20) { // Jeśli edytowana jest temperatura -20°C.
    lcd.setCursor(0, 0); // Ustawia kursor na tej wartości.
  }

  if (edycja == EDYCJA_10) lcd.setCursor(3, 0); // Wybiera pozycję temperatury -10°C.
  if (edycja == EDYCJA_0) lcd.setCursor(6, 0); // Wybiera pozycję temperatury 0°C.
  if (edycja == EDYCJA_PLUS10) lcd.setCursor(9, 0); // Wybiera pozycję temperatury +10°C.
  if (edycja == EDYCJA_OBNIZ) lcd.setCursor(5, 1); // Wybiera pozycję obniżenia.
  if (edycja == EDYCJA_HISTEREZA) lcd.setCursor(13, 1); // Wybiera pozycję histerezy.

  lcd.cursor(); // Włącza widoczny kursor.
  lcd.blink(); // Włącza miganie kursora.
}

// ====================== Klawiatura ======================
// Sprawdza, czy przycisk jest aktualnie wciśnięty.
bool przyciskNacisniety(uint8_t pin)
{
  return digitalRead(pin) == LOW; // Przyciski są aktywne w stanie niskim.
}

// Obsługuje wykrywanie zbocza naciśnięcia przycisku.
bool zboczePrzycisku(uint8_t pin)
{
  static bool stanPoprzedni[16]; // Tablica przechowuje poprzedni stan dla każdego pinu.
  bool aktualny = przyciskNacisniety(pin); // Odczytuje aktualny stan przycisku.

  if (aktualny != stanPoprzedni[pin]) { // Porównuje stan z poprzednim.
    stanPoprzedni[pin] = aktualny; // Aktualizuje zapamiętany stan.
    if (aktualny) return true; // Zgłasza naciśnięcie tylko przy przejściu do LOW.
  }

  return false; // Brak nowego naciśnięcia.
}

// Ogranicza wszystkie parametry do dopuszczalnych zakresów.
void ograniczParametry()
{
  TempZadana_20 = constrain(TempZadana_20, 20.0f, 49.0f); // Ogranicza temperaturę dla -20°C.
  TempZadana_10 = constrain(TempZadana_10, 20.0f, 49.0f); // Ogranicza temperaturę dla -10°C.
  TempZadana_0 = constrain(TempZadana_0, 20.0f, 49.0f); // Ogranicza temperaturę dla 0°C.
  TempZadana_plus10 = constrain(TempZadana_plus10, 20.0f, 49.0f); // Ogranicza temperaturę dla +10°C.
  ObnizTemp = constrain(ObnizTemp, 0.0f, 9.0f); // Ogranicza obniżenie.
  HisterezaReg = constrain(HisterezaReg, 0.5f, 5.0f); // Ogranicza histerezę.
}

// Zwiększa wartość aktualnie edytowanego parametru.
void dodaj1()
{
  switch (edycja) { // Sprawdza, który parametr jest edytowany.
    case EDYCJA_20: TempZadana_20 += 1; break; // Zwiększa temperaturę dla -20°C.
    case EDYCJA_10: TempZadana_10 += 1; break; // Zwiększa temperaturę dla -10°C.
    case EDYCJA_0: TempZadana_0 += 1; break; // Zwiększa temperaturę dla 0°C.
    case EDYCJA_PLUS10: TempZadana_plus10 += 1; break; // Zwiększa temperaturę dla +10°C.
    case EDYCJA_OBNIZ: ObnizTemp += 1; break; // Zwiększa obniżenie.
    case EDYCJA_HISTEREZA: HisterezaReg += 1; break; // Zwiększa histerezę.
  }

  ograniczParametry(); // Sprawdza dopuszczalne zakresy.
  zapiszEEPROM(); // Zapisuje nową wartość do EEPROM.
}

// Zmniejsza wartość aktualnie edytowanego parametru.
void odejmij1()
{
  switch (edycja) { // Sprawdza aktualny parametr.
    case EDYCJA_20: TempZadana_20 -= 1; break; // Zmniejsza temperaturę dla -20°C.
    case EDYCJA_10: TempZadana_10 -= 1; break; // Zmniejsza temperaturę dla -10°C.
    case EDYCJA_0: TempZadana_0 -= 1; break; // Zmniejsza temperaturę dla 0°C.
    case EDYCJA_PLUS10: TempZadana_plus10 -= 1; break; // Zmniejsza temperaturę dla +10°C.
    case EDYCJA_OBNIZ: ObnizTemp -= 1; break; // Zmniejsza obniżenie.
    case EDYCJA_HISTEREZA: HisterezaReg -= 1; break; // Zmniejsza histerezę.
  }

  ograniczParametry(); // Sprawdza zakresy.
  zapiszEEPROM(); // Zapisuje zmieniony parametr.
}

// Odczytuje przyciski co 30 ms i wykonuje przypisane czynności.
void odczytajPrzyciski()
{
  static unsigned long ostatniOdczyt = 0; // Czas ostatniego odczytu przycisków.

  if (millis() - ostatniOdczyt < 30) return; // Ogranicza częstotliwość odczytu do 30 ms.
  ostatniOdczyt = millis(); // Zapisuje czas bieżącego odczytu.

  if (zboczePrzycisku(PIN_PRZYCISK_1)) { // Jeśli naciśnięto przycisk 1.
    dodaj1(); // Zwiększa bieżącą wartość.
  }

  if (zboczePrzycisku(PIN_PRZYCISK_2)) { // Jeśli naciśnięto przycisk 2.
    odejmij1(); // Zmniejsza bieżącą wartość.
  }

  if (zboczePrzycisku(PIN_PRZYCISK_3)) { // Jeśli naciśnięto przycisk 3.
    edycja = static_cast<ElementEdycji>( // Przechodzi do kolejnego elementu edycji.
      (edycja + 1) % 6 // Po szóstym elemencie wraca do pierwszego.
    );
  }

  if (zboczePrzycisku(PIN_PRZYCISK_4)) { // Jeśli naciśnięto przycisk 4.
    if (tryb == TRYB_LATO) tryb = TRYB_WIOSNA; // Zmienia lato na wiosnę.
    else if (tryb == TRYB_WIOSNA) tryb = TRYB_ZIMA; // Zmienia wiosnę na zimę.
    else tryb = TRYB_LATO; // Zmienia zimę na lato.

    ustawTryb(); // Natychmiast konfiguruje nowe wyjścia i urządzenia.
  }
}// ====================== RTC / POWER DOWN ======================
// Pusta funkcja obsługi przerwania SQW.
void przerwanieSQW()
{
  // Powrót z trybu uśpienia następuje dzięki przerwaniu; obsługa nie wymaga dodatkowych operacji.
}

// Włącza tryb najgłębszego uśpienia mikrokontrolera.
void wejdzPowerDown()
{
  pinMode(PIN_SQW, INPUT); // Konfiguruje pin SQW jako wejście.

  attachInterrupt(digitalPinToInterrupt(PIN_SQW), // Rejestruje przerwanie dla pinu D2.
                  przerwanieSQW, // Wskazuje funkcję obsługi przerwania.
                  FALLING); // Aktywuje przerwanie na opadającym zboczu.

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Wybiera tryb uśpienia Power Down.
  sleep_enable(); // Włącza możliwość wejścia w uśpienie.
  sleep_cpu(); // Zatrzymuje CPU i czeka na przerwanie.
  sleep_disable(); // Wyłącza mechanizm uśpienia po obudzeniu.
  detachInterrupt(digitalPinToInterrupt(PIN_SQW)); // Usuwa obsługę przerwania po obudzeniu.
}

// Wykonuje początkową obsługę RTC i przechowuje jego stan.
void sprawdzAlarmSQW()
{
  static uint32_t stanSQW = 0; // Przechowuje stan czasu po pierwszym wykonaniu.

  if (stanSQW == 0) { // Jeśli czas jeszcze nie został pobrany.
    DateTime teraz = rtc.now(); // Pobiera aktualny czas z RTC.
    stanSQW = (uint32_t)teraz.unixtime(); // Zapisuje czas jako liczbę sekund.
  }

  if (rtc.begin()) { // Sprawdza, czy biblioteka RTC jest dostępna.
    // SQW obsługiwane jest przez przerwanie z pinu D2.
  }
}

// ====================== SETUP ======================
// Konfiguruje piny i uruchamia komponenty po resecie.
void setup()
{
  pinMode(PIN_ZAWOR_PRAWO, OUTPUT); // Konfiguruje wyjście zaworu w prawo.
  pinMode(PIN_ZAWOR_LEWO, OUTPUT); // Konfiguruje wyjście zaworu w lewo.
  pinMode(PIN_POMPA_POD, OUTPUT); // Konfiguruje wyjście pompy podłogowej.
  pinMode(PIN_POMPA_KAL, OUTPUT); // Konfiguruje wyjście pompy kaloryferowej.
  pinMode(PIN_KOCIOL, OUTPUT); // Konfiguruje wyjście kotła.
  pinMode(PIN_BUZZER, OUTPUT); // Konfiguruje wyjście buzzera.
  pinMode(PIN_LED_WIOSNA, OUTPUT); // Konfiguruje LED wiosny.
  pinMode(PIN_LED_ZIMA, OUTPUT); // Konfiguruje LED zimy.

  pinMode(PIN_TERMOSTAT, INPUT_PULLUP); // Konfiguruje termostat z wewnętrznym rezystorem podciągającym.
  pinMode(PIN_SQW, INPUT); // Konfiguruje wejście sygnału SQW.

  pinMode(PIN_PRZYCISK_1, INPUT_PULLUP); // Konfiguruje przycisk 1 z podciągnięciem.
  pinMode(PIN_PRZYCISK_2, INPUT_PULLUP); // Konfiguruje przycisk 2 z podciągnięciem.
  pinMode(PIN_PRZYCISK_3, INPUT_PULLUP); // Konfiguruje przycisk 3 z podciągnięciem.
  pinMode(PIN_PRZYCISK_4, INPUT_PULLUP); // Konfiguruje przycisk 4 z podciągnięciem.

  zatrzymajZawor(); // Wyłącza oba kierunki zaworu.
  digitalWrite(PIN_POMPA_POD, LOW); // Wyłącza pompę podłogową.
  digitalWrite(PIN_POMPA_KAL, LOW); // Wyłącza pompę kaloryferową.
  digitalWrite(PIN_KOCIOL, LOW); // Wyłącza kocioł.
  digitalWrite(PIN_BUZZER, LOW); // Wyłącza buzzer.
  digitalWrite(PIN_LED_WIOSNA, LOW); // Wyłącza LED wiosny.
  digitalWrite(PIN_LED_ZIMA, LOW); // Wyłącza LED zimy.

  Wire.begin(); // Uruchamia magistralę I2C.

  lcd.init(); // Inicjalizuje wyświetlacz LCD.
  lcd.backlight(); // Włącza podświetlenie LCD.
  lcd.clear(); // Czyści LCD.
  lcd.print("Sterownik zaworu"); // Wyświetla nazwę programu.

  rtc.begin(); // Inicjalizuje zegar RTC.
  odczytajEEPROM(); // Odtwarza zapisane parametry.
  inicjalizujCzujniki(); // Wykrywa i inicjalizuje czujniki temperatur.

  tryb = TRYB_LATO; // Ustawia domyślny tryb letni.
  ustawTryb(); // Konfiguruje wyjścia dla trybu letniego.

  lcd.clear(); // Czyści poprzedni napis.
  lcd.print("Tryb LATO"); // Wyświetla informację o trybie letnim.
  delay(1000); // Wyświetla komunikat przez 1 sekundę.
}

// ====================== LOOP ======================
// Nieskończona pętla programu wykonywana po setup().
void loop()
{
  unsigned long teraz = millis(); // Pobiera bieżący czas pracy mikrokontrolera.

  odczytajPrzyciski(); // Sprawdza naciśnięcia przycisków.

  if (teraz - ostatniPomiar >= ODCZYT_TEMPERATURY_MS) { // Czy minęła sekunda od ostatniego pomiaru.
    ostatniPomiar = teraz; // Aktualizuje czas ostatniego pomiaru.
    odczytajCzujniki(); // Odczytuje wszystkie temperatury.
  }

  if (teraz - ostatniRegulator >= INTERWAL_REGULACJI) { // Czy minęło 10 minut od ostatniej regulacji.
    ostatniRegulator = teraz; // Aktualizuje czas regulatora.
    regulator_zaworu(); // Wyznacza kierunek i stan zaworu.
  }

  sprawdzCzasRuchuZaworu(); // Sprawdza, czy zawór nie porusza się zbyt długo.
  sprawdzBezpieczenstwo(); // Wykonuje kontrolę czujników i temperatury.
  procesAntystop(); // Obsługuje procedurę antystopu.

  if (teraz - ostatniLCD >= ODCZYT_LCD_MS) { // Czy minęło 500 ms od ostatniego odświeżenia LCD.
    ostatniLCD = teraz; // Aktualizuje czas ostatniego odświeżenia.
    wyswietlLCD(); // Aktualizuje zawartość wyświetlacza.
  }
}
