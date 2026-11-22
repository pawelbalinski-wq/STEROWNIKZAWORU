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
