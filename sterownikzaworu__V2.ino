#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>

// ====================== PINY ======================
constexpr uint8_t PIN_ONEWIRE = 8;
constexpr uint8_t PIN_TERMOSTAT = 9;
constexpr uint8_t PIN_SQW = 2;

constexpr uint8_t PIN_ZAWOR_PRAWO = 10;
constexpr uint8_t PIN_ZAWOR_LEWO = 11;
constexpr uint8_t PIN_POMPA_POD = 12;
constexpr uint8_t PIN_POMPA_KAL = A0;
constexpr uint8_t PIN_KOCIOL = A1;
constexpr uint8_t PIN_BUZZER = A2;

constexpr uint8_t PIN_LED_WIOSNA = A3;
constexpr uint8_t PIN_LED_ZIMA = A4;

constexpr uint8_t PIN_PRZYCISK_1 = 3;
constexpr uint8_t PIN_PRZYCISK_2 = 4;
constexpr uint8_t PIN_PRZYCISK_3 = 5;
constexpr uint8_t PIN_PRZYCISK_4 = 6;

constexpr unsigned long CZAS_RUCHU_ZAWORU = 120000UL;
constexpr unsigned long INTERWAL_REGULACJI = 600000UL;
constexpr unsigned long ODCZYT_TEMPERATURY_MS = 20000UL;
constexpr unsigned long ODCZYT_LCD_MS = 500UL;
constexpr unsigned long CZAS_KONWERSJI_TEMP = 750UL;

// ====================== ENUMY ======================
enum Tryb {
  TRYB_LATO,
  TRYB_WIOSNA,
  TRYB_ZIMA
};

enum ElementEdycji {
  EDYCJA_20,
  EDYCJA_10,
  EDYCJA_0,
  EDYCJA_PLUS10,
  EDYCJA_OBNIZ,
  EDYCJA_HISTEREZA
};

// ====================== OBIEKTY ======================
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;

// ====================== DANE ======================
DeviceAddress adrZewn;
DeviceAddress adrZasilanie;
DeviceAddress adrWyjscie;
DeviceAddress adrPowrot;

Tryb tryb = TRYB_LATO;
ElementEdycji edycja = EDYCJA_20;

float TempZewn = 0.0f;
float TempZasilania = 0.0f;
float TempWyjscWody = 0.0f;
float TempPowrotu = 0.0f;

float TempZadana_20 = 40.0f;
float TempZadana_10 = 34.0f;
float TempZadana_0 = 30.0f;
float TempZadana_plus10 = 28.0f;
float TempZadana = 28.0f;
float ObnizTemp = 3.0f;
float HisterezaReg = 1.0f;

float pidIntegral = 0.0f;
float pidPreviousError = 0.0f;

bool zaworWlewoAktywny = false;
bool zaworWprawoAktywny = false;
bool alarmAktywny = false;
bool czujnikiOK = false;
bool antystopUruchomiony = false;
bool stanTermostatu = false;

unsigned long czasZaworu = 0;
unsigned long ostatniPomiar = 0;
unsigned long ostatniRegulator = 0;
unsigned long ostatniLCD = 0;
unsigned long czasAlarmu = 0;
unsigned long czasAntystopu = 0;
uint8_t etapAntystopu = 0;

unsigned long czasKonwersjiTemperatur = 0;
bool konwersjaTemperaturAktywna = false;
unsigned long ostatniaReinicjalizacjaCzujnikow = 0;

// ====================== EEPROM ======================
struct ParametryEEPROM {
  uint16_t magic;
  float temp20;
  float temp10;
  float temp0;
  float tempPlus10;
  float obniz;
  float histereza;
};

constexpr uint16_t MAGIC = 0x5A3C;
constexpr int EEPROM_ADRES = 0;

void zapiszEEPROM()
{
  ParametryEEPROM p;
  p.magic = MAGIC;
  p.temp20 = TempZadana_20;
  p.temp10 = TempZadana_10;
  p.temp0 = TempZadana_0;
  p.tempPlus10 = TempZadana_plus10;
  p.obniz = ObnizTemp;
  p.histereza = HisterezaReg;
  EEPROM.put(EEPROM_ADRES, p);
}

void odczytajEEPROM()
{
  ParametryEEPROM p;
  EEPROM.get(EEPROM_ADRES, p);

  if (p.magic != MAGIC) {
    zapiszEEPROM();
    return;
  }

  TempZadana_20 = constrain(p.temp20, 20.0f, 49.0f);
  TempZadana_10 = constrain(p.temp10, 20.0f, 49.0f);
  TempZadana_0 = constrain(p.temp0, 20.0f, 49.0f);
  TempZadana_plus10 = constrain(p.tempPlus10, 20.0f, 49.0f);
  ObnizTemp = constrain(p.obniz, 0.0f, 9.0f);
  HisterezaReg = constrain(p.histereza, 0.5f, 5.0f);
}

// ====================== ZAWÓR ======================
void zatrzymajZawor()
{
  digitalWrite(PIN_ZAWOR_LEWO, LOW);
  digitalWrite(PIN_ZAWOR_PRAWO, LOW);
  zaworWlewoAktywny = false;
  zaworWprawoAktywny = false;
}

void ZaworWlewo()
{
  if (zaworWprawoAktywny) zatrzymajZawor();

  if (!zaworWlewoAktywny) {
    digitalWrite(PIN_ZAWOR_LEWO, HIGH);
    zaworWlewoAktywny = true;
    czasZaworu = millis();
  }
}

void ZaworWprawo()
{
  if (zaworWlewoAktywny) zatrzymajZawor();

  if (!zaworWprawoAktywny) {
    digitalWrite(PIN_ZAWOR_PRAWO, HIGH);
    zaworWprawoAktywny = true;
    czasZaworu = millis();
  }
}

void sprawdzCzasRuchuZaworu()
{
  if ((zaworWlewoAktywny || zaworWprawoAktywny) &&
      millis() - czasZaworu >= CZAS_RUCHU_ZAWORU) {
    zatrzymajZawor();
  }
}

// ====================== CZUJNIKI ======================
void inicjalizujCzujniki()
{
  sensors.begin();
  konwersjaTemperaturAktywna = false;

  if (sensors.getDeviceCount() < 4) {
    czujnikiOK = false;
    return;
  }

  bool adresyOK = true;
  adresyOK &= sensors.getAddress(adrZewn, 0);
  adresyOK &= sensors.getAddress(adrZasilanie, 1);
  adresyOK &= sensors.getAddress(adrWyjscie, 2);
  adresyOK &= sensors.getAddress(adrPowrot, 3);

  if (!adresyOK) {
    czujnikiOK = false;
    return;
  }

  sensors.setResolution(12);
  sensors.setWaitForConversion(false);
  czujnikiOK = true;
  alarmAktywny = false;
  digitalWrite(PIN_BUZZER, LOW);
}

void rozpocznijPomiarCzujnikow()
{
  if (!czujnikiOK || konwersjaTemperaturAktywna) return;

  sensors.requestTemperatures();
  czasKonwersjiTemperatur = millis();
  konwersjaTemperaturAktywna = true;
}

const char* nazwaAwariiCzujnika()
{
  if (sensors.getTempC(adrZewn) == DEVICE_DISCONNECTED_C) {
    return "AWARIA: CZUJNIK ZEWN";
  }
  if (sensors.getTempC(adrZasilanie) == DEVICE_DISCONNECTED_C) {
    return "AWARIA: CZUJNIK ZASIL";
  }
  if (sensors.getTempC(adrWyjscie) == DEVICE_DISCONNECTED_C) {
    return "AWARIA: CZUJNIK WYJSCIE";
  }
  if (sensors.getTempC(adrPowrot) == DEVICE_DISCONNECTED_C) {
    return "AWARIA: CZUJNIK POWROT";
  }
  return "AWARIA CZUJNIKA";
}

void zakonczPomiarCzujnikow()
{
  if (!konwersjaTemperaturAktywna) return;
  if (millis() - czasKonwersjiTemperatur < CZAS_KONWERSJI_TEMP) return;

  konwersjaTemperaturAktywna = false;

  float z = sensors.getTempC(adrZewn);
  float zasilanie = sensors.getTempC(adrZasilanie);
  float wyjscie = sensors.getTempC(adrWyjscie);
  float powrot = sensors.getTempC(adrPowrot);

  if (z == DEVICE_DISCONNECTED_C ||
      zasilanie == DEVICE_DISCONNECTED_C ||
      wyjscie == DEVICE_DISCONNECTED_C ||
      powrot == DEVICE_DISCONNECTED_C) {
    czujnikiOK = false;
    alarmAktywny = true;

    zatrzymajZawor();
    digitalWrite(PIN_POMPA_POD, LOW);
    digitalWrite(PIN_POMPA_KAL, LOW);
    digitalWrite(PIN_KOCIOL, LOW);
    digitalWrite(PIN_BUZZER, HIGH);

    lcd.clear();
    lcd.print(nazwaAwariiCzujnika());
    return;
  }

  TempZewn = z;
  TempZasilania = zasilanie;
  TempWyjscWody = wyjscie;
  TempPowrotu = powrot;

  if (alarmAktywny) {
    alarmAktywny = false;
    digitalWrite(PIN_BUZZER, LOW);
  }
}

// ====================== KRZYWA GRZEWCZA ======================
float interpoluj(float x1, float y1, float x2, float y2, float x)
{
  if (x2 == x1) return y1;
  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

float obliczTempZadana()
{
  if (TempZewn <= -20.0f) return TempZadana_20;
  if (TempZewn >= 10.0f) return TempZadana_plus10;

  if (TempZewn < -10.0f) {
    return interpoluj(-20.0f, TempZadana_20,
                      -10.0f, TempZadana_10,
                      TempZewn);
  }

  if (TempZewn < 0.0f) {
    return interpoluj(-10.0f, TempZadana_10,
                       0.0f, TempZadana_0,
                       TempZewn);
  }

  return interpoluj(0.0f, TempZadana_0,
                    10.0f, TempZadana_plus10,
                    TempZewn);
}

// ====================== REGULATOR PID ======================
void regulator_zaworu()
{
  if (tryb != TRYB_ZIMA) {
    zatrzymajZawor();
    return;
  }

  TempZadana = obliczTempZadana();
  stanTermostatu = (digitalRead(PIN_TERMOSTAT) == LOW);

  if (stanTermostatu) {
    TempZadana -= ObnizTemp;
  }

  float blad = TempZadana - TempWyjscWody;
  pidIntegral += blad;
  pidIntegral = constrain(pidIntegral, -20.0f, 20.0f);

  float pochodna = blad - pidPreviousError;
  pidPreviousError = blad;

  float sterowanie =
      1.2f * blad +
      0.02f * pidIntegral +
      0.8f * pochodna;

  if (fabs(sterowanie) < HisterezaReg) {
    zatrzymajZawor();
    return;
  }

  if (sterowanie > 0.0f) ZaworWprawo();
  else ZaworWlewo();
}

// ====================== TRYBY PRACY ======================
void PracaKotla(bool stan)
{
  digitalWrite(PIN_KOCIOL, stan ? HIGH : LOW);
}

void pompaPod(bool stan)
{
  digitalWrite(PIN_POMPA_POD, stan ? HIGH : LOW);
}

void pompaKal(bool stan)
{
  digitalWrite(PIN_POMPA_KAL, stan ? HIGH : LOW);
}

void uruchomAntystop();

void ustawTryb()
{
  zatrzymajZawor();
  pidIntegral = 0.0f;
  pidPreviousError = 0.0f;

  if (tryb == TRYB_LATO) {
    digitalWrite(PIN_KOCIOL, LOW);
    digitalWrite(PIN_POMPA_POD, LOW);
    digitalWrite(PIN_POMPA_KAL, LOW);
    digitalWrite(PIN_LED_WIOSNA, LOW);
    digitalWrite(PIN_LED_ZIMA, LOW);

    ZaworWlewo();
    uruchomAntystop();
  }
  else if (tryb == TRYB_WIOSNA) {
    digitalWrite(PIN_KOCIOL, HIGH);
    digitalWrite(PIN_POMPA_POD, LOW);
    digitalWrite(PIN_POMPA_KAL, HIGH);
    digitalWrite(PIN_LED_WIOSNA, HIGH);
    digitalWrite(PIN_LED_ZIMA, LOW);
    ZaworWlewo();
  }
  else {
    digitalWrite(PIN_KOCIOL, HIGH);
    digitalWrite(PIN_POMPA_POD, HIGH);
    digitalWrite(PIN_POMPA_KAL, HIGH);
    digitalWrite(PIN_LED_WIOSNA, LOW);
    digitalWrite(PIN_LED_ZIMA, HIGH);
  }
}

// ====================== ANTYSTOP ======================
void uruchomAntystop()
{
  antystopUruchomiony = true;
  etapAntystopu = 0;
  czasAntystopu = millis();

  zatrzymajZawor();
  digitalWrite(PIN_POMPA_POD, HIGH);
  digitalWrite(PIN_POMPA_KAL, HIGH);
  digitalWrite(PIN_KOCIOL, LOW);
}

void wejdzPowerDown();

void procesAntystop()
{
  if (!antystopUruchomiony) return;

  unsigned long uplynelo = millis() - czasAntystopu;

  if (etapAntystopu == 0) {
    lcd.clear();
    lcd.print("URUCHOMIONY ANTYSTOP");
    digitalWrite(PIN_POMPA_POD, HIGH);
    digitalWrite(PIN_POMPA_KAL, HIGH);
    digitalWrite(PIN_LED_WIOSNA, HIGH);
    digitalWrite(PIN_LED_ZIMA, HIGH);

    if (uplynelo >= 60000UL) {
      etapAntystopu = 1;
      czasAntystopu = millis();
      ZaworWprawo();
    }
  }
  else if (etapAntystopu == 1) {
    ZaworWprawo();

    if (uplynelo >= 120000UL) {
      zatrzymajZawor();
      etapAntystopu = 2;
      czasAntystopu = millis();
    }
  }
  else if (etapAntystopu == 2) {
    if (uplynelo >= 2000UL) {
      etapAntystopu = 3;
      czasAntystopu = millis();
      ZaworWlewo();
    }
  }
  else if (etapAntystopu == 3) {
    ZaworWlewo();

    if (uplynelo >= 130000UL) {
      zatrzymajZawor();
      digitalWrite(PIN_POMPA_POD, LOW);
      digitalWrite(PIN_POMPA_KAL, LOW);
      digitalWrite(PIN_LED_WIOSNA, LOW);
      digitalWrite(PIN_LED_ZIMA, LOW);
      antystopUruchomiony = false;

      lcd.clear();
      lcd.print("Tryb LATO");
      lcd.setCursor(0, 1);
      lcd.print("Power Down");
      wejdzPowerDown();
    }
  }
}

// ====================== ZABEZPIECZENIE ======================
void sprawdzBezpieczenstwo()
{
  if (!czujnikiOK) {
    zatrzymajZawor();
    pompaPod(false);
    pompaKal(false);
    PracaKotla(false);
    digitalWrite(PIN_BUZZER, HIGH);

    lcd.clear();
    lcd.print(nazwaAwariiCzujnika());
    return;
  }

  if (TempWyjscWody > 49.0f) {
    if (!alarmAktywny) {
      alarmAktywny = true;
      czasAlarmu = millis();
    }

    pompaPod(false);
    pompaKal(false);
    zatrzymajZawor();
    ZaworWlewo();
    digitalWrite(PIN_BUZZER, HIGH);

    lcd.clear();
    lcd.print("NA WYJSCIU >49 C");

    if (millis() - czasAlarmu >= 1200000UL) {
      void (*resetuj)(void) = 0;
      resetuj();
    }
  }
  else {
    alarmAktywny = false;
  }
}

// ====================== WYŚWIETLACZ ======================
String formatujZima(float temperatura)
{
  char bufor[8];

  if (temperatura >= 0.0f) {
    snprintf(bufor, sizeof(bufor), "+%02d", (int)temperatura);
  }
  else {
    snprintf(bufor, sizeof(bufor), "-%02d", (int)abs(temperatura));
  }

  return String(bufor);
}

void wyswietlLCD()
{
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print((int)TempZadana_20);
  lcd.print(" ");
  lcd.print((int)TempZadana_10);
  lcd.print(" ");
  lcd.print((int)TempZadana_0);
  lcd.print(" ");
  lcd.print((int)TempZadana_plus10);

  lcd.setCursor(15, 0);
  if (zaworWlewoAktywny) lcd.print("<");
  else if (zaworWprawoAktywny) lcd.print(">");
  else lcd.print("0");

  lcd.setCursor(0, 1);
  lcd.print(formatujZima(TempZewn));

  lcd.setCursor(5, 1);
  lcd.print((int)ObnizTemp);

  lcd.setCursor(7, 1);
  lcd.print((int)TempWyjscWody);

  lcd.setCursor(10, 1);
  lcd.print((int)abs(TempWyjscWody - TempPowrotu));

  lcd.setCursor(13, 1);
  lcd.print((int)HisterezaReg);

  if (edycja == EDYCJA_20) lcd.setCursor(0, 0);
  if (edycja == EDYCJA_10) lcd.setCursor(3, 0);
  if (edycja == EDYCJA_0) lcd.setCursor(6, 0);
  if (edycja == EDYCJA_PLUS10) lcd.setCursor(9, 0);
  if (edycja == EDYCJA_OBNIZ) lcd.setCursor(5, 1);
  if (edycja == EDYCJA_HISTEREZA) lcd.setCursor(13, 1);

  lcd.cursor();
  lcd.blink();
}

// ====================== KLawiatura ======================
bool przyciskNacisniety(uint8_t pin)
{
  return digitalRead(pin) == LOW;
}

bool zboczePrzycisku(uint8_t pin)
{
  static bool stanPoprzedni[16];
  bool aktualny = przyciskNacisniety(pin);

  if (aktualny != stanPoprzedni[pin]) {
    stanPoprzedni[pin] = aktualny;
    if (aktualny) return true;
  }

  return false;
}

void ograniczParametry()
{
  TempZadana_20 = constrain(TempZadana_20, 20.0f, 49.0f);
  TempZadana_10 = constrain(TempZadana_10, 20.0f, 49.0f);
  TempZadana_0 = constrain(TempZadana_0, 20.0f, 49.0f);
  TempZadana_plus10 = constrain(TempZadana_plus10, 20.0f, 49.0f);
  ObnizTemp = constrain(ObnizTemp, 0.0f, 9.0f);
  HisterezaReg = constrain(HisterezaReg, 0.5f, 5.0f);
}

void dodaj1()
{
  switch (edycja) {
    case EDYCJA_20: TempZadana_20 += 1; break;
    case EDYCJA_10: TempZadana_10 += 1; break;
    case EDYCJA_0: TempZadana_0 += 1; break;
    case EDYCJA_PLUS10: TempZadana_plus10 += 1; break;
    case EDYCJA_OBNIZ: ObnizTemp += 1; break;
    case EDYCJA_HISTEREZA: HisterezaReg += 1; break;
  }

  ograniczParametry();
  zapiszEEPROM();
}

void odejmij1()
{
  switch (edycja) {
    case EDYCJA_20: TempZadana_20 -= 1; break;
    case EDYCJA_10: TempZadana_10 -= 1; break;
    case EDYCJA_0: TempZadana_0 -= 1; break;
    case EDYCJA_PLUS10: TempZadana_plus10 -= 1; break;
    case EDYCJA_OBNIZ: ObnizTemp -= 1; break;
    case EDYCJA_HISTEREZA: HisterezaReg -= 1; break;
  }

  ograniczParametry();
  zapiszEEPROM();
}

void odczytajPrzyciski()
{
  static unsigned long ostatniOdczyt = 0;

  if (millis() - ostatniOdczyt < 30) return;
  ostatniOdczyt = millis();

  if (zboczePrzycisku(PIN_PRZYCISK_1)) dodaj1();
  if (zboczePrzycisku(PIN_PRZYCISK_2)) odejmij1();

  if (zboczePrzycisku(PIN_PRZYCISK_3)) {
    edycja = static_cast<ElementEdycji>((edycja + 1) % 6);
  }

  if (zboczePrzycisku(PIN_PRZYCISK_4)) {
    if (tryb == TRYB_LATO) tryb = TRYB_WIOSNA;
    else if (tryb == TRYB_WIOSNA) tryb = TRYB_ZIMA;
    else tryb = TRYB_LATO;

    ustawTryb();
  }
}

// ====================== RTC / POWER DOWN ======================
void przerwanieSQW()
{
}

void wejdzPowerDown()
{
  pinMode(PIN_SQW, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_SQW), przerwanieSQW, FALLING);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(PIN_SQW));
}

void sprawdzAlarmSQW()
{
  static uint32_t stanSQW = 0;

  if (stanSQW == 0) {
    DateTime teraz = rtc.now();
    stanSQW = (uint32_t)teraz.unixtime();
  }

  if (rtc.begin()) {
  }
}

// ====================== SETUP ======================
void setup()
{
  pinMode(PIN_ZAWOR_PRAWO, OUTPUT);
  pinMode(PIN_ZAWOR_LEWO, OUTPUT);
  pinMode(PIN_POMPA_POD, OUTPUT);
  pinMode(PIN_POMPA_KAL, OUTPUT);
  pinMode(PIN_KOCIOL, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_WIOSNA, OUTPUT);
  pinMode(PIN_LED_ZIMA, OUTPUT);

  pinMode(PIN_TERMOSTAT, INPUT_PULLUP);
  pinMode(PIN_SQW, INPUT);

  pinMode(PIN_PRZYCISK_1, INPUT_PULLUP);
  pinMode(PIN_PRZYCISK_2, INPUT_PULLUP);
  pinMode(PIN_PRZYCISK_3, INPUT_PULLUP);
  pinMode(PIN_PRZYCISK_4, INPUT_PULLUP);

  zatrzymajZawor();
  digitalWrite(PIN_POMPA_POD, LOW);
  digitalWrite(PIN_POMPA_KAL, LOW);
  digitalWrite(PIN_KOCIOL, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED_WIOSNA, LOW);
  digitalWrite(PIN_LED_ZIMA, LOW);

  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Sterownik zaworu");

  rtc.begin();
  odczytajEEPROM();
  inicjalizujCzujniki();

  tryb = TRYB_LATO;
  ustawTryb();

  lcd.clear();
  lcd.print("Tryb LATO");
  delay(1000);
}

// ====================== LOOP ======================
void loop()
{
  unsigned long teraz = millis();

  odczytajPrzyciski();

  if (!czujnikiOK) {
    if (teraz - ostatniaReinicjalizacjaCzujnikow >= ODCZYT_TEMPERATURY_MS) {
      ostatniaReinicjalizacjaCzujnikow = teraz;
      inicjalizujCzujniki();
    }
  }
  else {
    if (teraz - ostatniPomiar >= ODCZYT_TEMPERATURY_MS) {
      ostatniPomiar = teraz;
      rozpocznijPomiarCzujnikow();
    }
  }

  zakonczPomiarCzujnikow();

  if (teraz - ostatniRegulator >= INTERWAL_REGULACJI) {
    ostatniRegulator = teraz;
    regulator_zaworu();
  }

  sprawdzCzasRuchuZaworu();
  sprawdzBezpieczenstwo();
  procesAntystop();

  if (teraz - ostatniLCD >= ODCZYT_LCD_MS) {
    ostatniLCD = teraz;
    wyswietlLCD();
  }
}void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
