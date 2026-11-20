Założenia:

Cztery czujniki DS18B20 mają osobne adresy na jednej magistrali.
DS3231 jest po I2C.
Wyjścia są realizowane przekaźnikami/sterownikami aktywowanymi stanem HIGH.
Wyjścia zaworu są blokowane wzajemnie.
Parametry są zapisane w EEPROM

Wymagane biblioteki Arduino:
OneWire
DallasTemperature
LiquidCrystal_I2C
RTClib
Warto pamiętać, że oryginalny opis nie podaje fizycznych adresów czujników. inicjalizujCzujniki() przypisuje czujniki kolejno według kolejności wykrycia przez bibliotekę; po wymianie czujnika kolejność może się zmienić i wymagane będzie zaprogramowanie stałych 64-bitowych adresów ROM.
