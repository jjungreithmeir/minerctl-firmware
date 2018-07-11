#include <EEPROM.h>
#include <Arduino.h>  // for type definitions

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); ++i)
    EEPROM.write(ee + i, *p++);
  return value;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); ++i)
    *p++ = EEPROM.read(ee + i);
  return value;
}

int EEPROM_readAnything(int ee)
{
  int store = 0;
  EEPROM_readAnything(ee, store);
  return store;
}

int EEPROM_write(int ee, const int value)
{
  EEPROM.update(ee, value);
  return value;
}

int EEPROM_read(int ee)
{
  return EEPROM.read(ee);
}
