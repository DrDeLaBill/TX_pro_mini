#ifndef TemperatureSensor_h
#define TemperatureSensor_h

#include "fonts.h"

#if defined(__AVR__)
  #include "Arduino.h"
#elif defined(__PIC32MX__)
  #include "WProgram.h"
#elif defined(__arm__)
  #include "Arduino.h"
#endif

#define BUF_SIZE                13      //ширина строки в символах
#define SLEEP_TIME              60000   //время сна в миллисекундах
#define RELOAD_TEMPERATURE      -100.0  //температура, при которой устройство должно перезагрузиться

extern const uint8_t LCDFont[];
extern const uint8_t Batary[];
extern const uint8_t LowBatery[];
extern const uint8_t ChargeBatery[];

class TemperatureSensor 
{
  public:
    uint16_t bat_abc = 0;               // значение с ацп 0...1023 напряжение с акб
    char *arrayTemp;

    TemperatureSensor();
    void init_temperature_sensor();
    void make_temp_string(float tempC);
};

#endif
