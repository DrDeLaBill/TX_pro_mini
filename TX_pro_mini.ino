#include <RF24.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <GyverPower.h>
#include "LCD5110.h"
#include "fonts.h"

#define ONE_WIRE_BUS        A2        // DS18B20
#define BAT_ADC_PIN         A1        // пин подключения к ацп напряжения с акб
#define BAT_CHRG_PIN        2         // пин подключения к ацп. детектор подключения зу
#define BAT_STDBY_PIN       A7        // пин подключения к ацп. окончание зарядки
#define NRF2401_POWER       A0        // пин питания nrf24L01
#define NRF24_CE_PIN        10
#define NRF24_CSN_PIN       9 
#define BAT_CHARGED_ADC     963       // значeние ацп, при котором напряжение на акб 4.2 вольт (заряжен)
#define BAT_DISCHARGED_ADC  573       // значeние ацп, при котором напряжение на акб 2.5 вольт (разряжен)
#define NRF24_VOLTAGE_DELAY 1500      // задержка млс для ожидания установки напряжения на понижайке
#define BUF_SIZE            13        // длинна строки температуры
#define SLEEP_TIME          60000     // время сна в миллисекундах
// настройка изображения аккумулятора
#define BATERY_POSX         84
#define BATERY_POSY         0
#define BATERY_HEIGHT       8
#define BATERY_WIDTH        14
// настройка LCD
#define CLK   4
#define DIN   5
#define DC    6
#define RST   8

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
RF24 radio(NRF24_CE_PIN, NRF24_CSN_PIN);
LCD5110 myGLCD(CLK, DIN, DC, RST); 
uint16_t bat_adc = 0;                 // значение с ацп 0...1023 напряжение с акб

extern const uint8_t LCDFont[];
extern const uint8_t Batery[];
extern const uint8_t Batery75[];
extern const uint8_t Batery50[];
extern const uint8_t Batery25[];
extern const uint8_t LowBatery[];
extern const uint8_t ChargeBatery[];

void isr_charge();
void make_temp_string(char *arrayTemp, short tempC);
void draw_temperature(short tempC);
void draw_batery();
bool is_charging();
void(*resetFunc) (void) = 0; // перезагрузка
void go_asleep();
void batery_report();

void setup() {
  Serial.begin(115200);

  analogReference(INTERNAL);
  
  pinMode(BAT_ADC_PIN, INPUT);
  pinMode(BAT_STDBY_PIN, INPUT);
  pinMode(NRF2401_POWER, OUTPUT);

  digitalWrite(NRF2401_POWER, HIGH);
  delay(NRF24_VOLTAGE_DELAY);
  if (radio.begin()) {
    Serial.println("radio init ok"); // Инициализация модуля NRF24L01
    radio.setChannel(0x6f);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_HIGH);
    radio.openWritingPipe(0x7878787878LL);
  }
  else {
    Serial.println("radio init error"); 
    delay(200);
    resetFunc();
  }

  myGLCD.InitLCD(70);          //запуск LCD контраст 65
  myGLCD.setFont(LCDFont);
  
  sensors.begin();

  //детектор подключения ЗУ с внутренней подтяжкой
  pinMode(BAT_CHRG_PIN, INPUT_PULLUP);
  // аппаратное прерывание: подключение зарядки
  // подключаем прерывание на пин D2 (Arduino NANO)
  attachInterrupt(0, isr_charge, CHANGE);
  // глубокий сон
  power.setSleepMode(POWERDOWN_SLEEP);
}

void loop() {
  bat_adc = analogRead(BAT_ADC_PIN);

  sensors.requestTemperatures();
  short tempC = sensors.getTempCByIndex(0);
  radio.write(&tempC, sizeof(tempC));

  draw_temperature(tempC);
  draw_batery();

  batery_report();
  
  go_asleep();
}

// обработчик аппаратного прерывания
void isr_charge() { 
  power.wakeUp();
  if (is_charging())
    Serial.println("Sensor charging!");
  else
    Serial.println("Sensor stop charging!");
}

void go_asleep() {
  // спим 1 минуту, но можем проснуться по прерыванию
  uint16_t mls_delay = SLEEP_TIME;
  Serial.print("Sleep mls: ");
  Serial.println(mls_delay);
  delay(10);
  power.sleepDelay(mls_delay);
  Serial.println("Wake up");
}

void batery_report() {
  Serial.print("bat adc value:  ");
  Serial.println(bat_adc);
}

void make_temp_string(char *arrayTemp, short tempC) {
  String temp_string_maker = "";
  
  // Выставление + или - в зависимости от температуры
  if (tempC > 0) {
    temp_string_maker += '+';
  }

  temp_string_maker += String(tempC) + "C";

  strcpy(arrayTemp, temp_string_maker.c_str());
  
  Serial.print("Output temperature string: ");
  Serial.println(arrayTemp);
}

void draw_temperature(short tempC) {
  char arrayTemp[BUF_SIZE] = {0};
  if (tempC < -100) { //ошибка измерений температуры
    Serial.println("SENSOR ERROR");
    char error[] = {'E', 'R', 'R'};
    myGLCD.print(error, CENTER, BATERY_HEIGHT + 1);
    delay(2000);
    resetFunc();
  } else {
    make_temp_string(arrayTemp, tempC);
    myGLCD.print(arrayTemp, CENTER, BATERY_HEIGHT + 1);
  }
}

void draw_batery() {
  Serial.print("Batery: ");
  uint16_t batery_charge_range = BAT_CHARGED_ADC - BAT_DISCHARGED_ADC;
  uint8_t *batery_pic;
  if (is_charging()) {
    Serial.println("100%");
    batery_pic = ChargeBatery;
  }
  else if (bat_adc > 3/4 * batery_charge_range + BAT_DISCHARGED_ADC) {
    Serial.println("100%");
    batery_pic = Batery;
  }
  else if (bat_adc > 1/2 * batery_charge_range + BAT_DISCHARGED_ADC) {
    Serial.println("75%");
    batery_pic = Batery75;
  }
  else if (bat_adc > 1/4 * batery_charge_range + BAT_DISCHARGED_ADC) {
    Serial.println("50%");
    batery_pic = Batery50;
  }
  else if (bat_adc > BAT_DISCHARGED_ADC) {
    Serial.println("25%");
    batery_pic = Batery25;
  }
  else {
    Serial.println("0%");
    batery_pic = LowBatery;
  }
  myGLCD.drawBitmap(BATERY_POSX - BATERY_WIDTH, BATERY_POSY, batery_pic, BATERY_WIDTH, BATERY_HEIGHT);
}

bool is_charging() {
  return digitalRead(BAT_CHRG_PIN) == LOW;
}
