#include <nRF24L01.h>
#include "RF24.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <GyverPower.h>
//#include "TemperatureSensor.h"
#include "LCD5110.h"
#include "fonts.h"

#define ONE_WIRE_BUS        A2        // DS18B20
#define BAT_ADC_PIN         A1        // пин подключения к ацп напряжения с акб
#define BAT_CHRG_PIN        2         // пин подключения к ацп. детектор подключения зу
#define BAT_STDBY_PIN       A7        // пин подключения к ацп. окончание зарядки
#define NRF2401_POWER       A0        // пин питания nrf24L01
#define NRF24_CE_PIN        10
#define NRF24_CSN_PIN       9 

#define BAT_CHARGED_ADC     824       // значeние ацп, при котором напряжение на акб 4.2 вольт (заряжен)
#define BAT_DISCHARGED_ADC  634       // значeние ацп, при котором напряжение на акб 3.0 вольт (разряжен)

#define BUF_SIZE            13

#define BATERY_POSX         0
#define BATERY_POSY         0
#define BATERY_HEIGHT       8
#define BATERY_WIDTH        14

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
RF24 radio(NRF24_CE_PIN, NRF24_CSN_PIN);    // nrf24l01
LCD5110 myGLCD(4, 5, 6, 8);           // CLK, DIN, DC, RST

const uint32_t sleep_time = 60000;    // время сна в миллисекундах
uint32_t t_timer_on_RX = 0;
float tempReload = -100.00;

uint16_t bat_adc = 0;                 // значение с ацп 0...1023 напряжение с акб

uint8_t iStep = 0;                    // шаг перемещения линии зарядки
bool no_sleeping_flag = false;        // флаг разрешения ухода в сон: "1" запрет, "0" разрешено

bool RF24_inited = true;              // флаг инициализации

extern uint8_t BigNumbers[];
extern const uint8_t LCDFont[];
extern const uint8_t Batery[];
extern const uint8_t LowBatery[];
extern const uint8_t ChargeBatery[];
//extern uint8_t DotMatrix_M_Slash[4184];

void isr_charge_up();
void isr_charge_down();
void make_temp_string(char *arrayTemp, float tempC);
void draw_batery();

void setup() {
  
  Serial.begin(115200);
  
  pinMode(BAT_ADC_PIN, INPUT);
  pinMode(BAT_STDBY_PIN, INPUT);
  pinMode(NRF2401_POWER, OUTPUT);

  myGLCD.InitLCD(70);          //запуск LCD контраст 65
  myGLCD.setFont(LCDFont);
//  myGLCD.setTextSize(1);  // установка размера шрифта
  
//  display.begin();
//  display.setRotation(2);
//  display.setFont(&FreeMonoBoldOblique18pt7b);
//  display.getPixel(0, 0, 0);

  digitalWrite(NRF2401_POWER, HIGH);
  delay(3000);
  
  sensors.begin();
  
  if (!radio.begin()) 
    Serial.println("radio init error"); // Инициализация модуля NRF24L01
  else 
    Serial.println("radio init ok");

  radio.setChannel(0x6f);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(0x7878787878LL);

  //детектор подключения ЗУ с внутренней подтяжкой
  pinMode(BAT_CHRG_PIN, INPUT_PULLUP);
  // аппаратное прерывание: подключение зарядки
  // подключаем прерывание на пин D2 (Arduino NANO), при подключении
  attachInterrupt(0, isr_charge_up, FALLING);
  // подключаем прерывание на пин D2 (Arduino NANO), при отключении
  attachInterrupt(0, isr_charge_down, RISING);
  // глубокий сон
  power.setSleepMode(POWERDOWN_SLEEP);
}

void loop() {
  bat_adc = analogRead(BAT_ADC_PIN);

  if (bat_adc >= BAT_CHARGED_ADC) {
    bat_adc = BAT_CHARGED_ADC;
  } else if (bat_adc <= BAT_DISCHARGED_ADC) {
    bat_adc = BAT_DISCHARGED_ADC;
  }

  sensors.requestTemperatures();          // Send the command to get temperatures
  float tempC = sensors.getTempCByIndex(0);
  radio.write(&tempC, sizeof(tempC));     // Передаём данные по радиоканалу
  //radio.powerDown();

  char arrayTemp[BUF_SIZE] = {0};
  make_temp_string(arrayTemp, tempC);
//  myGLCD.clrScr();
//  myGLCD.printNumI(tempC, CENTER, 13);
  myGLCD.print(arrayTemp, CENTER, BUF_SIZE);
  draw_batery();
  
//  display.clearDisplay();
//  display.fillRect(65, 1, 19, 7, 1);
//  display.fillRect(66, 2, 17, 5, 0);
//  display.fillRect(64, 3, 1, 3, 1);

  /*
   * надо создать буфер, в который поместить значок уровня заряда батареи
   * потом отправлять этот буфер в дисплей.
   */

  Serial.print("bat adc value:  ");
  Serial.println(bat_adc);
//
//  bat_adc = map(bat_adc, BAT_DISCHARGED_ADC, BAT_CHARGED_ADC, 0, 15);
//  Serial.print("bat_adc: ");
//  Serial.println(bat_adc);
//
//  if (analogRead(BAT_CHRG_PIN) < 500 || analogRead(BAT_STDBY_PIN) < 500) {
//    Serial.println("Зу подключено ");
//    //значок постоянный ток на дисплее

//    no_sleeping_flag = 1;
//    display.writeLine(48, 3, 61, 3, 1);
//    display.writeLine(48, 5, 51, 5, 1);
//    display.writeLine(53, 5, 56, 5, 1);
//    display.writeLine(58, 5, 61, 5, 1);
//
//    if (analogRead(BAT_STDBY_PIN) < 500) {
//      Serial.println("Зарядка АКБ ");
//      for (uint8_t i = 0; i <= iStep; i++) {
//        display.writeLine(82 - i, 2, 82 - i, 6, 1);
//      }
//      iStep ++;
//      if (iStep >= 15) iStep = 0;
//    } else {
//      Serial.println("Заряжено");
//      display.fillRect(65, 1, 19, 7, 1);
//      display.fillRect(68, 4, 13, 1, 0);
//    }
//  } else {
//    Serial.println("Разряжается");
//    no_sleeping_flag = 0;
//    for (uint8_t i = 0; i <= bat_adc; i++) {
//      display.writeLine(82 - i, 2, 82 - i, 6, 1);
//    }
//  }
//
//  if ((int)tempReload != (int)tempC) {
//    for (int i = 0; i <= BUF_SIZE; i++) {
//      if (i != BUF_SIZE) {
//        display.drawChar( (i * 17), 38, arrayTemp[i]);
//      } else {
//        display.drawChar( 5 + (i * 18), 38, arrayTemp[i]);
//        display.drawChar( (i * 18), 18, '.');
//      }
//    }
//    display.display();
//  }

  // спим 1 минуту, но можем проснуться по прерыванию
  uint32_t mls_delay = 60000;
  Serial.print("Sleep mls: ");
  Serial.println(mls_delay);
  delay(200);
  power.sleepDelay(mls_delay);
  Serial.println("Wake up");
}

// обработчик аппаратного прерывания
void isr_charge_up() { 
  power.wakeUp();  
  Serial.println("Sensor charging!");
}

void isr_charge_down() {
  power.wakeUp();  
  Serial.println("Sensor stop charging!");
}

void make_temp_string(char *arrayTemp, float tempC) {
  String temp_string_maker = "";
  
  // Выставление + или - в зависимости от температуры
  if (tempC > float(0)) {
    temp_string_maker += '+';
  } else if (tempC < float(0)) {
    temp_string_maker += '-';
  }

  temp_string_maker += String(int(tempC)) + "C";

  strcpy(arrayTemp, temp_string_maker.c_str());
  
  Serial.print("Output temperature string: ");
  Serial.println(arrayTemp);
}

void draw_batery() {
  if (digitalRead(BAT_CHRG_PIN) == LOW)
    myGLCD.drawBitmap(BATERY_POSX, BATERY_POSY, ChargeBatery, BATERY_WIDTH, BATERY_HEIGHT);
  else if (bat_adc == BAT_CHARGED_ADC)
    myGLCD.drawBitmap(BATERY_POSX, BATERY_POSY, Batery, BATERY_WIDTH, BATERY_HEIGHT);
  else
    myGLCD.drawBitmap(BATERY_POSX, BATERY_POSY, LowBatery, BATERY_WIDTH, BATERY_HEIGHT);
}
