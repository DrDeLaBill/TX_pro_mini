#include <nRF24L01.h>
#include "RF24.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LCD5110_SSVS.h>
#include <GyverPower.h>

#define ONE_WIRE_BUS    A2        // DS18B20
#define BAT_ADC_PIN     A1        // пин подключения к ацп напряжения с акб
#define BAT_CHRG_PIN    2         // пин подключения к ацп. детектор подключения зу
#define BAT_STDBY_PIN   A7        // пин подключения к ацп. окончание зарядки
#define NRF2401_POWER   A0        // пин питания nrf24L01
#define NRF24_CE_PIN    10
#define NRF24_CSN_PIN   9 

#define BAT_CHARGED_ADC 824       // значeние ацп, при котором напряжение на акб 4.2 вольт (заряжен)
#define BAT_DISCHARGED_ADC  634   // значeние ацп, при котором напряжение на акб 3.0 вольт (разряжен)

#define BUF_SIZE        5


OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

RF24 radio(NRF24_CE_PIN, NRF24_CSN_PIN);    // nrf24l01

//Adafruit_PCD8544 display = Adafruit_PCD8544(4, 5, 6, 24, 8);
//Sleep sleep;
LCD5110 myGLCD(4, 5, 6, 8);           // CLK, DIN, DC, RST

const uint32_t sleep_time = 60000;    // время сна в миллисекундах
uint32_t t_timer_on_RX = 0;
float tempReload = -150.00;

uint16_t bat_adc = 0;                 // значение с ацп 0...1023 напряжение с акб
//const uint8_t x_MaxSize = 84;
//const uint8_t y_MaxSize = 48;

uint8_t iStep = 0;                    // шаг перемещения линии зарядки
bool no_sleeping_flag = false;        // флаг разрешения ухода в сон: "1" запрет, "0" разрешено

bool RF24_inited = true;              // флаг инициализации

//boolean abortSleep;   //cancel sleep cycle
//int sleepCycleCount; //the number of times awake then asleep
//unsigned long sleepTime; //how long you want the arduino to sleep


extern uint8_t RusFont[];
extern uint8_t BigNumbers[];
//extern uint8_t DotMatrix_M_Slash[4184];

void isr_charge_up();

void setup() {
  
  Serial.begin(115200);
  
  pinMode(BAT_ADC_PIN, INPUT);
  pinMode(BAT_STDBY_PIN, INPUT);
  pinMode(NRF2401_POWER, OUTPUT);

  //детектор подключения ЗУ с внутренней подтяжкой
  pinMode(BAT_CHRG_PIN, INPUT_PULLUP);
  
  
  
//  abortSleep = false; //can be used to cancel the sleep cycle
//  sleepTime = 60000; //set sleep time in ms, max sleep time is 49.7 days
  // sleep.sleepPinInterrupt(BAT_CHRG_PIN, LOW); //(номер вывода прерывания, состояние прерывания)

  //radio.powerUp();

  myGLCD.InitLCD(70);          //запуск LCD контраст 65
  myGLCD.setFont(BigNumbers);
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

  // аппаратное прерывание: подключение зарядки
  // подключаем прерывание на пин D2 (Arduino NANO)
  attachInterrupt(0, isr_charge_up, RISING);

  // глубокий сон
  power.setSleepMode(POWERDOWN_SLEEP);
}



void loop() {

  int i = 0;

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

  if (tempC == float(0)) {
    itoa(tempC, arrayTemp, DEC);
  }  else {
    itoa(tempC, arrayTemp + 1, DEC);   //преобразоавние  из int в массив char во вторую ячейку, чтобы оставить место под знак
    
    // Выставление + или - в зависимости от температуры
    if (tempC == float(0)) {
      arrayTemp[0] = ' ';
    }else if (tempC > float(0)) {
      arrayTemp[0] = '+';
    } else if (tempC < float(0)) {
      arrayTemp[0] = '-';
    }
  }

  for (i = 0; i <= BUF_SIZE; i++) {
    if (arrayTemp[i] == 0) {
      arrayTemp[i] = 'C';
    }
  }

//  myGLCD.clrScr();
//  myGLCD.printNumI(tempC, CENTER, 13);
  myGLCD.print(arrayTemp, CENTER, 13);
  
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


  Serial.print("no_sleeping_flag: ");
  Serial.println(no_sleeping_flag);
  //  if (no_sleeping_flag == 0) {
  //    Serial.println("Ухожу в сон ");
  //    digitalWrite(NRF2401, LOW);  //Выключение питания на передатчике
  //    sleep.pwrDownMode(); //set sleep mode
  //    sleep.sleepDelay(sleepTime, abortSleep); //sleep for: sleepTime
  //
  //  }

  // спим 1 минуту, но можем проснуться по прерыванию
  power.sleepDelay(1 * 60 * 1000);
}

// обработчик аппаратного прерывания
void isr_charge_up() { 
  power.wakeUp();  
}
