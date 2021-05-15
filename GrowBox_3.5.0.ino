/*версия 3.5
  использет емкостный датчик влажности почвы
*/
/*версия 3.4
  добавлен датчик освещенности
*/
/*версия 3.3.2
  скоррект. имя переменной в закомментированной строке 24х - Temperature = TempDHT;
*/
/*версия 3.3.1
  - полив при срабатывании датчика влажности почвы
  - включение реле на увлажнение воздуха
  - продолжительность полива по датчику

  Youtube видео:
  Youtube канал:  https://www.youtube.com/channel/UC82y1uu7k08kq9SMpckNRoA
*/

#include <Wire.h>
//-------------часы --------
#include "RTClib.h"  //(библиотека RTClib ) DS1307 clock // SDA -A4 /SCL -A5  //билиотека папка RTClib //3.5
RTC_DS1307 ds1307;    // Create RTC object
int rtc[7];          // массив данных часов
#define LOAD_TIME 0     // 1- установить время, как на компьютере (при первой загрузке) , 0- не устанавливать время 
//---------------------------------------

#include <LCD_1602_RUS.h> //библиотека для вывода русс. символов (не более 8шт. не совпадающих с латиницей)
LCD_1602_RUS lcd(0x3F, 16, 2); //создаем объект дисплей lcd
//LCD_1602_RUS lcd(0x27, 16, 2); //создаем объект дисплей lcd - другой адрес (если не работает с предыдущей строкой)

//------------- датчик температуры 18b20--------------
#include <OneWire.h>
#define POWER_MODE  0 // режим питания, 0 - внешнее, 1 - паразитное
OneWire sensDs (10);  // датчик подключен к выводу 10
byte bufData[9];  // буфер данных
float Temperature;  // измеренная температура
//------------------------

//---------DHT11/DH22 датчик темпер и влаж --------
// !!! не забываем раскомментировать ниже строку "Temperature = TempDHT;" !!!
#include <iarduino_DHT.h> // Подключаем библиотеку для работы с датчиком DHT
iarduino_DHT  sensor1(9); //DHT подключен на пин ...
byte Hum = 0; // показания влажности
float TempDHT = 0.0; // показания температуры
unsigned long lastDHT = 0; // для контроля обновления показаний с интервалом не более 2 сек ( т.к. это медленные датчики)
#define DHT_corr_H 0 // коррекция показаний влажности (+- если надо)
#define DHT_corr_T 0.0 // коррекция показаний температуры (+- если надо)
//------------------------------
//-3.3-----------
//#define ground_Dpin 13// сигнальный контакт датчика влажности почвы
#define ground_Apin A1 // аналоговый контакт датчика влажности почвы


//--- кнопки----
#include <AmperkaKB_ms.h> // подключаем библиотеку для работы с матричной клавиатурой
// указывая номера arduino подключенные к шлейфу клавиатуры

AmperkaKB KB(6, 5, 4, 3, 2); //первый контакт= отдельный провод клавиатуры
//AmperkaKB KB(2,6, 5, 4, 3); //первый контакт= отдельный провод клавиатуры
//AmperkaKB KB(6, 2, 3, 4, 5); //первый контакт= отдельный провод клавиатуры
//----------работа с памятью--------------
#include <EEPROMex.h>
#include <EEPROMVar.h>
//----------------------------------------

boolean DAY = 0; // флаг времени суток 0-ночь, 1-день
#define Relay_Lamp 12 // пин реле включения освещения
#define Relay_Heat 11 // пин реле включения подогрева
#define Relay_Water 7 // пин реле включения полива
#define Relay_Vent 8 // пин реле включения вентиляции
#define Relay_Hum 16 // пин (16=А2) реле включения увлажнителя воздуха
#define Light_sens 17 // пин (17=А3) датчика освещенности //3.4

boolean heating = 1; // режим обогрева on/off   1/0                            - eeprom= 31
float Temp_Box_Day = 24.0; // необходимая температура при освещении   - eeprom= 1
float Temp_Box_Night = 21.0; // необходимая температура БЕЗ освещении - eeprom= 5
float delta_Temp = 0.4; //   отклонение температуры до включения (гистерезис)   - eeprom= 9
float step_Temp = 0.2; //   шаг измениения температуры

boolean lighting = 1; // режим освещения on/off                              - eeprom= 32
byte start_day_h = 17; // время начала дня (часы)                            - eeprom= 13
byte start_day_m = 00; // время начала дня (минуты)                          - eeprom= 14
byte duretion_day_h = 16 ; // продолжительность досветки (часы)              - eeprom= 15
byte duretion_day_m = 00; // продолжительность досветки (минуты)             - eeprom= 16
byte offL_h; // время выключения света(часы)
byte offL_m ; // время выключения света(минуты)
boolean sens_lighting = 0; // режим освещения - учитывать датчик освещенности on/off  - - eeprom= 44  //3.4
boolean No_Sunny = 0; // от солнца светло=0, темно=1   //3.4
#define AskSensLighTime 300000 //  интервал проверки датчика освещенности, мс (чтобы не щелкало реле освещения при пограничном освещении)300000=5мин   //3.4
unsigned long LastAskLS = 0; // время последней проверки датчика освещенности   //3.4

boolean watering = 1; // режим полива on/off                          - eeprom= 33
byte start_wat_h = 8; // время начала полива (часы)                   - eeprom= 17
byte start_wat_m = 00; // время начала полива (минуты)                - eeprom= 18
byte duretion_wat_h = 1 ; // продолжительность полива (часы)          - eeprom= 19
byte duretion_wat_m = 00; // продолжительность полива (минуты)        - eeprom= 20
byte offW_h; // время выключения полива(часы)
byte offW_m ; // время выключения полива(минуты)
//-3.2-----------
boolean ventilation = 1; // режим вентиляции on/off   1/0                            - eeprom= 34
float Temp_Vent_ON = 28.0; // температура включения вентиляции  - eeprom= 35
float delta_Temp_Vent = 2.0; //   отклонение температуры до отключения (гистерезис)- eeprom= 39
//-3.3-----------
boolean sens_watering = 1; // режим полива по датчику влаж. почвы on/off                          - eeprom= 40
byte ground_Hum ; // текущий уровень влажности почвы 0-100% (условный)
boolean vkl = 0; // флаг включения полива в фун. poliv()- также испол. в фун. poliv_Sen()
#define AskSensTime 600000 // интервал проверки датчика влаж.почвы, мс / 5-10мин (300000-600000) для исключения работы пограничного состояния (не убить насос на вкл/выкл)
unsigned long LastAskSens = 0; // время последней проверки датчика почвы
#define PolivTime 3600000 // мс, продолжительность полива при срабатывании датчика влаж. почвы. 10ч=36000000
unsigned long PolivOn = 0; // время вкл. полива по датчику
byte HumGND_ON = 45; //%, уровень влажности почвы, ниже которого включаем полив //3.5
#define CONFIRM_COUNT 3 //кол-во последовательных проверок (AskSensTime) датчика вл.почвы. Насос включаем,если все проверки показали низкую влажность 3.5
byte confirm_poliv; //счетчик последовательных проверок (AskSensTime) датчика вл.почвы с показниями ниже HumGND_ON  3.5

boolean humidification = 1; // режим увлажнения воздуха on/off   1/0  - eeprom= 41
byte Hum_ON = 30; // порог включения увлажнителя воздуха  - eeprom= 42
byte delta_Hum = 5; //   отклонение влажности до отключения (гистерезис)- eeprom= 43


unsigned long startMenuTime;   // время входа в меню
#define exitTime 5000 // время автовыхода из меню, мс
#define NUM_SETTINGS_MODES 6      // число режимов меню (включая 0)

String Temp , TempD, TempN;

//*********************************************************************************
void setup() {

  Serial.begin(9600);              // открыть порт для связи
  // Serial.println("setup");
  //-------------------------------------------

  pinMode(Relay_Lamp, OUTPUT);
  pinMode(Relay_Heat, OUTPUT);
  pinMode(Relay_Water, OUTPUT);
  pinMode(Relay_Vent, OUTPUT);//3.2
  pinMode(Relay_Hum, OUTPUT);//3.3

 // pinMode(ground_Dpin, INPUT); //3.3
  pinMode(ground_Apin, INPUT); //3.3
  analogReference(DEFAULT);   //3.3 опорное напряжение по питанию

  digitalWrite(Relay_Lamp, HIGH); //выключаем реле
  digitalWrite(Relay_Heat, HIGH);
  digitalWrite(Relay_Water, HIGH);
  digitalWrite(Relay_Vent, HIGH); //3.2
  digitalWrite(Relay_Hum, HIGH); //3.3
  //delay(1000);

  //-------Setup DS1307 RTC--------------------
#ifdef AVR
  Wire.begin();
#else
  Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino
#endif
  ds1307.begin(); //start RTC Clock

  if (! ds1307.isrunning()) {
    Serial.println("RTC is NOT running!");
  }
  if (LOAD_TIME == 1) ds1307.adjust(DateTime(__DATE__, __TIME__)); // установка времани = время на компе.

  load(); //инициализация

  KB.begin(KB1x4); // клавиатура
  //-----------------------------------------

  No_Sunny = digitalRead(Light_sens); // светло или темно по датчику овсещен. //3.4


  //------EEPROM--------------------

  //EEPROM.writeByte(100, 0); //  пишем 0, чтобы обновилась память в соответствии с переменными (для тестирования)

  // в ячейке 100 должен быть записан флажок 1, если его нет - делаем (ПЕРВЫЙ ЗАПУСК)
  if (EEPROM.read(100) != 1) {
    EEPROM.writeByte(100, 1);
    update_EEPROM(0); //записываем исходные данные в ячейки памяти
  }
  else { // записываем в переменные данные из памяти
    heating = EEPROM.read(31);// режим обогрева on/off
    Temp_Box_Day = EEPROM.readFloat(1); // необходимая температура при освещении
    Temp_Box_Night = EEPROM.readFloat(5); // необходимая температура  БЕЗ освещении
    delta_Temp = EEPROM.readFloat(9); //   отклонение температуры до включения

    lighting = EEPROM.read(32);// режим освещения on/off
    start_day_h = EEPROM.read(13); // время начала дня (часы)
    start_day_m = EEPROM.read(14); // время начала дня (минуты)
    duretion_day_h = EEPROM.read(15); // продолжительность досветки (часы)
    duretion_day_m = EEPROM.read(16); // продолжительность досветки (минуты)

    watering = EEPROM.read(33);//режим полива on/off
    start_wat_h = EEPROM.read(17);// время начала полива (часы)
    start_wat_m = EEPROM.read(18); // время начала полива (минуты)
    duretion_wat_h = EEPROM.read(19); // продолжительность полива (часы)
    duretion_wat_m = EEPROM.read(20);// продолжительность полива (минуты)
    //3.2
    ventilation = EEPROM.read(34); //режим вентиляции on/off
    Temp_Vent_ON = EEPROM.read(35);
    delta_Temp_Vent = EEPROM.read(39);
    //3.3
    sens_watering = EEPROM.read(40); //режим полива по датчику влаж. почвы on/off
    humidification = EEPROM.read(41); // режим увлажнения воздуха on/off
    Hum_ON = EEPROM.read(42); // порог включения увлажнителя воздуха
    delta_Hum = EEPROM.read(43);  //   отклонение влажности до отключения (гистерезис)
    sens_lighting = EEPROM.read(44);  //   учитывать датчик освещенности
    HumGND_ON = EEPROM.read(45); // уровень влажности почвы, ниже которого включаем полив

  }
}//setup
//*******************************************************************************************
//*******************************************************************************************

void loop() {
  delay(20);
  KB.read();
  if (KB.getNum == 1 )setup_menu();
  if (KB.getNum == 4 )displays(); // просмотр экранов

  //Serial.println("loop");

  //***************************************
  day_night(); // функ. назначает по времени день или ночь

  if (millis() - LastAskLS > AskSensLighTime) { // если пора проверить датчик освещенности    //3.4
    LastAskLS = millis(); // запоминаем
    No_Sunny = digitalRead(Light_sens); // светло или темно по датчику овсещен.
  }
  //**************************************************

  //------------ темпер и влаж------------------------
  if (millis() - lastDHT > 2000) {
    lastDHT = millis(); // запоминаем
    sensor1.read();
    Hum = sensor1.hum; // показания влажности DHT датчика
    TempDHT = sensor1.tem; // показания температуры  DHT датчика
   // Serial.println((String)"Hum=" + Hum + "+" + DHT_corr_H + "%");
   //  Serial.println((String)"TempDHT=" + TempDHT + "+" + DHT_corr_T +  "C");
    Hum = Hum + DHT_corr_H; //корректируем показания
    TempDHT = TempDHT + DHT_corr_T; //корректируем показания
  }

  Temperature = get_temp18b20(); // функ. вычисления температуры по 18b20
  //Temperature = TempDHT; // раскомментировать при изм. темпер. DHT

  //Serial.println (Temperature);
  //-------------------------------------

  //-3.5--- получаем данные датчика почвы------------------------

  ground_Hum = map(analog_aver(ground_Apin, 10, 5), 315, 622, 100, 0); // пересчитать в проценты( 306-вода 603-сухой  - опытные данные из 0-1023)
  // ground_Hum = map(analogRead(ground_Apin), 1, 1023, 100, 0);  // пересчитать в проценты(- опытные данные из 0-1023)
 // Serial.println((String)"analogRead= " + analogRead(ground_Apin));
  // Serial.println((String)"ground_Hum= " + ground_Hum);

  //-------------------------------------------------------

  //DAY=1; ////// для теста

  //---округляем t до 1го знака--------
  static char outstr1[15];// массив
  dtostrf(Temperature, 4, 1, outstr1); // (переменная типа float;, длина получаемого символьного значения, количество символов после запятой;,символьный массив для сохранения результата преобразован)
  Temp = (String)(outstr1);  //преобраз массава в строку

  static char outstr2[15];// массив
  dtostrf(Temp_Box_Day, 4, 1, outstr2); // (переменная типа float;, длина получаемого символьного значения, количество символов после запятой;,символьный массив для сохранения результата преобразован)
  TempD = (String)(outstr2); //преобраз массава в строку

  static char outstr3[15];// массив
  dtostrf(Temp_Box_Night, 4, 1, outstr3); // (переменная типа float;, длина получаемого символьного значения, количество символов после запятой;,символьный массив для сохранения результата преобразован)
  TempN = (String)(outstr3);  //преобраз массава в строку
  //-------------------------------------------

  //********выводим на экран  **************
  //lcd.clear(); //чистим экран
  lcd.setCursor(0, 0);
  lcd.print((String) zero(rtc[2]) + ":" + zero(rtc[1]) + F(" ТЕМП=") + Temp + char(223) );
  lcd.setCursor(0, 1);
  lcd.print((String)F("ВВ= ") + Hum + "%" + " " + F("ВП= ") + ground_Hum + "%  "); //3.3
  //lcd.print((String)F("ВЛАЖНОСТЬ= ") + Hum + "%");
  //**********************************************

  //+++++++++++++++++++++++++++++++++++++++++++++++++
  //+++++++++++++ Работа с реле +++++++++++++++++++++
  //+++++++++++++++++++++++++++++++++++++++++++++++++
  //*******  ПОЛИВ ************************************
  if (watering == 1) {
    poliv(); // если активно -фун. включения полива
  }
  else if (sens_watering == 0) digitalWrite(Relay_Water, HIGH); //если откл. по датчику -отключаем реле

  //-- полив по датчику влаж. почвы----

  if (sens_watering == 1) { // если мониторим датчик почвы
    if (millis() - LastAskSens > AskSensTime) { // если пора проверить датчик вл. почвы
      LastAskSens = millis(); // запоминаем
      if (vkl == 0)  { // если полив по времени откл.
        poliv_Sen(); // если активно -фун. включения полива
      }
    }
    else if (millis() - PolivOn > PolivTime) {
      digitalWrite(Relay_Water, HIGH);//отключаем реле полива;
    //  Serial.println("pump OFF!!!!!!!!!!!!!!!!!!!");
    }
  }
  else if (vkl == 0) { // если полив по времени откл.
    digitalWrite(Relay_Water, HIGH); //отключаем реле полива;
  }



  //**************************************************
  //---------включаем свет , если день и активно----

  if (lighting == 1) {//если активно
    if (sens_lighting == 1) { // если учитываем показания датчика освещ. //3.4
      if (DAY == 1 && No_Sunny == 1 ) { //если день и на датчике темно
        digitalWrite(Relay_Lamp, LOW); //включаем реле
      }
      else  {
        digitalWrite(Relay_Lamp, HIGH); //отключаем реле
      }

    }
    else {
      if (DAY == 1 ) {
        digitalWrite(Relay_Lamp, LOW); //включаем реле
      }
      else  {
        digitalWrite(Relay_Lamp, HIGH); //отключаем реле
      }
    }
  }
  else digitalWrite(Relay_Lamp, HIGH); //отключаем реле
  //-----------------------------------------------------------

  //-----------отопление------------------------------------
  if (heating == 1) {//если активно
    if ( DAY == 1 && Temperature < (Temp_Box_Day - delta_Temp)) { // если день и похолодало
      digitalWrite(Relay_Heat, LOW); // включаем подогрев дня
    }
    else if ( DAY == 0 && Temperature < (Temp_Box_Night - delta_Temp)) { // если ночь похолодало
      digitalWrite(Relay_Heat, LOW); // включаем подогрев ночи
    }

    else if (DAY == 1 && Temperature > (Temp_Box_Day)) { // если день и подогрели
      digitalWrite(Relay_Heat, HIGH); //выключаем реле подогрева
    }
    else if (DAY == 0 && Temperature > (Temp_Box_Night)) { // если ночь и подогрели
      digitalWrite(Relay_Heat, HIGH); //выключаем реле подогрева
    }
  }
  else {
    digitalWrite(Relay_Heat, HIGH); //выключаем реле подогрева
  }
  //----------------------------------------------

  //-----------вентиляция ----------------------------------------
  if (ventilation == 1) {//если активно
    if ( Temperature > Temp_Vent_ON) { // если жарко
      digitalWrite(Relay_Vent, LOW); // включаем реле вент.
    }

    else if (Temperature <= (Temp_Vent_ON - delta_Temp_Vent)) { // если остыли
      digitalWrite(Relay_Vent, HIGH); // выключаем реле вент.
      //  Serial.println((String) Temp_Vent_ON + "-" + delta_Temp_Vent );
    }
  }
  //-------------------------------------------------------

  //-----------увлажнение воздуха  -------------------------------
  if (humidification == 1) {//если активно
    if ( Hum < Hum_ON) { // если сухо
      digitalWrite(Relay_Hum, LOW); // включаем реле
    }

    else if (Hum >= (Hum_ON + delta_Hum)) { // если влажно
      digitalWrite(Relay_Hum, HIGH); // выключаем реле
      //  Serial.println((String) Hum_ON + "-" + delta_Hum );
    }
  }
  //-------------------------------------------------------


} //loop



//--------------------------
//*************ФУНКЦИИ*****************
//*****************************************
//------ смена экранов----------------
void displays() {
  byte disp = 1;
  byte firstrun = 1;

  startMenuTime = millis(); //запоминаем
  //  KB.read();
  //   KB.getNum=0; // обнуляем номер клавиши
  KB.read();

  while (KB.getNum != 2) {
    if ( (millis() - startMenuTime) > exitTime )
    {
      lcd.clear();
      break; //выходим
    }
    KB.read();

    if ( (KB.justPressed() && KB.getNum == 4) || firstrun == 1) {
      startMenuTime = millis(); //запоминаем

      disp++; // меняем тип вывода инфы
      if (firstrun == 1)firstrun = 0;
      if (disp > 8)disp = 1; // если превысили,то сброс*/
      lcd.clear();
    }

    switch (disp) {
      case 1:
        lcd.setCursor(0, 0);
        lcd.print((String) zero(rtc[2]) + ":" + zero(rtc[1]) + F(" ТЕМП=") + Temp + char(223) );
        lcd.setCursor(0, 1);
        lcd.print((String)F("ВВ= ") + Hum + "%" + " " + F("ВП= ") + ground_Hum + "%  "); //3.3
        // lcd.print((String)F("ВЛАЖНОСТЬ= ") + Hum + "%");
        break;

      case 2:
        lcd.setCursor(0, 0);
        lcd.print(F("ОБОРУДОВАНИЕ"));
        lcd.setCursor(0, 1);
        lcd.print((String)"L" + curR(Relay_Lamp) + " T" + curR(Relay_Heat) + " W" + curR(Relay_Water) + " V" + curR(Relay_Vent) + " H" + curR(Relay_Hum));
        break;

      case 3://
        lcd.setCursor(0, 0);
        lcd.print((String) F("СВЕТ: ВКЛ  ") +  zero(start_day_h) + ":" + zero(start_day_m));
        lcd.setCursor(0, 1);
        lcd.print((String)zero(duretion_day_h) + ":" + zero(duretion_day_m) + F(" ОТКЛ ") + zero(offL_h) + ":" + zero(offL_m));
        break;

      case 4:
        lcd.setCursor(0, 0);
        lcd.print((String) F("ТЕМП: ДЕНЬ  ") +  TempD);
        lcd.setCursor(0, 1);
        lcd.print((String) F("d=") + delta_Temp + F(" НОЧЬ ") +  TempN );
        break;

      case 5://
        lcd.setCursor(0, 0);
        lcd.print((String) F("ПОЛИВ: ВКЛ ") +  zero(start_wat_h) + ":" + zero(start_wat_m));
        lcd.setCursor(0, 1);
        lcd.print((String)F("ПРОДОЛЖ. ") + zero(duretion_wat_h) + ":" + zero(duretion_wat_m));
        break;

      case 6://3.2
        lcd.setCursor(0, 0);
        lcd.print(F("ВЕНТИЛЯЦИЯ"));//
        lcd.setCursor(0, 1);
        lcd.print((String)  F("ТЕМП:") +  Temp_Vent_ON + F(" d=") + delta_Temp_Vent);
        break;

      case 7://3.2
        lcd.setCursor(0, 0);
        lcd.print(F("УВЛАЖНЕНИЕ ВОЗД."));
        lcd.setCursor(0, 1);
        lcd.print((String)  F("ВЛАЖ:") +  Hum_ON + F("% d=") + delta_Hum);
        break;

      case 8:


        lcd.setCursor(0, 0); lcd.print(F("L LD T WT WD V H"));   //3.4
        lcd.setCursor(0, 1); lcd.print(curSet(lighting));
        lcd.setCursor(2, 1); lcd.print(curSet(sens_lighting));   //3.4
        lcd.setCursor(5, 1); lcd.print(curSet(heating));
        lcd.setCursor(7, 1); lcd.print(curSet(watering));
        lcd.setCursor(10, 1); lcd.print(curSet(sens_watering));
        lcd.setCursor(13, 1); lcd.print(curSet(ventilation));
        lcd.setCursor(15, 1); lcd.print(curSet(humidification));


        break;
    }

    delay(30);
  } //while
  KB.getNum = 0; // обнуляем номер клавиши
}

//------------------------------------
// добавление нуля,если время однозначное
String zero(int data) {
  String str;
  if (data < 10) str = "0" + (String)data;
  else str = (String)data;
  //Serial.println((String)"str="+str); // выводим
  return str;
}
//-----------читает состояние пина и выдает символ----------------------------
char curR(int pin) {
  if (digitalRead(pin) == LOW) return '+';
  else if (digitalRead(pin) == HIGH)   return '-';
  else  return 'E';
}

//-----------3,3-читает значение и выдает on/off----------------------------
String curSet(byte per) {
  if (per == 1) return "+";
  else if (per == 0)   return "-";
  else  return "err";
}


//------- получение данных темпер. датчика---
float get_temp18b20 () {
  //----------------18b20-------------
  sensDs.reset();  // сброс шины
  sensDs.write(0xCC, POWER_MODE); // пропуск ROM
  sensDs.write(0x44, POWER_MODE); // инициализация измерения
  delay(40);

  sensDs.reset();  // сброс шины
  sensDs.write(0xCC, POWER_MODE); // пропуск ROM
  sensDs.write(0xBE, POWER_MODE); // команда чтения памяти датчика
  sensDs.read_bytes(bufData, 9);  // чтение памяти датчика, 9 байтов

  if ( OneWire::crc8(bufData, 8) == bufData[8] ) {  // проверка CRC
    // данные правильные
    Temperature =  (float)((int)bufData[0] | (((int)bufData[1]) << 8)) * 0.0625 ;
  }
  // Serial.print("temp18b20 : ");
  //  Serial.println(temperature);
  return Temperature;
}
//------- конец -получение данных темпер. датчика---

//----------------------------------------------------
// обновление данных в энергонезависимой памяти
void update_EEPROM(int cod)
{
  switch (cod) {
    case 0:
      EEPROM.write(31, heating);
      EEPROM.writeFloat(1, Temp_Box_Day);
      EEPROM.writeFloat(5, Temp_Box_Night);
      EEPROM.writeFloat(9, delta_Temp);

      EEPROM.write(32, lighting);
      EEPROM.write(13, start_day_h);
      EEPROM.write(14, start_day_m);
      EEPROM.write(15, duretion_day_h);
      EEPROM.write(16, duretion_day_m);

      EEPROM.write(33, watering);
      EEPROM.write(17, start_wat_h);
      EEPROM.write(18, start_wat_m);
      EEPROM.write(19, duretion_wat_h);
      EEPROM.write(20, duretion_wat_m);
      //3.2
      EEPROM.write(34, ventilation);
      EEPROM.write(35, Temp_Vent_ON);
      EEPROM.write(39, delta_Temp_Vent);
      //3.3
      EEPROM.write(40, sens_watering);
      EEPROM.write(41, humidification);
      EEPROM.write(42, Hum_ON);
      EEPROM.write(43, delta_Hum);
      EEPROM.write(44, sens_lighting); //3.4
      EEPROM.write(45, HumGND_ON); //3.5
      break;

    case 1:  EEPROM.updateFloat(cod, Temp_Box_Day); break;
    case 5:  EEPROM.updateFloat(cod, Temp_Box_Night); break;
    case 9:  EEPROM.updateFloat(cod, delta_Temp); break;
    case 13:  EEPROM.updateByte(cod, start_day_h); break;
    case 14:  EEPROM.updateByte(cod, start_day_m); break;
    case 15:  EEPROM.updateByte(cod, duretion_day_h); break;
    case 16:  EEPROM.updateByte(cod, duretion_day_m); break;
    case 17:  EEPROM.updateByte(cod, start_wat_h); break;
    case 18:  EEPROM.updateByte(cod, start_wat_m); break;
    case 19:  EEPROM.updateByte(cod, duretion_wat_h); break;
    case 20:  EEPROM.updateByte(cod, duretion_wat_m); break;
    case 31:  EEPROM.updateByte(cod, heating); break;
    case 32:  EEPROM.updateByte(cod, lighting); break;
    case 33:  EEPROM.updateByte(cod, watering); break;
    case 34:  EEPROM.updateByte(cod, ventilation); break; //3.2
    case 35:  EEPROM.updateByte(cod, Temp_Vent_ON); break; //3.2
    case 39:  EEPROM.updateByte(cod, delta_Temp_Vent); break; //3.2
    case 40:  EEPROM.updateByte(cod, sens_watering); break; //3.3
    case 41:  EEPROM.updateByte(cod, humidification); break; //3.3
    case 42:  EEPROM.updateByte(cod, Hum_ON); break; //3.3
    case 43:  EEPROM.updateByte(cod, delta_Hum); break; //3.3
    case 44:  EEPROM.updateByte(cod, sens_lighting); break; //3.4
    case 45:  EEPROM.updateByte(cod, HumGND_ON); break; //3.5

  }

}
