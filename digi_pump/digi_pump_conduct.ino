
 
//const int AirValue = 630;             // Максимальное значение сухого датчика
//const int WaterValue = 266;           // Минимальное значение погруженного датчика
const int WaterMax = 400;             // Предельное значение после которого включаем помпу
int soilMoistureValue = 0;            // Создаем переменную soilMoistureValue

 
//#define PERIOD 259200   // период работы в секундах (пример: 60*60*24*3 = 259200 - три дня!)
#define WORK 30         // время работы в секундах
#define MOS 1           // пин мосфета
#define VET 5           // пин датчика влажности

uint32_t mainTimer, myTimer;
boolean state = false;

#include <avr/wdt.h>
#include <avr/interrupt.h>
//#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
//#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC
// http://alexgyver.ru/arduino/DigiDrivers.rar

void setup() 
{
  pinMode(soilMoistureValue, INPUT);    // Устанавливаем вывод как вход - датчик.
  pinMode(MOS, OUTPUT);					// Устанавливаем вход как вывод - Мосфет.
}


void loop() {
  soilMoistureValue = analogRead(VET);   // Считываем данные с порта P5 и записываем их в переменную

  if (soilMoistureValue > WaterMax) { 	// ЕСли показатель с датчика больше значение WaterMax
	pinMode(MOS, OUTPUT);               // пин как выход
	digitalWrite(MOS, HIGH);            // врубить Мосфет
  } else {
	  digitalWrite(MOS, LOW);           // вырубить
      pinMode(MOS, INPUT);              // пин как вход (экономия энергии)

}

void loop() {

  if (!state) {                           // если помпа не включена
    if ((long)mainTimer - myTimer > PERIOD) {   // таймер периода
      myTimer = mainTimer;                // сброс таймера
      state = true;                       // флаг на запуск
      pinMode(MOS, OUTPUT);               // пин как выход
      digitalWrite(MOS, HIGH);            // врубить
    }
  } else {                                // если помпа включена
    if ((long)mainTimer - myTimer > WORK) {     // таймер времени работы
      myTimer = mainTimer;                // сброс
      state = false;                      // флаг на выкл
      digitalWrite(MOS, LOW);             // вырубить
      pinMode(MOS, INPUT);                // пин как вход (экономия энергии)
    }
  }

  sleep_enable();   // разрешаем сон
  sleep_cpu();      // спать!
}

ISR (WDT_vect) {
  WDTCR |= _BV(WDIE); // разрешаем прерывания по ватчдогу. Иначе будет реcет.
}
