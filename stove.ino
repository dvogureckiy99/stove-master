


/*
Nextion_2 is a new version. С клавиатурой , вводом коэфиициентов Kp, Ki, setpoint , output . 
Реализовано переключение между страницами.
Добавлен простейший протокол передачи данных.
*/

#include <MAX31856_my.h>
#include <PID_v2.h>
#include <TimerThree.h>
#include <stdint.h>


#include <TimeLib.h>
time_t t_MCU ; //записываем время перового запуска микроконтроллера
time_t t_ALG; // время запуска программы
time_t t_INT; // время запуска работы на участке
time_t t_NOW; // время, необходимое для вывода

//Расшифровка управляющих команд протокола передачи данных
//формат сообщения #12;
// # - признак начала сообщения
//1- байт номера команды
//2- байт номера действия или данные
// ; - признак конца команды
//номер команды
#define LED  1 
#define PID_SWITCH  2 
#define SETPOINT 3
#define OUTPUT 4
#define KOEFF_PROPORTIONAL 5
#define KOEFF_INTEGRAL 6
#define TEMP_POINT 7  //точки установки температуры
#define POINT_SWITCH  8//включение выключение изменение температуры по заданному алгоритму
#define POINT_RUN  9
#define RETURN_INTERVAL  10 //вернуться на интервал  
//номер действия
#define ON  1
#define OFF 0

//Define Variables we'll be connecting to
unsigned int Setpoint = 400; //установочная темепратура
unsigned int Input; //настоящая температура
int Output = 1023;  //  скважность сигнала
double Kp=10; //коэффициент пропорнционального звена 
double Ki=0; //коэффициент интегрирующего звена
byte SampleTime = 1 ; // частота вызова ПИД (в сек)

//Specify the links and initial tuning parameters
//по умолчанию(SetOutputLimits(0, 255);   inAuto = false;)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, REVERSE);
#define PIN_OUTPUT 2 //выход , решулирующий температуру
unsigned long last_time= 0; //последнее время заупска алгоритма
  
                                                        

//connecting IC
//подключаю питание через сопротивление 1,3 кОМ т.е. ток питания 3,8 мА , что соответствует даташиту
#define SCK    11
#define CS     10
#define SDI    13 //
#define SDO    12 //


#define CR0_INIT  (CR0_AUTOMATIC_CONVERSION + CR0_OPEN_CIRCUIT_FAULT_TYPE_K  + CR0_NOISE_FILTER_50HZ )
#define CR1_INIT  (CR1_AVERAGE_2_SAMPLES + CR1_THERMOCOUPLE_TYPE_K)
#define MASK_INIT (~(MASK_VOLTAGE_UNDER_OVER_FAULT + MASK_THERMOCOUPLE_OPEN_FAULT))

MAX31856 *temperature;
#define LED_1_PIN  6

// Использование константы вместо переменной позволяет задать размер для массива.
const int numReadings = 5;
 
unsigned int readings[numReadings];      // данные, считанные с входного аналогового контакта
byte index = 0;                  // индекс для значения, которое считывается в данный момент
unsigned int total = 0;                  // суммарное значение

//точки изменения температуры
uint16_t  temp_point[10];
uint16_t  time_point[10];
uint8_t flag_point_switch = 0 ; //разрешение на изменение температуры по заданному алгоритму
uint16_t time_step[10] ;//время ,через которое меняется установочная температура (в сек) для каждого участка
uint8_t interval = 0; // номер участка
uint8_t compare = 0; //больше или меньше температура в начале участка, чем температура в конце участка
//1  температура в начале участка больше 0 - температура в начале участка меньше
uint8_t flag_first_run_interval = 0 ; //флаг(показатель) первого запуска интервала 1-уже запущен  0- не запущен
uint16_t lastsecond = 0 ;//время крайнего вызова изменения setpoint

void setup() 
{
   Serial.begin(9600);
   //Отправка времени, с начала запуска микроконтроллера
   setTime(0,0,0,1,12,18);
   t_MCU = now(); //записываем время перового запуска микроконтроллера
   //turn the PID on
  //при первом запуске происходит также инициализация Initialize()
  myPID.SetMode(MANUAL);
  myPID.SetSampleTime(SampleTime);
  myPID.SetOutputLimits(0, 1023);
   //------PWM----------------
  Timer3.initialize((long) SampleTime* 1000000);         // инициализировать timer1, и установить период равный периоду вызова ПИД
  Timer3.pwm( PIN_OUTPUT,  Output );                // задать шим сигнал на выводе 9 с  коэффициентом заполнения 

  pinMode(LED_1_PIN, OUTPUT);   
  digitalWrite(LED_1_PIN, HIGH);
  
  // Display temperatures using the serial port
  Serial.println("MAX31856 Sample application");
  
  // Define the pins used to communicate with the MAX31856
  temperature = new MAX31856(SDI, SDO, CS, SCK);
  
  // Initializing the MAX31855's registers
  temperature->writeRegister(REGISTER_CR0, CR0_INIT);
  temperature->writeRegister(REGISTER_CR1, CR1_INIT);
  temperature->writeRegister(REGISTER_MASK, MASK_INIT);
  
  // Wait for the first sample to be taken
  delay(200);

   // инициализация массива окна усредения (инициализируем первые 9 значений, т.к. 10 мы измерим сразу
   //------------------------------------------------
  while ( index < (numReadings-1))
  {
    readings[index] =  temperature->readThermocouple(CELSIUS);
    total= total + readings[index];  
    index ++;
  }
  readings[index]=0; // значение последенго показания равно 0
  //---------------------------------------------------

    //отправка на экран необходимых данных
    delay(100);
    
    Serial.print((String)"settings.kp.txt=\""+"Kp="+Kp+"\""+char(255)+char(255)+char(255));
    delay(50); 
    Serial.print((String)"settings.ki.txt=\""+"Ki="+Ki+"\""+char(255)+char(255)+char(255)); 
    delay(50);
    Serial.print((String)"settings.sp.txt=\""+"Setpoint="+Setpoint+"\""+char(255)+char(255)+char(255));
    delay(50);
    
}


void loop () 
{  

  if((millis() - last_time) >= SampleTime*1000  )
  {
    //----усредение----------------------
    total= total - readings[index]; // вычитаем самое раннее значения из окна усреднения
    readings[index] = temperature->readThermocouple(CELSIUS); //считывание показания 
    // добавляем его к общей сумме:
    total= total + readings[index];      
    // продвигаемся к следующему значению в массиве:  
    index ++;                    
    // если мы в конце массива...
    if (index >= numReadings)              
      // ...возвращаемся к началу:
      index = 0;                          
    // вычисляем среднее значение:
    Input = total / numReadings;
    //----------------------------------------
   
    myPID.Compute();  
    
    Timer3.setPwmDuty(PIN_OUTPUT,  Output); //выставляем скважность выходного сигнала

    //отправка  температуры и выхода 
    Serial.print((String)"main.temp.val="+Input+char(255)+char(255)+char(255)); 
    delay(50);
    Serial.print((String)"settings.ot.val="+Output+char(255)+char(255)+char(255));
    delay(50);
    Serial.print((String)"settings.kp.txt=\""+"Kp="+Kp+"\""+char(255)+char(255)+char(255));
    delay(50); 
    Serial.print((String)"settings.ki.txt=\""+"Ki="+Ki+"\""+char(255)+char(255)+char(255)); 
    delay(50);
    Serial.print((String)"settings.sp.txt=\""+"Setpoint="+Setpoint+"\""+char(255)+char(255)+char(255));
    delay(50);
    if(flag_point_switch == 1) //если алгоритм включен
    {
        //времени с момента запуска алгоритма

        uint16_t day_t = day()-day(t_ALG)+(month() - month(t_ALG))*31+(year() - year(t_ALG))*372;
        //Serial.print((String)"main.day.val="+day_t+char(255)+char(255)+char(255));
        uint8_t minute_t = minute()-minute(t_ALG); 
        Serial.print((String)"main.minute1.val="+minute_t+char(255)+char(255)+char(255));
        uint8_t hour_t = hour()-hour(t_ALG);
        Serial.print((String)"main.hour1.val="+hour_t+char(255)+char(255)+char(255));
    }
    last_time = millis();
  }

    if(flag_point_switch) //алгоритм слежения за целевой функцией
    {   
        uint32_t second_t = second()-second(t_ALG) + (minute()-minute(t_ALG))*60+(hour()-hour(t_ALG))*3600+(day()-day(t_ALG))*86400+(month() - month(t_ALG))*2678400;//время с момента запуска алгоритма
        
       
        for(int i = 0; i <= 9; i++)//проверяем на каком мы интервале находимся
        { 
          
            Serial.print((String)"main.day.val="+time_step[0]+char(255)+char(255)+char(255));
            
            if( ((second_t-lastsecond) >= time_step[i] )&&(interval == i) )
            {   
                Serial.print((String)"main.day.val="+1+char(255)+char(255)+char(255));
                
                if(flag_first_run_interval == 0) 
                {
                    t_INT = now(); //запись времени первого запуска интервала
                    flag_first_run_interval = 1 ; //интервал запущен
                    //проверка больше или меньше температура в начале участка, чем температура в конце участка
                    if(Input >= temp_point[i])
                    compare=1;
                    else
                    compare=0; 
                } 
                if(compare==0) //если нужно увеличивать температуру, то идем вверх
                Setpoint = Setpoint+1;
                else ////если нет, то идём  вниз
                Setpoint = Setpoint-1;
                if( ((second_t/60) >= time_point[i]))//достижение установленного времени
                {
                    Setpoint = temp_point[i];//достигаем установленного значения температуры
                    if(((compare=1)&&(Input <= temp_point[i]))||((compare=0)&&(Input >= temp_point[i])))
                    {
                        interval++;//переход на следующий интервал по достижению установленного времени и температуры
                        flag_first_run_interval = 0 ; //интервал закончен
                        if(i=9)
                        { // то есть закончился алгооритм
                            flag_point_switch = 0; // запрещаем алгоритму вызываться во избежания повторения алгоритма
                                        //для повторного вызова нужно перезапустить алгоритм через экран 
                        }
                    }
                }
                lastsecond = second_t;
            }
        }
        
  
    }
    static String  inStr = ""; // Это будет приемник информации от Nextion (здесь только //1- байт номера команды /2- байт номера действия или данные)
    static bool serialReadFlag = false; // А это флаг появление сообщения.

   /******************************************************************/
    // обработка кнопок
    if (Serial.available()) 
    {
        uint8_t inn = Serial.read(); // читаем один байт
        if(serialReadFlag) 
        { // Если установлен флаг приема - действуем
            if(inn == 59)
            {     // ASCII : ";" // Находим конец передачи ";"
                if(inStr.length() > 0) 
                { // Проверяем длину сообщения и отправляем в "переработку"
                    checkCommand(inStr); // В этой функции будем парсить сообщение
                }
                serialReadFlag = false; // Сбрасываем флаг приема
            }
            else 
            { // А это нормальный прием
                inStr += (char)inn; // Считываем данные
            }
        }
        else 
        { // А здесь отлавливается начало передачи от Nextion
            if(inn == 35) 
            { // ASCII : "#"
                serialReadFlag = true; // После # начинаем чтение при следующем заходе
                inStr = ""; // Но до этого очистим стринг приема
            }
        }
    }
    /**************************************************************************/
}

void checkCommand(String ins) 
{
    // У нас информация от Nextion состоит из двух частей,
    // первая буква - идентификатор действия
    // ловим его:
    String command = ins.substring(0,1); //0 - позиция символа 1 - число символов
    // А все остальное - данные:
    String last = ins.substring(1);
    switch (command.toInt())
    {
        case LED: 
        {
            if(last.toInt() == ON)
            digitalWrite(LED_1_PIN, HIGH);
            else
            digitalWrite(LED_1_PIN, LOW);
            break;
        }
        case PID_SWITCH: 
        {
            if(last.toInt() == ON)
            {
            myPID.SetMode(AUTOMATIC);
            }
            else
            {
            myPID.SetMode(MANUAL);
            }
            break;
        }
        case OUTPUT: 
        {
            // Значит остальное парсим :
            unsigned int output = last.toFloat();
            // И проверяем  на допустимость (возможный диапазон)
            // переменная target была объявлена заранее, ее и устанавливаем, если
            // в пределах допустимого:
            ((output >= 0) && (output <= 255)) ? Output = output : 1==1;
            //отправка на экран
            Serial.print((String)"settings.ot.val=\""+Output+"\""+char(255)+char(255)+char(255));
            break;
        }
        case SETPOINT: 
        {
            // Значит остальное парсим :
            unsigned int setpoint = last.toFloat();
            // И проверяем  на допустимость (возможный диапазон)
            // переменная target была объявлена заранее, ее и устанавливаем, если
            // в пределах допустимого:
            ((setpoint >= 0) && (setpoint <= 2000)) ? Setpoint = setpoint : 1==1;
             //отправка на экран
            Serial.print((String)"page0.sp.txt=\""+"Setpoint="+Setpoint+"\""+char(255)+char(255)+char(255));
            break;
        }
        case KOEFF_PROPORTIONAL: 
        {
            // Значит остальное парсим на float:
            float new_kp = last.toFloat();
            // И проверяем  на допустимость (возможный диапазон)
            // переменная target была объявлена заранее, ее и устанавливаем, если
            // в пределах допустимого:
            Kp = new_kp ;
            myPID.SetTunings(Kp, Ki);
            //отправка на экран
            Serial.print((String)"page0.kp.txt=\""+"Kp="+Kp+"\""+char(255)+char(255)+char(255)); 
            break;
        }
        case KOEFF_INTEGRAL :
        {
            // Значит остальное парсим на float:
            float new_ki = last.toFloat();
            // И проверяем  на допустимость (возможный диапазон)
            // переменная target была объявлена заранее, ее и устанавливаем, если
            // в пределах допустимого:
            Ki = new_ki ;
            myPID.SetTunings(Kp, Ki);
            //отправка на экран
            Serial.print((String)"page0.ki.txt=\""+"Ki="+Ki+"\""+char(255)+char(255)+char(255)); 
            break;
        }
        
        case TEMP_POINT  :
        {
            // наше сообщение (пример) 1,20,600:2,40,300:3,60,300:4,0,0:
            String value = ""; 
            int point_number = 0;
            int parametrs_num = 0;
            int i;
            for(i = 0; i < last.length(); i++ )
            {   
                value += last[i];
                if(last[i] == ',')
                {
                    if(parametrs_num == 0)//номер точки
                    {
                        point_number = value.toInt();
                        parametrs_num ++;
                        value = ""; //удаляем значение
                        continue;
                    }
                    if(parametrs_num == 1)//время
                    {
                        time_point[point_number-1] = value.toInt();
                        parametrs_num = 0; // достигли max-1 параметра
                        value = ""; //удаляем значение
                        continue;
                    }
                }
                if(last[i] == ':')
                {
                    temp_point[point_number-1] = value.toInt(); //записываем температуру
                    if(point_number == 1) //записываем шаг времени
                    time_step[point_number-1] = (60*time_point[point_number-1])/(temp_point[point_number-1]-Input); //1 шаг времени
                    time_step[point_number-1] = (60*(time_point[point_number-1]-time_point[point_number-2]))/(temp_point[point_number-1]-temp_point[point_number-2]); //последующие шаги времени
                    value = ""; //удаляем значение            
                }
            } 
            break;
        }
        case POINT_SWITCH :  //включение новой программы  слежения за заданной температурой
        {
            if(last.toInt() == ON)
            {
                t_ALG = now();
                flag_point_switch = 1;
                Setpoint = Input ; // записываем текущую температуру
                interval = 0; //возвращение на 1-ый интервал
            }
            else
            flag_point_switch = 0;
            break;
        }
        
        case POINT_RUN :  // включение и отсановка слежения за заданной температурой
        {
            if(last.toInt() == ON)//алгоритм работает
            flag_point_switch = 1;
            else //пауза
            {            
                flag_point_switch = 0;
                time_step[interval] = 60*(time_point[interval]/(temp_point[interval]-Input)); //пересчет шага
                Setpoint = Input ; // записываем текущую температуру, с неё начинаем
            }
            
            break;
        }
        
        case RETURN_INTERVAL :  // включение и отсановка слежения за заданной температурой
        {
            interval = last.toInt()-1;
            time_step[interval] = 60*(time_point[interval]/(temp_point[interval]-Input)); //пересчет шага
            Setpoint = Input ; // записываем текущую температуру, с неё начинаем
            flag_first_run_interval = 0; //так как теперь мы запускаем интервал заново
            break;
        }
        
        
        
    }
}
