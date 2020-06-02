/*
Ogureckiy Dmitriy 
ogureckiy98@mail.ru
*/
//проблема с расчетом шага при температуре большей температуры установочной, шаг как будто слишком маленький и setpoint очень быстро изменяется
#include "stove.h" //все основные определения

void setup() 
{

   Serial.begin(115200);
   Serial.print("baud=115200ÿÿÿ"); //установка скорости экрана
   t_MCU = now(); //записываем время перового запуска микроконтроллера
   
   //turn the PID on
  //при первом запуске происходит также инициализация Initialize()
  myPID.SetMode(MANUAL);
  myPID.SetSampleTime(SAMPLE_TIME);
  myPID.SetOutputLimits(0, 1023);
  
   //------PWM----------------
  Timer3.initialize((long) SAMPLE_TIME * 1000);         // инициализировать timer1, и установить период равный периоду вызова ПИД
  Timer3.pwm( PIN_OUTPUT,  Output );                // задать шим сигнал  с  коэффициентом заполнения 

  // Создание объекта MAX31856 с определением пинов
  temperature = new MAX31856(SDI, SDO, CS, SCK);
  // Инициализация решистров
  temperature->writeRegister(REGISTER_CR0, CR0_INIT);
  temperature->writeRegister(REGISTER_CR1, CR1_INIT);
  temperature->writeRegister(REGISTER_MASK, MASK_INIT);
  // Ожидание отправки всех байтов
  delay(200);

   // инициализация массива окна усредения (инициализируем первые 4 значений, т.к. 5 мы измерим сразу
   //------------------------------------------------
  while ( index < (numReadings-1))
  {
    readings[index] =  temperature->readThermocouple(CELSIUS);
    total= total + readings[index];  
    index ++;
  }
  readings[index]=0; // значение последенго показания равно 0
  //так как в основном алгоритме мы вычитаем самое раннее значения из окна усреднения
  //---------------------------------------------------

    //отправка на экран необходимых данны 
    Serial.print((String)"debug.kp.txt=\""+"Kp="+Kp+"\""+char(255)+char(255)+char(255));
    delay(50); 
    Serial.print((String)"debug.ki.txt=\""+"Ki="+Ki+"\""+char(255)+char(255)+char(255)); 
    delay(50);
    Serial.print((String)"debug.sp.txt=\""+"Setpoint="+Setpoint+"\""+char(255)+char(255)+char(255));
    delay(50);
    
}

void loop () 
{  

  if((millis() - last_time) >= SAMPLE_TIME  )
  {
    movingAverage();
   
    myPID.Compute(); 

    checkMAX31856();

    Timer3.setPwmDuty(PIN_OUTPUT,  Output); //выставляем скважность выходного сигнала

    jumpderivative();
    
    senddata();

    last_time = millis();
  }

    reachingSetpoint();

    exitpause();

    changeLinearly();
    
    readNextioncommand();
}

void checkCommand(String ins) //парсинг сообщение от экрана
{
    // У нас информация от Nextion состоит из двух частей,
    // первая буква - идентификатор действия
    // ловим его:
    String command = ins.substring(0,1); //0 - позиция символа 1 - число символов
    // А все остальное - данные:
    String last = ins.substring(1);
    switch (command.toInt())
    {
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
            ((output >= 0) && (output <= 1023)) ? Output = output : 1==1;
            //отправка на экран
            Serial.print((String)"debug.ot.val=\""+Output+"\""+char(255)+char(255)+char(255));
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
            Serial.print((String)"debug.ki.txt=\""+"Ki="+Ki+"\""+char(255)+char(255)+char(255)); 
            break;
        }
        
        case TEMP_POINT  :
        {
            // наше сообщение (пример) 1,20,600:2,40,300:3,60,300:4,0,0:           
            String value = ""; //проверяемый символ(строка)
            int point_number = 0; //номер точки
            int parametrs_num = 0; //номер параметра 
            //(0-номер точки,1-время в мин,2-температура)
            int i;//счетчик
            for(i = 0; i < last.length(); i++ )//проходим по всей строке
            {   
                value += last[i];//запись следующего символа
                if(last[i] == ',')//запятая в роли разделителя
                {
                    if(parametrs_num == 0)//номер точки
                    {
                        point_number = value.toInt();//запись номера точки
                        parametrs_num ++;//переход следующий парметр
                        value = ""; //удаляем значение
                        continue;
                    }
                    if(parametrs_num == 1)//время
                    {
                        time_point[point_number-1] = value.toInt();//запись времени
                        parametrs_num = 0; // достигли max-1 параметра
                        value = ""; //удаляем значение
                        continue;
                    }
                }
                if(last[i] == ':')
                {
                    temp_point[point_number - 1] = value.toInt(); //записываем температуру
                    //расчет шага времени
                    if (point_number == 1) //записываем шаг времени
                    {//расчёт шага изменения температуры
                        int16_t denominator = (temp_point[point_number - 1] - Input);
                        if (denominator != 0)
                        {
                            time_step[point_number - 1] = (60 * time_point[point_number - 1]);
                            time_step[point_number - 1] /= (denominator); //1 шаг времени
                            time_step[point_number - 1] = abs(time_step[point_number - 1]);
                        }
                        else
                            time_step[point_number - 1] = 0;//если температура постоянна на участке
                                                             //шаг времени = 0
                    }
                    else
                    {
                        int16_t denominator = (temp_point[point_number - 1] - temp_point[point_number - 2]);
                        if (denominator != 0)
                        {
                            time_step[point_number - 1] = 60 * (time_point[point_number - 1]);
                            time_step[point_number - 1] /= denominator;
                            time_step[point_number - 1] = abs(time_step[point_number - 1]);  //последующие шаги времени 
                        }
                        else
                            time_step[point_number - 1] = 0;//если температура постоянна на участке
                                                            //шаг времени = 0
                    }
                    value = ""; //удаляем значение            
		                     
                }
            }
            break;
        }
        case POINT_SWITCH :  //8 включение новой программы  слежения за заданной температурой
        {//сообщение  действие = "номер программы" 
                t_ALG = now(); // запись времени начала запуска  алгоритма 
                Serial.print((String)"main.programm.val="+last.toInt()+char(255)+char(255)+char(255));//вывод номера программы
                flag_point_switch = 1;
                Setpoint = Input ; // записываем текущую температуру
                interval = 0; //возвращение на 1-ый интервал
                Serial.print((String)"cle 24,0"+char(255)+char(255)+char(255)); //очистить график
            break;
        }
        
        case POINT_RUN :  //9 пауза и выход из паузы слежения за заданной температурой 
        {
            if(last.toInt() == ON)//выход из паузы
            {
                myPID.SetMode(AUTOMATIC);
                flag_pause_exit = 1; // флаг выхода из паузы (разрешение проверки условий выхода из паузы)
                //операции перед выходом из паузы
                flag_reaching_Setpoint = 0; //разрешение операции перед выходом из паузы : достижение установленного значение 
            }
            else //пауза
            {   
                if(flag_point_switch == 1)
                { //вход в паузу возможен только, если алгоритм уже работает    
                    myPID.SetMode(MANUAL); 
                    Timer3.setPwmDuty(PIN_OUTPUT,  1023); //закрваем семистр, выключаем нагрев  
                    flag_point_switch = 0; //выключаем линейное изменение по функции
                    t_ALG_last_run =t_ALG_last_run + now() - t_ALG; // сохранение время работы алгоритма
                    t_INT_last_run =t_INT_last_run + now() - t_INT; // сохранение время работы интервала
                }
            }
            
            break;
        }
        
        case RETURN_INTERVAL :  //10 возвращение на начало какого-либо интервала 
        {
            //условия для возвращения
            if(flag_point_switch == 1)//возвращение возможно только, если алгоритм уже работает
            {
                flag_reaching_Setpoint = 0; //разрешение операции перед возвращением на начало интервала : достижение установленного значение
                interval = last.toInt(); //запись номера интервала 0-8
                Setpoint = temp_point[interval] ; // записываем температуру начала интервала 
                flag_first_run_interval = 0 ;//перед запуском интервала нужно проделать необходимые операции
                interval++; //начинать быдем с следующего интервала
                //если была нажата первая точка , то в Setpoint будет записано значение температуры  1 точки  
                //и далее после достижения этой температуры, будут выполняться последующие интервалы
            }
            break;
        }  
        case INTERVAL_HIGHLIGHING :  //11 подсвечивание интервала
        {
            if(last.toInt() == 1)// вызвано из окна с первыми 5 точками
            {
                if(interval < 5)
                {
                    int i = interval*8+1;
                    int max_id = (interval+1)*8;
                    for(i; i <= max_id  ; i++)
                    {
                        Serial.print((String)"points_1_5.b["+i+"].bco=64528"+char(255)+char(255)+char(255)); //поменять цвет
                    }
                }
            }
            else//вызвано из окна с последними 5 точками
            {
                if(interval > 4)
                {
                    int number_line = interval-4; 
                    int i = number_line*8+1;
                    int max_id = number_line*8;
                    for(i; i <= max_id  ; i++)
                    {
                        Serial.print((String)"points_5_10.b["+i+"].bco=64528"+char(255)+char(255)+char(255)); //поменять цвет
                    }
                }
            }
            break;
        }  
        case MISTAKE :  //12 вывод сообщения об ошибке
        {          
            Serial.print((String)"vis "+mistake_id+",1"+char(255)+char(255)+char(255)); //показать сообщение об ошибке
            break;
        }   
    }    
}


void checkMAX31856(void)//-------------------обработка оповещений от термоконтроллера----------------
{
    //-------------------обработка оповещений от термоконтроллера----------------
    if((Input == FAULT_OPEN)&&(flag_point_switch == 1)) // No thermocouple
    {//pause
        mistake_id = 1; //запись номера ошибки 
        Output = 1023 ; //выключаем нагрев
        myPID.SetMode(MANUAL); //ручной режим ПИД
        flag_point_switch = 0; //останавливаем работу на  интервале
        t_ALG_last_run =t_ALG_last_run + now() - t_ALG; // сохранение время работы алгоритма
        t_INT_last_run =t_INT_last_run + now() - t_INT; // сохранение время работы интервала
        Serial.print((String)"page FAULT_OPEN"+char(255)+char(255)+char(255)); //страница с оповещением
        Serial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255)); //изменение состояния индикатора паузы
        Serial.print((String)"main.ERROR.en=1"+char(255)+char(255)+char(255)); //включаем мигание экрана
    }
    if((Input == FAULT_VOLTAGE)&&(flag_point_switch == 1)) //  Under/over voltage error.  Wrong thermocouple type?
    {//pause
        mistake_id = 2;//запись номера ошибки
        Output = 1023 ; //выключаем нагрев
        myPID.SetMode(MANUAL); //ручной режим ПИД
        flag_point_switch = 0;//останавливаем работу на  интервале
        t_ALG_last_run =t_ALG_last_run + now() - t_ALG; // сохранение время работы алгоритма
        t_INT_last_run =t_INT_last_run + now() - t_INT; // сохранение время работы интервала
        Serial.print((String)"page FAULT_VOLTAGE"+char(255)+char(255)+char(255)); //страница с оповещением
        Serial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255)); //изменение состояния индикатора паузы
        Serial.print((String)"main.ERROR.en=1"+char(255)+char(255)+char(255)); //включаем мигание экрана
    }
    if((Input == NO_MAX31856)&&(flag_point_switch == 1)) // MAX31856 not communicating or not connected
    {//pause
        mistake_id = 3;//запись номера ошибки
        Output = 1023 ; //выключаем нагрев
        myPID.SetMode(MANUAL); //ручной режим ПИД
        flag_point_switch = 0;//останавливаем работу на  интервале
        t_ALG_last_run =t_ALG_last_run + now() - t_ALG; // сохранение время работы алгоритма
        t_INT_last_run =t_INT_last_run + now() - t_INT; // сохранение время работы интервала
        Serial.print((String)"page NO_MAX31856"+char(255)+char(255)+char(255)); //страница с оповещением
        Serial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255)); //изменение состояния индикатора паузы
        Serial.print((String)"main.ERROR.en=1"+char(255)+char(255)+char(255)); //включаем мигание экрана
    }
    if((Input == FAULT_OPEN)||(Input == FAULT_VOLTAGE )||(Input == NO_MAX31856)) { Input = 0 ;}; //вывод нулевой температуры
    if((Input != FAULT_OPEN)&&(Input != FAULT_VOLTAGE )&&(Input != NO_MAX31856))
    Serial.print((String)"main.ERROR.en=0"+char(255)+char(255)+char(255)); //выключаем мигание экрана, если нет оповищения об ошибке
    //---------------------------------------------------------------------------
}

void readNextioncommand()// обработка приходящих данных
{
    static String  inStr = ""; // Это будет приемник информации от Nextion (здесь только //1- байт номера команды /2- байт номера действия или данные)
    static bool serialReadFlag = false; // А это флаг появление сообщения.

   /******************************************************************/
    // обработка приходящих данных
    if (Serial.available()) 
    {
        uint8_t inn = Serial.read(); // читаем один байт
        if(serialReadFlag) 
        { // Если установлен флаг приема - действуем
            if(inn == 59)// ASCII : ";" // Находим конец передачи ";"
            {     
                if(inStr.length() > 0) // Проверяем длину сообщения и отправляем в "переработку"
                { 
                    checkCommand(inStr); // В этой функции будем парсить сообщение
                }
                serialReadFlag = false; // Сбрасываем флаг приема
            }
            else // А это нормальный прием
            { 
                inStr += (char)inn; // Считываем данные
            }
        }
        else 
        { // А здесь отлавливается начало передачи от Nextion
            if(inn == 35) // ASCII : "#"
            { 
                serialReadFlag = true; // После # начинаем чтение при следующем заходе
                inStr = ""; // Но до этого очистим стринг приема
            }
        }
    }
    /**************************************************************************/
}

void changeLinearly(void)//алгоритм слежения за целевой функцией
{
    if(flag_point_switch) //алгоритм слежения за целевой функцией
    {   
        if(flag_reaching_Setpoint == 1) //включаем только при достижении температуры установленного значения
        {
            for(int i = 0; i <= 9; i++)//проверяем на каком мы интервале находимся
            {   
                if(interval == i)
                {
                    if(flag_first_run_interval == 0) //операции перед включением интервала
                    {  
                        flag_first_run_interval = 1 ; //интервал запущен
                        t_INT = now(); //запись времени первого запуска интервала 
                        t_INT_last_run = 0 ;
                        t_INT_run = t_INT_last_run + now() - t_INT;
                        int intreval_1 = interval + 1 ;                                                    //вывод номера интервала
                        Serial.print((String)"main.point.val="+intreval_1+char(255)+char(255)+char(255));//вывод номера интервала 
                        //проверка больше или меньше температура в начале участка, чем температура в конце участка
                        if(Input >= temp_point[i])
                        compare=1;
                        else
                        compare=0; 
                        lastsecond = millis();
                    }
                    if(time_step[i] != 0) //линейное изменение температуры
                    {
                        uint32_t time_after_step = millis()-lastsecond ; //время, пройденное после последнего шага (мс)
                        if(  time_after_step >= (uint32_t)(time_step[i]*1000)  ) // прошло время шага 
                        {   
                            if(compare==0) //если нужно увеличивать температуру, то идем вверх на 1 градус 
                            Setpoint = Setpoint+1;
                            else ////если нет, то идём  вниз
                            Setpoint = Setpoint-1;     
                            if( get_minute(t_INT_run) >= time_point[i])//достижение установленного времени работы интервала
                            {
                                Setpoint = temp_point[i];//поддерживаем установленное значения температуры
                                if(((compare==1)&&(Input <= temp_point[i]))||((compare==0)&&(Input >= temp_point[i])))
                                {
                                    flag_first_run_interval = 0 ; //интервал закончен, разрешение операций перед включением интервала 
                                    if(i==9)
                                    { // то есть закончился алгооритм
                                        flag_point_switch = 0; // запрещаем алгоритму вызываться во избежания повторения алгоритма
                                                            //для повторного вызова нужно перезапустить алгоритм через экран 
                                    }
                                    interval++;//переход на следующий интервал по достижению установленного времени и температуры
                                }
                            }
                            lastsecond = millis() ;
                        }
                    }
                    else //температура постоянна на участке
                    {
                        //Setpoint не изменяется, так как он установлен в предыдщем интервале, либо при запуске алгоритма
                        if( get_minute(t_INT_run) >= time_point[i])//достижение установленного времени работы интервала
                        {
                            flag_first_run_interval = 0 ; //интервал закончен, разрешение операций перед включением интервала 
                            if(i==9)
                            { // то есть закончился алгооритм
                                flag_point_switch = 0; // запрещаем алгоритму вызываться во избежания повторения алгоритма
                                                    //для повторного вызова нужно перезапустить алгоритм через экран 
                            }
                            interval++;//переход на следующий интервал по достижению установленного времени и температуры
                        }
                    }
                    break;
                }
            }
        }
        
  
    }
}

void reachingSetpoint(void)// проверка: достигли ли мы установленного значения?
{
    if((flag_reaching_Setpoint == 2)||(flag_reaching_Setpoint == 0)) // проверка: достигли ли мы установленного значения?
    {
        if(flag_reaching_Setpoint == 0)//первый запуск участка кода
        {
        //проверка больше или меньше температура в начале участка, чем установочное значение
                    if(Input >= Setpoint)
                    compare=1;
                    else
                    compare=0; 
                    flag_reaching_Setpoint == 2;
        }
        if(((compare==1)&&(Input <= Setpoint))||((compare==0)&&(Input >= Setpoint))) //достижение последнего установочного значения 
        {
            flag_reaching_Setpoint = 1; // достигли
        }
    }
}

void exitpause(void)//проверка выполнения операций перед выходом из паузы
{
    if(flag_pause_exit == 1) //проверка выполнения операций перед выходом из паузы
    {
        if(flag_reaching_Setpoint == 1) //достигли установленного значения
        {
           t_ALG = now();//перезаписываем время последнего включения алгоритма 
           t_INT = now();//и интервала
           flag_point_switch = 1; //разрешение выполнения алгоритма
           flag_pause_exit = 0; //выход из паузы осуществлен 
        }
    }
}

void jumpderivative(void)//----------обработке экстренных скачков производной------------------
{
    //----------обработке экстренных скачков производной------------------
    int dInput; 
    dInput  = myPID.GetdInput(); //получаем значение производной (градус/сек)
    if((dInput>= MAX_DERIVATIVE)&&(flag_point_switch == 1))
    {//вход в паузу, если значение производной больше максимального и включен алгоритм
        Output = 1023 ; //выключаем нагрев
        myPID.SetMode(MANUAL); //ручной режим ПИД
        flag_point_switch = 0;//останавливаем работу на  интервале
        t_ALG_last_run =t_ALG_last_run + now() - t_ALG; // сохранение время работы алгоритма
        t_INT_last_run =t_INT_last_run + now() - t_INT; // сохранение время работы интервала
        Serial.print((String)"page jump_der"+char(255)+char(255)+char(255)); //страница с оповещением
        Serial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255)); //изменение состояния индикатора паузы
    }
    //--------------------------------------------------------------------
}

void senddata(void)//отправка  данных на экран
{
    //отправка  данных на экран
    Serial.print((String)"main.temp.val="+Input+char(255)+char(255)+char(255)); 
    delay(50);
    Serial.print((String)"debug.ot.val="+Output+char(255)+char(255)+char(255));
    delay(50);
    Serial.print((String)"debug.kp.txt=\""+"Kp="+Kp+"\""+char(255)+char(255)+char(255));
    delay(50); 
    Serial.print((String)"debug.ki.txt=\""+"Ki="+Ki+"\""+char(255)+char(255)+char(255)); 
    delay(50);
    Serial.print((String)"debug.sp.txt=\""+"Setpoint="+Setpoint+"\""+char(255)+char(255)+char(255));
    delay(50);
    uint8_t temp = Input*180/500; 
    Serial.print((String)"add 7,0,"+temp+char(255)+char(255)+char(255)); //рисуем график
    if(flag_point_switch)
    {
        //отправка времени с момента запуска алгоритма
        t_ALG_run = t_ALG_last_run + now() - t_ALG;//расчет времени работы алг. и инт.
        t_INT_run = t_INT_last_run + now() - t_INT;//------------------------------
        Serial.print((String)"main.day.val="+get_day(t_ALG_run)+char(255)+char(255)+char(255)); //(0-inf)
        Serial.print((String)"main.minute1.val="+minute(t_ALG_run)+char(255)+char(255)+char(255)); //(0-60)
        Serial.print((String)"main.hour1.val="+hour(t_ALG_run)+char(255)+char(255)+char(255)); //(0-24)
        //отправка времени с момента запуска интервала
        Serial.print((String)"main.day2.val="+get_day(t_INT_run)+char(255)+char(255)+char(255));//(0-inf)
        Serial.print((String)"main.hour2.val="+hour(t_INT_run)+char(255)+char(255)+char(255));//(0-24)
        Serial.print((String)"main.minute2.val="+minute(t_INT_run)+char(255)+char(255)+char(255)); //(0-60)      
    }
}

void  movingAverage(void)//скользящее среднее
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
}
