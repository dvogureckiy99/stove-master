/*
Ogureckiy Dmitriy 
ogureckiy98@mail.ru
*/
#include "stove.h" //все основные определения

void setup() 
{
   // Serial: 0 (rx) и 1 (tx), debugSerial: 19 (rx) 18 (tx)
   Serial.begin(SERIAL_BAUD);
   debugSerial.begin(SERIAL_BAUD);
   String baud = "baud="         ;
   baud += (String)SERIAL_BAUD   ;
   baud += "ÿÿÿ"                 ;
   Serial.print(baud)            ; //установка скорости экрана
   //t_MCU = now();                //записываем время перового запуска микроконтроллера
   
  //инициализация флагов
  flags.flag_pause_exit         = 0;
  flags.flag_first_run_interval = 0;
  flags.flag_point_switch       = 0;
  flags.flag_reaching_Setpoint  = 1;
  flags.flag_true_init          = 0; 
  //инициализация ошибок
  mistake.mistake_2             = 0;
  mistake.mistake_id            = 0;
  
   //инициализация SD-card
   pinMode(CS_SD, OUTPUT);
   init_SD();

   //turn the PID on
  //при первом запуске происходит также инициализация Initialize()
  myPID.SetMode(MANUAL);
  myPID.SetSampleTime(SAMPLE_TIME);
  myPID.SetOutputLimits(0, MINIMUM_HEATING);
  
   //------PWM----------------
  Timer3.initialize((long) SAMPLE_TIME * 1000);     // инициализировать timer1, и установить период равный периоду вызова ПИД
  Timer3.pwm( PIN_OUTPUT,  Output );                // задать шим сигнал  с  коэффициентом заполнения 

  // Создание объекта MAX31856 с определением пинов
  temperature = new MAX31856(SDI, SDO, CS_IC, SCK);
  // Инициализация регистров
  temperature->writeRegister(REGISTER_CR0, CR0_INIT);
  temperature->writeRegister(REGISTER_CR1, CR1_INIT);
  temperature->writeRegister(REGISTER_MASK, MASK_INIT);
  delay(200);// Ожидание отправки всех байтов

   // инициализация массива окна усредения (инициализируем первые значений кроме последнего для того , чтобы выводилась корректная температура)
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
}

void loop () 
{  
    if((now() - last_time_PI) >= SAMPLE_TIME_PI  )
    {
        movingAverage();

        myPID.Compute(); 

        Timer3.setPwmDuty(PIN_OUTPUT,  Output); //выставляем скважность выходного сигнала

        jumpderivative();

        selection_of_PID_coefficients();
   
        last_time_PI = now();
    }
    
    if((now()-last_time_send_data) >= SAMPLE_TIME_SEND_DATA )
    {
        senddata();
        if((!mistake.mistake_id))   //проверка наличия ошибок,вызывающих мигание экрана
        {
            
            Serial.print((String)"main.ERROR.en=0"+char(255)+char(255)+char(255)); //выключаем мигание экрана, если нет оповищения об ошибке
            if(!mistake.mistake_2) //проверка остальных ошибок ошибки SD-card
            {
                Serial.print((String)"main.mistake_vis.val=0"+char(255)+char(255)+char(255));
            }
        }
        last_time_send_data = now();
    }

    checkMAX31856();

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
    switch (command[0])
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
            ((output >= 0) && (output <= MINIMUM_HEATING)) ? Output = output : 1==1;
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
        
        case TEMP_POINT  : //(last - номер программы)
        {
            uint32_t time = 0;
            String value = "";
            struct point point1 ;
            int N = last.toInt();//номер программы
            for(int i = 1; i<11;i++) // i - номер точки
            {
                int address = (N-1)*100+(i-1)*10 ;       //формируем адресс точки в EEPROM
                EEPROM.get(address,point1);                
                
                value = point1.hour; //часы
                time += value.toInt();
                time *= 60; //перевод в минуты
                value = point1.minute; //минуты
                time += value.toInt();
                time_point[i-1] = time;//запись времени в мин
                time = 0;
                value = point1.temp;//темп.
                temp_point[i - 1] = value.toInt(); //записываем температуру

                //расчет шага времени
                if (i == 1) //записываем шаг времени
                {//расчёт шага изменения температуры
                    int16_t denominator = (temp_point[i - 1] - Input);
                    if (denominator != 0)
                    {
                        time_step[i - 1] = (60 * time_point[i - 1]);
                        time_step[i- 1] /= (denominator); //1 шаг времени
                        time_step[i - 1] = abs(time_step[i - 1]);
                    }
                    else
                        time_step[i - 1] = 0;   //если температура постоянна на участке
                                                //шаг времени = 0
                }
                else
                {
                    int16_t denominator = (temp_point[i - 1] - temp_point[i - 2]);
                    if (denominator != 0)
                    {
                        time_step[i- 1] = 60 * (time_point[i - 1]);
                        time_step[i - 1] /= denominator;
                        time_step[i - 1] = abs(time_step[i- 1]);  //последующие шаги времени 
                    }
                    else
                        time_step[i - 1] = 0;           //если температура постоянна на участке
                                                        //шаг времени = 0
                }
            }        
            break;
        }
        case POINT_SWITCH :  //8 включение новой программы  слежения за заданной температурой
        {//сообщение  действие = "номер программы" 
                t_ALG = now(); // запись времени начала запуска  алгоритма 
                Serial.print((String)"main.programm.val="+last.toInt()+char(255)+char(255)+char(255));//вывод номера программы
                flags.flag_point_switch = 1;
                Setpoint = Input ; // записываем текущую температуру
                interval = 0; //возвращение на 1-ый интервал
            break;
        }
        
        case POINT_RUN :  //9 пауза и выход из паузы слежения за заданной температурой 
        {
            if(last.toInt() == ON)//выход из паузы
            {
                myPID.SetMode(AUTOMATIC);
                flags.flag_pause_exit = 1; // флаг выхода из паузы (разрешение проверки условий выхода из паузы)
                //операции перед выходом из паузы
                flags.flag_reaching_Setpoint = 0; //разрешение операции перед выходом из паузы : достижение установленного значение 
            }
            else //пауза
            {   
                if(flags.flag_point_switch == 1)
                { //вход в паузу возможен только, если алгоритм уже работает    
                    myPID.SetMode(MANUAL); 
                    Timer3.setPwmDuty(PIN_OUTPUT,  MINIMUM_HEATING); //закрваем семистр, выключаем нагрев  
                    flags.flag_point_switch = 0; //выключаем линейное изменение по функции
                    t_ALG_last_run =t_ALG_last_run + now() - t_ALG; // сохранение время работы алгоритма
                    t_INT_last_run =t_INT_last_run + now() - t_INT; // сохранение время работы интервала
                }
            }
            
            break;
        }
        
        case RETURN_INTERVAL :  //10 возвращение на начало какого-либо интервала 
        {
            //условия для возвращения
            if(flags.flag_point_switch == 1)//возвращение возможно только, если алгоритм уже работает
            {
                flags.flag_reaching_Setpoint = 0; //разрешение операции перед возвращением на начало интервала : достижение установленного значение
                interval = last.toInt(); //запись номера интервала 0-8
                Setpoint = temp_point[interval] ; // записываем температуру начала интервала 
                flags.flag_first_run_interval = 0 ;//перед запуском интервала нужно проделать необходимые операции
                interval++; //начинать быдем с следующего интервала
                //если была нажата первая точка , то в Setpoint будет записано значение температуры  1 точки  
                //и далее после достижения этой температуры, будут выполняться последующие интервалы
            }
            break;
        }  
        case INTERVAL_HIGHLIGHING :  //11 подсвечивание интервала
        {
            if(flags.flag_first_run_interval)
            {
                debugSerial.println("INTERVAL_HIGHLIGHING");
                if(last.toInt() == 1)// вызвано из окна с первыми 5 точками
                {
                    
                    if(interval < 5) //первый запуск равно 0
                    {
                        int i = interval*8+1; //1
                        int max_id = (interval+1)*8;//8
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
            }
            break;
        }  
        case MISTAKE :  //12 вывод сообщения об ошибке
        {          
            if(mistake.mistake_id)
            Serial.print((String)"vis "+mistake.mistake_id+",1"+char(255)+char(255)+char(255)); //показать сообщение об ошибке
            if(mistake.mistake_2)
            Serial.print((String)"vis 4,1"+char(255)+char(255)+char(255)); //показать сообщение об ошибке initialization SD-card failed!
            break;
        }   
        case INITIALIZATION_SD :  //инициализация SD-карты
        {          
            init_SD();
            break;
        } 
        case WRITE_POINT://операция записи на EEPROM данных конкретной точки программы
        {          
            String value = "" ;                     //проверяемый элемент
            value = last.substring(0,2);     //символы номера программы
            int N = value.toInt();
            value = last.substring(3,5);     //символы номера точки
            int P = value.toInt();
            struct point point1 ;
            int address = (N-1)*100+(P-1)*10 ;       //формируем адресс точки в EEPROM
            strncpy(point1.hour,(last.substring(5,9)).c_str(), 4);
            strncpy(point1.minute,(last.substring(9,11)).c_str(), 2);
            strncpy(point1.temp,(last.substring(11,15)).c_str(), 4);
            EEPROM.put(address,point1) ; 
           
            break;
        } 
        case READ_POINT :  //операция чтения с EEPROM и записи в HMI данных точки на страницу "set_points"
        {          
            int P = 0;//номер точки
            int N = 0;//номер программы
            String value = ""; //проверяемый символ(строка)
            for(int i = 0; i < last.length(); i++ )//проходим по всей строке
            {
                if(last[i] == ',')//запятая в роли разделителя
                {
                        N = value.toInt();//запись номера программы
                        value = ""; //удаляем значение
                        continue;
                }
                value += last[i];//запись следующего символа
            }
            P = value.toInt();//запись номера точки
            int address = (N-1)*100+(P-1)*10 ;       //формируем адресс точки в EEPROM
            struct point point1 ;
            EEPROM.get(address,point1);
            set_point_data(point1);//последовательная установка в поле дисплея на стр. set_points
            break;
        }
        case READ_POINTS :  //операция чтения точек программы с DS-карты и записи в HMI данных точек на страницах points...
        {          
            int P = 0;//номер точки
            int N = 0;//номер программы
            String value = ""; //проверяемый символ(строка)
            struct point point1;
            for(int i = 0; i < last.length(); i++ )//проходим по всей строке
            {
                if(last[i] == ',')//запятая в роли разделителя
                {
                        N = value.toInt();//запись номера программы
                        value = ""; //удаляем значение
                        continue;
                }
                value += last[i];//запись следующего символа
            }
            P = value.toInt();//запись номера точки
            int address = (N-1)*100;       //формируем адресс точки в EEPROM
            if(P) //считываем(устанавливаем) 6-10 строки
            {
                address+=50;//вычисляем новый адресс(для 6 точки)
                for(int i = 6; i<11;i++,address+=10)
                {
                    EEPROM.get(address,point1);
                    set_point_data2(point1, i);// отправляем данные в дисплей
                }
            }
            else //считываем(устанавливаем) первые 5 строк
            {
                address+=50;//вычисляем новый адресс(для 6 точки)
                for(int i = 1; i<6;i++,address+=10)
                {
                    EEPROM.get(address,point1);
                    set_point_data2(point1, i);// отправляем данные в дисплей
                }  
            break;
            }
        } 
        case SET_MAX_DERIVATIVE:
        {
            int max_der = last.toInt();
            if(max_der > 0)
            max_derivative = max_der;
            break;
        }
        case SET_MIN_DERIVATIVE:
        {
            int min_der = last.toInt();
            if(min_der < 0)
            min_derivative = min_der;
            break;
        }
        case GET_JUMP_DERIVATIVE:
        {
            Serial.print((String)"set_jump_der_2.max_der.val="+max_derivative+char(255)+char(255)+char(255));
            Serial.print((String)"set_jump_der_2.min_der.val="+min_derivative+char(255)+char(255)+char(255));
            break;
        }
    }    
}


void checkMAX31856(void)//-------------------обработка оповещений от термоконтроллера----------------
{
    //-------------------обработка оповещений от термоконтроллера----------------
    if((Input == FAULT_OPEN)) // No thermocouple
    {//pause
        mistake.mistake_id = 1; //запись номера ошибки 
        Output = MINIMUM_HEATING ; //выключаем нагрев
        myPID.SetMode(MANUAL); //ручной режим ПИД
        if(flags.flag_point_switch==1)
        {
            t_ALG_last_run =t_ALG_last_run + now() - t_ALG; // сохранение время работы алгоритма
            t_INT_last_run =t_INT_last_run + now() - t_INT; // сохранение время работы интервала
            flags.flag_point_switch = 0; //останавливаем работу на  интервале
        }
        Serial.print((String)"page FAULT_OPEN"+char(255)+char(255)+char(255)); //страница с оповещением
        Serial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255)); //изменение состояния индикатора паузы
        Serial.print((String)"main.ERROR.en=1"+char(255)+char(255)+char(255)); //включаем мигание экрана
        Serial.print((String)"main.mistake_vis.val=1"+char(255)+char(255)+char(255)); //появление кнопки с ошибками
    }
    if((Input == FAULT_VOLTAGE)) //  Under/over voltage error.  Wrong thermocouple type?
    {//pause
        mistake.mistake_id = 2;//запись номера ошибки
        Output = MINIMUM_HEATING ; //выключаем нагрев
        myPID.SetMode(MANUAL); //ручной режим ПИД
        if(flags.flag_point_switch==1)
        {
            t_ALG_last_run =t_ALG_last_run + now() - t_ALG; // сохранение время работы алгоритма
            t_INT_last_run =t_INT_last_run + now() - t_INT; // сохранение время работы интервала
            flags.flag_point_switch = 0; //останавливаем работу на  интервале
        }
        Serial.print((String)"page FAULT_VOLTAGE"+char(255)+char(255)+char(255)); //страница с оповещением
        Serial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255)); //изменение состояния индикатора паузы
        Serial.print((String)"main.ERROR.en=1"+char(255)+char(255)+char(255)); //включаем мигание экрана
        Serial.print((String)"main.mistake_vis.val=1"+char(255)+char(255)+char(255)); //появление кнопки с ошибками
    }
    if((Input == NO_MAX31856)) // MAX31856 not communicating or not connected
    {//pause
        mistake.mistake_id = 3;//запись номера ошибки
        Output = 1023 ; //выключаем нагрев
        myPID.SetMode(MANUAL); //ручной режим ПИД
        if(flags.flag_point_switch==1)
        {
            t_ALG_last_run =t_ALG_last_run + now() - t_ALG; // сохранение время работы алгоритма
            t_INT_last_run =t_INT_last_run + now() - t_INT; // сохранение время работы интервала
            flags.flag_point_switch = 0; //останавливаем работу на  интервале
        }
        Serial.print((String)"page NO_MAX31856"+char(255)+char(255)+char(255)); //страница с оповещением
        Serial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255)); //изменение состояния индикатора паузы
        Serial.print((String)"main.ERROR.en=1"+char(255)+char(255)+char(255)); //включаем мигание экрана
        Serial.print((String)"main.mistake_vis.val=1"+char(255)+char(255)+char(255)); //появление кнопки с ошибками
    }
    if((Input == FAULT_OPEN)||(Input == FAULT_VOLTAGE )||(Input == NO_MAX31856)) { Input = 0 ;}; //вывод нулевой температуры
    if((Input != FAULT_OPEN)&&(Input != FAULT_VOLTAGE )&&(Input != NO_MAX31856))
    {
        mistake.mistake_id = 0 ; //нет ошибки
    }
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
    if(flags.flag_point_switch) //алгоритм слежения за целевой функцией
    {   
        if(flags.flag_reaching_Setpoint == 1) //включаем только при достижении температуры установленного значения
        {
            for(int i = 0; i <= 9; i++)//проверяем на каком мы интервале находимся
            {   
                if(interval == i)
                {
                    if(flags.flag_first_run_interval == 0) //операции перед включением интервала
                    {   
                        flags.flag_first_run_interval = 1 ; //интервал запущен
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
                            if( get_minute(t_INT_run) >= time_point[i] )//достижение установленного времени работы интервала
                            {
                                Setpoint = temp_point[i];//поддерживаем установленное значения температуры
                                if(((compare==1)&&(Input <= temp_point[i]))||((compare==0)&&(Input >= temp_point[i])))
                                {
                                    flags.flag_first_run_interval = 0 ; //интервал закончен, разрешение операций перед включением интервала 
                                    if(i==9)
                                    { // то есть закончился алгооритм
                                        flags.flag_point_switch = 0; // запрещаем алгоритму вызываться во избежания повторения алгоритма
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
                            flags.flag_first_run_interval = 0 ; //интервал закончен, разрешение операций перед включением интервала 
                            if(i==9)
                            { // то есть закончился алгооритм
                                flags.flag_point_switch = 0; // запрещаем алгоритму вызываться во избежания повторения алгоритма
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
    if((flags.flag_reaching_Setpoint == 2)||(flags.flag_reaching_Setpoint == 0)) // проверка: достигли ли мы установленного значения?
    {
        if(flags.flag_reaching_Setpoint == 0)//первый запуск участка кода
        {
        //проверка больше или меньше температура в начале участка, чем установочное значение
                    if(Input >= Setpoint)
                    compare=1;
                    else
                    compare=0; 
                    flags.flag_reaching_Setpoint == 2;
        }
        if(((compare==1)&&(Input <= Setpoint))||((compare==0)&&(Input >= Setpoint))) //достижение последнего установочного значения 
        {
            flags.flag_reaching_Setpoint = 1; // достигли
        }
    }
}

void exitpause(void)//проверка выполнения операций перед выходом из паузы
{
    if(flags.flag_pause_exit == 1) //проверка выполнения операций перед выходом из паузы
    {
        if(flags.flag_reaching_Setpoint == 1) //достигли установленного значения
        {
           t_ALG = now();//перезаписываем время последнего включения алгоритма 
           t_INT = now();//и интервала
           flags.flag_point_switch = 1; //разрешение выполнения алгоритма
           flags.flag_pause_exit = 0; //выход из паузы осуществлен 
        }
    }
}

void jumpderivative(void)//----------обработке экстренных скачков производной------------------
{
    //----------обработке экстренных скачков производной------------------
    int dInput; 
    dInput  = myPID.GetdInput(); //получаем значение производной (градус/сек)
    if(flags.flag_point_switch) //включен алгоритм
    {
        if(dInput > max_derivative )
        {//вход в паузу, если значение производной больше максимального и включен алгоритм
            Output = MINIMUM_HEATING ;                                              //выключаем нагрев
            myPID.SetMode(MANUAL);                                                  //ручной режим ПИД
            flags.flag_point_switch = 0;                                            //останавливаем работу на  интервале
            t_ALG_last_run =t_ALG_last_run + now() - t_ALG;                         // сохранение время работы алгоритма
            t_INT_last_run =t_INT_last_run + now() - t_INT;                         // сохранение время работы интервала
            Serial.print((String)"page jump_der"+char(255)+char(255)+char(255));    //страница с оповещением
            Serial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255));   //изменение состояния индикатора паузы
        }
        if(dInput < min_derivative )
        {//вход в паузу, если значение производной меньше минимального и включен алгоритм
            Output = MINIMUM_HEATING ;                                              //выключаем нагрев
            myPID.SetMode(MANUAL);                                                  //ручной режим ПИД
            flags.flag_point_switch = 0;                                            //останавливаем работу на  интервале
            t_ALG_last_run =t_ALG_last_run + now() - t_ALG;                         // сохранение время работы алгоритма
            t_INT_last_run =t_INT_last_run + now() - t_INT;                         // сохранение время работы интервала
            Serial.print((String)"page jump_der"+char(255)+char(255)+char(255));    //страница с оповещением
            Serial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255));   //изменение состояния индикатора паузы
        }
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
    if(flags.flag_point_switch)
    {
        //отправка времени с момента запуска алгоритма
        t_ALG_run = t_ALG_last_run + now() - t_ALG;//расчет времени работы алг. и инт.
        t_INT_run = t_INT_last_run + now() - t_INT;//------------------------------
        Serial.print((String)"main.day.val="+get_day(t_ALG_run)+char(255)+char(255)+char(255));      //(0-inf)
        Serial.print((String)"main.minute1.val="+minute(t_ALG_run)+char(255)+char(255)+char(255));   //(0-60)
        Serial.print((String)"main.hour1.val="+hour(t_ALG_run)+char(255)+char(255)+char(255));       //(0-24)
        //отправка времени с момента запуска интервала
        Serial.print((String)"main.day2.val="+get_day(t_INT_run)+char(255)+char(255)+char(255));     //(0-inf)
        Serial.print((String)"main.minute2.val="+minute(t_INT_run)+char(255)+char(255)+char(255));   //(0-60) 
        Serial.print((String)"main.hour2.val="+hour(t_INT_run)+char(255)+char(255)+char(255));       //(0-24)      
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

void init_SD() //инициализация SD-карты
{
    //инициализация
    debugSerial.println("Initializing SD card...");
    if(SD.begin(CS_SD)) 
    {
         flags.flag_true_init++;
    }
    else
    {  
      if(flags.flag_true_init==0)
      {
        SD_CARD_FAILED();//карта не была раньше активирована
        return;  
      }
      else
      {
        return; //если карта была раньше активирована, то попытка повторной активации будет неуспешной, поэтому просто выходим из функции
      }
    }

    if(!SD.exists("selection_of_PID_coefficients"))   //нет папки для отображения результата настройки
    {
        if(SD.mkdir("selection_of_PID_coefficients")) //то создаем
        {
            flags.flag_true_init++;                   //если создана успешно
        }
    }
    
    if(flags.flag_true_init==2) //успешная иниц.
    {
        debugSerial.println("initialization done.");
        if(mistake.mistake_2)
        {   //показать, что SD-карта успешно инициализирована
            Serial.print((String)"page SD_CARD_FAILED"+char(255)+char(255)+char(255)); //страница с оповещением
            Serial.print((String)"vis t0,0"+char(255)+char(255)+char(255)); 
            Serial.print((String)"vis t1,1"+char(255)+char(255)+char(255));
            Serial.print((String)"vis b0,0"+char(255)+char(255)+char(255)); 
            mistake.mistake_2 = 0;
        }
    }
    else //неуспешная иниц.
    {
        SD_CARD_FAILED();
        flags.flag_true_init = 0;//обновляем флаг инициализации
    }
}

void SD_CARD_FAILED()//при неуспешной инициализации файла
{
    debugSerial.println("initialization failed!"); 
    mistake.mistake_2 = 1;//запись номера ошибки initialization SD-card failed!
    Serial.print((String)"main.mistake_vis.val=1"+char(255)+char(255)+char(255)); //появление кнопки с ошибками
}

void set_point_data(struct point point1) //установка параметров точки в дисплей на стр.set_points
{
    String value = ""; //проверяемый символ(строка)
    value = point1.hour ; //часы
    Serial.print((String)"set_points.t1.txt=\""+value+"\""+char(255)+char(255)+char(255));
    value = point1.minute; //минуты
    Serial.print((String)"set_points.t7.txt=\""+value+"\""+char(255)+char(255)+char(255));
    value = point1.temp;//темп.
    Serial.print((String)"set_points.t8.txt=\""+value+"\""+char(255)+char(255)+char(255));    
}

void set_point_data2(struct point point1, int s) //установка парметров точки в дисплей на стр. points... s-номер точки (1-10)
{
    String value = ""; //проверяемый символ(строка)
    int num = 0;//номер поля
    if(s>5)
    {
        s-=5; //поправка для дисплея (теперь s=1...5 всегда !!!)
    }
    value = point1.hour ; //часы
    num = 3+8*(s-1) ;
    Serial.print((String)"b["+num+"].txt=\""+value+"\""+char(255)+char(255)+char(255));
    value = point1.minute; //минуты
    num = 5+8*(s-1) ;
    Serial.print((String)"b["+num+"].txt=\""+value+"\""+char(255)+char(255)+char(255));
    value = point1.temp;//темп.
    num = 7+8*(s-1) ;
    Serial.print((String)"b["+num+"].txt=\""+value+"\""+char(255)+char(255)+char(255));    
}

void selection_of_PID_coefficients()
{
    
}