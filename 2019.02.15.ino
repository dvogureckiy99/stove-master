/*
Ogureckiy Dmitriy 
ogureckiy98@mail.ru
Убраны вспомогательные сообщения , выводимые на debugSerial для ускорения работы
*/
#include "stove.h" //все основные определения

void setup() 
{
   //  debugSerial: 19 (rx) 18 (tx) nextionSerial: 17 (rx) 16 (tx)
   nextionSerial.begin(SERIAL_BAUD);
   debugSerial.begin(SERIAL_BAUD);
   String baud = "baud="         ;
   baud += (String)SERIAL_BAUD   ;
   baud += "ÿÿÿ"                 ;
   nextionSerial.print(baud)            ; //установка скорости экрана
   //t_MCU = now();                //записываем время перового запуска микроконтроллера
   
  //инициализация флагов
  flags.flag_pause_exit                          = 0;
  flags.flag_first_run_interval                  = 0;
  flags.flag_point_switch                        = 0;
  flags.flag_reaching_Setpoint                   = 1;
  flags.flag_true_init_SD                        = 0; 
  flags.flag_selection_of_PID_coefficients       = 0;
  flags.flag_start_selection_of_PID_coefficients = 0;
  flags.flag_save_data_on_SD                     = 0;
  flags.flag_on_page_graph                       = 0;
  //инициализация ошибок
  mistake.mistake_2                              = 0;
  mistake.mistake_id                             = 0;
  
   //инициализация SD-card
   pinMode(CS_SD, OUTPUT);
   init_SD();

   //turn the PID on
  //при первом запуске происходит также инициализация Initialize()
  myPID.SetMode(MANUAL);
  myPID.SetSampleTime(SAMPLE_TIME_PI);
  myPID.SetOutputLimits(0, 1023);
  
   //------PWM----------------
  Timer3.initialize((long) SAMPLE_TIME_PI * 1000000);     // инициализировать timer1, и установить период равный периоду вызова ПИД
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
    if((now() - t_last_time_PI) >= SAMPLE_TIME_PI  )
    {
        movingAverage();

        myPID.Compute(); 

        Timer3.setPwmDuty(PIN_OUTPUT,  Output); //выставляем скважность выходного сигнала

        jumpderivative();

        start_selection_of_PID_coefficients();

        selection_of_PID_coefficients();

        save_temp_for_graph();

        plotting();
   
        t_last_time_PI = now();
    }
    
    if((now()-t_last_time_send_data) >= SAMPLE_TIME_SEND_DATA )
    {
        senddata();
        if((!mistake.mistake_id))   //проверка наличия ошибок,вызывающих мигание экрана
        {
            
            nextionSerial.print((String)"main.ERROR.en=0"+char(255)+char(255)+char(255)); //выключаем мигание экрана, если нет оповищения об ошибке
            if(!mistake.mistake_2) //проверка остальных ошибок ошибки SD-card
            {
                nextionSerial.print((String)"main.mistake_vis.val=0"+char(255)+char(255)+char(255));
            }
        }
        
        t_last_time_send_data = now();
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
            nextionSerial.print((String)"debug.ot.val=\""+Output+"\""+char(255)+char(255)+char(255));
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
            nextionSerial.print((String)"debug.ki.txt=\""+"Ki="+Ki+"\""+char(255)+char(255)+char(255)); 
            break;
        }
        
        case TEMP_POINT  : //(last - номер программы)
        {
            //debugSerial.println("---operation TEMP_POINT--------");
            uint32_t time = 0;
            char value[4] = {0};
            char value1[2] = {0};
            struct point point1 ;
            int N = last.toInt();//номер программы
            for(int i = 1; i<11;i++) // i - номер точки
            {
                int address = (N-1)*100+(i-1)*10 ;       //формируем адресс точки в EEPROM
                EEPROM.get(address,point1);                                
                strncpy(value,point1.hour, 4);
                time += atoi(value);
                time *= 60; //перевод в минуты
                //debugSerial.print("hour=");
                //debugSerial.print(atoi(value));
                strncpy(value1,point1.minute, 2);
                time += atoi(value1);
                time_point[i-1] = time;//запись времени в мин
                time = 0;
                //debugSerial.print(":minute=");
                //debugSerial.print(atoi(value1));
                strncpy(value,point1.temp, 4);
                temp_point[i - 1] = atoi(value);; //записываем температуру
                //debugSerial.print(":temp=");
                //debugSerial.print(atoi(value));
                //debugSerial.println();
                
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
            //debugSerial.println("------------------------------------");       
            break;
        }
        case POINT_SWITCH :  //8 включение новой программы  слежения за заданной температурой
        {//сообщение  действие = "номер программы" 
                t_ALG = now(); // запись времени начала запуска  алгоритма 
                t_INT = now(); // запись времени начала запуска  интервала
                nextionSerial.print((String)"main.programm.val="+last.toInt()+char(255)+char(255)+char(255));//вывод номера программы
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
                //debugSerial.println("INTERVAL_HIGHLIGHING");
                if(last.toInt() == 1)// вызвано из окна с первыми 5 точками
                {
                    
                    if(interval < 5) //первый запуск равно 0
                    {
                        int i = interval*8+1; //1
                        int max_id = (interval+1)*8;//8
                        for(i; i <= max_id  ; i++)
                        {
                            nextionSerial.print((String)"points_1_5.b["+i+"].bco=64528"+char(255)+char(255)+char(255)); //поменять цвет
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
                            nextionSerial.print((String)"points_5_10.b["+i+"].bco=64528"+char(255)+char(255)+char(255)); //поменять цвет
                        }
                    }
                }
            }
            break;
        }  
        case MISTAKE :  //12 вывод сообщения об ошибке
        {          
            if(mistake.mistake_id)
            nextionSerial.print((String)"vis "+mistake.mistake_id+",1"+char(255)+char(255)+char(255)); //показать сообщение об ошибке
            if(mistake.mistake_2)
            nextionSerial.print((String)"vis 4,1"+char(255)+char(255)+char(255)); //показать сообщение об ошибке initialization SD-card failed!
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
            strncpy(point1.hour,(last.substring(6,10)).c_str(), 4);
            strncpy(point1.minute,(last.substring(11,13)).c_str(), 2);
            strncpy(point1.temp,(last.substring(14,18)).c_str(), 4);
            /*debugSerial.println("--operation WRITE_POINT----------");
            debugSerial.print("point1=");
                    debugSerial.write(point1.hour,4);
                    debugSerial.print(":");
                    debugSerial.write(point1.minute,2);
                    debugSerial.print(":");
                    debugSerial.write(point1.temp,4);
                    debugSerial.println();
                    debugSerial.println("-------------------------------");*/
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
            EEPROM.get(address,point1);/*
            debugSerial.println("--operation READ_POINT----------");
            debugSerial.print("point1=");
                    debugSerial.write(point1.hour,4);
                    debugSerial.print(":");
                    debugSerial.write(point1.minute,2);
                    debugSerial.print(":");
                    debugSerial.write(point1.temp,4);
                    debugSerial.println();
                    debugSerial.println("-------------------------------");*/
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
                //debugSerial.println("--operation READ_POINTS----------");
                address+=50;//вычисляем новый адресс(для 6 точки)
                for(int i = 6; i<11;i++,address+=10)
                {
                    EEPROM.get(address,point1);
                    /*debugSerial.print("point1=");
                    debugSerial.write(point1.hour,4);
                    debugSerial.print(":");
                    debugSerial.write(point1.minute,2);
                    debugSerial.print(":");
                    debugSerial.write(point1.temp,4);
                    debugSerial.print(",i=");
                    debugSerial.println(i);*/
                    
                    set_point_data2(point1, i);// отправляем данные в дисплей
                }
                //debugSerial.println("---------------------------");
            }
            else //считываем(устанавливаем) первые 5 строк
            {
                //debugSerial.println("--operation READ_POINTS----------");
                for(int i = 1; i<6;i++,address+=10)
                {
                    EEPROM.get(address,point1);
                    /*debugSerial.print("point1=");
                    debugSerial.write(point1.hour,4);
                    debugSerial.print(":");
                    debugSerial.write(point1.minute,2);
                    debugSerial.print(":");
                    debugSerial.write(point1.temp,4);
                    debugSerial.print(",i=");
                    debugSerial.println(i);
                    */
                    set_point_data2(point1, i);// отправляем данные в дисплей
                }  
                //debugSerial.println("-----------------------------");
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
            nextionSerial.print((String)"set_jump_der.max_der.val="+max_derivative+char(255)+char(255)+char(255));
            nextionSerial.print((String)"set_jump_der.min_der.val="+min_derivative+char(255)+char(255)+char(255));
            break;
        }
        case SELECTION_OF_PID_COEFFICIENTS:
        {
            if(last.toInt() == ON)
            {
                nextionSerial.print((String)"PIR_setup.progress_bar.val=0"+char(255)+char(255)+char(255)); 
                debugSerial.println("Run program the selection_of_PID_coefficients");
                debugSerial.println("Further");
                debugSerial.println("Setpoint=...;Sattling_time=...(or unstable system");
                String address = "";
                String address1 = "";
                flags.flag_point_switch = 0 ;//запрет изменения температуры по заданному алгоритму
                flags.flag_start_selection_of_PID_coefficients = 1; //разрешение на подготовку к выполнению алгоритма
                Setpoint = temp_start ; // перед настройкой необходимо вернуться к начальной температуре
                flags.flag_reaching_Setpoint = 0; //благодаря установлению нового задающего воздействия
                myPID.SetMode(AUTOMATIC);
                //--------------------создание необходимых файлов--------------------------------------
                //формируем адресс и открываем файл для записи температуры
                address += "PIR_reg/temp.txt";
                /*
                if(SD.exists(address))
                {
                    if(SD.remove(address))
                    {
                        debugSerial.print("successfully delete last file:");
                        debugSerial.println(address);
                    }
                    else
                    {
                        SD_CARD_FAILED();
                    }
                    
                }*/
                file_obj = SD.open(address,FILE_WRITE);//открытие файла, файл должен быть создан раннее
                if(file_obj)
                {
                    //debugSerial.print("successfully open file:");
                    //debugSerial.println(address);
                }else
                {
                    SD_CARD_FAILED();
                    //debugSerial.print("unsuccessfully open file:");
                    //debugSerial.println(address);
                }
                //открытие файла для записии времени
                address1 += "PIR_reg/time.txt";
                /*
                if(SD.exists(address1))
                {
                    if(SD.remove(address1))
                    {
                        debugSerial.print("successfully delete last file:");
                        debugSerial.println(address1);
                    }
                    else
                    {
                        {SD_CARD_FAILED();}
                    }
                    
                }*/
                file_obj1 = SD.open(address1,FILE_WRITE);//открытие файла, файл должен быть создан раннее
                if(file_obj1)
                {
                    //debugSerial.print("successfully open file:");
                    //debugSerial.println(address1);
                }else
                {
                    SD_CARD_FAILED();
                    //debugSerial.print("unsuccessfully open file:");
                    //debugSerial.println(address1);
                }
                //открытие файла для записии времени регулирования 
                /*
                if(SD.exists("PIR_reg/set_time.txt"))
                {
                    if(SD.remove("PIR_reg/set_time.txt"))
                    {
                        debugSerial.print("successfully delete last file:");
                        debugSerial.println("PIR_reg/set_time.txt");
                    }
                    else
                    {
                        {SD_CARD_FAILED();}
                    }
                    
                }*/
                file_obj2 = SD.open("PIR_reg/set_time.txt",FILE_WRITE);//открытие файла, файл должен быть создан раннее
                if(file_obj2)
                {
                    //debugSerial.print("successfully open file:");
                    //debugSerial.println("PIR_reg/set_time.txt");
                }else
                {
                    SD_CARD_FAILED();
                    //debugSerial.print("unsuccessfully open file:");
                    //debugSerial.println("PIR_reg/set_time.txt");
                }
                file_obj3 = SD.open("PIR_reg/Kp.txt",FILE_WRITE);//открытие файла, файл должен быть создан раннее
                if(file_obj2)
                {
                    //debugSerial.print("successfully open file:");
                    //debugSerial.println("PIR_reg/Kp.txt");
                }else
                {
                    SD_CARD_FAILED();
                    //debugSerial.print("unsuccessfully open file:");
                    //debugSerial.println("PIR_reg/Kp.txt");
                }
                file_obj4 = SD.open("PIR_reg/Ki.txt",FILE_WRITE);//открытие файла, файл должен быть создан раннее
                if(file_obj2)
                {
                    //debugSerial.print("successfully open file:");
                    ///debugSerial.println("PIR_reg/Ki.txt");
                }else
                {
                    SD_CARD_FAILED();
                    //debugSerial.print("unsuccessfully open file:");
                    //debugSerial.println("PIR_reg/Ki.txt");
                }
                //добавка 
                file_obj.print("New iteration.Kp=");file_obj.print(Kp);file_obj.print("Ki=");file_obj.println(Ki);
                file_obj.print("              temp_end=");file_obj.print(temp_end);file_obj.print("temp_start=");file_obj.println(temp_start);
                file_obj1.print("New iteration.Kp=");file_obj.print(Kp);file_obj.print("Ki=");file_obj.println(Ki);
                file_obj1.print("              temp_end=");file_obj.print(temp_end);file_obj.print("temp_start=");file_obj.println(temp_start);
                //---------------------------------------------------------------------------------------
            }else //last==OFF
            {
                if(flags.flag_selection_of_PID_coefficients||flags.flag_start_selection_of_PID_coefficients) //только если алгоритм работает, во избежание срабатывания при случайном нажатии
                {
                    flags.flag_selection_of_PID_coefficients = 0; //запрет на работу
                    flags.flag_start_selection_of_PID_coefficients = 0; //запрет на подготовку к выполнению алгоритма
                    flags.flag_reaching_Setpoint = 1; //показываем, как-будто мы достигли нужного значения
                    myPID.SetMode(MANUAL); 
                    Timer3.setPwmDuty(PIN_OUTPUT,  MINIMUM_HEATING); //закрваем симистор, выключаем нагрев
                }
            }       
            break;
        }
        case  SET_TEMP_MAX_MIN_US :
        {
            nextionSerial.print((String)"PIR_setup.n2.val="+temp_end+char(255)+char(255)+char(255));
            nextionSerial.print((String)"PIR_setup.n3.val="+temp_start+char(255)+char(255)+char(255));
            nextionSerial.print((String)"PIR_setup.n0.val="+time_unstable_system+char(255)+char(255)+char(255));
            break;
        } 
        case GET_TEMP_MAX_MIN_US :
        {
            debugSerial.println("----operation GET_TEMP_MAX_MIN_US-------") ;
            //формат last=id+value
            //id = 0-temp_end , 1-time_unstable_system, 2-temp_start
            String value = "";
            value = last.substring(0,1);
            if (value.toInt()==2) //2
            {
                value = last.substring(1);
                temp_start = value.toInt();
                debugSerial.print("temp_start=");
                debugSerial.println(temp_start); 
            }
            if (value.toInt()==0)//0
            {
                value = last.substring(1);
                temp_end = value.toInt();
                debugSerial.print("temp_end=");
                debugSerial.println(temp_end); 
            }
            if (value.toInt()==1)//1
            {
                value = last.substring(1);
                time_unstable_system = value.toInt();
                debugSerial.print("time_unstable_system=");
                debugSerial.println(time_unstable_system); 
            }debugSerial.println("------------------------------------------");
            break;
        }
        case SET_KP_KI_SETTLING_TIME:
        {
            debugSerial.println("----operation SET_KP_KI_SETTLING_TIME-------") ;
            for(int i = 0; i < 5 ; i++ )
            {
                debugSerial.print("i=");debugSerial.println(i);
                debugSerial.print("Kp=");
                debugSerial.println(Kp_Ki_settling_time[i].kp);
                nextionSerial.print((String)"kp"+(i+1)+".txt=\""+Kp_Ki_settling_time[i].kp+"\""+char(255)+char(255)+char(255));
                debugSerial.print("Ki=");
                debugSerial.println(Kp_Ki_settling_time[i].ki);
                nextionSerial.print((String)"ki"+(i+1)+".txt=\""+Kp_Ki_settling_time[i].ki+"\""+char(255)+char(255)+char(255));
                debugSerial.print("settling_time=");
                debugSerial.println(Kp_Ki_settling_time[i].settling_time);
                nextionSerial.print((String)"settling_time"+(i+1)+".val="+Kp_Ki_settling_time[i].settling_time+char(255)+char(255)+char(255));
            }debugSerial.println("------------------------------------------");
            break;
        }
        case SAVE_DATA_ON_SD_CARD :
        {
            if(last.toInt())
            {
                flags.flag_save_data_on_SD = 1;
                nextionSerial.print((String)"SD.val="+flags.flag_save_data_on_SD+char(255)+char(255)+char(255));
                nextionSerial.print((String)"page save_SD"+char(255)+char(255)+char(255));
                file_graph = SD.open("graph.txt", FILE_WRITE); 
                line = file_graph.size()/SIZE_SAVE_TEMP;
                line++;
            }
            else
            {
                nextionSerial.print((String)"SD.val="+flags.flag_save_data_on_SD+char(255)+char(255)+char(255));
                nextionSerial.print((String)"page save_SD"+char(255)+char(255)+char(255));
                flags.flag_save_data_on_SD = 0;
            }            
            break;
        }
        case SET_SD_STATUS :
        {
            if(last.toInt()==0) //вызов со стр. save_SD
            {
                nextionSerial.print((String)"SD.val="+flags.flag_save_data_on_SD+char(255)+char(255)+char(255));
            }
            break;
        }
        case PAGE_GRAPH  :
        {
            if(last.toInt())
            {
                flags.flag_on_page_graph = 1;
            }else{ flags.flag_on_page_graph = 0;}
            break;
        }
        case PI_STATUS:
        {
           nextionSerial.print((String)"bt0.val="+myPID.GetMode()+char(255)+char(255)+char(255));
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
        nextionSerial.print((String)"page FAULT_OPEN"+char(255)+char(255)+char(255)); //страница с оповещением
        nextionSerial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255)); //изменение состояния индикатора паузы
        nextionSerial.print((String)"main.ERROR.en=1"+char(255)+char(255)+char(255)); //включаем мигание экрана
        nextionSerial.print((String)"main.mistake_vis.val=1"+char(255)+char(255)+char(255)); //появление кнопки с ошибками
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
        nextionSerial.print((String)"page FAULT_VOLTAGE"+char(255)+char(255)+char(255)); //страница с оповещением
        nextionSerial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255)); //изменение состояния индикатора паузы
        nextionSerial.print((String)"main.ERROR.en=1"+char(255)+char(255)+char(255)); //включаем мигание экрана
        nextionSerial.print((String)"main.mistake_vis.val=1"+char(255)+char(255)+char(255)); //появление кнопки с ошибками
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
        nextionSerial.print((String)"page NO_MAX31856"+char(255)+char(255)+char(255)); //страница с оповещением
        nextionSerial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255)); //изменение состояния индикатора паузы
        nextionSerial.print((String)"main.ERROR.en=1"+char(255)+char(255)+char(255)); //включаем мигание экрана
        nextionSerial.print((String)"main.mistake_vis.val=1"+char(255)+char(255)+char(255)); //появление кнопки с ошибками
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
    if (nextionSerial.available()) 
    {
        uint8_t inn = nextionSerial.read(); // читаем один байт
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
                        nextionSerial.print((String)"main.point.val="+intreval_1+char(255)+char(255)+char(255));//вывод номера интервала 
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
            nextionSerial.print((String)"page jump_der"+char(255)+char(255)+char(255));    //страница с оповещением
            nextionSerial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255));   //изменение состояния индикатора паузы
        }
        if(dInput < min_derivative )
        {//вход в паузу, если значение производной меньше минимального и включен алгоритм
            Output = MINIMUM_HEATING ;                                              //выключаем нагрев
            myPID.SetMode(MANUAL);                                                  //ручной режим ПИД
            flags.flag_point_switch = 0;                                            //останавливаем работу на  интервале
            t_ALG_last_run =t_ALG_last_run + now() - t_ALG;                         // сохранение время работы алгоритма
            t_INT_last_run =t_INT_last_run + now() - t_INT;                         // сохранение время работы интервала
            nextionSerial.print((String)"page jump_der"+char(255)+char(255)+char(255));    //страница с оповещением
            nextionSerial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255));   //изменение состояния индикатора паузы
        }
    }
    //--------------------------------------------------------------------
}

void senddata(void)//отправка  данных на экран
{
    //отправка  данных на экран
    nextionSerial.print((String)"main.temp.val="+Input+char(255)+char(255)+char(255)); 
    delay(50);
    nextionSerial.print((String)"debug.ot.val="+Output+char(255)+char(255)+char(255));
    delay(50);
    nextionSerial.print((String)"debug.kp.txt=\""+"Kp="+Kp+"\""+char(255)+char(255)+char(255));
    delay(50); 
    nextionSerial.print((String)"debug.ki.txt=\""+"Ki="+Ki+"\""+char(255)+char(255)+char(255)); 
    delay(50);
    nextionSerial.print((String)"debug.sp.txt=\""+"Setpoint="+Setpoint+"\""+char(255)+char(255)+char(255));
    if(flags.flag_point_switch)
    {
        //отправка времени с момента запуска алгоритма
        t_ALG_run = t_ALG_last_run + now() - t_ALG;//расчет времени работы алг. и инт.
        t_INT_run = t_INT_last_run + now() - t_INT;//------------------------------
        nextionSerial.print((String)"main.day.val="+get_day(t_ALG_run)+char(255)+char(255)+char(255));      //(0-inf)
        nextionSerial.print((String)"main.minute1.val="+minute(t_ALG_run)+char(255)+char(255)+char(255));   //(0-60)
        nextionSerial.print((String)"main.hour1.val="+hour(t_ALG_run)+char(255)+char(255)+char(255));       //(0-24)
        //отправка времени с момента запуска интервала
        nextionSerial.print((String)"main.day2.val="+get_day(t_INT_run)+char(255)+char(255)+char(255));     //(0-inf)
        nextionSerial.print((String)"main.minute2.val="+minute(t_INT_run)+char(255)+char(255)+char(255));   //(0-60) 
        nextionSerial.print((String)"main.hour2.val="+hour(t_INT_run)+char(255)+char(255)+char(255));       //(0-24)      
    }
    if(flags.flag_selection_of_PID_coefficients)
    {
        time_t time = now()-t_new_setpoint;
        nextionSerial.print((String)"PIR_setup.minute2.val="+minute(time)+char(255)+char(255)+char(255));   //(0-60)
        nextionSerial.print((String)"PIR_setup.hour2.val="+get_hour(time)+char(255)+char(255)+char(255));       //(0-24)     
        uint8_t progress_bar = time*100/(time_unstable_system) ; 
        nextionSerial.print((String)"PIR_setup.progress_bar.val="+progress_bar+char(255)+char(255)+char(255)); 
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
    /*
    if(!SD.begin(CS_SD)) //не активировалась
    {
         //SD_CARD_FAILED();
        flags.flag_true_init_SD = 0;//обновляем флаг инициализации
    }*/
    if(SD.begin(CS_SD)) 
    {
         flags.flag_true_init_SD++;
    }
    else
    {  SD_CARD_FAILED();
    return;
      /*
      if(flags.flag_true_init_SD==0)
      {
        SD_CARD_FAILED();//карта не была раньше активирована
        return;  
      }
      else
      {
        return; //если карта была раньше активирована, то попытка повторной активации будет неуспешной, поэтому просто выходим из функции
      }*/
    }

    if(!SD.exists("PIR_reg"))   //нет папки для отображения результата настройки
    {
        SD.mkdir("PIR_reg"); //то создаем
    }
    
    if(flags.flag_true_init_SD==1) //успешная иниц.
    {
        debugSerial.println("initialization done.");
        if(mistake.mistake_2)
        {   //показать, что SD-карта успешно инициализирована
            nextionSerial.print((String)"page SD_CARD_FAILED"+char(255)+char(255)+char(255)); //страница с оповещением
            nextionSerial.print((String)"page SD_CARD_FAILED"+char(255)+char(255)+char(255)); //страница с оповещением
            nextionSerial.print((String)"vis t0,0"+char(255)+char(255)+char(255)); 
            nextionSerial.print((String)"vis t1,1"+char(255)+char(255)+char(255));
            nextionSerial.print((String)"vis b0,0"+char(255)+char(255)+char(255)); 
            mistake.mistake_2 = 0;
        }
        
    }
    else //неуспешная иниц.
    {
        SD_CARD_FAILED();
    }
}

void SD_CARD_FAILED()//при неуспешной инициализации файла
{
    debugSerial.println("initialization failed!"); 
    flags.flag_save_data_on_SD = 0; 
    nextionSerial.print((String)"save_SD.SD.val=0"+char(255)+char(255)+char(255)); //показать, что нет SD карты
    flags.flag_true_init_SD = 0;//обновляем флаг инициализации
    mistake.mistake_2 = 1;//запись номера ошибки initialization SD-card failed!
    nextionSerial.print((String)"page SD_CARD_FAILED"+char(255)+char(255)+char(255)); //страница с оповещением
    nextionSerial.print((String)"page SD_CARD_FAILED"+char(255)+char(255)+char(255)); //страница с оповещением
    nextionSerial.print((String)"main.mistake_vis.val=1"+char(255)+char(255)+char(255)); //появление кнопки с ошибками
}

void set_point_data(struct point point1) //установка параметров точки в дисплей на стр.set_points
{
    String value = ""; //проверяемый символ(строка)
    value = point1.hour ; //часы
    nextionSerial.print((String)"set_points.t1.txt=\""+value+"\""+char(255)+char(255)+char(255));
    value = point1.minute; //минуты
    nextionSerial.print((String)"set_points.t7.txt=\""+value+"\""+char(255)+char(255)+char(255));
    value = point1.temp;//темп.
    nextionSerial.print((String)"set_points.t8.txt=\""+value+"\""+char(255)+char(255)+char(255));    
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
    nextionSerial.print((String)"b["+num+"].txt=\""+value+"\""+char(255)+char(255)+char(255));
    value = point1.minute; //минуты
    num = 5+8*(s-1) ;
    nextionSerial.print((String)"b["+num+"].txt=\""+value+"\""+char(255)+char(255)+char(255));
    value = point1.temp;//темп.
    num = 7+8*(s-1) ;
    nextionSerial.print((String)"b["+num+"].txt=\""+value+"\""+char(255)+char(255)+char(255));    
}

void selection_of_PID_coefficients() //алгоритм подбора коэффициентов
{
    
    if(flags.flag_selection_of_PID_coefficients)
    {            
            static uint8_t flag_end_heat = 0;
            uint16_t settling_time = 0;
            time_t time_from_new_setpoint = now()-t_new_setpoint;
            file_obj.println(Input);                //температура
            file_obj1.println(time_from_new_setpoint);  //время в секундах с начала нагрева по алгоритму
            if(time_from_new_setpoint>=time_unstable_system) //система неустойчива
            {
                settling_time = INF ;
                file_obj2.println(INF) ;
                debugSerial.print("Setpoint=");debugSerial.print(Setpoint);debugSerial.print(";Sattling_time=INF");
            }
            if(equality_of_array_values(readings,numReadings, Setpoint)) //установившейся режим , т.к. все члены окна усреднения равны setpoint  
            { 
                settling_time = time_from_new_setpoint ;
                file_obj2.println(time_from_new_setpoint) ;
                debugSerial.print("Setpoint=");debugSerial.print(Setpoint);debugSerial.print(";Sattling_time=");
                debugSerial.println(time_from_new_setpoint);
            }

            //условие конца настройки
            if(flag_end_heat) 
            {
                nextionSerial.print((String)"page PIR_setup"+char(255)+char(255)+char(255)); //перейти на страницу
                nextionSerial.print((String)"PIR_setup.progress_bar.val=100"+char(255)+char(255)+char(255)); //полностью заполнить стутус-бар
                nextionSerial.print((String)"vis ready_status,1"+char(255)+char(255)+char(255)); //показать сообщение о завершении
                flags.flag_selection_of_PID_coefficients = 0 ; //запрет на работу
                myPID.SetMode(MANUAL); 
                Timer3.setPwmDuty(PIN_OUTPUT,  MINIMUM_HEATING); //закрываем симистор, выключаем нагрев
                //запись в память последних измерений
                
                //сдвигаем все значение вверх и далее всегда записываем в последний
                    for(int i = 0 ; i < 4; i++)
                    {
                        Kp_Ki_settling_time[i].kp = Kp_Ki_settling_time[i+1].kp ;
                        Kp_Ki_settling_time[i].ki = Kp_Ki_settling_time[i+1].ki ;
                        Kp_Ki_settling_time[i].settling_time = Kp_Ki_settling_time[i+1].settling_time ;
                    }
                Kp_Ki_settling_time[4].kp = Kp;
                Kp_Ki_settling_time[4].ki = Ki;
                Kp_Ki_settling_time[4].settling_time = settling_time;
                //обновляем страницу 
                nextionSerial.print((String)"page PIR_setup"+char(255)+char(255)+char(255));
                
                file_obj3.println(Kp);
                file_obj4.println(Ki);

                file_obj.close();
                file_obj1.close();
                file_obj2.close();
                file_obj3.close();
                file_obj4.close();

                flag_end_heat = 0;
            }

            //сохранение    
            static time_t t_last_save = 0; //последнее сохранение
            if((now()-t_last_save)>=SAVE_FILE )
            {
                file_obj.flush();
                file_obj1.flush();
                file_obj2.flush();
                file_obj3.flush();
                file_obj4.flush();
                t_last_save = now();
            }
    }
}

void start_selection_of_PID_coefficients() //операции перед началом выполнения алгоритма настройки ПИР
{
    if(flags.flag_start_selection_of_PID_coefficients)
    {
        if(flags.flag_reaching_Setpoint == 1)
        {
            t_new_setpoint =now() ; 
            Setpoint = temp_end ; 
            flags.flag_selection_of_PID_coefficients = 1; //разрешение выполнения
            flags.flag_start_selection_of_PID_coefficients = 0 ;//операции перед стартом выполнены
        }
    }
}

uint8_t equality_of_array_values(uint16_t *array, uint8_t n , uint16_t value  ) //проверка равенства всех членов массива какому-либо значению 
{ //n кол-во членов , *array - указатель на массив 
    uint8_t equal = 0; 
    for(int i = 0; i < n;i++)
    {
        if(array[i]==value)
        {
            equal++;
        }
    }
    if(equal==n) {return 1;} 
    else { return 0 ;}
}

void save_temp_for_graph() //сохранение температуры для работы графика
{
    if(flags.flag_save_data_on_SD)
    {
        if(line<=POINT_COUNT)
        {
            uint16_t line_beginning = (line-1)*SIZE_SAVE_TEMP ;
            if(file_graph.seek(line_beginning))
            {
                file_graph.println(fix_size(Input,SIZE_TEMP));
                line++;
                file_graph.flush();
            }
            else//файл не проинициализирован (не имеет нужного размера).Нужно инициализировать Sd-карту заново и потом повторить
            {
                SD_CARD_FAILED();
            }
        }
        else
        {
            line=1;
        }
    }
}

char* fix_size(uint16_t number,uint8_t size)
{
    /*
    переводит целое число number в строку char* и дополняет слева незначащими нулями 
    до длины size. Если количество цифр в числе больше желаемого размера массива, то
    возвращается указатель на нуль символ. 
    При успешном выполнении возвращается указатель на требуемую строку.
    */
	uint8_t digits = 1; 
	uint16_t for_dig = number;
	while (for_dig /= 10) digits++;
	char * str;

	if (digits <= size)
	{
		str = (char*)malloc(size + 1);
		for (int i = 0; i < size; i++)
		{
			str[i] = '0';
		}
		str[size] = '\0';

		char *str2 = (char*)malloc(size + 1);
		for (int i = 0; i < size; i++)
		{
			str2[i] = '0';
		}
		str2[size] = '\0';


		itoa(number, str2, 10);

		//перенос цифр в конечный массив с незначащими 0 слева
		for (int i = 0; i < digits; i++)
		{
			str[size - 1 - i] = str2[digits - 1 - i];
		}
	}
	else
	{
		str = (char*)malloc(1);
		str[0] = '\0';
	}

	return str; 
}

void plotting()
{
    if(flags.flag_on_page_graph)//если мы находимся на странице graph
    {
       //debugSerial.println("------------------------plotting graph----------------");
       int16_t last_y = -1;
       nextionSerial.print((String)"ref 0"+char(255)+char(255)+char(255)); //обновление страницы
       uint16_t x ;
       int write_line;
       for(x = X_Y_MIN,write_line = (line-1) ; x <= X_MAX ; x++, write_line--) 
       { //write_line - номер считываемой строки, 
           uint16_t size_file = SIZE_SAVE_TEMP*POINT_COUNT ;
           if(file_graph.size()>=size_file) //файл полностью заполнени нужным количеством точек
           {
               if(write_line==0)//дошли до начала файла
               {
                   write_line=POINT_COUNT ; //переходим в конец файла
               }
               uint16_t line_beginning = (write_line-1)*SIZE_SAVE_TEMP ;
                if(file_graph.seek(line_beginning))
                {
                    uint16_t temp = 0;
                    char buf[5] = {0}; // buf : char[16] -> char* -> void*
                    //buf[4]='\0';
                    file_graph.read(buf,4); // первую строку   
                    temp = atoi(buf) ;
                    //debugSerial.print("buf=");debugSerial.print(buf);debugSerial.println("|");
                    //debugSerial.print("temp=");debugSerial.println(temp);
                    //debugSerial.print("x=");debugSerial.println(x);
                    last_y = graph_point(temp,x,last_y);
                    //debugSerial.print("y=");debugSerial.println(last_y);
                    
                }
                else//файл не проинициализирован (не имеет нужного размера).Нужно инициализировать Sd-карту заново и потом повторить
                {
                    SD_CARD_FAILED();
                }
           }
           else
           {
               if(write_line==0)//дошли до начала файла
               {
                   break ; //заканчиваем рисование
               }
               uint16_t line_beginning = (write_line-1)*SIZE_SAVE_TEMP ;
                if(file_graph.seek(line_beginning))
                {
                    uint16_t temp = 0;
                    char buf[5] = {0}; // buf : char[16] -> char* -> void*
                    //buf[4]='\0';
                    file_graph.read(buf,4); // первую строку   
                    temp = atoi(buf) ; 
                    //debugSerial.print("buf=");debugSerial.print(buf);debugSerial.println("|");
                    //debugSerial.print("temp=");debugSerial.println(temp);
                    //debugSerial.print("x=");debugSerial.println(x);
                    last_y = graph_point(temp,x,last_y);
                    //debugSerial.print("y=");debugSerial.println(last_y);
                }
                else//файл не проинициализирован (не имеет нужного размера).Нужно инициализировать Sd-карту заново и потом повторить
                {
                    SD_CARD_FAILED();
                }
           }
       }
       //debugSerial.println("------------------------------------------------------");
    }
}

uint16_t graph_point(uint16_t value, uint16_t x, int16_t last_y)
{
    /*
    построение кривой графика к следующей точке со значение value, в координату x
    возвращает координату y
    */
    uint32_t y = value ;
    y *= Y_MAX ;
    y /= graph_temp_max;
    if(!value)
    {
        y = X_Y_MIN ;
    }
    uint32_t val = value;
    val *= Y_MAX ;
    if(val<graph_temp_max) {y = X_Y_MIN;}
    if(value>graph_temp_max) {y=Y_MAX;}
    y = Y_MAX + 1 - y ; //инверсия 
    nextionSerial.print((String)"line "+x+","+y+","+x+","+y+","+"GREEN"+char(255)+char(255)+char(255)); //рисуем точку
    //дополнительные построения, для соединения точек
    if(last_y!=-1) //строится не первая точка 
    {
        if((last_y+1)<y) //новая точка ниже предыдущей на 2 пикселя
        {
            for(int j = 0; j < (y-last_y);j++)
            {
               nextionSerial.print((String)"line "+(x-1)+","+(y-j)+","+(x-1)+","+(y-j)+","+"GREEN"+char(255)+char(255)+char(255)); //рисуем точку 
            }
        }
        if((last_y-1)>y)//новая точка выше предыдущей на 2 пискеля
        {
            for(int j = 1; j <= (last_y-y);j++)
            {
               nextionSerial.print((String)"line "+x+","+(y+j)+","+x+","+(y+j)+","+"GREEN"+char(255)+char(255)+char(255)); //рисуем точку 
            }
        }
    }
    return y ;
}
