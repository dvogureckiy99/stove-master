/*
Ogureckiy Dmitriy 
ogureckiy98@mail.ru
*/
//проблема с расчетом шага при температуре большей температуры установочной, шаг как будто слишком маленький и setpoint очень быстро изменяется
#include "stove.h" //все                                                           основные определения

void setup() 
{
   // Serial: 0 (rx) и 1 (tx), Serial1: 19 (rx) 18 (tx)
   Serial.begin(115200);
   debugSerial.begin(115200);
   Serial.print("baud=115200ÿÿÿ"); //установка скорости экрана
   t_MCU = now(); //записываем время перового запуска микроконтроллера
   

   //инициализация SD-card
   pinMode(CS_SD, OUTPUT);
   init_SD();

   //turn the PID on
  //при первом запуске происходит также инициализация Initialize()
  myPID.SetMode(MANUAL);
  myPID.SetSampleTime(SAMPLE_TIME);
  myPID.SetOutputLimits(0, 1023);
  
   //------PWM----------------
  Timer3.initialize((long) SAMPLE_TIME * 1000);         // инициализировать timer1, и установить период равный периоду вызова ПИД
  Timer3.pwm( PIN_OUTPUT,  Output );                // задать шим сигнал  с  коэффициентом заполнения 

  // Создание объекта MAX31856 с определением пинов
  temperature = new MAX31856(SDI, SDO, CS_IC, SCK);
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
    //Serial.print((String)"page SD_CARD_FAILED"+char(255)+char(255)+char(255)); //выключаем мигание экрана, если нет оповищения об ошибке

    last_time = millis();
  }

    reachingSetpoint();

    exitpause();

    changeLinearly();
    
    readNextioncommand();
    
    //проверка наличия ошибок
    if((mistake_id == 0)&&(mistake_2 == 0))
    Serial.print((String)"main.ERROR.en=0"+char(255)+char(255)+char(255)); //выключаем мигание экрана, если нет оповищения об ошибке
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
        
        case TEMP_POINT  : //(last - номер программы)
        {
            String adress = "";
            uint32_t time = 0;
            //формируем адресс
            adress += "program/";
            adress += last;
            adress += ".txt";
            file_obj = SD.open(adress,FILE_READ);//открытие файла, файл должен быть создан раннее
            for(int i = 1; i<11;i++) // i - номер точки
            {
                if(file_obj.seek((i-1)*17))//перенос каретки на нужную строку
                {
                    char buf[16] = {0}; // buf : char[16] -> char* -> void*
                    file_obj.read(buf,15); // первую строку   
                    last = buf ;             
                    String value = ""; //проверяемый символ(строка)
                    value = last.substring(3,7); //часы
                    time += value.toInt();
                    time *= 60; //перевод в минуты
                    value = last.substring(8,10); //минуты
                    time += value.toInt();
                    time_point[i-1] = time;//запись времени в мин
                    time = 0;
                    value = last.substring(11,15);//темп.
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
                            time_step[i - 1] = 0;//если температура постоянна на участке
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
                            time_step[i - 1] = 0;//если температура постоянна на участке
                                                            //шаг времени = 0
                    }
                }
                else//файл не проинициализирован (не имеет нужного размера).Нужно инициализировать Sd-карту заново и потом повторить
                {
                    SD_CARD_FAILED();
                    break; //выходи из цикла
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
            if(flag_first_run_interval)
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
            if(mistake_id)
            Serial.print((String)"vis "+mistake_id+",1"+char(255)+char(255)+char(255)); //показать сообщение об ошибке
            if(mistake_2)
            Serial.print((String)"vis 4,1"+char(255)+char(255)+char(255)); //показать сообщение об ошибке initialization SD-card failed!
            break;
        }   
        case INITIALIZATION_SD :  //инициализация SD-карты
        {          
            init_SD();
            break;
        } 
        case WRITE_POINT:  //операция записи на флэш-носитель данных конкретной точки программы
        {          
            int P=0 ;//номер точки
            String Np = last.substring(0,2);//символы номера программы
            int N = Np.toInt();
            String nP = last.substring(3,5);//символы номера точки
            P = nP.toInt(); //преобразуем в число
            String adress = "";//адрес файла программы
            last = last.substring(3) ; //обрезаем номер программы
            //формируем адресс
            adress += "program/";
            adress += N;
            adress += ".txt";
            file_obj = SD.open(adress,FILE_WRITE);//открытие файла, файл должен быть создан раннее
            if(file_obj.seek((P-1)*17))//перенос каретки на нужную строки
            {
                file_obj.print(last);//запись данных точки
                file_obj.close();
            }
            else//файл не проинициализирован (не имеет нужного размера).Нужно инициализировать Sd-карту заново и потом повторить
            {
                debugSerial.println("wrong write on SD-card");
                SD_CARD_FAILED();
            }
            break;
        } 
        case READ_POINT :  //операция чтения с DS-карты и записи в HMI данных точки на страницу "set_points"
        {          
            String adress = "";
            int P = 0;//номер точки
            String N ="";//номер программы
            String value = ""; //проверяемый символ(строка)
            for(int i = 0; i < last.length(); i++ )//проходим по всей строке
            {
                if(last[i] == ',')//запятая в роли разделителя
                {
                        N = value;//запись номера программы
                        value = ""; //удаляем значение
                        continue;
                }
                value += last[i];//запись следующего символа
            }
            P = value.toInt();//запись номера точки
            //формируем адресс
            adress += "program/";
            adress += N;
            adress += ".txt";
            file_obj = SD.open(adress,FILE_WRITE);//открытие файла, файл должен быть создан раннее
            if(file_obj.seek((P-1)*17))//перенос каретки на нужную строки
            {                   
                char buf[16] = {0}; // buf : char[16] -> char* -> void*
                file_obj.read(buf,15); // первую строку   
                last = buf ; 
                file_obj.close();
                set_point_data(last);//последовательная установка в поле дисплея на стр. set_points
            }
            else//файл не проинициализирован (не имеет нужного размера).Нужно инициализировать Sd-карту заново и потом повторить
            {
                debugSerial.println("wrong read point from SD-card");
                SD_CARD_FAILED();
            }
            break;
        }
        case READ_POINTS :  //операция чтения точек программы с DS-карты и записи в HMI данных точек на страницах points...
        {          
            String adress = "";
            int P = 0;//номер точки
            String N ="";//номер программы
            String value = ""; //проверяемый символ(строка)
            for(int i = 0; i < last.length(); i++ )//проходим по всей строке
            {         
                if(last[i] == ',')//запятая в роли разделителя
                {
                        N = value;//запись номера программы
                        value = ""; //удаляем значение
                        continue;
                }
                value += last[i];//запись следующего символа
            }
            P = value.toInt();//запись номера страницы
            //формируем адресс
            adress += "program/";
            adress += N;
            adress += ".txt";
            file_obj = SD.open(adress,FILE_WRITE);//открытие файла, файл должен быть создан раннее
            if(file_obj)
            {
              debugSerial.print("successfully open file:");
              debugSerial.println(adress);
            }else
            {
              debugSerial.print("unsuccessfully open file:");
              debugSerial.println(adress);
            }
            if(P) //считываем(устанавливаем) 6-10 строки
            {
                for(int i = 6; i<11;i++)
                {
                     if(file_obj.seek((i-1)*17))//перенос каретки на нужную строки
                    {
                        debugSerial.print("successfully cursor shift:");
                        debugSerial.println(i);                      
                        char buf[16] = {0}; // buf : char[16] -> char* -> void*
                        file_obj.read(buf,15); // первую строку   
                        last = buf ;
                        debugSerial.print("last =");
                        debugSerial.println(last);           
                        set_point_data2(last, i);// отправляем данные в дисплей
                    }
                    else//файл не проинициализирован (не имеет нужного размера).Нужно инициализировать Sd-карту заново и потом повторить
                    {
                        debugSerial.println("wrong read points from SD-card");
                        SD_CARD_FAILED();
                        break; //выходи из цикла
                    }
                }
            }
            else //считываем(устанавливаем) первые 5 строк
            {
                for(int i = 1; i<6;i++)
                {
                     if(file_obj.seek((i-1)*17))//перенос каретки на нужную строки
                    {
                        debugSerial.print("successfully cursor shift:");
                        debugSerial.println(i);                      
                        char buf[16] = {0}; // buf : char[16] -> char* -> void*
                        file_obj.read(buf,15); // первую строку   
                        last = buf ;
                        debugSerial.print("last =");
                        debugSerial.println(last);              
                        set_point_data2(last, i);//отправляем данные в дисплей
                    }
                    else//файл не проинициализирован (не имеет нужного размера).Нужно инициализировать Sd-карту заново и потом повторить
                    {
                        debugSerial.println("wrong read points from SD-card");
                        SD_CARD_FAILED();
                        break; //выходи из цикла
                    }
                }
            }
            file_obj.close();
            debugSerial.print("successfully close file:");
            debugSerial.println(adress);
            break;
        } 
    }    
}


void checkMAX31856(void)//-------------------обработка оповещений от термоконтроллера----------------
{
    //-------------------обработка оповещений от термоконтроллера----------------
    if((Input == FAULT_OPEN)) // No thermocouple
    {//pause
        mistake_id = 1; //запись номера ошибки 
        Output = 1023 ; //выключаем нагрев
        myPID.SetMode(MANUAL); //ручной режим ПИД
        if(flag_point_switch==1)
        {
            t_ALG_last_run =t_ALG_last_run + now() - t_ALG; // сохранение время работы алгоритма
            t_INT_last_run =t_INT_last_run + now() - t_INT; // сохранение время работы интервала
            flag_point_switch = 0; //останавливаем работу на  интервале
        }
        Serial.print((String)"page FAULT_OPEN"+char(255)+char(255)+char(255)); //страница с оповещением
        Serial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255)); //изменение состояния индикатора паузы
        Serial.print((String)"main.ERROR.en=1"+char(255)+char(255)+char(255)); //включаем мигание экрана
    }
    if((Input == FAULT_VOLTAGE)) //  Under/over voltage error.  Wrong thermocouple type?
    {//pause
        mistake_id = 2;//запись номера ошибки
        Output = 1023 ; //выключаем нагрев
        myPID.SetMode(MANUAL); //ручной режим ПИД
        if(flag_point_switch==1)
        {
            t_ALG_last_run =t_ALG_last_run + now() - t_ALG; // сохранение время работы алгоритма
            t_INT_last_run =t_INT_last_run + now() - t_INT; // сохранение время работы интервала
            flag_point_switch = 0; //останавливаем работу на  интервале
        }
        Serial.print((String)"page FAULT_VOLTAGE"+char(255)+char(255)+char(255)); //страница с оповещением
        Serial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255)); //изменение состояния индикатора паузы
        Serial.print((String)"main.ERROR.en=1"+char(255)+char(255)+char(255)); //включаем мигание экрана
    }
    if((Input == NO_MAX31856)) // MAX31856 not communicating or not connected
    {//pause
        mistake_id = 3;//запись номера ошибки
        Output = 1023 ; //выключаем нагрев
        myPID.SetMode(MANUAL); //ручной режим ПИД
        if(flag_point_switch==1)
        {
            t_ALG_last_run =t_ALG_last_run + now() - t_ALG; // сохранение время работы алгоритма
            t_INT_last_run =t_INT_last_run + now() - t_INT; // сохранение время работы интервала
            flag_point_switch = 0; //останавливаем работу на  интервале
        }
        Serial.print((String)"page NO_MAX31856"+char(255)+char(255)+char(255)); //страница с оповещением
        Serial.print((String)"main.bt1.val=1"+char(255)+char(255)+char(255)); //изменение состояния индикатора паузы
        Serial.print((String)"main.ERROR.en=1"+char(255)+char(255)+char(255)); //включаем мигание экрана
    }
    if((Input == FAULT_OPEN)||(Input == FAULT_VOLTAGE )||(Input == NO_MAX31856)) { Input = 0 ;}; //вывод нулевой температуры
    if((Input != FAULT_OPEN)&&(Input != FAULT_VOLTAGE )&&(Input != NO_MAX31856))
    {
        mistake_id = 0 ; //нет ошибки
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
                            if( get_minute(t_INT_run) >= time_point[i] )//достижение установленного времени работы интервала
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

void init_SD() //инициализация SD-карты
{
    
    //становится true, если он равен 11 (1 успешная инициализация + 10 успешно созданных файла)
    
    //инициализация
    debugSerial.println("Initializing SD card...");
    if(SD.begin(CS_SD)) 
    {
         flag_true_init++;
    }
    else
    {  
      if(flag_true_init==0)
      {
        SD_CARD_FAILED();//карта не была раньше активирована
        return;  
      }
      else
      {
        return;
      }
    }
    if(!SD.exists("program"))//нет папки для программ
    {
        SD.mkdir("program"); //то создаем
    }

    String name_txt = "" ;
    for(int i = 1; i <= 10; i++ ) //проверяем, созданы ли файлы программ, если надо создаём
    {
        name_txt = "program/";
        name_txt += (String)(i);
        name_txt += ".txt";
        file_obj = SD.open(name_txt, FILE_WRITE); //создаём/открываем файл
        debugSerial.print("open file ");
        debugSerial.println(i);
        if(file_obj) //файл есть
        {
            if(file_obj.size()<SIZE_PROGRAM_FILE)//файл не проинициализирован
            {
                for(int j = 1; j < 10;j++) //заполняем строковыми значениями
                {
                  file_obj.print(0);
                  file_obj.print(j);
                  file_obj.println(",0000,00,0000");
                }
                file_obj.print(10);
                file_obj.println(",0000,00,0000");
            }
            flag_true_init++;
        }
        file_obj.close();//закрыли и сохранили
    }
    
    if(flag_true_init==11) //успешная иниц.
    {
        debugSerial.println("initialization done.");
        if(mistake_2)
        {   //показать, что SD-карта успешно инициализирована
            Serial.print((String)"page SD_CARD_FAILED"+char(255)+char(255)+char(255)); //страница с оповещением
            Serial.print((String)"vis t0,0"+char(255)+char(255)+char(255)); 
            Serial.print((String)"vis t1,1"+char(255)+char(255)+char(255));
            Serial.print((String)"vis b0,0"+char(255)+char(255)+char(255)); 
            mistake_2 = 0;
        }
    }
    else //неуспешная иниц.
    {
        SD_CARD_FAILED();
        flag_true_init = 0;//обновляем флаг инициализации
    }
}

void SD_CARD_FAILED()//при неуспешной инициализации файла
{
    debugSerial.println("initialization failed!"); 
    mistake_2 = 1;//запись номера ошибки initialization SD-card failed!
    Serial.print((String)"page SD_CARD_FAILED"+char(255)+char(255)+char(255)); //страница с оповещением
    Serial.print((String)"main.ERROR.en=1"+char(255)+char(255)+char(255)); //включаем мигание экрана и окна с ошибками
}

void set_point_data(String str)
{
    String value = ""; //проверяемый символ(строка)
    value = str.substring(3,7); //часы
    Serial.print((String)"set_points.t1.txt=\""+value+"\""+char(255)+char(255)+char(255));
    value = str.substring(8,10); //минуты
    Serial.print((String)"set_points.t7.txt=\""+value+"\""+char(255)+char(255)+char(255));
    value = str.substring(11,15);//темп.
    Serial.print((String)"set_points.t8.txt=\""+value+"\""+char(255)+char(255)+char(255));    
}

void set_point_data2(String str, int s) //s-номер точки (1-10)
{
    String value = ""; //проверяемый символ(строка)
    int num = 0;//номер поля
    if(s>5)
    {
        s-=5; //поправка для дисплея (теперь s=1...5)
    }
    value = str.substring(3,7); //часы
    num = 3+8*(s-1) ;
    Serial.print((String)"b["+num+"].txt=\""+value+"\""+char(255)+char(255)+char(255));
    value = str.substring(8,10); //минуты
    num = 5+8*(s-1) ;
    Serial.print((String)"b["+num+"].txt=\""+value+"\""+char(255)+char(255)+char(255));
    value = str.substring(11,15);//темп.
    num = 7+8*(s-1) ;
    Serial.print((String)"b["+num+"].txt=\""+value+"\""+char(255)+char(255)+char(255));    
}
