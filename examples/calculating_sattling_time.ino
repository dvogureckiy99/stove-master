/*
Ogureckiy Dmitriy 
ogureckiy98@mail.ru
Расчёт времени решулирования и запись в файл
*/
#include <MAX31856_my.h>
#include <PID_v2.h>
#include <TimerThree.h>
#include <stdint.h>
#include <SD.h>

// Serial: 0 (rx) и 1 (tx), Serial1: 19 (rx) 18 (tx)
#define debugSerial Serial1 //порт(отладочный) для вывода сообщений о работе контроллера 

void setup()
{
    // Serial: 0 (rx) и 1 (tx), debugSerial: 19 (rx) 18 (tx)
   debugSerial.begin(115200);

      //инициализация SD-card
   pinMode(CS_SD, OUTPUT);
   init_SD();
}

void init_SD() //инициализация SD-карты
{
    flags.flag_true_init = 0; //флаг успешной инициализации
    //становится true, если он равен 11 (1 успешная инициализация + 10 успешно созданных файла)
    
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
            flags.flag_true_init++;
        }
        file_obj.close();//закрыли и сохранили
    }
    
    if(flags.flag_true_init==11) //успешная иниц.
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
        flags.flag_true_init = 0;//обновляем флаг инициализации
    }
}

void SD_CARD_FAILED()//при неуспешной инициализации файла
{
    debugSerial.println("initialization failed!"); 
}