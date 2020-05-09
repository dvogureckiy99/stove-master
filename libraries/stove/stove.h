/*
по сравнению со старой версией убран расчет времени регулирования для нескольких температурных интервалов и нет 
алгоритма настройки коэфф. ПИР
*/

#ifndef stove_h
#define stove_h
#include <EEPROM.h>         
#include <MAX31856_my.h>
#include <PID_v2.h>
#include <TimerThree.h>
#include <stdint.h>
#include <SD.h>
//#include <stdlib.h>
typedef float  float32_t;
typedef double double32_t;

//определение переменных для вывода времени
#include <TimeLib.h>
time_t t_MCU ;              //записываем время перового запуска микроконтроллера
time_t t_ALG=0;             // время запуска программы
time_t t_ALG_last_run = 0;  //время работы алгоритма перед включением паузы
time_t t_ALG_run=0;         // время работы алгоритма
time_t t_INT=0;             // время запуска интервала
time_t t_INT_last_run = 0;  //время работы интервала перед включением паузы
time_t t_INT_run = 0;       // время работы интервала
//вспомогательные (для хранения времени)    
time_t t_last_time_PI = 0        ;  //последнее время заупска алгоритма
time_t t_last_time_send_data = 0 ;  //последнее время вызова операций (функций)
#define SAMPLE_TIME_SEND_DATA 1   //частота опроса основных условий


//Расшифровка управляющих команд протокола передачи данных
//формат сообщения #12;
//# - признак начала сообщения
//1 - байт номера команды
//2 - байт номера действия или данные
//; - признак конца команды
//номер команды
#define PID_SWITCH                      '2'   //включение,отключение ПИ-регулятора
#define SETPOINT                        '3'   //установка задающего воздействия
#define OUTPUT                          '4'   //установка к.з. ШИМ (0-1024).Мощность нагревательного элемента (0-100%)
#define KOEFF_PROPORTIONAL              '5'   //установка коэффициента пропорционального регулятора
#define KOEFF_INTEGRAL                  '6'   //установка коэффициента интегрального регулятора
#define TEMP_POINT                      '7'   //считывает точки программы изменения температуры из SD-card в ОЗУ MCU
#define POINT_SWITCH                    '8'   //включение выключение изменение температуры по заданному алгоритму
#define POINT_RUN                       '9'   //пауза и выход из паузы
#define RETURN_INTERVAL                 '<'   //вернуться на интервал 
#define INTERVAL_HIGHLIGHING            '='   //подсвечивание интервала
#define MISTAKE                         '?'   //отправка информации об ошибках
#define WRITE_POINT                     '>'   //операция записи на EEPROM данных конкретной точки программы
#define READ_POINT                      '@'   //операция чтения с EEPROM и записи в HMI данных точки на страницу "set_points"
#define READ_POINTS                     'A'   //операция чтения точек программы с EEPROM и записи в HMI данных точек на страницах points...
#define INITIALIZATION_SD               'B'   //инициализация SD-карты
#define SET_MAX_DERIVATIVE              'D'   //принять и записать максимально допустимый скачок производной 
#define SET_MIN_DERIVATIVE              'E'   //принять и записать минимально допустимый скачок производной 
#define GET_JUMP_DERIVATIVE             'C'   //установать на дисплее значения скачков производной
#define SELECTION_OF_PID_COEFFICIENTS   'F'   //включение алгоритма подбора коэффициентов  (2=ON включить,2=OFF отключить)
#define SET_TEMP_MAX_MIN_US             'G'   //установка праметров на странице PID_SETUP
#define GET_TEMP_MAX_MIN_US             'H'   //получение параметров от страницы PID_setting
#define SET_KP_KI_SETTLING_TIME         'I'   //установка последних 5-ти измерений времени регулирования
#define SAVE_DATA_ON_SD_CARD            'J'   //сохранение данных о работе устройства на SD-card 
#define SET_SD_STATUS                   'K'   //поставить status SD-card (есть,нет)  на нужной странице дисплея
#define PAGE_GRAPH                      'L'   //установка или снятие флага нахождения на странице
#define PI_STATUS                       'M'   //установить статус иконки PIR на стр. debug

//номер действия
#define ON  1
#define OFF 0
 
// Serial: 0 (rx) и 1 (tx), Serial1: 19 (rx) 18 (tx) Serial2: 17 (rx) 16 (tx)
#define debugSerial Serial1 //порт(отладочный) для вывода сообщений о работе контроллера 
#define nextionSerial Serial2 //порт для для работы с дисплеем Nextion 
#define SERIAL_BAUD 115200 

//константы Пи регулятора
#define MAXIMUM_HEATING 0       //максимальная мощность 
#define MINIMUM_HEATING 1023    //минимальная мощность (нагрев выключен) 
//определение пременных для ПИ регулятора
uint16_t Setpoint = 400; //установочная темепратура
uint16_t Input;          //текущая температура
int Output = MINIMUM_HEATING ;           // коэфф. заполн. в относительных единицах 
double Kp=8.5;                 //коэффициент пропорнционального звена 
double Ki=1;                 //коэффициент интегрирующего звена
// частота вызова ПИ (в сек)
#define SAMPLE_TIME_PI 1    
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, REVERSE); //создание объекта класса myPID REVERS указывает на то, 
                                                        //что направление изменения выходного сигнала идёт в обратном направлении
                                                        //по умолчанию(SetOutputLimits(0, 255);   inAuto = false;)
#define PIN_OUTPUT 2         //выход , регулирующий температуру

//для работы алгориитма настройки коэф. ПИР
time_t t_new_setpoint = 0;    //время начала нагрева с новым значением setpoint 
#define SAVE_FILE 30 //частота сохранения in sec
#define INF 66000 //время регулирования при нестабильной системе
uint16_t temp_start = 400;  //начальная температура, от которой идёт настройка
uint16_t temp_end   = 500;//конечная температура, до которой идёт настройка
uint16_t time_unstable_system = 300 ; //sec, время , при котором считаем , что система неустойчива 

//экстремальные значения производной
int max_derivative =  150 ;    //максимальное значение производной от температуры     
int min_derivative = -150 ;    //минимальное значение производной от температуры  

//подключение термоконтроллера
//подключаю питание через сопротивление 1,3 кОМ т.е. ток питания 3,8 мА , что соответствует даташиту
#define SCK    11     //выбор пинов
#define CS_IC     10  //
#define SDI    13     //
#define SDO    12     //
#define CR0_INIT  (CR0_AUTOMATIC_CONVERSION + CR0_OPEN_CIRCUIT_FAULT_TYPE_K  + CR0_NOISE_FILTER_50HZ )
#define CR1_INIT  (CR1_AVERAGE_2_SAMPLES + CR1_THERMOCOUPLE_TYPE_K)
#define MASK_INIT (~(MASK_VOLTAGE_UNDER_OVER_FAULT + MASK_THERMOCOUPLE_OPEN_FAULT))
MAX31856 *temperature;

//Для реализации скользящего среднего
const int numReadings = 5;               // Использование константы вместо переменной позволяет задать размер для массива.
uint16_t readings[numReadings];      // данные, считанные с входного аналогового контакта
byte index = 0;                          // индекс для значения, которое считывается в данный момент
unsigned int total = 0;                  // суммарное значение

//точки изменения температуры
uint16_t  temp_point[10];   //содержит информацию о температуре в точке
uint16_t  time_point[10];   //содержит информацию о времени нахождении на интервале точки
float32_t time_step[10] ;   //время ,через которое меняется установочная температура (в сек) для каждого участка
uint8_t interval = 0;       // номер участка 0-8
uint8_t compare = 0;        //больше или меньше температура в начале участка, чем температура в конце участка
                            //1  температура в начале участка больше 0 - температура в начале участка меньше
uint32_t lastsecond = 0 ;   //время крайнего вызова изменения setpoint (такого значения переменной хватит на 50 дней)

//ошибки
struct 
{
    unsigned mistake_id     : 2;            //ошибка термопары. номера ошибок:1 - FAULT_OPEN,2 - FAULT_VOLTAGE,3 - NO_MAX31856
    unsigned mistake_2      : 1;            //ошибка initialization SD-card failed!
} mistake;

//flags
struct 
{
    unsigned flag_pause_exit                                    : 1;   //флаг выхода из паузы
    unsigned flag_first_run_interval                            : 1;   //флаг(показатель) первого запуска интервала 1-уже запущен  0- не запущен
    unsigned flag_point_switch                                  : 1;   //разрешение на изменение температуры по заданному алгоритму
    unsigned flag_reaching_Setpoint                             : 2;   //флаг достижение установившегося значения
    unsigned flag_true_init_SD                                  : 2;   //флаг успешной инициализации (2 - успешно, любое другое знач. — неуспешно)
                                                                       //становится true, если он равен 2
    unsigned flag_selection_of_PID_coefficients                 : 1;   //разрешение на работу алгоритма подбора параметров ПИ
    unsigned flag_start_selection_of_PID_coefficients           : 1;   //флаг разрешения выполнения операций перед стартом алгоритма подбора параметров ПИ
    unsigned flag_save_data_on_SD                               : 1;   //сохранять параметры на SD (1-да,0-нет)
    unsigned flag_on_page_graph                                 : 1;   //нахождение на странице graph (1-да,0-нет)
} flags ;

//структура точки для работы с EEPROM
struct point{
    char hour[4];
    char minute[2];
    char temp[4];
} ;

//структура Kp,Ki,settling_time для запоминания последних 5-ти Kp,Ki,settling_time 
struct Kp_Ki_settling_time{
    double kp = 0 ;
    double ki = 0 ;
    uint16_t settling_time = 0; 
} Kp_Ki_settling_time[5];
uint8_t number = 0;

//для построения графика и сохранения на SD
#define POINT_COUNT 3600
#define SIZE_SAVE_TEMP 6  
#define SIZE_TEMP 4
uint16_t line = 1; //номер строки, в которую будет производиться запись
#define X_MAX 799 //максимальная координата по x
#define Y_MAX 479 //максимальная координата по Y_MAX
#define X_Y_MIN 1 //минимальная координата
uint16_t graph_temp_max = 800;
uint8_t graph_temp_min = 0;

// SD-card . порты SPI:  51 (MOSI), 50 (MISO), 52 (SCK)
#define CS_SD 53 
#define SIZE_PROGRAM_FILE 168 //размер файла с одной программой
File file_obj, file_obj1,file_obj2,file_obj3,file_obj4, file_graph; // вспомогательный объект файла 
#endif