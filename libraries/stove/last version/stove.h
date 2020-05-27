#ifndef stove_h
#define stove_h
#include <EEPROM.h>


#include <MAX31856_my.h>
#include <PID_v2.h>
#include <TimerThree.h>
#include <stdint.h>
#include <SD.h>
typedef float  float32_t;
typedef double double32_t;

//определение пременных для вывода времени
#include <TimeLib.h>
time_t t_MCU ; //записываем время перового запуска микроконтроллера
time_t t_ALG=0; // время запуска программы
time_t t_ALG_last_run = 0; //время работы алгоритма перед включением паузы
time_t t_ALG_run=0; // время работы алгоритма
time_t t_INT=0; // время запуска интервала
time_t t_INT_last_run = 0; //время работы интервала перед включением паузы
time_t t_INT_run = 0; // время работы интервала

//Расшифровка управляющих команд протокола передачи данных
//формат сообщения #12;
// # - признак начала сообщения
//1- байт номера команды
//2- байт номера действия или данные
// ; - признак конца команды
//номер команды
 
#define PID_SWITCH  '2' 
#define SETPOINT '3'
#define OUTPUT '4'
#define KOEFF_PROPORTIONAL '5'
#define KOEFF_INTEGRAL '6'
#define TEMP_POINT '7'  //считывает точки программы изменения температуры из SD-card в ОЗУ MCU
#define POINT_SWITCH  '8'//включение выключение изменение температуры по заданному алгоритму
#define POINT_RUN  '9'  //пауза и выход из паузы
#define RETURN_INTERVAL  '<' //вернуться на интервал 
#define INTERVAL_HIGHLIGHING '='//подсвечивание интервала
#define MISTAKE '?' //отправка информации об ошибках
#define WRITE_POINT '>' //операция записи на флэш-носитель данных конкретной точки программы
#define READ_POINT '@'  //операция чтения с DS-карты и записи в HMI данных точки на страницу "set_points"
#define READ_POINTS 'A' //операция чтения точек программы с DS-карты и записи в HMI данных точек на страницах points...
#define INITIALIZATION_SD 'B' //инициализация SD-карты
//номер действия
#define ON  1
#define OFF 0

//serial
// Serial: 0 (rx) и 1 (tx), Serial1: 19 (rx) 18 (tx)
#define debugSerial Serial1 

//определение пременных для ПИ
unsigned int Setpoint = 400; //установочная темепратура
unsigned int Input; //настоящая температура
int Output = 1023;  //  коэфф. заполн. в относительных единицах 
double Kp=8; //коэффициент пропорнционального звена 
double Ki=1; //коэффициент интегрирующего звена
#define SAMPLE_TIME 1000 //// частота вызова ПИ (в мсек)
//по умолчанию(SetOutputLimits(0, 255);   inAuto = false;)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, REVERSE); //создание объекта класса myPID REVERS указывает на то, 
//что направление изменения выходного сигнала идёт в обратном направлении
#define PIN_OUTPUT 2 //выход , решулирующий температуру
uint32_t last_time= 0; //последнее время заупска алгоритма 

#define MAX_DERIVATIVE 10 //максимальное значение производной от температуры     

//connecting IC
//подключаю питание через сопротивление 1,3 кОМ т.е. ток питания 3,8 мА , что соответствует даташиту
#define SCK    11  //выбор пинов
#define CS_IC     10  //
#define SDI    13 //
#define SDO    12 //
#define CR0_INIT  (CR0_AUTOMATIC_CONVERSION + CR0_OPEN_CIRCUIT_FAULT_TYPE_K  + CR0_NOISE_FILTER_50HZ )
#define CR1_INIT  (CR1_AVERAGE_2_SAMPLES + CR1_THERMOCOUPLE_TYPE_K)
#define MASK_INIT (~(MASK_VOLTAGE_UNDER_OVER_FAULT + MASK_THERMOCOUPLE_OPEN_FAULT))
MAX31856 *temperature;

//Для реализации скользящего среднего
const int numReadings = 5;// Использование константы вместо переменной позволяет задать размер для массива.
unsigned int readings[numReadings];      // данные, считанные с входного аналогового контакта
byte index = 0;                  // индекс для значения, которое считывается в данный момент
unsigned int total = 0;                  // суммарное значение

//точки изменения температуры
uint16_t  temp_point[10];   //содержит информацию о температуре в точке
uint16_t  time_point[10];//содержит информацию о времени нахождении на интервале точки
float32_t time_step[10] ;//время ,через которое меняется установочная температура (в сек) для каждого участка
uint8_t interval = 0; // номер участка 0-8
uint8_t compare = 0; //больше или меньше температура в начале участка, чем температура в конце участка
//1  температура в начале участка больше 0 - температура в начале участка меньше
uint32_t lastsecond = 0 ;//время крайнего вызова изменения setpoint (такого значения переменной хватит на 50 дней)

uint8_t mistake_id = 0; //1 ошибка термопары
//номера ошибок
//1 - FAULT_OPEN
//2 - FAULT_VOLTAGE
//3 - NO_MAX31856
uint8_t mistake_2 = 0; //2 ошибка initialization SD-card failed!


//flags
struct
{
    unsigned 

} flags;
/*
uint8_t flag_pause_exit = 0; //флаг выхода из паузы
uint8_t flag_first_run_interval = 0 ; //флаг(показатель) первого запуска интервала 1-уже запущен  0- не запущен
uint8_t flag_point_switch = 0 ; //разрешение на изменение температуры по заданному алгоритму
uint8_t flag_reaching_Setpoint = 1; //флаг достижение установившего значения
uint8_t flag_true_init = 0; //флаг успешной инициализации
*/

// SD-card . порты SPI:  51 (MOSI), 50 (MISO), 52 (SCK)
#define CS_SD 53 
#define SIZE_PROGRAM_FILE 168 //размер файла с одной программой
File file_obj; // вспомогательный объект файла 
#endif