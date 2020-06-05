//парсинг строки с дисплея Nextion. Отработка алгоритма
#include "stdafx.h"
#include <iostream>
#include <cstring>
#include <string>
#include <stdio.h>
#include <cctype>
#include <conio.h> //библиотека содержит функции для работы с экраном


#define OUTPUT_STRING "1,10,100:2,20,100:3,25,300:4,60,200:" //Выводимая строка (обязательно прописными буквами !!!)
//   ТС1=600/71=8,45   8*71=568   поднимется на 32 секунды раньше , чем нужно
//   
using namespace std;

int main() {
	// наше сообщение (пример) 1,10,100:2,20,100:3,25,300:4,60,300:
	uint16_t time_point[10];
	uint16_t temp_point[10];
	float time_step[10];
	string value = "";
	int point_number = 0;
	int parametrs_num = 0;
	int i;
	int Input = 105;
	string last = OUTPUT_STRING;
	for (i = 0; i < last.length(); i++)
	{
		value += last[i];
		if (last[i] == ',')
		{
			if (parametrs_num == 0)//номер точки
			{
				point_number = stoi(value);
				parametrs_num++;
				value = ""; //удаляем значение
				continue;
			}
			if (parametrs_num == 1)//время
			{
				time_point[point_number - 1] = stoi(value);
				parametrs_num = 0; // достигли max-1 параметра
				value = ""; //удаляем значение
				continue;
			}
		}
		if (last[i] == ':')
		{
			temp_point[point_number - 1] = stoi(value); //записываем температуру
															//расчет шага времени
			if (point_number == 1) //записываем шаг времени
			{
				int16_t denominator = (temp_point[point_number - 1] - Input);
				if (denominator != 0)
				{
					time_step[point_number - 1] = (60 * time_point[point_number - 1]);
					//cout << "denominator[" << point_number - 1 << "]=" << denominator << endl;
					time_step[point_number - 1] /= (denominator); //1 шаг времени
					//cout << " time_step_before_abs[" << point_number - 1 << "]=" << time_step[point_number - 1] << endl;
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
					time_step[point_number - 1] = 60 * (time_point[point_number - 1] - time_point[point_number - 2]);
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
	for (int i = 0; i < 10; i++)
	{
		cout << "temp_point[" << i << "]=" << temp_point[i] << endl;
		cout << "time_point[" << i << "]=" << time_point[i] << endl;
		cout << " time_step[" << i << "]=" << time_step[i] << endl;
	}
	cout << endl;
	

	_getch(); //функция ввода символа с клавиатуры. Используется для задержки
	return 0;
}
