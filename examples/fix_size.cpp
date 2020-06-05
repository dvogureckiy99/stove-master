#define _CRT_SECURE_NO_WARNINGS

// #include "stdafx.h"
#include <iostream>
#include <cstring>
#include <string>
#include <stdio.h>
#include <cctype>
#include <conio.h> //библиотека содержит функции для работы с экраном

char* fix_size(uint16_t number, uint8_t size);

using namespace std;

int main() {
	uint16_t Input = 35; 
	uint16_t n = 0;
	char *str;
	char a = '0';

	while (a != '3')
	{
		cout << "Write Input=";
		cin >> Input;
		cout << "Write n=";
		cin >> n;
		//n -= 48;
		str = fix_size(Input, n);

		cout << "str=" << str << "\n" ;
		
		cout << "turn 3 to close:";
		cin >> a;
	}
	//_getch(); //функция ввода символа с клавиатуры. Используется для задержки
	return 0;
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


		_itoa(number, str2, 10);

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
