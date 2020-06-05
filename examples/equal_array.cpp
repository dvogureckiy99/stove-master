//#include "stdafx.h"
#include <iostream>
#include <cstring>
#include <string>
#include <stdio.h>
#include <cctype>
#include <conio.h> //библиотека содержит функции для работы с экраном

uint8_t equality_of_array_values(unsigned int *array, uint8_t n);

using namespace std;

int main() {
	const uint8_t n = 5;
	unsigned int A[n] = { 0 , 0, 0, 1 };
	if (equality_of_array_values(A, n)) { cout << "all member A are equal\n"; }
	else { cout << "all members A are not equal\n"; }
	
	_getch(); //функция ввода символа с клавиатуры. Используется для задержки
	return 0;
}

uint8_t equality_of_array_values(unsigned int *array, uint8_t n)
{
	uint8_t equal = 0;
	for (int i = 0; i < (n - 1); i++)
	{
		if (array[i] == array[i + 1])
		{
			equal++;
		}
	}
	if (equal == (n - 1)) { return 1; }
	else { return 0; }
}