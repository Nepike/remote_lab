/**
 * SERV library (Linux)
 * Сервисная библиотека. Много полезных функций
 * \author Robofob
 * \version 1.05
 * \date 18.01.2014
 * \date LP 03.03.2016
*/

#ifndef _SERV_LIB_H_
#define _SERV_LIB_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>

typedef char l_string[512];

/// Символы строки-комментария для функции ReadString
extern char RemStr[];

/// Флаг ожидания нажатия клавиши Enter после выдачи сообщения об ошибке (функция error)
extern int error_wait_for_enter;

char *newstr(char *s);
void error(const char *fmt, ...);
void warning(const char *fmt, ...);
void trim(char *s);

/// Чтение строк. Игнорируются пустые строки и строки-комментарии (начинающиеся с символов '#' или ';', см.RemStr)
int ReadString(FILE *f, char *s);
int SkipRemarkLine(FILE *f, char *s); // Аналог ReadString
int ReadInt(FILE *f, int *n);
int ReadInt(FILE *f);
int Read2Int(FILE *f, int *n1, int *n2);
int Read2Float(FILE *f, float *f1, float *f2);

/// Чтение целочисленного вектора из строки. Формат: <количество элементов> v[0] v[1] ... 
/// Возвращает количество элементов или -1 в случае ошибки
int ReadIntVector(FILE *f, int v[]);

/// Считать следующую лексему из строки
int getfragment(const char *stext, int *n, const char *delimstr, char *result, int ignorefirstdelims = 0);

/// Считать строку формата С из строки
int getCstrfragment(const char *stext, int *n, char *result);

void delay_ms(unsigned int ms);

#endif
