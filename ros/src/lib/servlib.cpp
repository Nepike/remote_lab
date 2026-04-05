/**
 * SERV library (Linux)
 * Сервисная библиотека. Много полезных функций
 * \author Robofob
 * \version 1.04
 * \date 18.01.2014
 * \date LP 06.03.2020
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include "servlib.h"

/// Символы строки-комментария для функции ReadString
char RemStr[] = "#;";

/// Флаг ожидания нажатия клавиши Enter после выдачи сообщения об ошибке (функция error)
int error_wait_for_enter = 0;

char *newstr(char *s)
{
  char *v = new char[strlen(s)+1];
  strcpy(v,s);
  return v;
}

void error(const char *fmt, ...)
{
  char s[512];
  va_list argptr;
  va_start(argptr, fmt);
  vsprintf(s, fmt, argptr);
  va_end(argptr);

  fprintf(stderr,"\nError: %s\n",s);
  perror("");
  if(error_wait_for_enter)
    //gets(s);
    getchar();
  exit(1);
}

void warning(const char *fmt, ...)
{
  char s[512];
  va_list argptr;
  va_start(argptr, fmt);
  vsprintf(s, fmt, argptr);
  va_end(argptr);
  printf("\nWarning: %s\n",s);
}

void trim(char *s)
{
  l_string v;
  strcpy(v,s);
  int n=(int)strlen(v)-1;
  while((unsigned char)v[n]<=32 && n>0)
  { v[n]=0; n--; }

  char *vv = v;
  while((int)strlen(vv)>0 && ((unsigned char)vv[0]<=32))
    vv++;
  strcpy(s,vv);
}

int find_char(char c, const char *s)
{
  int i;
  for(i=0;*s;i++) if(*s++==c) return i;
  return -1;
}

int V_scanf(FILE *point, char *msg, int len)
{
  msg[0] = 0;
  if(fgets(msg, len, point) == NULL)
  {
    if(msg[0]) return 1;
    return 0;
  }
  trim(msg);
  return 1;
}

int ReadString(FILE *f, char *s)
{
  int k;
  s[0] = 0;
  do
  {
    if(!V_scanf(f,s,sizeof(l_string)))
    {
      s[0] = 0;
      return 0;
    }
    trim(s);
    if(s[0])
    {
      k = find_char(s[0], RemStr);
      if(k>=0) s[0] = 0;
    }
  } while (s[0]==0);
  return 1;
}

int SkipRemarkLine(FILE *f, char *s)
{
  return ReadString(f, s);
}

int ReadInt(FILE *f, int *n)
{
  l_string s;
  if(!ReadString(f, s)) return 0;
  *n = atoi(s);
  return 1;
}

int ReadInt(FILE *f)
{
  l_string s;
  if(!ReadString(f, s)) return 0;
  int n = atoi(s);
  return n;
}

int Read2Int(FILE *f, int *n1, int *n2)
{
  l_string s;
  if(!ReadString(f, s)) return 0;
  if(sscanf(s, "%d %d", n1, n2)!=2) return 0;
  return 1;
}

int Read2Float(FILE *f, float *f1, float *f2)
{
  l_string s;
  if(!ReadString(f, s)) return 0;
  if(sscanf(s, "%f %f", f1, f2)!=2) return 0;
  return 1;
}

int ReadIntVector(FILE *f, int v[])
{
  char *p;
  l_string s;
  if(!ReadString(f, s)) return -1;
  p = strtok(s, " ,");
  int n = atoi(p);
  for(int i=0;i<n;i++)
  {
    p = strtok(NULL, " ,");
    if(!p) return -1;
    v[i] = atoi(p);
  }
  return n;
}

int getfragment(const char *stext, int *n, const char *delimstr, char *result, int ignorefirstdelims)
{
  int len = strlen(stext), k = *n, i = 0;
  result[0] = 0;
  if(k>=len) return 0;
  if(!ignorefirstdelims)
  {
    while(k<len && find_char(stext[k],delimstr)>=0) k++;
    if(k>=len) return 0;
  }
  while(k<len && find_char(stext[k],delimstr)<0)
    result[i++] = stext[k++];
  k++;
  *n = k;
  result[i] = 0;
  return 1;
}

int getCstrfragment(const char *stext, int *n, char *result)
{
  int len = strlen(stext), k = *n, i = 0;
  int cn;
  char s[4];
  result[0] = 0;
  if(k>=len) return 0;
  while(k<len && stext[k]!='\"') k++;
  k++;
  if(k>=len) return 0;
  int eoj = 0;
  char c;
  do
  {
    if(k>=len) break;
    c = stext[k++];
    switch(c)
    {
      case '\\':
      {
        if(k>=len) { result[0] = 0; return 0; }
        c = stext[k++];
        switch(c)
        {
          case 'n': c = '\n'; break;
          case 'r': c = '\r'; break;
          case 't': c = '\t'; break;
          case '0': s[0]= stext[k++];
                    s[1]= stext[k++];
                    s[2]= stext[k++];
                    s[3] = 0;
                    sscanf(s,"%o",&cn);
                    c = (char)cn; break;
        }
        result[i++] = c;
        break;
      }
      case '\"': eoj = 1; break;
      default: result[i++] = c;
    }
  } while(!eoj);
  k++;
  *n = k;
  result[i] = 0;
  return 1;
}

void delay_ms(unsigned int ms)
{
  usleep(ms*1000);
}
