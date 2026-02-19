/*
 * Meley FSM
 * 25.03.2016
 * 25.03.2016
*/

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "auto.h"

void info(const char *fmt, ...)
{
  char s[512];
  va_list argptr;
  va_start(argptr, fmt);
  vsprintf(s, fmt, argptr);
  va_end(argptr);

  printf("\r\n%s\r\n",s);
  fflush(stdout);
}

void AError(TAutomaton *a, const char *fmt, ...)
{
  char s[512];
  va_list argptr;
  va_start(argptr, fmt);
  vsprintf(s, fmt, argptr);
  va_end(argptr);

  if(a) a->Show(stderr);
  fprintf(stderr, "\n%s\n",s);
  fflush(stderr);
  getchar();
  exit(1);
}

void TAutomaton::AddRules(TRule *R[]) 
{
  int i=0;
  while(R[i])
    AddRule(R[i++]);
}

int TAutomaton::Step(void)
{
  tcnt++;
  // Ищем подходящее правило
  int nr = Rules.size();
  for(int i=0;i<nr;i++)
  {
    if(Rules[i]->src==q)
    {
      if(Rules[i]->Eval()==1) // Все хорошо
      {
        Rules[i]->RunProc();
        q = Rules[i]->dest;
        if(q==qt) return 0;
        return 1;
      }
    }
  }
  // Ищем else-правило
  for(int i=0;i<nr;i++)
  {    
    if(Rules[i]->src==q)
    {
      if(Rules[i]->isDefaultRule())
      {
        Rules[i]->RunProc();
        q = Rules[i]->dest;
        if(q==qt) return 0;
        return 1;
      }
    }
  }
  return -1;
}
