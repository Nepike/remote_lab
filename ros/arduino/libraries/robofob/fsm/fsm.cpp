/**
 * Primitive Finite-state machine library
 * 
 * 20.11.2017
 * Version 1.02
 * LP 20.01.2018
 */

#include "fsm.h"

int TFSM::Accept(byte s)
{
  for(byte i=0;i<rnum;i++)
    if(Rules[i].q1==q && (Rules[i].a==s || Rules[i].a==0))
    {
      q = Rules[i].q2;
      return Rules[i].proc;
    }
  return -1;
}
