/**
 * Primitive Finite-state machine library
 *
 * 20.11.2017
 * Version 1.02
 * LP 20.01.2018
 */

#ifndef _FSM_LIB_H_
#define _FSM_LIB_H_

#ifndef byte
  typedef unsigned char byte;
#endif

struct TRule
{
  byte q1, q2;
  byte a;
  byte proc;
};

struct TFSM
{
  byte q0;
  byte q;
  TRule *Rules;
  byte rnum;
  TFSM(byte nq0, TRule R[], byte nrnum)
  {
    q0 = q = nq0;
    rnum = nrnum;
    Rules = R;
  }
  int Accept(byte s);
  void Reset(void) {q = q0;}
};

#endif
