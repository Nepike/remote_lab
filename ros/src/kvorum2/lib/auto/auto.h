/*
 * Meley FSM
 * 25.03.2016
 * 25.03.2016
*/

#ifndef _MFSM_AUTO_H_
#define _MFSM_AUTO_H_

#include <stdarg.h>
#include <vector>

class TAutomaton;

typedef const char* (*FUNC_PREDICATE)(TAutomaton *, int);
typedef const char* (*FUNC_PROCESS)(TAutomaton *, int);

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

void info(const char *fmt, ...);
void AError(TAutomaton *a, const char *fmt, ...);

class TRule
{
private:
  FUNC_PREDICATE predicate;
  FUNC_PROCESS procedure;
public:
  TAutomaton *parent;
  int src, dest; // Вершины источника и приемника дуги
  TRule(int nsrc, int ndest, FUNC_PREDICATE npred, FUNC_PROCESS nproc)
  {
    src = nsrc;
    dest = ndest;
    predicate = npred;
    procedure = nproc;
    parent = NULL;
  }
  void SetParent(TAutomaton *p)
  {
    parent = p;
  }
  int Eval(void)
  {
    if(predicate==NULL) return -1;
    int res = (*predicate)(parent, 0)!=NULL;
    return res;
  }
  int RunProc(void)
  {
    if(procedure==NULL) return -1;
    int res = (*procedure)(parent, 0)!=NULL;
    return res;
  }
  int isDefaultRule(void)
  {
    return (predicate==NULL);
  }
  const char *GetPredicateName(void)
  {
    if(predicate==NULL) return "NULL";
    const char *s=(*predicate)(parent, 1);
    return s;
  }
  const char *GetProcedureName(void)
  {
    if(procedure==NULL) return "NULL";
    const char *s=(*procedure)(parent, 1);
    return s;
  }
  void Show(const char *msg=NULL)
  {
    if(msg) printf("%s", msg);
    printf("Rule: %d->%d cond = %s, proc = %s\n", src, dest, GetPredicateName(), GetProcedureName());
  }
};

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

class TAutomaton
{
private:
  std::vector<TRule*> Rules;
  int q;  // Текущее состояние
  int q0; // Начальное состояние
  int qt; // Конечное состояние
  const char *name;
  int id;
public:
  int tcnt;
  int tcnt_lim;
  TAutomaton(int nid, const char *cname, int nq0, int nqt, int ntcnt_lim)
  {
    id = nid;
    name = cname;
    q0 = nq0;
    qt = nqt;
    q = q0;
    tcnt = 0;
    tcnt_lim = ntcnt_lim;
  }
  void AddRules(TRule *R[]);
  void AddRule(TRule *R)
  { 
    R->SetParent(this);
    Rules.push_back(R);
  }
  int CurrState(void) { return q; }
  void SetCurrState(int nq) { q = nq; }
  void Reset(void) { q = q0; tcnt = 0; }
  int Step(void); // 0 - переход в конечное состояние, 1 - продолжение работы, -1 - ошибка
  void Show(FILE *f = stdout)
  {
    fprintf(f, "-- %s.q = %d\n", GetName(), q);
    fflush(f);
  }
  const char* GetName(void) { return name; }
  int GetId(void) { return id; }
  int TcntCheck(void) { return (tcnt<tcnt_lim); }
};

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

#define MAX_STACK 10
class TAStack
{
private:
  int top;
  TAutomaton *D[MAX_STACK];
public:
  TAStack(void) { top = 0; }
  void Push(TAutomaton *a, int ignoreoverflow = 1)
  {
    D[top] = a;
    top++;
    if(top>=MAX_STACK)
    {
      if(!ignoreoverflow) AError(NULL, "TStack Push");
      else top--;
    }
  }
  TAutomaton *Pop(void)
  {
    if(top==0)
      AError(NULL, "TStack Pop");
    top--;
    return D[top];
  }
  int Len(void) { return top; }
  TAutomaton *GetLast(void)
  {
    if(top==0)
      return NULL;
    return D[top-1];
  }
  void Clear(void) { top = 0; }
};

#endif
