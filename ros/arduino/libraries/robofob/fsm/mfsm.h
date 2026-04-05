/*
 * \author Maxim Rovbo
 * \version 1.02
 * \date 19.02.2018
 */


#ifndef _M_FSM_H_
#define _M_FSM_H_

#include <Arduino.h>

// memory saving definitions
#define _NO_INSPECT_RULES 1


//---------------------------
// Автомат
//---------------------------
struct FsmRule
{
  char from_;
  char to_;
  bool (*condition_)(void);
  void (*action_)(void);
  int priority_;

  bool do_timer_reset_; // for resetting fsm timer on action
  void (**extra_actions_)(void); // usually used for utility operations with timer, 
    // counters, etc.
  byte n_extra_actions_; // number of extra actions
};

#define FSM_FINISHED 0
#define FSM_NOT_FINISHED 1

class FiniteStateMachine
{
  /*
   * TODO: states are useless, the actual ones are only in rules. states in the fsm DO NOT correspond to the ones in the 
   * rules and only the rules actually matter.
   */
  public:
    FiniteStateMachine(const char * name, const char * states, byte n_states, const char * terminals, byte n_terminals, char starting_state, FsmRule rules[], byte n_rules)
      : trace_(false), name_(name), states_(states), n_states_(n_states), terminals_(terminals), n_terminals_(n_terminals),
        starting_state_(starting_state), state_(starting_state), status_(FSM_NOT_FINISHED), rules_(rules), n_rules_(n_rules)
    {
    }

    boolean trace_;

    void SetTimerCounter(unsigned int counter)
    {
      timer_counter_ = counter;
    }

    /*
     * Returns true while timer is not over yet.
     */
    bool CheckTimer()
    {
#ifndef _NEED_MEMORY
      if (trace_)
      {
        Serial.print(F("Checking timer [time "));
        Serial.print(time_);
        Serial.print(F("]) and threshold ["));
        Serial.print(timer_counter_);
        Serial.println(F("])"));
      }
#endif // _NEED_MEMORY
      return time_ < timer_counter_;
    }

    byte Step();

    void Reset()
    {
      state_ = starting_state_;
      time_ = 0;
      status_ = FSM_NOT_FINISHED;
    }
    
    // for Debug
    char GetState()
    {
      return state_;
    }
    
    const char * GetName()
    {
      return name_;
    }

    static FiniteStateMachine * current_fsm_ptr;

private:    

    const char * name_;
    
    const char * states_;
    byte n_states_;
    
    const char * terminals_;
    byte n_terminals_;
    
    char starting_state_;
    
    unsigned int timer_counter_; // threshold for timer
    unsigned int time_;          // number of cycles ran in step

    char state_;
    byte status_;
    
    FsmRule * rules_;
    byte n_rules_;

};

//---------------------------
// Предикаты для автоматов
//---------------------------

namespace predicates
{
// Check time (number of ticks) for the current FSM. A workaround, because all function pointers need
// to point to global space (not to object methods)
bool CheckTimer();

extern bool (*ELSE)(void);
extern bool (*TRUE)(void);

} // namespace predicates

//---------------------------
// Процедуры для автоматов
//---------------------------
namespace actions
{
  
extern void (*DO_NOTHING)(void);

} // namespace actions

#endif
