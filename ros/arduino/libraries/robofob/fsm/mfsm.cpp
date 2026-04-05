
/*
 * \author Maxim Rovbo
 * \version 1.02
 * \date 19.02.2018
 */

#include <Arduino.h>
#include "mfsm.h"


//---------------------------
// Вспомогательные функции
//---------------------------

static void PrintFsmRule(FsmRule * rule)
{
  Serial.print(F("From ["));
  Serial.print(rule->from_);
  Serial.print(F("] to ["));
  Serial.print(rule->to_);
  Serial.println(F("]"));
}

//---------------------------
// Автомат
//---------------------------

byte FiniteStateMachine::Step()
{
  time_++;

  if (trace_)
  {
    Serial.print(F("FSM STEP (name "));
    Serial.print(name_);
    Serial.print(F(") Q = "));
    Serial.println(state_);
  }

  // check if current state is the final one
  if (strchr(terminals_, state_))
  {
    status_ = FSM_FINISHED;
    return FSM_FINISHED;
  }

  // determine a rule to execute
  FsmRule * active_rule_ptr = NULL; // activated rule with the topmost priority (least priority value)

#ifndef _NO_INSPECT_RULES
  if (trace_)
    Serial.println(F("Inspecting applicable rules"));
#endif // _NO_INSPECT_RULES

  FsmRule * default_rule_ptr = NULL; // get executed if no other rule was activated
  for (byte i = 0; i < n_rules_; i++)
  {
    FsmRule * rule_ptr = rules_ + i;
    if (rule_ptr->from_ == state_)
    {
      // no conditions means this is a default rule
      if ( !(rule_ptr->condition_) )
      {
#ifndef _NO_INSPECT_RULES
        if (trace_)
          Serial.println(F("Default rule encountered"));
#endif // _NO_INSPECT_RULES

        default_rule_ptr = rule_ptr;
      }
      else
      {
        // if rule is active
        if (rule_ptr->condition_())
        {
          if(trace_)
            PrintFsmRule(rule_ptr);

          // if no active rule yet
          if ( !( active_rule_ptr ) )
          {
            active_rule_ptr = rule_ptr;
          }
          else
          {              
            // compare priorities
            if ( rule_ptr->priority_ < active_rule_ptr->priority_)
            {
              active_rule_ptr = rule_ptr;
            }
          }
        }
      }
    }
  }
  
  // if no active rule found, use default rule instead
  if ( !(active_rule_ptr) )
  {
    if (!default_rule_ptr)
      // no rule found is very bad
      Serial.println(F("CRITICAL: No rule selected"));
    else
      active_rule_ptr = default_rule_ptr;
  }

  // execute active rule with highest priority
  if (active_rule_ptr)
  {
#ifndef _NO_INSPECT_RULES
    if(trace_)
    {
      Serial.println(F("Executing rule: "));
      PrintFsmRule(active_rule_ptr);
    }
#endif // _NO_INSPECT_RULES

    // Resetting timer if rule asks for it
    if (active_rule_ptr->do_timer_reset_)
      time_ = 0;
      
    // if any action required (pointer is 0 otherwise)
    if (active_rule_ptr->action_)
      active_rule_ptr->action_();
    
    // if any extra actions exist, do them too
    if (active_rule_ptr->extra_actions_)
    {
      for (int i = 0; i < active_rule_ptr->n_extra_actions_; i++)
        active_rule_ptr->extra_actions_[i]();
    }

    // transferring to the next state
    state_ = active_rule_ptr->to_;
  }
  else 
  {
    Serial.println(F("CRITICAL: No active rule, yet terminal state not reached"));
  }
  
  status_ = FSM_NOT_FINISHED;
  
  return status_;
}

FiniteStateMachine * FiniteStateMachine::current_fsm_ptr = 0;

//---------------------------
// Предикаты для автоматов
//---------------------------

// Check time (number of ticks) for the current FSM. A workaround, because all function pointers need
// to point to global space (not to object methods)
bool predicates::CheckTimer()
{
  return FiniteStateMachine::current_fsm_ptr->CheckTimer();
}

bool (*predicates::ELSE)(void) = NULL;
bool (*predicates::TRUE)(void) = NULL;

void (*actions::DO_NOTHING)(void);

