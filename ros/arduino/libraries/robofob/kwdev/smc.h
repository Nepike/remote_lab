/*
 * Simple Motor Controller
 * 22.08.2018
 * LP 22.08.2018
 */
 
#ifndef _SMC_H_
#define _SMC_H_

class Tsmc
{
public:
  Tsmc(void (*writefuncptr)(byte))
  {
    CWrite = writefuncptr;
  }
  void Init(void)
  {
    // if the Simple Motor Controller has automatic baud detection
    // enabled, we first need to send it the byte 0xAA (170 in decimal)
    // so that it can learn the baud rate
    CWrite(0xAA); // send baud-indicator byte
  }
  
  void Go(byte device, int sp);

  // Send the Exit Safe Start command, which
  // clears the safe-start violation and lets the motor run.
  // Required to allow motors to move.
  // Must be called when controller restarts and after any error.
  void exitSafeStart(byte device)
  {
    CWrite(0xAA);
    CWrite(device);
    CWrite(0x03);
  }
  
private:
    void (*CWrite)(byte);
};

#endif
