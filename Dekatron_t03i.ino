/*
  Dekatron Slinky Example - with Timer IRQ

  modified April 7, 2017
  by Michael Moorrees
 */
#include <TimerOne.h>

int swing = 2;        // swing range - 2 to 30
int Scnt = 0;         // step count
int state = 0;        // State - Machine State
int dk_ct = 0;        // Dekatron Guide count - 0,1, or 2
int LED = 13;          // Test LED
int Guide1 = 5;       // Guide 1 - G1 pin of 2-guide Dekatron
int Guide2 = 6;       // Guide 2 - G2 pin of 2-guide Dekatron
int Index  = 7;       // Index   - NDX input pin. High when glow at K0
int Ndx = 0;          // K0 index indicator variable
int Tick = false;     // 1mS tick

void d_step(int dk_bt) // Dekatron Step
{
  if (digitalRead(Index)) Ndx = true;   // Sample for glow at K0
  switch (dk_bt) {
    case 0:                             // glow at a main cathode
      digitalWrite(Guide1, LOW);
      digitalWrite(Guide2, LOW); 
      break; 
    case 1:                             // glow to a Guide 1 cathode
      digitalWrite(Guide1, HIGH);
      digitalWrite(Guide2, LOW); 
      break; 
    case 2:                             // glow to a Guide 2 cathode
      digitalWrite(Guide1, LOW);
      digitalWrite(Guide2, HIGH); 
  }
}

void D_adv()                    // Dekatron Advance - Clockwise
{
  dk_ct++;
  if (dk_ct == 3) dk_ct = 0;
  d_step(dk_ct);
}

void D_rev()                    // Dekatron Reverse - Counter-Clockwise
{
  dk_ct--;
  if (dk_ct == -1) dk_ct = 2;
  d_step(dk_ct);
}

void dk_action0() {             // Dekatron Action Routine 0 - Grow from Bottom, Forward (cw)
   if (Ndx) {                   //   When swing hits Ndx [K0] cathode, then max swing is achieved 
    Ndx = false;                //   Set vars for next state, and jump to state 2
    state = 2;
    swing = 30;
    Scnt = (swing / 2) - 1;
    digitalWrite(LED, HIGH);
  }
  else {                        // If not at K0 yet, but stepped "swing" number of steps
    if (Scnt == 0) {            //   increment swing length, preset count
      swing++;                  //   and jump to state 1, for reverse stepping
      Scnt = swing;
      state = 1;
    }
    else {                      // otherwise, continue stepping forward,
      Scnt--;                   //   while decrementing step counter.
      D_adv();
    }
  }
}

void dk_action1() {             // Dekatron Action Routine 1 - Grow from, either bottom, or top, Reverse (ccw)
  if (Scnt == 0) {              //   at full steps (count = 0), increase swing, preset count
    state--;                    //   and jump to forward growth state (0 or 4)
    swing++;                    //   This routine is called while in either state 1 (bottom) or state 5 (top)
    Scnt = swing;
  }
  else {                        // If not yet at full step
    Scnt--;                     //   contiue stepping in reverse
    D_rev();                    //   while drecrementing counter
  }
}

void dk_action2() {             // Dekatron Action Routine 2 - Shrink from, either bottom, or top, Forward (cw)
  if (swing < 2) {              //   until swing is at minimum (2)
    state = (state + 2) & 7;    //   then jump to state 4 (if at 2 - top) or 0 (if at 6 - bottom)
//  if (state == 2) state = 4;
//  if (state == 6) state = 0;
    swing = 2;
    Scnt = swing;
    Ndx = false;
  }
  else {                        // at full forward step (count = 0),
    if (Scnt == 0) {            //   decrease swing length, preset count
      swing--;                  //   and go to reverse shrink state:
      Scnt = swing;             //   state 3 (if at state 2 - top) or
      state++;                  //   state 7 (if at state 6 - bottom)
    }
    else {
      Scnt--;                   // Otherwise, contine stepping forward
      D_adv();                  //   while decrementing counter
    }
  }
}

void dk_action3() {             // Dekatron Action Routine 3 - Shrink from, either bottom, or top, Reverse (ccw)
  if (Scnt == 0) {              //   at full steps (count = 0),
    state--;                    //   decrease swing length, and jump to
    swing--;                    //   state 2 (if at state 3 - top) or
    Scnt = swing;               //   state 6 (if at state 7 - bottom)
  }
  else {                        // Otherwise, continue stepping in reverse
    Scnt--;                     //   while decrementing counter
    D_rev();
  }
}

void dk_action4() {             // Dekatron Action Routine 4 - Grow from Top, Forward (cw)
  if (swing >= 30) {            //   When swing length reaches 30 (maximum),
   state = 6;                   //   jump to state 6, and preset swing and count
   swing = 30;
   Scnt = (swing / 2) - 1;
  }
  else {                        // If not yet at full swing, but hit maximum steps,
   if (Scnt == 0) {             //   (count = 0), then increase swing length,
    swing++;                    //   preset count and go to state 5 (reverse)
    Scnt = swing;
    state = 5;
   }
   else {                       // Otherwisw, continue stepping forward
    Scnt--;                     //   while decrementing counter.
    D_adv();
   }
  } 
}

// setup() runs once, at reset, to initialize system
// initialize hardware and preset variables
void setup() {
  Timer1.initialize(1000);
  Timer1.attachInterrupt( timerISR );
  pinMode(Guide1, OUTPUT);
  pinMode(Guide2, OUTPUT);
  pinMode(Index, INPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(LED, LOW);
  swing = 2;
  Scnt = 0;
  state = 0;
  Ndx = 0;
  D_adv();
}

// Main Loop - Slinky Pattern State Machine
//
//  States: 0 <--> 1 until max swing [Growing, at bottom]
//          2 <--> 3 until min swing [Shrinking, at top]
//          4 <--> 5 until max swing [Growing, at top]
//          6 <--> 7 until min swing [Shrinking, at bottom]
//          at min go back to state 0
//
void loop() {
  if (Tick) {
    Tick = false;
    switch (state) {             // Do action per current state
    case 0:
      dk_action0();            // Grow from Bottom, Forward
      break;
    case 1:
      dk_action1();            // Grow, Reverse
      break;
    case 2:
      dk_action2();            // Shrink, Forward
      digitalWrite(LED, LOW);
      break;
    case 3:
      dk_action3();            // Shrink, Reverse
      break;
    case 4:
      dk_action4();            // Grow from Top, Forward
      break;
    case 5:
      dk_action1();            // Grow, Reverse
      break;
    case 6:
      dk_action2();            // Shrink, Forward
      break;
    case 7:
      dk_action3();            // Shrink, Reverse
  }
 // delay(1);                    // Pause 1mS
}
}

void timerISR() {
  Tick = true;
}
