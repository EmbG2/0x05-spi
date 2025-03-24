/*
 * File:   timer.c
 * Author: EmbeddedG2
 *
 * Created on February 28, 2025, 11:04 AM
 */


 #include "xc.h"
 #include "timer.h"
 #define FCY 72000000UL
 
 void tmr_setup_period(int timer, int ms) {
     unsigned int prescaler;
     double period;
     // Set the maximum period as <200 ms>
     if (ms > 200){
         ms = 200;
     }
     // Set the pre-scaler layers and the corresponding type
     prescaler = 1;
     int prescaler_type = 0;
     if(ms > 2) {
         prescaler *= 8;     // 8
         prescaler_type++;   // 1
     }
     if(ms > 10) {
         prescaler *= 8;     // 64
         prescaler_type++;   // 2
     }
     if(ms > 50) {
         prescaler *= 4;     // 256
         prescaler_type++;   // 3
     }
     // <f_tick> = f_clock / prescaler = <1 / T_tick>
     // Period of the timer with respect to the period of the tick
     // <period> = duration / (T_tick * 1000)
     // it is divided by 1000 to convert from seconds to milliseconds
     period  = ((double)FCY / (prescaler * 1000)) * ms;
 
     switch(timer){
         case TIMER1:
             T1CONbits.TON = 0;                  // Turn off the Timer1 (stop counting)
             T1CONbits.TCKPS = prescaler_type;   // Set the Timer Clock Prescaler value
             PR1 = period;                       // Period Register for Timer1
             TMR1 = 0;                           // Reset the number of tick counted by the timer
 //            T1CONbits.TON = 1;                  // Turn on the Timer1 (start counting)
             break;
         case TIMER2:
             T2CONbits.TON = 0;
             T2CONbits.TCKPS = prescaler_type;
             PR2 = period;
             TMR2 = 0;
 //            T2CONbits.TON = 1;
             break;
          case TIMER3:
             T3CONbits.TON = 0;
             T3CONbits.TCKPS = prescaler_type;
             PR3 = period;
             TMR3 = 0;
 //            T2CONbits.TON = 1;
             break;
     }
 }
 
 void tmr_wait_period(int timer){
     switch (timer){
         case TIMER1:
             while (IFS0bits.T1IF == 0); // Blocking mode till the duration of the period is reached and signaled by interrupt flag
             IFS0bits.T1IF = 0;          // Put the flag at zero to notify a new event
             break;
         case TIMER2:
             while (IFS0bits.T2IF == 0);
             IFS0bits.T2IF = 0;
             break;
     }
 }
 
 void tmr_wait_ms(int timer, int ms){
     tmr_setup_period(timer, ms);
     tmr_turn(timer, 1);
     tmr_wait_period(timer);
     if (timer == TIMER1){
         T1CONbits.TON = 0;
     } else if (timer == TIMER2){
         T2CONbits.TON = 0;
     }
 }
 
 int tmr_wait_period_3(int timer){
     int expired = 0;
     switch (timer){
         case TIMER1:
             if (IFS0bits.T1IF) {
                 expired = 1;
                 break;
             }
             while (IFS0bits.T1IF == 0);
             IFS0bits.T1IF = 0;
             break;
         case TIMER2:
             if (IFS0bits.T2IF) {
                 expired = 1;
                 break;
             }
             while (IFS0bits.T2IF == 0);
             IFS0bits.T2IF = 0;
             break;
     }
     return expired;
 }
 
 void tmr_wait_ms_3(int timer, int ms){
     int expired;
     while (ms > 0) {
         int wait_time = (ms > 200) ? 200 : ms; 
         tmr_setup_period(timer, wait_time);
         tmr_turn(timer, 1);
         expired = tmr_wait_period_3(timer);
         ms -= wait_time;
     }
 }
 
 void tmr_turn(int timer, int value){
     switch(timer){
         case TIMER1:
             T1CONbits.TON = value;                  // Turn on the Timer1 (start counting)
             break;
         case TIMER2:
             T2CONbits.TON = value;
             break;
         case TIMER3:
             T3CONbits.TON = value;
             break;
     }
 }
 