/* 
 * File:   
 * Author: EMBG2
 * Comments:
 * Revision history: 
 */

#ifndef TIMER_H
#define	TIMER_H

#include <xc.h> 
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3

void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void tmr_wait_ms(int timer, int ms);
int tmr_wait_period_3(int timer);
void tmr_wait_ms_3(int timer, int ms);
void tmr_turn(int timer, int value);


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* TIMER_H */

