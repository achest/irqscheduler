/* Copyright (C) 2019 Alexander Chestnov  All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Alexander Chestnov 
 Web      :  https://github.com/achest/
 */

#ifndef _IrqScheduler_h
#define _IrqScheduler_h

#include <stdlib.h>
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif

#define RESOLUTION 65536  
// These defs cause trouble on some versions of Arduino
#undef round

#ifndef ESP8266
  #include <avr/io.h>
  #include <avr/interrupt.h>
#endif

#include "LinkedList.h"

//#define  DEBUG

extern "C" {
	typedef void (*SchedulerTask)(void);
	
}

class Task {
	public:
		SchedulerTask callback;
		long  scheduledtime;
		
		Task(SchedulerTask  func,long stime) {
			callback = func;
			scheduledtime = stime;
		};
		long getScheduledtime() {
			 return scheduledtime;
		} ;
		
};

class IrqScheduler {
public:
    IrqScheduler() {
   
    };
	
	boolean addTask(SchedulerTask callback , long time) {
		
		long now = micros();
		Task * newtask = new Task (callback,now+time*1000); 
		
		#ifdef DEBUG
				printTime();
				Serial.print("addTask:Task "); 
				Serial.print(time*1000); 

		#endif
		
		if (taskList.size() == 0) {
			taskList.add(newtask);
			initTimer(time*1000);
			 #ifdef DEBUG
				printTime();
				Serial.print("addTask:firstTask"); 
			#endif
		}
		boolean ready=false;
		for (int i = 0; i < taskList.size(); i++ ) {
			Task *schedTask = taskList.get(i);
			if (schedTask->getScheduledtime() > now+time*1000 ) {
				taskList.add(i, newtask); 
				ready = true;
			 #ifdef DEBUG
				printTime();
				Serial.print("addTask:ready"); 
			#endif

				break;
				}
			
		}
		if (!ready ) {
			taskList.add(newtask);
		    #ifdef DEBUG
				printTime();
				Serial.print("addTask:addEnd"); 
			#endif

		}
		
	}	;


    
    void isrCallback() {
		
		if (taskList.size()== 0) {
			#ifdef DEBUG
				printTime();
				Serial.print("isrCallback:stopTimer, Detatch intterupt"); 
			#endif
			#ifndef ESP8266
				TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));          // clears all clock selects bits
			#else
				timer0_detachInterrupt();
			#endif	
			return;
		}
		
		Task * t = taskList.get(0);
		taskList.remove(0);
		#ifdef DEBUG
			printTime();
			Serial.print("isrCallback:callTask"); 
		#endif
		t->callback ();

		if (taskList.size()>=1) {
			Task * nextTask = taskList.get(0);
			long now = micros();
			if (nextTask->getScheduledtime()-now > 0) { 
				
				setPeriod (nextTask->getScheduledtime()- now);
				#ifdef DEBUG
					printTime();
					Serial.print("isrCallback:setPeriod"); 
				#endif
			} else { 		    
				#ifdef DEBUG
					printTime();
					Serial.print(" isrCallback:too late"); 
				#endif
				setPeriod (1); // too late
			} 
	    }
	    
		
	};
	
private:
	LinkedList <Task * > taskList = LinkedList<Task*>();

    void setPeriod(long microseconds) ;
    void initTimer(long microseconds) ;
    unsigned char clockSelectBits;
    
    void printTime() {
		#ifdef DEBUG
					Serial.print("\n "); 
					Serial.print(micros()); 
					Serial.print(": "); 
					
		#endif
		
	}
    
};

extern IrqScheduler irqScheduler;
IrqScheduler irqScheduler;   // preinstatiate



#ifndef ESP8266
ISR(TIMER1_OVF_vect)          // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  irqScheduler.isrCallback();
}
#else

void inline timer0ISR(void){
	
  irqScheduler.isrCallback();	
  
}
#endif 

void IrqScheduler::setPeriod(long microseconds) {
	
	  #ifndef ESP8266	 
		
		  long cycles = (F_CPU * microseconds) / 2000000;                                // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
		  if(cycles < RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal
		  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
		  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
		  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
		  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
		  else        cycles = RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum
		  ICR1 = cycles;                                                     // ICR1 is TOP in p & f correct pwm mode
		  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
		  TCCR1B |= clockSelectBits;                                                     // reset clock select register
	  #else
		 timer0_write(ESP.getCycleCount() +clockCyclesPerMicrosecond()*microseconds);
	  #endif
	};
    void IrqScheduler::initTimer(long microseconds) {
	
		#ifndef ESP8266
		
		  TCCR1A = 0;                 // clear control register A 
		  TCCR1B = _BV(WGM13);        // set mode as phase and frequency correct pwm, stop the timer
		  if(microseconds > 0) setPeriod(microseconds);
		  TIMSK1 = _BV(TOIE1);                                     // sets the timer overflow interrupt enable bit
		  sei();                                                   // ensures that interrupts are globally enabled
		   TCCR1B |= clockSelectBits;
	  #else
		  timer0_isr_init();
		  timer0_attachInterrupt(timer0ISR);
		  setPeriod (microseconds);
	  #endif	
		
	};



#endif
