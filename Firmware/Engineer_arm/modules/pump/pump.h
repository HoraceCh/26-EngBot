#ifndef _PUMP_H__
#define _PUMP_H__

#include <stdint.h>

extern void pump_run(uint8_t id);
extern void pump_stop(uint8_t id);
extern uint8_t* get_pump_state_ptr(void);

#endif