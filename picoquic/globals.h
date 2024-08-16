#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>
#include <stdbool.h>
 
extern uint64_t new_ack_on_path0; 
extern uint64_t new_ack_on_path1;
extern bool acked_path0; 
extern bool acked_path1; 
extern bool path0init;
extern bool path1init;

#ifdef __cplusplus
extern "C" {
#endif
extern bool learning;
extern bool deployment;
extern int current_timestep;
extern double learning_bytes;
extern bool stop_collect;

#ifdef __cplusplus
}
#endif

#endif // GLOBALS_H
