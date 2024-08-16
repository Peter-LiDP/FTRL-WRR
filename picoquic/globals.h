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
extern int var_blest;

#ifdef __cplusplus
extern "C" {
#endif

extern bool updated;

#ifdef __cplusplus
}
#endif

#endif // GLOBALS_H
