#ifndef CLUBBOT2__MSG__ENCODERS_H_
#define CLUBBOT2__MSG__ENCODERS_H_

#ifdef __cplusplus 
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

// ---- Encoders message ----

typedef struct clubbot2__msg__Encoders
{
  int32_t left;
  int32_t right;
} clubbot2__msg__Encoders;

#ifdef __cplusplus
}
#endif

#endif  // CLUBBOT2__MSG__ENCODERS_H_
