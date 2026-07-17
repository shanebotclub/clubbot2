#ifndef CLUBBOT2__MSG__BUMPERS_H_
#define CLUBBOT2__MSG__BUMPERS_H_

#ifdef __cplusplus 
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

// ---- Bumpers message ----

typedef struct clubbot2__msg__Bumpers
{
  bool LF_bumper;
  bool MF_bumper;
  bool RF_bumper;
  bool LB_bumper;
  bool MB_bumper;
  bool RB_bumper;
} clubbot2__msg__Bumpers;

#ifdef __cplusplus
}
#endif

#endif  // CLUBBOT2__MSG__BUMPERS_H_
