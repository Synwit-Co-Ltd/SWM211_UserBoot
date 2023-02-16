#ifndef __SWM2X1_H__
#define __SWM2X1_H__


#if defined(CHIP_SWM201)

#include "SWM201.h"

#elif defined(CHIP_SWM211)

#include "SWM211.h"

#else

#error define CHIP_SWMXXX please!

#endif


#endif //__SWM2X1_H__
