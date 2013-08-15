#include "p33fxxxx.h"
#include "sensorMCUSlugsMKII.h"
#include "sensorMCUSlugsMKII_private.h"

volatile uint16_T ADCBuffChannelDMA[(4)] __attribute__((space(dma),aligned(256)));
