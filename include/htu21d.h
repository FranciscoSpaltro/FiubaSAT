#ifndef HTU21D_H
#define HTU21D_H

#include "i2c.h"

// HTU21D
#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5
#define SOFT_RESET  0xFE
#define HTU21D_ADDRESS 0x40

BaseType_t reset_htu21d(uint32_t i2c_id);
BaseType_t request_htu21d(uint32_t i2c_id, uint8_t command, uint8_t *data);

#endif 