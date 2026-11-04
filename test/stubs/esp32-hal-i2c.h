#pragma once
#include <Wire.h>
#include <cassert>
inline void* i2cBusHandle(uint8_t bus) {
    assert(bus == 1);
    return TwoWire::active && TwoWire::active->busHandleAvailable ? TwoWire::active : nullptr;
}
