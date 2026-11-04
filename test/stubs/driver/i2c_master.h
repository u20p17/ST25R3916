#pragma once
#include <Wire.h>
#include <cassert>
#include <freertos/task.h>
#include <array>
#include <deque>
#include <vector>
#include <algorithm>
#include <cstring>
using i2c_master_bus_handle_t = TwoWire*;
struct Device { TwoWire* bus; };
using i2c_master_dev_handle_t = Device*;
inline constexpr int I2C_ADDR_BIT_LEN_7 = 0;
struct i2c_device_config_t {
    int dev_addr_length; uint16_t device_address; uint32_t scl_speed_hz;
};
struct Reply { std::vector<uint8_t> bytes; bool badCrc = false; };
inline std::array<uint8_t,64> registers{};
inline std::deque<Reply> replies;
inline std::vector<uint8_t> tx, rx;
inline std::vector<std::vector<uint8_t>> sentFrames;
inline bool busFails = false;
inline unsigned busTime = 100;
inline uint8_t chipIdentity = 0x28;
inline esp_err_t i2c_master_bus_add_device(TwoWire* bus, const i2c_device_config_t* cfg, Device** out) {
    assert(bus->busInitialized && cfg->device_address == 0x50 && cfg->scl_speed_hz == 400000);
    if (bus->deviceAddStatus != ESP_OK) return bus->deviceAddStatus;
    *out = new Device{bus}; ++bus->liveDevices; ++bus->deviceAdds;
    registers[0x3F] = chipIdentity;
    return ESP_OK;
}
inline esp_err_t i2c_master_bus_rm_device(Device* dev) {
    assert(dev && dev->bus->liveDevices == 1);
    --dev->bus->liveDevices; ++dev->bus->deviceRemoves; delete dev; return ESP_OK;
}
inline esp_err_t i2c_master_transmit(Device* dev, const uint8_t* data, size_t len, int timeout) {
    assert(dev && timeout == 50); fakeMicros += busTime;
    if (busFails) return ESP_ERR_TIMEOUT;
    if (data[0] == 0x80) { tx.assign(data+1,data+len); return ESP_OK; }
    if (len == 2 && data[0] < 64) {
        if (data[0] == 2 && !(registers[2]&0x80) && (data[1]&0x80)) registers[0x1A] |= 0x80;
        registers[data[0]]=data[1]; return ESP_OK;
    }
    assert(len == 1);
    if (data[0] == 0xC1) { registers.fill(0); registers[0x3F]=chipIdentity; }
    if (data[0] == 0xDB) { tx.clear(); rx.clear(); }
    if (data[0] == 0xC4 || data[0] == 0xC5 || data[0] == 0xC7) {
        sentFrames.push_back(data[0] == 0xC7 ? std::vector<uint8_t>{0x52} : tx);
        registers[0x1A]=8;
        if ((data[0] == 0xC5 && !tx.empty() && tx[0] == 0x6A) || replies.empty()) {
            registers[0x1B]=0x40; return ESP_OK;
        }
        auto reply=replies.front();replies.pop_front();rx=reply.bytes;
        if (!(registers[0x0A]&0x80)) {rx.push_back(0);rx.push_back(0);}
        registers[0x1A]|=0x10;
        registers[0x1C]=reply.badCrc ? 0x80 : 0;
    }
    return ESP_OK;
}
inline esp_err_t i2c_master_transmit_receive(Device* dev, const uint8_t* command, size_t n,
    uint8_t* out, size_t len, int timeout) {
    assert(dev && n == 1 && timeout == 50);fakeMicros += busTime;
    if (busFails) return ESP_ERR_TIMEOUT;
    if (*command == 0x9F) {
        if (len > rx.size()) return ESP_FAIL;
        std::copy_n(rx.begin(),len,out);rx.erase(rx.begin(),rx.begin()+len);return ESP_OK;
    }
    assert(len == 1 && (*command&0x40));
    auto reg=*command&0x3F;
    *out = reg == 0x1E ? rx.size() & 255 : reg == 0x1F ? ((rx.size()>>8)<<6) : registers[reg];
    if (reg >= 0x1A && reg <= 0x1D) registers[reg]=0;
    return ESP_OK;
}
