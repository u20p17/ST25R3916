// Adapted from rednblkx/HomeKey-ESP32, commit f167cba1950aa9e37afb9cb06163a78299599a67.
// Copyright (c) 2026 rednblkx. MIT; see ../LICENSE.
#pragma once

// ESP32 Arduino shared-I2C ST25R3916 NFC-A / ISO-DEP driver.
// Owns its I2C device handle, never the TwoWire bus. Call from one task;
// synchronize externally if more than one task uses this instance.
#include <Wire.h>

#include <driver/i2c_master.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

class ST25R3916 {
public:
    // Borrow a shared bus. SDA/SCL must match other users of that bus.
    ST25R3916(TwoWire& wire, int sda, int scl,
              uint8_t i2cAddr = 0x50, uint32_t i2cHz = 400000);
    ~ST25R3916();

    ST25R3916(const ST25R3916&) = delete;
    ST25R3916& operator=(const ST25R3916&) = delete;
    ST25R3916(ST25R3916&&) = delete;
    ST25R3916& operator=(ST25R3916&&) = delete;

    // Switch between presentations. The selection is volatile; reboot uses fast.
    static void setFastPolling(bool enabled) { s_fastPolling.store(enabled); }
    static bool fastPolling() { return s_fastPolling.load(); }
    bool init();
    void stop();
    bool isConnected() const;

    uint8_t getFwMajor() const { return m_icType; }
    uint8_t getFwMinor() const { return m_icRev; }

    bool beginDiscovery();
    bool pollForTag(std::vector<uint8_t>& uid,
                    std::array<uint8_t, 2>& atqa,
                    uint8_t& sak,
                    uint32_t timeoutMs);
    bool isTagStillPresent();
    void releaseTag();
    void endDiscovery();

    bool exchangeApdu(const std::vector<uint8_t>& send,
                      std::vector<uint8_t>& recv,
                      uint32_t timeoutMs);
    bool healthCheck();

    // Copies an optional raw ECP frame, including CRC-A (max 18 bytes).
    // nullptr/0 disables ECP. Invalid input leaves the previous frame intact.
    // The driver never generates HomeKey identifiers or credentials.
    bool setEcpFrame(const uint8_t* frame, size_t length);

private:
    inline static std::atomic<bool> s_fastPolling{true};
#ifdef ST25R3916_TIMING_TEST
    bool m_timingFast = false;
    uint32_t m_timingLastReport = 0, m_timingPolls = 0;
    uint64_t m_timingPollUs = 0;
#endif
    uint32_t m_exchangeStart = 0, m_exchangeBudget = 0;
    bool m_exchanging = false;

    // ---- low level bus access -------------------------------------------
    bool writeReg(uint8_t reg, uint8_t val);
    bool readReg(uint8_t reg, uint8_t& out);
    uint8_t readRegOr(uint8_t reg, uint8_t fallback = 0);
    bool modifyReg(uint8_t reg, uint8_t clearMask, uint8_t setMask);
    bool command(uint8_t cmd);
    bool writeFifo(const uint8_t* data, size_t len);
    size_t readFifo(uint8_t* out, size_t maxLen);
    uint16_t fifoLen();

    // ---- interrupts (polled; no external IRQ wire required) ---------
    uint32_t readInterrupts();
    void clearInterrupts();
    uint32_t waitInterrupt(uint32_t mask, uint32_t timeoutMs);

    // ---- RF --------------------------------------------------------------
    void setField(bool on);
    void setNumTxBytes(uint16_t bytes, uint8_t trailingBits = 0);

    struct RxResult {
        bool ok = false;
        size_t len = 0;
        uint32_t irqs = 0;
        const char* error = nullptr;
    };

    RxResult transceive(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxMax,
                        bool txCrc, bool rxCrc, uint32_t timeoutMs = 20, bool expectReply = true);
    RxResult shortFrame(uint8_t cmd, uint8_t* rx, size_t rxMax, uint32_t timeoutMs = 20);

    // ---- ISO14443-3 / -4 --------------------------------------------------
    bool cascadeLevel(uint8_t sel, uint8_t outUid[4], uint8_t& outSak);
    bool anticollision(std::vector<uint8_t>& uid, uint8_t& sak);
    bool rats(uint8_t fsdi);

    std::array<uint8_t, 18> m_ecpData{};
    size_t m_ecpSize = 0;
    TwoWire& m_wire;
    int m_sda, m_scl;
    uint8_t m_i2cAddr;
    uint32_t m_i2cHz;

    i2c_master_bus_handle_t m_bus = nullptr;
    i2c_master_dev_handle_t m_dev = nullptr;

    bool m_connected = false;
    bool m_fieldUp = false;
    uint8_t m_icType = 0;
    uint8_t m_icRev = 0;

    // ISO-DEP session state, valid between a successful pollForTag() and the
    // next releaseTag().
    bool m_isodepActive = false;
    uint8_t m_blockNum = 0;
    // Rate-limits the informational block-number mismatch notice to once per
    // ISO-DEP session; the presence check runs every ~65 ms.
    bool m_blockMismatchLogged = false;
    uint16_t m_fsc = 32;      // max frame the card accepts, from ATS
    uint32_t m_fwtMs = 5;     // frame waiting time, from ATS

    static constexpr const char* TAG = "ST25R3916";
};

