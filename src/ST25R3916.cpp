// ST25R3916 I2C reader adapted from rednblkx/HomeKey-ESP32.
// Copyright (c) 2026 rednblkx. MIT; see ../LICENSE.
// Reference commit f167cba1950aa9e37afb9cb06163a78299599a67.
#include "ST25R3916.hpp"
#include <algorithm>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstring>
#ifdef ST25R3916_TIMING_TEST
#include <cstdio>
#endif

namespace {

// ---- I2C access modes -----------------------------------------------------
constexpr uint8_t OP_WRITE_REGISTER = 0x00;  // 00aaaaaa
constexpr uint8_t OP_READ_REGISTER = 0x40;   // 01aaaaaa
constexpr uint8_t OP_LOAD_FIFO = 0x80;       // 10000000
constexpr uint8_t OP_READ_FIFO = 0x9F;       // 10011111

// ---- registers (space A) --------------------------------------------------
constexpr uint8_t REG_IO_CONF2 = 0x01;
constexpr uint8_t REG_AD_RESULT = 0x25;
constexpr uint8_t REG_REGULATOR_CONTROL = 0x2C;
constexpr uint8_t REG_OP_CONTROL = 0x02;
constexpr uint8_t REG_MODE = 0x03;
constexpr uint8_t REG_BITRATE = 0x04;
constexpr uint8_t REG_AUX = 0x0A;
constexpr uint8_t REG_MASK_MAIN_IRQ = 0x16;
constexpr uint8_t REG_MASK_TIMER_NFC_IRQ = 0x17;
constexpr uint8_t REG_MASK_ERROR_WAKEUP_IRQ = 0x18;
constexpr uint8_t REG_MASK_PASSIVE_TARGET_IRQ = 0x19;
constexpr uint8_t REG_MAIN_IRQ = 0x1A;
constexpr uint8_t REG_TIMER_NFC_IRQ = 0x1B;
constexpr uint8_t REG_ERROR_WAKEUP_IRQ = 0x1C;
constexpr uint8_t REG_PASSIVE_TARGET_IRQ = 0x1D;
constexpr uint8_t REG_FIFO_STATUS1 = 0x1E;
constexpr uint8_t REG_FIFO_STATUS2 = 0x1F;
constexpr uint8_t REG_NUM_TX_BYTES1 = 0x22;
constexpr uint8_t REG_NUM_TX_BYTES2 = 0x23;
constexpr uint8_t REG_IC_IDENTITY = 0x3F;

// ---- direct commands ------------------------------------------------------
constexpr uint8_t CMD_SET_DEFAULT = 0xC1;
constexpr uint8_t CMD_TRANSMIT_WITH_CRC = 0xC4;
constexpr uint8_t CMD_TRANSMIT_WITHOUT_CRC = 0xC5;
constexpr uint8_t CMD_TRANSMIT_WUPA = 0xC7;
constexpr uint8_t CMD_ADJUST_REGULATORS = 0xD6;
constexpr uint8_t CMD_CLEAR_FIFO = 0xDB;
constexpr uint8_t CMD_MEASURE_VDD = 0xDF;
constexpr uint8_t CMD_RC_CAL = 0xEA;

// ---- bits -----------------------------------------------------------------
constexpr uint8_t OP_EN = 0x80;     // oscillator + regulator
constexpr uint8_t OP_RX_EN = 0x40;
constexpr uint8_t OP_TX_EN = 0x08;  // drives the RF field
constexpr uint8_t MODE_OM_ISO14443A = 0x01 << 3;
constexpr uint8_t AUX_NO_CRC_RX = 0x80;

// Interrupt status lives in four read-to-clear registers, packed into one word:
//   0x1A main << 24 | 0x1B timer/nfc << 16 | 0x1C error/wakeup << 8 | 0x1D pt
constexpr uint32_t IRQ_DCT = 0x80ul << 16;
constexpr uint32_t IRQ_OSC = 0x80ul << 24;
constexpr uint32_t IRQ_RXE = 0x10ul << 24;
constexpr uint32_t IRQ_TXE = 0x08ul << 24;
constexpr uint32_t IRQ_NRE = 0x40ul << 16;
constexpr uint32_t IRQ_CRC = 0x80ul << 8;
constexpr uint32_t IRQ_PAR = 0x40ul << 8;
constexpr uint32_t IRQ_ERR2 = 0x20ul << 8;
constexpr uint32_t IRQ_ERR1 = 0x10ul << 8;
constexpr uint32_t IRQ_ANY_ERROR = IRQ_CRC | IRQ_PAR | IRQ_ERR2 | IRQ_ERR1;

constexpr uint8_t IC_TYPE_ST25R3916 = 0x05;
constexpr uint8_t IC_TYPE_ST25R3916B = 0x06;

bool supportedIdentity(uint8_t identity) {
    const uint8_t type = (identity >> 3) & 0x1F;
    // ST RFAL also excludes pre-production B silicon (revision zero).
    return type == IC_TYPE_ST25R3916 ||
        (type == IC_TYPE_ST25R3916B && (identity & 0x07) >= 1);
}

// ---- ISO14443 -------------------------------------------------------------
constexpr uint8_t SEL_CL1 = 0x93;
constexpr uint8_t SEL_CL2 = 0x95;
constexpr uint8_t SEL_CL3 = 0x97;
constexpr uint8_t SAK_CASCADE = 0x04;
constexpr uint8_t SAK_ISODEP = 0x20;
constexpr uint8_t CASCADE_TAG = 0x88;

constexpr uint8_t PCB_I_BLOCK = 0x02;
constexpr uint8_t PCB_BLOCK_NUM = 0x01;
constexpr uint8_t PCB_CHAINING = 0x10;
constexpr uint8_t PCB_TYPE_MASK = 0xC0;
constexpr uint8_t PCB_TYPE_R = 0x80;
constexpr uint8_t PCB_TYPE_S = 0xC0;
constexpr uint8_t S_BLOCK_MASK = 0x30;
constexpr uint8_t S_WTX = 0x30;
constexpr uint8_t PCB_S_DESELECT = 0xC2;

// R-block PCB: 1 0 1 x 0 0 1 n. Bit 0x10 selects NAK over ACK, 0x02 is fixed,
// and the low bit carries the block number.
constexpr uint8_t PCB_R_ACK = 0xA2;
constexpr uint8_t PCB_R_NAK = 0xB2;

constexpr int I2C_TIMEOUT_MS = 50;

inline uint32_t nowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

// FSCI -> frame size, ISO14443-4 table 6.
uint16_t fsciToBytes(uint8_t fsci) {
    static const uint16_t table[] = {16, 24, 32, 40, 48, 64, 96, 128, 256};
    return fsci < (sizeof(table) / sizeof(table[0])) ? table[fsci] : 256;
}

// ISO14443-4 FWT = (4096 / 13.56 MHz) * 2^FWI, rounded up to ms.
uint32_t fwiToMs(uint8_t fwi) {
    if (fwi > 14) fwi = 14;
    const uint64_t microseconds = (uint64_t(4096) * 1000000 * (uint64_t(1) << fwi) + 13560000 - 1) / 13560000;
    return std::min(uint32_t((microseconds + 999) / 1000 + 1), uint32_t(2000));
}

// Largest frame either side puts on the wire: PCB + 256 byte INF + CRC + slack.
// Bounds the FIFO write buffer and the ISO-DEP frame buffers.
constexpr size_t kMaxFrame = 264;

}  // namespace

// ---------------------------------------------------------------- lifecycle

ST25R3916::ST25R3916(i2c_master_bus_handle_t bus,
                                 uint8_t i2cAddr,
                                 uint32_t i2cHz)
    : m_i2cAddr(i2cAddr), m_i2cHz(i2cHz), m_bus(bus) {}

bool ST25R3916::setEcpFrame(const uint8_t* frame, size_t length) {
    if (length > m_ecpData.size() || (length && !frame)) return false;
    m_ecpData.fill(0);
    if (length) std::copy(frame, frame + length, m_ecpData.begin());
    m_ecpSize = length;
    return true;
}

ST25R3916::~ST25R3916() {
    stop();
}

bool ST25R3916::init() {
    if (m_connected) return true;
    if (!m_bus || m_i2cAddr < 8 || m_i2cAddr > 0x77 || m_i2cHz != 400000)
        return false;
    if (!m_dev) {
        i2c_device_config_t devcfg = {};
        devcfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        devcfg.device_address = m_i2cAddr;
        devcfg.scl_speed_hz = m_i2cHz;

        esp_err_t err = i2c_master_bus_add_device(m_bus, &devcfg, &m_dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
            stop();
            return false;
        }
    }

    uint8_t identity = 0;
    if (!readReg(REG_IC_IDENTITY, identity)) {
        ESP_LOGE(TAG, "No response at I2C 0x%02X -- check wiring and 5V", m_i2cAddr);
        stop();
        return false;
    }
    m_icType = (identity >> 3) & 0x1F;
    m_icRev = identity & 0x07;
    if (!supportedIdentity(identity)) {
        ESP_LOGE(TAG, "IC_IDENTITY 0x%02X: unsupported ST25R3916/B type 0x%02X or revision", identity, m_icType);
        stop();
        return false;
    }
    ESP_LOGI(TAG, "Found %s, IC_IDENTITY 0x%02X (type 0x%02X rev %u)",
             m_icType == IC_TYPE_ST25R3916B ? "ST25R3916B" : "ST25R3916", identity, m_icType, m_icRev);

    if (!command(CMD_SET_DEFAULT)) { stop(); return false; }
    vTaskDelay(pdMS_TO_TICKS(1));

    // No IRQ line on the Grove connector, so unmask everything and poll the
    // status registers. Masked sources do not latch, which would hang the waits.
    if (!writeReg(REG_MASK_MAIN_IRQ, 0x00) || !writeReg(REG_MASK_TIMER_NFC_IRQ, 0x00) ||
        !writeReg(REG_MASK_ERROR_WAKEUP_IRQ, 0x00) || !writeReg(REG_MASK_PASSIVE_TARGET_IRQ, 0x00))
    { stop(); return false; }
    clearInterrupts();

    if (!modifyReg(REG_OP_CONTROL, 0, OP_EN)) {
        ESP_LOGE(TAG, "Failed to enable oscillator");
        stop();
        return false;
    }
    // int64_t, not uint32_t: esp_timer_get_time() is a 64-bit microsecond
    // counter, and truncating it would make this delta wrong after ~71 minutes
    // of uptime.
    [[maybe_unused]] const int64_t t0 = esp_timer_get_time();
    if (!(waitInterrupt(IRQ_OSC, 20) & IRQ_OSC)) {
        ESP_LOGE(TAG, "Oscillator did not stabilise");
        stop();
        return false;
    }
    ESP_LOGI(TAG, "Oscillator stable after %lld us", esp_timer_get_time() - t0);

    // ST DS13541 / RFAL initialization: B silicon requires RC calibration
    // after every reset. Do not send this B-only command to the original IC.
    if (m_icType == IC_TYPE_ST25R3916B) {
        clearInterrupts();
        if (!command(CMD_RC_CAL) || !(waitInterrupt(IRQ_DCT, 10) & IRQ_DCT)) {
            ESP_LOGE(TAG, "ST25R3916B RC calibration failed");
            stop(); return false;
        }
    }

    // Module input voltage is not necessarily the IC's VDD (Mini boards have
    // on-board regulation). Measure the chip supply instead of assuming 5 V.
    clearInterrupts();
    uint8_t supply = 0;
    if (!modifyReg(REG_REGULATOR_CONTROL, 0x07, 0) || !command(CMD_MEASURE_VDD) ||
        !(waitInterrupt(IRQ_DCT, 5) & IRQ_DCT) || !readReg(REG_AD_RESULT, supply)) {
        ESP_LOGE(TAG, "Supply measurement failed"); stop(); return false;
    }
    const uint32_t supplyMv = uint32_t(supply) * 234 / 10;
    if (supplyMv < 2400 || supplyMv > 5500 ||
        !modifyReg(REG_IO_CONF2, 0x80, supplyMv < 3600 ? 0x80 : 0)) {
        ESP_LOGE(TAG, "Unsupported chip supply: %u mV", unsigned(supplyMv)); stop(); return false;
    }
    clearInterrupts();
    if (!command(CMD_ADJUST_REGULATORS) || !(waitInterrupt(IRQ_DCT, 10) & IRQ_DCT)) {
        ESP_LOGE(TAG, "Regulator adjustment failed"); stop(); return false;
    }
    ESP_LOGI(TAG, "Chip supply: %u mV (%s mode)", unsigned(supplyMv), supplyMv < 3600 ? "3.3V" : "5V");

    if (!writeReg(REG_MODE, MODE_OM_ISO14443A) || !writeReg(REG_BITRATE, 0x00))
    { stop(); return false; }

    m_connected = true;
    ESP_LOGI(TAG, "Reader initialized. Waiting for tags...");
    return true;
}

void ST25R3916::stop() {
    if (m_dev) {
        setField(false);
        i2c_master_bus_rm_device(m_dev);
        m_dev = nullptr;
    }
    // The caller owns the bus. Retain it for reinitialization after stop().
    m_connected = false;
    m_isodepActive = false;
    m_fieldUp = false;
    m_icType = 0;
    m_icRev = 0;
}

bool ST25R3916::isConnected() const {
    return m_connected;
}

bool ST25R3916::healthCheck() {
    if (!m_connected) return false;
    uint8_t identity = 0;
    if (!readReg(REG_IC_IDENTITY, identity) || !supportedIdentity(identity) ||
        ((identity >> 3) & 0x1F) != m_icType || (identity & 0x07) != m_icRev) {
        m_connected = false;
        m_icType = 0;
        m_icRev = 0;
        return false;
    }
    return true;
}

// ------------------------------------------------------------- bus plumbing

bool ST25R3916::writeReg(uint8_t reg, uint8_t val) {
    if (!m_dev) return false;
    const uint8_t buf[2] = {static_cast<uint8_t>(OP_WRITE_REGISTER | (reg & 0x3F)), val};
    const bool ok=i2c_master_transmit(m_dev,buf,sizeof(buf),I2C_TIMEOUT_MS)==ESP_OK;
    if(!ok)m_connected=false;
    return ok;
}

bool ST25R3916::readReg(uint8_t reg, uint8_t& out) {
    if (!m_dev) return false;
    const uint8_t mode = OP_READ_REGISTER | (reg & 0x3F);
    // transmit_receive issues a repeated start; releasing the bus in between
    // would drop the pending register selection.
    const bool ok=i2c_master_transmit_receive(m_dev,&mode,1,&out,1,I2C_TIMEOUT_MS)==ESP_OK;
    if(!ok)m_connected=false;
    return ok;
}

uint8_t ST25R3916::readRegOr(uint8_t reg, uint8_t fallback) {
    uint8_t v = fallback;
    readReg(reg, v);
    return v;
}

bool ST25R3916::modifyReg(uint8_t reg, uint8_t clearMask, uint8_t setMask) {
    uint8_t v = 0;
    if (!readReg(reg, v)) return false;
    return writeReg(reg, static_cast<uint8_t>((v & ~clearMask) | setMask));
}

bool ST25R3916::command(uint8_t cmd) {
    if (!m_dev) return false;
    // Command constants already carry the 0b11 mode bits.
    const bool ok=i2c_master_transmit(m_dev,&cmd,1,I2C_TIMEOUT_MS)==ESP_OK;
    if(!ok)m_connected=false;
    return ok;
}

bool ST25R3916::writeFifo(const uint8_t* data, size_t len) {
    // Bounded by the largest frame a caller can build, not by the 512-byte FIFO.
    // transceive() calls this while exchangeApdu() already holds tx[264] and
    // rx[264] on the same stack, and that path also runs on the 4096-byte
    // precompute task -- a 513-byte buffer here is pure stack pressure for
    // capacity no caller can reach.
    if (!m_dev || len + 1 > kMaxFrame) return false;
    uint8_t buf[kMaxFrame];
    buf[0] = OP_LOAD_FIFO;
    memcpy(buf + 1, data, len);
    const bool ok=i2c_master_transmit(m_dev,buf,len+1,I2C_TIMEOUT_MS)==ESP_OK;
    if(!ok)m_connected=false;
    return ok;
}

size_t ST25R3916::readFifo(uint8_t* out, size_t maxLen) {
    if (!m_dev || maxLen == 0) return 0;
    const uint8_t mode = OP_READ_FIFO;
    if (i2c_master_transmit_receive(m_dev, &mode, 1, out, maxLen, I2C_TIMEOUT_MS) != ESP_OK) {
        m_connected=false;return 0;
    }
    return maxLen;
}

uint16_t ST25R3916::fifoLen() {
    const uint8_t s1 = readRegOr(REG_FIFO_STATUS1);
    const uint8_t s2 = readRegOr(REG_FIFO_STATUS2);
    // fifo_b spans status2[7:6] as the high bits and status1[7:0] as the low.
    return static_cast<uint16_t>((static_cast<uint16_t>(s2 >> 6) << 8) | s1);
}

// -------------------------------------------------------------- interrupts

uint32_t ST25R3916::readInterrupts() {
    // Read all four read-to-clear status registers in a single I2C burst,
    // as required by the ST register interface. This also shortens RF polling.
    uint8_t irq[4] = {};
    const uint8_t mode = OP_READ_REGISTER | REG_MAIN_IRQ;
    if (!m_dev || i2c_master_transmit_receive(m_dev, &mode, 1, irq, sizeof(irq), I2C_TIMEOUT_MS) != ESP_OK) {
        m_connected = false; return 0;
    }
    return (static_cast<uint32_t>(irq[0]) << 24) | (static_cast<uint32_t>(irq[1]) << 16) |
           (static_cast<uint32_t>(irq[2]) << 8) | static_cast<uint32_t>(irq[3]);
}

void ST25R3916::clearInterrupts() {
    (void)readInterrupts();
}

uint32_t ST25R3916::waitInterrupt(uint32_t mask, uint32_t timeoutMs) {
    // Elapsed-delta rather than an absolute deadline: nowMs() wraps every ~49.7
    // days, and a wrapped "nowMs() + timeoutMs" is already in the past, so every
    // wait would return after a single poll for the length of that window.
    // Unsigned subtraction stays correct across the wrap.
    const uint32_t start = nowMs();
    uint32_t seen = 0;
    do {
        seen |= readInterrupts();
        if (seen & mask) break;
        if (m_exchanging && nowMs() - m_exchangeStart >= m_exchangeBudget) break;
        taskYIELD(); // Let KNX run without adding a millisecond to each RF poll.
    } while ((nowMs() - start) < timeoutMs);
    return seen;
}

// --------------------------------------------------------------------- RF

void ST25R3916::setField(bool on) {
    if (on) {
        modifyReg(REG_OP_CONTROL, 0, static_cast<uint8_t>(OP_TX_EN | OP_RX_EN));
        // ISO14443 requires >= 5 ms for a card to power up before the first
        // command.
        vTaskDelay(pdMS_TO_TICKS(6));
    } else {
        modifyReg(REG_OP_CONTROL, static_cast<uint8_t>(OP_TX_EN | OP_RX_EN), 0);
    }
    m_fieldUp = on;
}

void ST25R3916::setNumTxBytes(uint16_t bytes, uint8_t trailingBits) {
    writeReg(REG_NUM_TX_BYTES1, static_cast<uint8_t>(bytes >> 5));
    writeReg(REG_NUM_TX_BYTES2,
             static_cast<uint8_t>(((bytes & 0x1F) << 3) | (trailingBits & 0x07)));
}

ST25R3916::RxResult ST25R3916::transceive(const uint8_t* tx, size_t txLen,
                                                      uint8_t* rx, size_t rxMax,
                                                      bool txCrc, bool rxCrc,
                                                      uint32_t timeoutMs, bool expectReply) {
    RxResult r;

    // Responses that genuinely carry no CRC -- notably the ISO14443-3
    // ANTICOLLISION reply and the ECP frame's (absent) answer -- must have CRC
    // checking disabled, or a valid response is flagged as a CRC error.
    if (!modifyReg(REG_AUX, AUX_NO_CRC_RX, rxCrc ? 0 : AUX_NO_CRC_RX) || !command(CMD_CLEAR_FIFO))
    { r.error = "I2C configuration failed"; return r; }
    clearInterrupts();

    if (txLen) {
        if (!writeFifo(tx, txLen)) {
            r.error = "FIFO load failed";
            return r;
        }
        setNumTxBytes(static_cast<uint16_t>(txLen));
    }

    if (!command(txCrc ? CMD_TRANSMIT_WITH_CRC : CMD_TRANSMIT_WITHOUT_CRC))
    { r.error = "I2C transmit failed"; return r; }

    uint32_t seen = waitInterrupt(IRQ_TXE, timeoutMs);
    if (!(seen & IRQ_TXE)) {
        r.irqs = seen;
        r.error = "transmit did not complete";
        return r;
    }

    // ECP is a broadcast, with no direct RF response. Only the TX-complete
    // interrupt is required; ordinary card/APDU exchanges still wait for RX.
    if (!expectReply) { r.ok = true; r.irqs = seen; return r; }

    // readInterrupts() clears the status registers as it reads them, so the poll
    // that observed IRQ_TXE may have latched IRQ_RXE in the same read. Entering
    // the RX wait in that case is fatal to latency, not correctness: the bit has
    // already been consumed, so the wait can never see it and spins for the whole
    // timeout before the exchange succeeds anyway.
    const uint32_t kRxSettled = IRQ_RXE | IRQ_NRE | IRQ_ANY_ERROR;
    int64_t rxWaitUs = 0;
    if (!(seen & kRxSettled)) {
        const int64_t rxStart = esp_timer_get_time();
        seen |= waitInterrupt(kRxSettled, timeoutMs);
        rxWaitUs = esp_timer_get_time() - rxStart;
    }
    r.irqs = seen;

    if (!(seen & IRQ_RXE)) {
        r.error = (seen & IRQ_NRE) ? "no response" : "no RXE";
        return r;
    }

    // Separates "the card took a long time to answer" from "our polling was
    // slow": this is purely the card's think time.
    if (rxWaitUs > 50000) {
        ESP_LOGD(TAG, "card took %lld ms to answer (timeout was %u ms)", rxWaitUs / 1000,
                 timeoutMs);
    }

    const uint16_t avail = fifoLen();
    if (avail == 0) {
        r.error = "RXE but FIFO empty";
        return r;
    }
    if (avail > rxMax) { command(CMD_CLEAR_FIFO); r.error = "RF frame exceeds receive buffer"; return r; }
    r.len = readFifo(rx, avail);
    if (!r.len || (rxCrc && r.len < 2)) { r.error = "FIFO read failed or truncated CRC"; return r; }

    // The chip verifies the RX CRC but leaves those two bytes in the FIFO.
    // Trim them so callers see only the payload; otherwise the last two bytes
    // of an APDU response are the CRC rather than SW1 SW2.
    if (rxCrc && r.len >= 2) r.len -= 2;

    if (seen & IRQ_CRC) r.error = "CRC error";
    else if (seen & IRQ_PAR) r.error = "parity error";
    else if (seen & (IRQ_ERR1 | IRQ_ERR2)) r.error = "framing error";

    r.ok = (r.error == nullptr);
    return r;
}

// REQA/WUPA are 7-bit short frames, which cannot be expressed as whole bytes,
// so the chip provides dedicated direct commands for them.
ST25R3916::RxResult ST25R3916::shortFrame(uint8_t cmd, uint8_t* rx, size_t rxMax,
                                                      uint32_t timeoutMs) {
    RxResult r;

    // ATQA carries no CRC. AUX_NO_CRC_RX is otherwise whatever the previous
    // transceive() left behind -- and in isTagStillPresent() that is an R(NAK)
    // exchange with rxCrc = true, which clears the bit. The WUPA fallback would
    // then have its reply checked against a CRC that ATQA does not contain, and
    // only pass because the error flags are ignored below. Set it explicitly so
    // the frame does not depend on unrelated state.
    modifyReg(REG_AUX, 0, AUX_NO_CRC_RX);

    command(CMD_CLEAR_FIFO);
    clearInterrupts();
    command(cmd);

    uint32_t seen = waitInterrupt(IRQ_TXE, timeoutMs);
    if (!(seen & IRQ_TXE)) {
        r.irqs = seen;
        r.error = "no TXE";
        return r;
    }

    // Same latch-in-one-read case as transceive(); see the comment there.
    const uint32_t kRxSettled = IRQ_RXE | IRQ_NRE | IRQ_ANY_ERROR;
    if (!(seen & kRxSettled)) {
        seen |= waitInterrupt(kRxSettled, timeoutMs);
    }
    r.irqs = seen;
    if (!(seen & IRQ_RXE)) {
        r.error = "no card responded";
        return r;
    }

    const uint16_t avail = fifoLen();
    if (avail == 0) {
        r.error = "RXE but FIFO empty";
        return r;
    }
    if (avail != 2 || avail > rxMax) { command(CMD_CLEAR_FIFO); r.error = "Invalid ATQA length"; return r; }
    r.len = readFifo(rx, avail);
    r.ok = r.len == 2 && !(seen & IRQ_ANY_ERROR);
    if (!r.ok) r.error = "ATQA receive error";
    return r;
}

// -------------------------------------------------------- ISO14443-3 / -4

bool ST25R3916::cascadeLevel(uint8_t sel, uint8_t outUid[4], uint8_t& outSak) {
    uint8_t rx[16];

    // Neither ANTICOLLISION nor its UID+BCC reply carries a CRC.
    const uint8_t anticoll[2] = {sel, 0x20};
    RxResult a = transceive(anticoll, sizeof(anticoll), rx, sizeof(rx), false, false);
    if (!a.ok || a.len < 5) return false;
    if ((rx[0] ^ rx[1] ^ rx[2] ^ rx[3]) != rx[4]) {
        ESP_LOGW(TAG, "BCC mismatch during anticollision");
        return false;
    }
    memcpy(outUid, rx, 4);

    const uint8_t select[7] = {sel, 0x70, rx[0], rx[1], rx[2], rx[3], rx[4]};
    RxResult s = transceive(select, sizeof(select), rx, sizeof(rx), true, true);
    if (!s.ok || s.len < 1) return false;
    outSak = rx[0];
    return true;
}

bool ST25R3916::anticollision(std::vector<uint8_t>& uid, uint8_t& sak) {
    static const uint8_t levels[3] = {SEL_CL1, SEL_CL2, SEL_CL3};
    uid.clear();

    for (uint8_t i = 0; i < 3; i++) {
        uint8_t part[4] = {0};
        uint8_t levelSak = 0;
        if (!cascadeLevel(levels[i], part, levelSak)) return false;
        sak = levelSak;

        if (levelSak & SAK_CASCADE) {
            // part[0] is the cascade tag; only the remaining three are UID.
            if (part[0] != CASCADE_TAG) return false;
            uid.insert(uid.end(), part + 1, part + 4);
            continue;
        }
        uid.insert(uid.end(), part, part + 4);
        return true;
    }
    return false;
}

bool ST25R3916::rats(uint8_t fsdi) {
    const uint8_t cmd[2] = {0xE0, static_cast<uint8_t>((fsdi << 4) | 0x00)};  // CID 0
    uint8_t rx[32];

    RxResult r = transceive(cmd, sizeof(cmd), rx, sizeof(rx), true, true, 50);
    if (!r.ok || r.len < 1) return false;

    // TL counts itself and must not claim more bytes than arrived.
    const uint8_t tl = rx[0];
    if (tl < 2 || tl != r.len) return false;

    m_fsc = 32;
    m_fwtMs = 5;

    if (tl >= 2) {
        const uint8_t t0 = rx[1];
        if ((t0 & 0x0F) > 8) return false;
        m_fsc = fsciToBytes(t0 & 0x0F);

        // Interface bytes appear in TA1, TB1, TC1 order, each present only if
        // its flag is set -- so TB1's offset depends on whether TA1 is there.
        size_t idx = 2;
        if (t0 & 0x10) { if (idx >= tl) return false; ++idx; }
        if (t0 & 0x20) {
            if (idx >= tl || (rx[idx] >> 4) == 15) return false;
            m_fwtMs = fwiToMs((rx[idx++] >> 4) & 0x0F);
        }
        if ((t0 & 0x40) && idx >= tl) return false;
    }

    m_blockNum = 0;
    m_isodepActive = true;
    m_blockMismatchLogged = false;
    return true;
}

// ---------------------------------------------------------------- polling

bool ST25R3916::beginDiscovery() {
    if (!m_connected) return false;
    setField(true);
    return true;
}

void ST25R3916::endDiscovery() {
    m_isodepActive = false;
    setField(false);
}

bool ST25R3916::pollForTag(std::vector<uint8_t>& uid,
                                 std::array<uint8_t, 2>& atqa,
                                 uint8_t& sak,
                                 uint32_t timeoutMs) {
    if (!m_connected) return false;

    m_isodepActive = false;
    if (!m_fieldUp) setField(true);

    uint8_t rx[32];
    const bool fast = fastPolling();
#ifdef ST25R3916_TIMING_TEST
    if (fast != m_timingFast) {
        m_timingFast=fast;m_timingLastReport=nowMs();m_timingPolls=0;m_timingPollUs=0;
    }
    const int64_t pollStart = esp_timer_get_time();
#endif
    // Fast is the default for both chip variants. Baseline remains selectable
    // for comparisons. ISO-DEP/FWT/WTX timeouts are unchanged.
    if (m_ecpSize)
        transceive(m_ecpData.data(), m_ecpSize, rx, sizeof(rx), false, false, 12, !fast);
#ifdef ST25R3916_TIMING_TEST
    const int64_t ecpEnd = esp_timer_get_time();
#endif
    vTaskDelay(pdMS_TO_TICKS(5));
    const uint32_t requested = timeoutMs ? timeoutMs : 20;
    const uint32_t discoveryTimeout = fast ? std::min(requested, uint32_t(10)) : requested;
    RxResult p = shortFrame(CMD_TRANSMIT_WUPA, rx, sizeof(rx), discoveryTimeout);
#ifdef ST25R3916_TIMING_TEST
    const int64_t wakeEnd = esp_timer_get_time();
    ++m_timingPolls; m_timingPollUs += wakeEnd-pollStart;
    if (nowMs()-m_timingLastReport >= 5000) {
        printf("ST25 timing: mode=%s polls=%u average_probe_us=%llu limit_ms=%u\n",
            fast ? "fast" : "baseline", unsigned(m_timingPolls),
            static_cast<unsigned long long>(m_timingPollUs/m_timingPolls), unsigned(discoveryTimeout));
        m_timingLastReport=nowMs();m_timingPolls=0;m_timingPollUs=0;
    }
#endif
    if (!p.ok || p.len < 2) return false;
    atqa[0] = rx[0];
    atqa[1] = rx[1];

    if (!anticollision(uid, sak)) return false;

    // The PN532 performs RATS inside InListPassiveTarget, so callers expect a
    // tag reported here to be ready for exchangeApdu(). Do the same.
    if (sak & SAK_ISODEP) {
        if (!rats(8)) {
            ESP_LOGW(TAG, "Activated tag but RATS failed");
            return false;
        }
    }
#ifdef ST25R3916_TIMING_TEST
    printf("ST25 timing: detected mode=%s ecp_us=%lld wake_us=%lld activation_us=%lld total_poll_us=%lld\n",
        fast ? "fast" : "baseline", static_cast<long long>(ecpEnd-pollStart),
        static_cast<long long>(wakeEnd-ecpEnd), static_cast<long long>(esp_timer_get_time()-wakeEnd),
        static_cast<long long>(esp_timer_get_time()-pollStart));
#endif
    return true;
}

bool ST25R3916::isTagStillPresent() {
    if (!m_connected) return false;
    // Deliberately does not drop the field: cycling it would reset a device
    // that is merely between commands.
    if (!m_fieldUp) setField(true);

    // Preferred: the ISO 14443-4 R(NAK) presence check. While an ISO-DEP
    // session is alive the card answers R(ACK) purely at the protocol layer,
    // with no application involvement -- so it still reports presence after the
    // applet has finished its transaction.
    //
    // This matters for phones specifically: an iPhone stops answering WUPA the
    // moment Home Key authentication completes, so a WUPA-based check reports
    // "gone" while the handset is still physically on the antenna. The polling
    // loop then immediately re-polls and starts a second transaction that iOS
    // refuses to finish.
    if (m_isodepActive) {
        // R-block PCB: 1 0 1 x 0 0 1 n, where x = 1 for NAK and n is the
        // current block number. R(NAK) must not toggle the block number.
        const uint8_t rnak[1] = {static_cast<uint8_t>(PCB_R_NAK | (m_blockNum & PCB_BLOCK_NUM))};
        uint8_t rx[8];
        RxResult r = transceive(rnak, sizeof(rnak), rx, sizeof(rx),
                                /*txCrc=*/true, /*rxCrc=*/true, /*timeoutMs=*/20);
        if (r.ok && r.len >= 1) {
            // Any well-formed reply proves the card is still powered and
            // listening: R(ACK) is the expected one, but an S(WTX) or a
            // straggling I-block answers the question just as well.
            const bool isAck = (rx[0] & ~PCB_BLOCK_NUM) == PCB_R_ACK;
            // Log both block numbers: an R(ACK) whose block number differs from
            // ours means the card and reader have diverged, which would corrupt
            // the next I-block exchange.
            const uint8_t theirs = rx[0] & PCB_BLOCK_NUM;
            const uint8_t ours = m_blockNum & PCB_BLOCK_NUM;
            // The card consistently answers with its own block number, which
            // differs from ours by one after an odd number of exchanges. This
            // appears to be convention rather than an error -- a card that
            // genuinely disagreed would retransmit its last I-block instead of
            // acknowledging. Reported once per session rather than on every
            // poll, which would otherwise bury everything else in the log.
            if (isAck && theirs != ours && !m_blockMismatchLogged) {
                m_blockMismatchLogged = true;
                ESP_LOGD(TAG, "presence: card R(ACK) blk=%u vs our blk=%u "
                              "(informational; presence detection unaffected)",
                         theirs, ours);
            }
            ESP_LOGV(TAG, "presence: still here, PCB 0x%02X blk=%u%s", rx[0], theirs,
                     isAck ? " (R(ACK))" : "");
            return true;
        }
        // Silence means the session is finished. Drop it and fall through, so a
        // device that merely deselected can still be re-detected below.
        ESP_LOGD(TAG, "presence: ISO-DEP session ended (%s)", r.error ? r.error : "no reply");
        m_isodepActive = false;
    }

    uint8_t rx[8];
    RxResult r = shortFrame(CMD_TRANSMIT_WUPA, rx, sizeof(rx), 10);
    ESP_LOGV(TAG, "presence: WUPA fallback -> %s", (r.ok && r.len >= 2) ? "present" : "gone");
    return r.ok && r.len >= 2;
}

void ST25R3916::releaseTag() {
    if (!m_connected) return;
    if (m_isodepActive) {
        // Best-effort S(DESELECT); ignore the outcome, the field stays up so a
        // fresh poll can follow immediately.
        const uint8_t deselect[1] = {PCB_S_DESELECT};
        uint8_t rx[8];
        transceive(deselect, sizeof(deselect), rx, sizeof(rx), true, true, 10);
        m_isodepActive = false;
    }
}

// ---------------------------------------------------------------- ISO-DEP

bool ST25R3916::exchangeApdu(const std::vector<uint8_t>& send,
                                   std::vector<uint8_t>& recv,
                                   uint32_t timeoutMs) {
    recv.clear();
    if (!m_connected || !m_isodepActive || send.empty() || send.size() > 4096 || timeoutMs == 0)
        return false;
    m_exchangeStart = nowMs();
    m_exchangeBudget = std::min(timeoutMs, uint32_t(10000));
    m_exchanging = true;
    struct Finish {
        bool& active; std::vector<uint8_t>& response; bool succeeded = false;
        ~Finish() { active = false; if (!succeeded) response.clear(); }
    } finish{m_exchanging, recv};
    auto remaining = [this](uint32_t requested) {
        const uint32_t elapsed = nowMs() - m_exchangeStart;
        return elapsed >= m_exchangeBudget ? 0 : std::min(requested, m_exchangeBudget - elapsed);
    };

    // FSC bounds what the *card* is willing to receive, so it caps our INF per
    // frame: FSC = PCB + INF + 2 CRC bytes. Anything longer is split across
    // chained I-blocks. FSC never bounds the card's own responses -- those are
    // limited by the FSD we advertised in RATS -- so the RX buffer is sized from
    // the frame buffer, not from FSC.
    const size_t maxInf = (m_fsc > 3) ? static_cast<size_t>(m_fsc) - 3 : 13;

    // Sized from the largest frame either side may send (FSD/FSC max 256) rather
    // than the 512-byte FIFO. This runs on the 4096-byte precompute task as well
    // as the polling task, so every byte of stack matters -- an overflow there
    // reboots the device mid-transaction. The PN532 backend keeps its buffers on
    // the heap for the same reason.
    static constexpr size_t kFrameBuf = 264;  // PCB + 256 payload + CRC + slack
    uint8_t tx[kFrameBuf];
    uint8_t rx[kFrameBuf];

    // A chained response is assembled here across several frames. Bounded so a
    // misbehaving card cannot grow the heap without limit.
    static constexpr size_t kMaxResponse = 4096;

    // Caller timeout acts as a floor; the card's advertised FWT may be longer.
    uint32_t timeout = timeoutMs > m_fwtMs ? timeoutMs : m_fwtMs;

    // Exchanges that approach the card's FWT are worth surfacing: a phone that
    // is made to wait too long deselects and reports a protocol error, even
    // though the exchange itself eventually succeeded.
    const int64_t exchangeStart = esp_timer_get_time();
    uint8_t wtxRounds = 0;

    // ISO 14443-4 allows the reader to request retransmission after a
    // transmission error. Two attempts is the conventional limit -- beyond that
    // the link is genuinely bad and failing fast beats stalling the phone.
    // The budget is per recovery episode, not per transaction: it is cleared
    // whenever a frame arrives intact. Otherwise a chained command that spent
    // both retries on its first chunk would have none left for the remaining
    // frames, which is precisely when errors are most likely.
    constexpr uint8_t kMaxNakRetries = 2;
    uint8_t nakRetries = 0;
    // Tracks only whether the frame we are waiting on is a reply to an R(NAK),
    // which is a different question from how much budget is left. Conflating
    // the two left the resend-on-R-block branch armed after a recovery had
    // already succeeded, so a later unexpected R-block resent the I-block
    // instead of failing -- looping until the attempt cap.
    bool awaitingNakReply = false;

    size_t txOff = 0;      // how much of `send` has been acknowledged
    size_t rxFrames = 0;   // response frames collected (1 unless the card chains)
    size_t txFrames = 0;

    // Outer loop walks the command chunks. A command that fits in one frame runs
    // this once, which is every Home Key exchange observed so far except
    // attestation.
    for (;;) {
        const size_t remain = send.size() - txOff;
        const size_t chunk = remain > maxInf ? maxInf : remain;
        const bool moreToSend = (txOff + chunk) < send.size();

        auto buildIBlock = [&]() -> size_t {
            tx[0] = static_cast<uint8_t>(PCB_I_BLOCK | (m_blockNum & PCB_BLOCK_NUM) |
                                         (moreToSend ? PCB_CHAINING : 0));
            memcpy(tx + 1, send.data() + txOff, chunk);
            return chunk + 1;
        };

        size_t txLen = buildIBlock();
        txFrames++;
        bool chunkSettled = false;

        // Inner loop resolves one chunk: S(WTX) extensions and R(NAK)
        // retransmissions do not advance the chain, they just re-run this.
        for (uint8_t attempt = 0; attempt < 8 && !chunkSettled; attempt++) {
            const uint32_t budget = remaining(timeout);
            if (!budget) return false;
            RxResult r = transceive(tx, txLen, rx, kFrameBuf, true, true, budget);

            if (!r.ok || r.len < 1) {
                // ISO 14443-4 error recovery. A corrupted frame is a normal RF
                // event, not a fatal one: the reader is supposed to send R(NAK)
                // to request retransmission rather than abandon the transaction.
                //
                // This matters most for the 87-byte Auth0 response, the largest
                // frame in a FAST-flow exchange, and for every frame of a chained
                // attestation payload. Without recovery a single bad CRC aborts
                // authentication and the phone reports a protocol error.
                const bool recoverable =
                    (r.irqs & (IRQ_CRC | IRQ_PAR | IRQ_ERR1 | IRQ_ERR2)) != 0 || (r.irqs & IRQ_RXE);

                if (recoverable && nakRetries < kMaxNakRetries) {
                    nakRetries++;
                    awaitingNakReply = true;
                    ESP_LOGW(TAG, "recoverable RX error (%s), sending R(NAK) %u/%u",
                             r.error ? r.error : "?", nakRetries, kMaxNakRetries);
                    // R(NAK) carries the current block number and must not toggle it.
                    tx[0] = static_cast<uint8_t>(PCB_R_NAK | (m_blockNum & PCB_BLOCK_NUM));
                    txLen = 1;
                    continue;
                }

                // A failure here leaves the session half-finished: our block
                // number has not toggled but the card's may have.
                ESP_LOGE(TAG, "ISO-DEP exchange FAILED after %lld ms: %s "
                              "(blk=%u, frame %u, %u WTX round%s, %u NAK retr%s, "
                              "irqs=0x%08X, %u byte cmd)",
                         (esp_timer_get_time() - exchangeStart) / 1000,
                         r.error ? r.error : "no response", static_cast<unsigned>(m_blockNum),
                         static_cast<unsigned>(txFrames), wtxRounds, wtxRounds == 1 ? "" : "s",
                         nakRetries, nakRetries == 1 ? "y" : "ies", r.irqs,
                         static_cast<unsigned>(send.size()));
                return false;
            }

            const uint8_t pcb = rx[0];

            // ---- S(WTX): card wants more time, typically while doing crypto.
            if ((pcb & PCB_TYPE_MASK) == PCB_TYPE_S) {
                if ((pcb & S_BLOCK_MASK) == S_WTX && r.len >= 2) {
                    const uint8_t wtxm = rx[1] & 0x3F;
                    if (!wtxm || wtxm > 59 || wtxRounds >= 8) return false;
                    timeout = m_fwtMs * wtxm + 10;
                    wtxRounds++;
                    ESP_LOGD(TAG, "S(WTX) round %u: WTXM=%u, extending timeout to %u ms",
                             wtxRounds, wtxm, timeout);
                    tx[0] = pcb;
                    tx[1] = wtxm;
                    txLen = 2;
                    continue;
                }
                ESP_LOGW(TAG, "Unexpected S-block 0x%02X", pcb);
                return false;
            }

            // ---- R-block.
            if ((pcb & PCB_TYPE_MASK) == PCB_TYPE_R) {
                const bool isAck = (pcb & ~PCB_BLOCK_NUM) == PCB_R_ACK;

                if (moreToSend && isAck) {
                    // Expected mid-chain acknowledgement: this chunk is in, so
                    // the next one starts with a full retry budget.
                    m_blockNum ^= 1;
                    txOff += chunk;
                    nakRetries = 0;
                    awaitingNakReply = false;
                    chunkSettled = true;
                    break;
                }
                if (awaitingNakReply) {
                    // The card answered our R(NAK) with an R-block, meaning it
                    // never received the I-block. Resend it. Cleared first so a
                    // later unrelated R-block is treated as the protocol error
                    // it is rather than triggering another resend.
                    awaitingNakReply = false;
                    ESP_LOGW(TAG, "card answered R-block 0x%02X to our R(NAK); resending I-block",
                             pcb);
                    txLen = buildIBlock();
                    continue;
                }
                ESP_LOGW(TAG, "Unexpected R-block 0x%02X", pcb);
                return false;
            }

            // ---- I-block. Only legitimate once the whole command is sent.
            if (moreToSend) {
                ESP_LOGW(TAG, "card sent I-block 0x%02X mid-chain, expected R(ACK)", pcb);
                return false;
            }

            if (r.len < 2) {
                ESP_LOGW(TAG, "ISO-DEP I-block carries no payload");
                return false;
            }
            // A good I-block ends any recovery episode: the continuation frames
            // below get a full budget again.
            nakRetries = 0;
            awaitingNakReply = false;
            if (recv.size() + r.len - 1 > kMaxResponse) return false;
            recv.insert(recv.end(), rx + 1, rx + r.len);

            // ---- Receive chaining: the card signals more with the M bit. Each
            // continuation is requested with R(ACK) carrying the received block
            // number toggled, per ISO 14443-4. S(WTX) can appear between frames.
            uint8_t respPcb = pcb;
            while (respPcb & PCB_CHAINING) {
                if (recv.size() > kMaxResponse) {
                    ESP_LOGE(TAG, "chained response exceeded %u bytes; aborting",
                             static_cast<unsigned>(kMaxResponse));
                    return false;
                }
                tx[0] = static_cast<uint8_t>(PCB_R_ACK |
                                             ((respPcb ^ PCB_BLOCK_NUM) & PCB_BLOCK_NUM));
                txLen = 1;

                bool gotNext = false;
                for (uint8_t chainAttempt = 0; chainAttempt < 8 && !gotNext; chainAttempt++) {
                    const uint32_t budget = remaining(timeout);
                    if (!budget) return false;
                    RxResult cr = transceive(tx, txLen, rx, kFrameBuf, true, true, budget);
                    if (!cr.ok || cr.len < 1) {
                        ESP_LOGE(TAG, "chained RX failed after %u frame%s: %s",
                                 static_cast<unsigned>(rxFrames + 1), rxFrames == 0 ? "" : "s",
                                 cr.error ? cr.error : "no response");
                        return false;
                    }
                    const uint8_t cpcb = rx[0];
                    if ((cpcb & PCB_TYPE_MASK) == PCB_TYPE_S &&
                        (cpcb & S_BLOCK_MASK) == S_WTX && cr.len >= 2) {
                        const uint8_t wtxm = rx[1] & 0x3F;
                        if (!wtxm || wtxm > 59 || wtxRounds >= 8) return false;
                        timeout = m_fwtMs * wtxm + 10;
                        wtxRounds++;
                        tx[0] = cpcb;
                        tx[1] = wtxm;
                        txLen = 2;
                        continue;
                    }
                    if ((cpcb & PCB_TYPE_MASK) != 0x00 && (cpcb & PCB_TYPE_MASK) != PCB_I_BLOCK) {
                        ESP_LOGW(TAG, "unexpected PCB 0x%02X during receive chaining", cpcb);
                        return false;
                    }
                    if (cr.len < 2) {
                        ESP_LOGW(TAG, "empty continuation frame during receive chaining");
                        return false;
                    }
                    if (recv.size() + cr.len - 1 > kMaxResponse) return false;
                    recv.insert(recv.end(), rx + 1, rx + cr.len);
                    respPcb = cpcb;
                    rxFrames++;
                    gotNext = true;
                }
                if (!gotNext) {
                    ESP_LOGW(TAG, "too many WTX rounds while receiving a chained response");
                    return false;
                }
            }

            // The card echoes our block number; take the next one from what it
            // actually sent rather than assuming, so a chained response leaves
            // the session correctly aligned.
            m_blockNum = static_cast<uint8_t>((respPcb & PCB_BLOCK_NUM) ^ 1);

            if (recv.size() < 2) {
                ESP_LOGW(TAG, "ISO-DEP response shorter than SW1 SW2");
                return false;
            }

            rxFrames++;
            const int64_t elapsedMs = (esp_timer_get_time() - exchangeStart) / 1000;
            const bool chained = (txFrames > 1 || rxFrames > 1);
            if (chained) {
                ESP_LOGI(TAG, "chained APDU ok: %lld ms, %u in / %u out "
                              "(%u TX frame%s, %u RX frame%s, FSC %u, %u WTX)",
                         elapsedMs, static_cast<unsigned>(send.size()),
                         static_cast<unsigned>(recv.size()), static_cast<unsigned>(txFrames),
                         txFrames == 1 ? "" : "s", static_cast<unsigned>(rxFrames),
                         rxFrames == 1 ? "" : "s", m_fsc, wtxRounds);
            } else if (elapsedMs > static_cast<int64_t>(m_fwtMs) / 2 || wtxRounds) {
                // Anything past half the frame waiting time is worth reporting:
                // a phone that is kept waiting near FWT deselects and reports a
                // protocol error even though the exchange itself succeeded.
                ESP_LOGW(TAG, "slow APDU: %lld ms (FWT %u ms, %u WTX round%s, %u byte cmd)",
                         elapsedMs, m_fwtMs, wtxRounds, wtxRounds == 1 ? "" : "s",
                         static_cast<unsigned>(send.size()));
            } else {
                ESP_LOGV(TAG, "APDU ok: %lld ms, %u in / %u out, blk %u->%u", elapsedMs,
                         static_cast<unsigned>(send.size()), static_cast<unsigned>(recv.size()),
                         static_cast<unsigned>(m_blockNum ^ 1), static_cast<unsigned>(m_blockNum));
            }
            if (nowMs() - m_exchangeStart > m_exchangeBudget) return false;
            finish.succeeded = true;
            return true;
        }

        if (!chunkSettled) {
            ESP_LOGW(TAG, "chunk %u unresolved after 8 attempts (%u WTX rounds)",
                     static_cast<unsigned>(txFrames), wtxRounds);
            return false;
        }
    }
}
