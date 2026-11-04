# ST25R3916

ESP32 Arduino hardware-I2C driver for the Elechouse ST25R3916 in I2C mode.
Default address **0x50**, bus **400 kHz**, no external IRQ wire required.
Requires Arduino ESP32 with ESP-IDF 5.4+ and its native I2C bus-handle API.

## Features

- NFC-A discovery and 4/7/10-byte UID selection.
- RATS/ATS and ISO-DEP APDU exchange, chaining, CRC checking and bounded WTX/recovery.
- Optional caller-supplied ECP frames. ECP is disabled by default.
- Borrows the shared TwoWire bus, removes only its own device handle on stop.
- No dependency on OpenKNX, ETS, HomeSpan, HomeKey or a credential store.

This implementation targets ST25R3916, not an unverified ST25R3916B variant.
SPI and NFC-B/F/V are not implemented. Physical Elechouse/iPhone validation is pending.

## Usage

```cpp
#include <ST25R3916.hpp>
ST25R3916 reader(Wire, 21, 22);
// During setup:
// reader.init(); reader.beginDiscovery();
// During loop, call pollForTag(uid, atqa, sak, timeoutMs).
// For a selected ISO-DEP card, call exchangeApdu(command, response, timeoutMs).
// releaseTag() after use; stop() removes only this reader's I2C device handle.
```

Use the module's documented 5 V supply and ESP32-compatible logic levels.
All devices on the shared bus must use compatible pins and unique addresses.
Call this instance from one task, or synchronize access externally.

`setEcpFrame(frame, length)` copies up to 18 bytes including CRC-A.
`setEcpFrame(nullptr, 0)` disables ECP. The caller generates any application
payload, identifiers and CRC. Changing ECP does not require reinitialization.

Run standalone protocol and bus-lifecycle tests with `python3 test/run.py`.
The tests use a scripted I2C/RF device and AddressSanitizer/UBSan.

## Attribution

Adapted from rednblkx/HomeKey-ESP32 at
[f167cba1950aa9e37afb9cb06163a78299599a67](https://github.com/rednblkx/HomeKey-ESP32/tree/f167cba1950aa9e37afb9cb06163a78299599a67).
The MIT license and upstream copyright are retained in LICENSE.
Local adaptations include shared-bus ownership, optional copied ECP frames,
ATS validation and stricter exchange bounds, deadlines and failure handling.
