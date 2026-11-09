# ST25R3916

ESP32 Arduino hardware-I2C driver for the Elechouse ST25R3916 and ST25R3916B (including Mini) in I2C mode.
Default address **0x50**, bus **400 kHz**, no external IRQ wire required.
Requires Arduino ESP32 with ESP-IDF 5.4+ and its native I2C bus-handle API.

## Features

- NFC-A discovery and 4/7/10-byte UID selection.
- RATS/ATS and ISO-DEP APDU exchange, chaining, CRC checking and bounded WTX/recovery.
- Optional caller-supplied ECP frames. ECP is disabled by default.
- Borrows the shared TwoWire bus, removes only its own device handle on stop.
- No dependency on OpenKNX, ETS, HomeSpan, HomeKey or a credential store.

ST25R3916 (type 0x05) and production ST25R3916B (type 0x06, revision 1+)
are recognized at runtime. B silicon performs the mandatory RC calibration after
reset. Both variants measure their IC supply before configuring the regulator
mode; a module's input voltage can differ from its regulated IC supply.
Identity checks also run during health monitoring; other chip types are rejected.
SPI and NFC-B/F/V are not implemented.
Physical validation: Elechouse ST25R3916B Mini on XIAO ESP32-S3 over I2C;
ordinary UID card detection and iPhone HomeKey authentication confirmed.
Performance comparisons and longer stability testing remain pending.

Initialization follows [ST DS13541](https://www.st.com/resource/en/datasheet/st25r3916b.pdf)
and [AN5768](https://www.st.com/resource/en/application_note/an5768-migrating-from-st25r39xx-to-st25r39xxb-stmicroelectronics.pdf).
The shared NFC-A/ISO-DEP path retains the chip's default TX-driver and AWS settings;
original-chip modulation values are not copied to the B variant.

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

Use the module's documented input supply and ESP32-compatible logic levels.
The Elechouse Mini V2 specifies 3.3–5 V input; other modules may require 5 V.
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

## Polling modes

Fast polling is the default in every build, for both ST25R3916 and ST25R3916B.
It sends ECP as a broadcast (wait for TX completion, no RX wait) and caps the
WUPA response window at 10 ms. Baseline retains the original receive wait and
caller-supplied discovery timeout. Use `ST25R3916::setFastPolling(false)` to
select baseline, or `true` for fast. Selection is volatile and shared by reader
instances; reboot returns to fast. Switch only between card presentations.
APDU, FWT/WTX, RATS, settling and field-on guard times are unchanged.
`ST25R3916_TIMING_TEST` enables diagnostic measurements only.
