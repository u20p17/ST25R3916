#include <array>
#include <vector>
#define private public
#include "ST25R3916.hpp"
#undef private
#include <cassert>
#include <iostream>

static void isoDep(ST25R3916& reader) {
    reader.m_isodepActive=true;reader.m_blockNum=0;reader.m_fsc=32;reader.m_fwtMs=5;
}
int main(int argc, char** argv) {
    assert(ST25R3916::fastPolling()); // Production and diagnostic builds default fast.
    if(argc > 1) ST25R3916::setFastPolling(std::string(argv[1]) != "baseline");
    FakeBus bus;std::array<uint8_t,18> ecp{};
    ST25R3916 nullBus(nullptr);
    assert(!nullBus.init());
    ST25R3916 invalidAddress(&bus, 0x78);
    assert(!invalidAddress.init() && bus.deviceAdds == 0);
    ST25R3916 invalidClock(&bus, 0x50, 100000);
    assert(!invalidClock.init() && bus.deviceAdds == 0);
    ST25R3916 reader(&bus);
    assert(reader.init() && reader.init());
    assert(bus.deviceAdds==1 && bus.liveDevices==2);
    assert(reader.beginDiscovery());
    std::vector<uint8_t> uid, response;std::array<uint8_t,2> atqa;uint8_t sak=0;
    replies={{{4,0}},{{1,2,3,4,4}},{{0}}};
    assert(reader.pollForTag(uid,atqa,sak,30));
    assert(uid==std::vector<uint8_t>({1,2,3,4}) && sak==0);
    for(const auto& f:sentFrames)assert(f.empty()||f[0]!=0x6A); // No ECP in UID-only mode.
    replies={{{4,0}},{{0x88,1,2,3,0x88}},{{4}},{{4,5,6,7,0}},{{0}}};
    assert(reader.pollForTag(uid,atqa,sak,30) && uid.size()==7 && uid.back()==7);
    replies={{{4,0}},{{0x88,1,2,3,0x88}},{{4}},{{0x88,4,5,6,uint8_t(0x88^4^5^6)}},{{4}},{{7,8,9,10,uint8_t(7^8^9^10)}},{{0}}};
    assert(reader.pollForTag(uid,atqa,sak,30) && uid.size()==10 && uid.back()==10);
    replies={{{4,0}},{{1,2,3,4,4}},{{0x20}},{{3,0x28,0x40}}};
    assert(reader.pollForTag(uid,atqa,sak,30) && reader.m_isodepActive && reader.m_fwtMs==6);
    replies={{{4,0},true}};
    assert(!reader.pollForTag(uid,atqa,sak,30)); // Corrupt wake-up response is rejected.
    ecp={0x6a,2,0xcb,2,6,2,0x11,0,1,2,3,4,5,6,7,8,9,10};
    assert(!reader.setEcpFrame(nullptr,1));
    assert(!reader.setEcpFrame(ecp.data(),19));
    assert(reader.setEcpFrame(ecp.data(),ecp.size()));
    const auto before=sentFrames.size();replies.clear();
    assert(!reader.pollForTag(uid,atqa,sak,30));
    assert(sentFrames[before]==std::vector<uint8_t>(ecp.begin(),ecp.end()));
    assert(reader.setEcpFrame(nullptr,0));
    assert(reader.m_ecpSize==0);
    isoDep(reader);replies={{{0x02,0x90,0},true},{{0x02,0x90,0}}};
    assert(reader.exchangeApdu({0xA4},response,1000)); // Bounded CRC recovery.
    isoDep(reader);replies={{{0x02,0x90,0}}};
    assert(reader.exchangeApdu({0,0xA4},response,1000));
    assert(response==std::vector<uint8_t>({0x90,0})); // CRC bytes are removed once.
    isoDep(reader);replies={{{0xA3}},{{0x03,0x90,0}}};
    assert(reader.exchangeApdu(std::vector<uint8_t>(50,0xA5),response,1000));
    assert(response==std::vector<uint8_t>({0x90,0})); // TX chaining.
    isoDep(reader);replies={{{0x12,1,2}},{{0x03,0x90,0}}};
    assert(reader.exchangeApdu({0xA4},response,1000));
    assert(response==std::vector<uint8_t>({1,2,0x90,0})); // RX chaining.
    isoDep(reader);replies={{{0xF2,1}},{{0x02,0x90,0}}};
    assert(reader.exchangeApdu({0xA4},response,1000)); // Valid WTX then reply.
    for(uint8_t wtx:{0,60}) {
        isoDep(reader);replies={{{0xF2,wtx}}};
        assert(!reader.exchangeApdu({0xA4},response,1000) && response.empty());
    }
    isoDep(reader);replies.clear();for(int i=0;i<10;i++)replies.push_back({{0xF2,1}});
    assert(!reader.exchangeApdu({0xA4},response,1000) && response.empty());
    isoDep(reader);replies.clear();
    for(int i=0;i<18;i++) {std::vector<uint8_t> frame(250,0xA5);frame[0]=i==17?0x03:0x12;replies.push_back({frame});}
    assert(!reader.exchangeApdu({0xA4},response,1000) && response.empty()); // Strict 4096-byte bound.
    isoDep(reader);assert(!reader.exchangeApdu({},response,1000));
    assert(!reader.exchangeApdu(std::vector<uint8_t>(4097),response,1000));
    isoDep(reader);busTime=5000;const auto start=fakeMicros;replies={{{0xF2,1}},{{0x02,0x90,0}}};
    assert(!reader.exchangeApdu({0xA4},response,1) && response.empty());
    assert(fakeMicros-start<200000 && !reader.m_exchanging);busTime=100;
    busFails=true;assert(!reader.healthCheck());busFails=false;
    reader.stop();reader.stop();assert(bus.liveDevices==1 && bus.busInitialized);
    assert(reader.init());reader.stop();assert(bus.deviceAdds==2 && bus.deviceRemoves==2);
    chipIdentity=0;assert(!reader.init() && bus.liveDevices==1);chipIdentity=0x28;
    // Both IC families must retain UID/ECP/ISO-DEP support and health checks.
    for (uint8_t identity : {0x28, 0x31}) {
        chipIdentity=identity;commands.clear();supplyAdc=identity==0x31?128:214;
        assert(reader.init() && reader.healthCheck());
        assert(reader.getFwMajor()==(identity>>3));
        assert(bool(registers[1]&0x80)==(identity==0x31));
        assert((std::find(commands.begin(),commands.end(),0xEA)!=commands.end())==(identity==0x31));
        assert(reader.beginDiscovery());
        replies={{{4,0}},{{1,2,3,4,4}},{{0x20}},{{3,0x28,0x40}}};
        assert(reader.pollForTag(uid,atqa,sak,30) && reader.m_isodepActive);
        assert(reader.setEcpFrame(ecp.data(),ecp.size()));
        const auto sent=sentFrames.size();replies.clear();
        assert(!reader.pollForTag(uid,atqa,sak,30));
        assert(sentFrames[sent]==std::vector<uint8_t>(ecp.begin(),ecp.end()));
        isoDep(reader);replies={{{0x02,0x90,0}}};
        assert(reader.exchangeApdu({0xA4},response,1000));
        registers[0x3F]=identity==0x31?0x28:0x31;
        assert(!reader.healthCheck());reader.stop();
    }
    for(uint8_t invalid : {0x00,0x30,0x38,0xFF}) {
        chipIdentity=invalid;assert(!reader.init() && bus.liveDevices==1);
    }
    chipIdentity=0x31;calibrationFails=true;
    assert(!reader.init() && bus.liveDevices==1);calibrationFails=false;
    measurementFails=true;assert(!reader.init() && bus.liveDevices==1);measurementFails=false;
    supplyAdc=0;assert(!reader.init() && bus.liveDevices==1);supplyAdc=128;
    assert(reader.init() && reader.healthCheck());reader.stop();
#ifdef ST25R3916_TIMING_TEST
    assert(reader.init());assert(reader.beginDiscovery());
    noResponseIrq=false;replies.clear();
    int64_t ecpUs[2],probeUs[2];
    for(int fast=0;fast<2;fast++) {
        ST25R3916::setFastPolling(fast);
        const auto start=fakeMicros;
        auto result=reader.transceive(ecp.data(),ecp.size(),nullptr,0,false,false,12,!fast);
        ecpUs[fast]=fakeMicros-start;
        assert(result.ok==bool(fast));assert(!result.len);
        const auto probeStart=fakeMicros;
        assert(!reader.pollForTag(uid,atqa,sak,250));
        probeUs[fast]=fakeMicros-probeStart;
    }
    assert(ecpUs[0]>=12000 && ecpUs[1]<2000);
    assert(probeUs[0]>=250000 && probeUs[1]<30000);
    noResponseIrq=true;ST25R3916::setFastPolling(false);reader.stop();
    std::cout << "A/B timing model: ECP " << ecpUs[0] << " -> " << ecpUs[1]
        << " us; idle probe " << probeUs[0] << " -> " << probeUs[1] << " us (simulated bus)\n";
#endif
    bus.deviceAddStatus=ESP_ERR_NO_MEM;assert(!reader.init() && bus.liveDevices==1);
    std::cout << "ST25R3916/3916B I2C identity, calibration, supply, UID, ECP, ISO-DEP chaining/CRC/WTX/bounds and shared-bus lifecycle passed\n";
}
