#include <array>
#include <vector>
#include <Wire.h>
#define private public
#include "ST25R3916.hpp"
#undef private
#include <cassert>
#include <iostream>

static void isoDep(ST25R3916& reader) {
    reader.m_isodepActive=true;reader.m_blockNum=0;reader.m_fsc=32;reader.m_fwtMs=5;
}
int main() {
    TwoWire wire;std::array<uint8_t,18> ecp{};
    ST25R3916 reader(wire,21,22);
    assert(reader.init() && reader.init());
    assert(wire.sda==21 && wire.scl==22 && wire.deviceAdds==1);
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
    reader.stop();reader.stop();assert(wire.liveDevices==0 && wire.busInitialized);
    assert(reader.init());reader.stop();assert(wire.deviceAdds==2 && wire.deviceRemoves==2);
    chipIdentity=0;assert(!reader.init() && wire.liveDevices==0);chipIdentity=0x28;
    wire.deviceAddStatus=ESP_ERR_NO_MEM;assert(!reader.init() && wire.liveDevices==0);
    std::cout << "ST25R3916 I2C UID, ECP, ISO-DEP chaining/CRC/WTX/bounds and shared-bus lifecycle passed\n";
}
