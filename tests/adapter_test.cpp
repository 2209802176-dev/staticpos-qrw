// SPDX-License-Identifier: GPL-3.0-or-later
// All positions below are generated around the synthetic 10 deg / 20 deg origin.
#include "adapter_stub/Copter.h"
#include <iostream>
#include <stdexcept>
#include <map>
#include <iomanip>
namespace adapter_stub {
uint64_t now_us=0; AP_GPS gps; AP_InertialSensor ins; FakeAHRS ahrs;
FakeGCS gcs; FakeLogger logger; std::vector<std::string> notices;
}
#include "../ardupilot/UserCode.cpp"
namespace {
Copter vehicle;
void require(bool value,const char *message) { if(!value) throw std::runtime_error(message); }
void reset(uint64_t us=1000000) {
    adapter_stub::now_us=us; adapter_stub::gps=AP_GPS();
    adapter_stub::ins=AP_InertialSensor(); adapter_stub::ahrs=FakeAHRS();
    adapter_stub::gcs.packets.clear(); adapter_stub::logger.records.clear();
    adapter_stub::notices.clear(); vehicle.storage.value=false; vehicle.userhook_init();
}
void fix(uint64_t us,int32_t lat=100000000,int32_t lon=200000000) {
    adapter_stub::now_us=us; auto &gps=adapter_stub::gps;
    gps.sensors=1; gps.fix=AP_GPS::GPS_OK_FIX_3D;
    gps.fix_ms=static_cast<uint32_t>(us/1000); gps.message_ms=gps.fix_ms+1;
    gps.loc.lat=lat; gps.loc.lng=lon; vehicle.userhook_MediumLoop();
}
void ticks(unsigned count) {
    for(unsigned n=0;n<count;++n) {
        vehicle.userhook_50Hz(); adapter_stub::now_us+=20000;
    }
}
std::map<std::string,CapturedPacket> batch(uint32_t time) {
    std::map<std::string,CapturedPacket> out;
    for(const auto &packet:adapter_stub::gcs.packets) if(packet.time_ms==time) out[packet.name]=packet;
    return out;
}
void run() {
    reset(); fix(1000000); ticks(30);
    auto first=batch(1000);
    require(first.at("ST_STATE").integer==1 && first.at("ST_VER").integer==3,"first fix/version");
    require(first.at("ST_GPSMS").integer==1000 && first.at("ST_MSGMS").integer==1001,"source/message clocks");
    require(first.at("ST_LAT").integer==100000000,"e7 precision");
    require(first.at("ST_COUNT").integer==1 && first.at("ST_NWIN").integer==1,"counts");
    require(first.at("ST_FCOK").integer==0,"official unavailable");
    for(size_t n=1;n<adapter_stub::gcs.packets.size();++n)
        require(adapter_stub::gcs.packets[n].sent_us>adapter_stub::gcs.packets[n-1].sent_us,"one field per tick");
    require(first.at("ST_IMU").number==0,"packet type sanity");
    require(adapter_stub::gps.loc.lat==100000000 && adapter_stub::gps.loc.lng==200000000,"raw unchanged");
    ticks(20); // t=2s; a new raw fix arrives before the next estimator hook.
    adapter_stub::gps.fix_ms=2000; adapter_stub::gps.message_ms=2001;
    ticks(30); auto second=batch(2000);
    require(second.at("ST_GPSMS").integer==1000 && second.at("ST_COUNT").integer==1,"snapshot source must match estimate");
    require(second.at("ST_AGE").number>=1.0f,"repeated estimate age");
    require(second.at("ST_SEQ").integer>first.at("ST_SEQ").integer,"snapshot sequence");

    reset(); fix(1000000); ticks(5); vehicle.storage.value=true; ticks(4);
    require(batch(1100).at("ST_STATE").integer==4,"armed preempts partial snapshot");
    require(estimator.diagnostics().update_count==0,"armed resets once");
    vehicle.storage.value=false; fix(1300000); ticks(30);
    require(batch(1300).at("ST_SEG").integer>first.at("ST_SEG").integer,"segment monotonic across reset");

    reset(); fix(1000000); ticks(30);
    adapter_stub::now_us=3001000; ticks(4);
    require(batch(3001).at("ST_STATE").integer==3,"stale preempts without estimator hook");
    fix(6000000); ticks(30);
    require(estimator.diagnostics().window_count==2,"short interruption keeps history");
    fix(16001000); ticks(30);
    require(estimator.diagnostics().window_count==1 && segment_generation==2,"long gap makes new segment");

    reset(); fix(1000000); ticks(30);
    adapter_stub::gps.sensors=2; ticks(4);
    require(state==MULTI_GPS && estimator.diagnostics().window_count==0,"multi GPS gate");
    adapter_stub::gps.sensors=1; ticks(4); vehicle.userhook_MediumLoop();
    require(state==WAIT_GPS,"same fix after multi reset cannot reenter");

    reset(); adapter_stub::ins.w.z=.2f; fix(1000000);
    fix(1200000,100000010,200000000); ticks(30);
    require(estimator.diagnostics().lambda==4,"causal gyro gate");
    adapter_stub::ins.healthy=false; fix(1400000); ticks(30);
    require(!estimator.diagnostics().imu_valid,"unhealthy IMU visible");

    reset((UINT64_C(1)<<32)*1000-200000); fix(adapter_stub::now_us); ticks(2);
    fix((UINT64_C(1)<<32)*1000); ticks(2);
    fix((UINT64_C(1)<<32)*1000+200000); ticks(30);
    require(estimator.diagnostics().update_count==3,"source millis rollover");
    require(state==VALID,"source zero at rollover remains valid");

    reset(); fix(1000000); ticks(30);
    fix(2000000,0,0); ticks(4);
    require(state==WAIT_GPS,"invalid coordinates block publication");
    fix(2200000); ticks(30);
    require(state==VALID && estimator.diagnostics().window_count==2,"valid recovery preserves usable history");
    require(!adapter_stub::logger.records.empty(),"dataflash records");
}
void fixture() {
    reset(); adapter_stub::ahrs.ok=true;
    adapter_stub::ahrs.location.lat=100000000;
    adapter_stub::ahrs.location.lng=200000000;
    for(unsigned elapsed=0;elapsed<20000;elapsed+=20) {
        const uint64_t now=UINT64_C(1000000)+static_cast<uint64_t>(elapsed)*1000U;
        adapter_stub::now_us=now;
        vehicle.storage.value=elapsed>=14000 && elapsed<15000;
        adapter_stub::ins.w.z=elapsed>=2000 && elapsed<3000?.12f:0;
        const bool gps_available=elapsed<6000 || elapsed>=9000;
        if(gps_available && elapsed%200==0) {
            fix(now,100000000+static_cast<int32_t>((elapsed/200)%7),200000000);
        } else if(elapsed%100==0) { vehicle.userhook_MediumLoop(); }
        vehicle.userhook_50Hz();
    }
    std::cout<<std::setprecision(9);
    for(const auto &p:adapter_stub::gcs.packets) {
        const uint64_t elapsed=(p.sent_us-UINT64_C(1000000))/1000;
        const char *phase=elapsed<6000?"normal":elapsed<9000?"gap":
            elapsed<14000?"recovery":elapsed<15000?"armed":"new_session";
        std::cout<<"{\"sent_us\":"<<p.sent_us<<",\"phase\":\""<<phase
            <<"\",\"message\":{\"mavpackettype\":\"NAMED_VALUE_"<<(p.id==252?"INT":"FLOAT")
            <<"\",\"name\":\""<<p.name<<"\",\"time_boot_ms\":"<<p.time_ms<<",\"value\":";
        if(p.id==252) std::cout<<p.integer; else std::cout<<p.number;
        std::cout<<"}}\n";
    }
}
}
int main(int argc,char **argv) {
    if(argc==2 && std::string(argv[1])=="--telemetry-fixture") { fixture(); return 0; }
    try { run(); std::cout<<"{\"status\":\"PASS\",\"scope\":\"actual V3 adapter with host boundary doubles\"}\n"; return 0; }
    catch(const std::exception &e) { std::cerr<<"Adapter FAIL: "<<e.what()<<"\n"; return 1; }
}
