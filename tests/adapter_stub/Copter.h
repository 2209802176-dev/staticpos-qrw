#pragma once
// Boundary doubles compile the actual adapter; never included in firmware.
#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cassert>
#include <string>
#include <vector>
#include <cmath>
using std::isfinite;
#define USERHOOK_INIT 1
#define USERHOOK_MEDIUMLOOP 1
#define USERHOOK_50HZLOOP 1
#define HAL_LOGGING_ENABLED 1
#define MAV_SEVERITY_NOTICE 5
#define MAVLINK_MSG_ID_NAMED_VALUE_INT 252
#define MAVLINK_MSG_ID_NAMED_VALUE_FLOAT 251
struct Location { int32_t lat=0,lng=0; };
struct mavlink_named_value_int_t { uint32_t time_boot_ms; int32_t value; char name[10]; };
struct mavlink_named_value_float_t { uint32_t time_boot_ms; float value; char name[10]; };
struct Vector3f { float x=0,y=0,z=0; };
struct AP_GPS {
    enum GPS_Status { NO_GPS=0,NO_FIX=1,GPS_OK_FIX_2D=2,GPS_OK_FIX_3D=3 };
    uint8_t sensors=0,satellites=12;
    uint32_t fix_ms=0,message_ms=0;
    uint16_t hdop=100;
    float hacc=1;
    bool have_hacc=true;
    GPS_Status fix=NO_GPS;
    Location loc;
    uint8_t num_sensors() const { return sensors; }
    GPS_Status status(unsigned) const { return fix; }
    uint32_t last_fix_time_ms(unsigned) const { return fix_ms; }
    uint32_t last_message_time_ms(unsigned) const { return message_ms; }
    uint16_t get_hdop(unsigned) const { return hdop; }
    uint8_t num_sats(unsigned) const { return satellites; }
    const Location &location(unsigned) const { return loc; }
    bool horizontal_accuracy(unsigned,float &out) const { out=hacc; return have_hacc; }
};
struct AP_InertialSensor {
    Vector3f w;
    bool healthy=true;
    const Vector3f &get_gyro(unsigned) const { return w; }
    bool get_gyro_health(unsigned) const { return healthy; }
};
struct FakeAHRS {
    bool ok=false; Location location;
    bool get_location(Location &out) const { out=location; return ok; }
};
struct CapturedPacket {
    uint32_t id=0,time_ms=0;
    uint64_t sent_us=0;
    std::string name;
    int32_t integer=0;
    float number=0;
};
namespace adapter_stub { extern uint64_t now_us; }
struct FakeGCS {
    std::vector<CapturedPacket> packets;
    void send_to_active_channels(uint32_t id,const char *data) {
        CapturedPacket p; p.id=id; p.sent_us=adapter_stub::now_us;
        if (id==252) {
            mavlink_named_value_int_t q; std::memcpy(&q,data,sizeof(q));
            p.time_ms=q.time_boot_ms; p.integer=q.value;
            p.name=std::string(q.name,bounded(q.name));
        } else {
            mavlink_named_value_float_t q; std::memcpy(&q,data,sizeof(q));
            p.time_ms=q.time_boot_ms; p.number=q.value;
            p.name=std::string(q.name,bounded(q.name));
        }
        packets.push_back(p);
    }
    static size_t bounded(const char *p) { size_t n=0; while(n<10 && p[n]) ++n; return n; }
};
struct CapturedLog { std::string name,labels,format; };
struct FakeLogger {
    std::vector<CapturedLog> records;
    void Write(const char *name,const char *labels,const char *format,...) {
        assert(std::strlen(labels)<=64);
        unsigned fields=1; for(const char *p=labels;*p;++p) if(*p==',') ++fields;
        assert(fields==std::strlen(format));
        records.push_back({name,labels,format});
    }
};
namespace adapter_stub {
    extern AP_GPS gps; extern AP_InertialSensor ins; extern FakeAHRS ahrs;
    extern FakeGCS gcs; extern FakeLogger logger; extern std::vector<std::string> notices;
}
namespace AP_HAL {
    inline uint64_t micros64() { return adapter_stub::now_us; }
    inline uint32_t millis() { return static_cast<uint32_t>(adapter_stub::now_us/1000U); }
}
namespace AP {
    inline AP_GPS &gps() { return adapter_stub::gps; }
    inline AP_InertialSensor &ins() { return adapter_stub::ins; }
    inline FakeAHRS &ahrs() { return adapter_stub::ahrs; }
    inline FakeLogger &logger() { return adapter_stub::logger; }
}
inline FakeGCS &gcs() { return adapter_stub::gcs; }
#define GCS_SEND_TEXT(severity,text) adapter_stub::notices.push_back(text)
struct FakeMotors { bool value=false; bool armed() const { return value; } };
struct Copter {
    FakeMotors storage; FakeMotors *motors=&storage;
    void userhook_init(); void userhook_MediumLoop(); void userhook_50Hz();
};
