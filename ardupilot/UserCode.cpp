/* SPDX-License-Identifier: GPL-3.0-or-later
 * ArduCopter 4.6.3 / fmuv3: disarmed, one GPS, independent qrw_w90_k3.
 * No writes to GPS, AHRS, EKF, home, parameters, or control targets.
 */
#define ALLOW_DOUBLE_MATH_FUNCTIONS
#include "Copter.h"
#include "RobustWindowEstimator.h"
#include <cstring>

namespace {
staticpos_v3::RobustWindowEstimator estimator;
enum State : uint8_t { WAIT_GPS=0, VALID=1, STALE=3, ARMED=4,
    MULTI_GPS=5, CAPACITY_LOSS=6, NUMERICAL_ERROR=7 };
State state=WAIT_GPS;
bool have_seen_fix=false;
uint32_t seen_fix_ms=0, calculation_us=0, maximum_us=0;
uint32_t segment_generation=0, previous_core_segments=0;
uint32_t budget_overruns=0;
uint32_t sequence=0, previous_capture_ms=0, capture_interval_ms=0;
bool have_capture=false;
State captured_state=WAIT_GPS;
uint32_t used_message_ms=0;

struct Field {
    const char *name;
    bool integer;
    int32_t iv;
    float fv;
};
struct Snapshot {
    Field fields[32];
    uint8_t size=0, next=0;
    uint32_t capture_ms=0;
    bool active=false;
    void add_i(const char *name,int32_t value) {
        if (size<32) { fields[size++]={name,true,value,0}; }
    }
    void add_f(const char *name,float value) {
        if (size<32) { fields[size++]={name,false,0,value}; }
    }
} snapshot;

int32_t wire_u32(uint32_t n) {
    int32_t out;
    static_assert(sizeof(out)==sizeof(n),"32-bit wire integer required");
    std::memcpy(&out,&n,sizeof(out));
    return out;
}

void send_field(const Field &field,uint32_t capture_ms) {
    if (field.integer) {
        mavlink_named_value_int_t packet {};
        packet.time_boot_ms=capture_ms; packet.value=field.iv;
        std::memcpy(packet.name,field.name,std::strlen(field.name));
        gcs().send_to_active_channels(MAVLINK_MSG_ID_NAMED_VALUE_INT,
                                     reinterpret_cast<const char *>(&packet));
    } else {
        mavlink_named_value_float_t packet {};
        packet.time_boot_ms=capture_ms; packet.value=field.fv;
        std::memcpy(packet.name,field.name,std::strlen(field.name));
        gcs().send_to_active_channels(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,
                                     reinterpret_cast<const char *>(&packet));
    }
}

// Invalidating a session occurs once per gate transition, not every 50 Hz tick.
void set_gate(State next) {
    if (state!=next && (next==ARMED || next==MULTI_GPS)) {
        estimator.reset(); previous_core_segments=0;
        seen_fix_ms=AP::gps().last_fix_time_ms(0); have_seen_fix=true;
    }
    state=next;
}

void check_gate(uint64_t now_us) {
    const AP_GPS &gps=AP::gps();
    if (gps.num_sensors()!=1) {
        set_gate(gps.num_sensors()==0?WAIT_GPS:MULTI_GPS); return;
    }
    if (gps.status(0)<AP_GPS::GPS_OK_FIX_3D) { state=WAIT_GPS; return; }
    const uint32_t now_ms=static_cast<uint32_t>(now_us/1000U);
    const uint32_t age_ms=now_ms-gps.last_fix_time_ms(0);
    if (age_ms>2000U) { state=STALE; return; }
    if (estimator.valid(now_us)) { state=VALID; }
    else if (estimator.last_result()==staticpos_v3::UPDATE_OVERFLOW) { state=CAPACITY_LOSS; }
    else if (estimator.last_result()==staticpos_v3::UPDATE_NUMERICAL_ERROR) { state=NUMERICAL_ERROR; }
    else { state=WAIT_GPS; }
}

void log_epoch(uint64_t now_us,uint32_t source_ms,uint32_t message_ms,
               const Location &raw,double hacc,double hdop,uint8_t satellites) {
#if HAL_LOGGING_ENABLED
    const auto &d=estimator.diagnostics();
    // Split records keep each logger labels string and packet below limits.
    AP::logger().Write("S3PO","TimeUS,GPSms,MsgMS,RLat,RLon,Lat,Lon,E,N,Seq",
        "QIILLLLffI",now_us,source_ms,message_ms,raw.lat,raw.lng,
        estimator.latitude_e7(),estimator.longitude_e7(),
        estimator.east_m(),estimator.north_m(),d.update_count);
    AP::logger().Write("S3DG","TimeUS,Sig,Lam,Hub,Wgt,Neff,Span,NWin,Seg,US,MaxUS,Over",
        "QffffffIIIII",now_us,d.sigma_m,d.lambda,d.robust_factor,d.newest_weight,
        d.effective_n,d.window_span_s,d.window_count,segment_generation,
        calculation_us,maximum_us,budget_overruns);
    AP::logger().Write("S3IN","TimeUS,GPSms,HAcc,HDop,NSat,Fix,IMU,State,Lost",
        "QIffBBBBI",now_us,source_ms,hacc,hdop,
        static_cast<unsigned int>(satellites),static_cast<unsigned int>(AP::gps().status(0)),
        static_cast<unsigned int>(d.imu_valid?(d.lambda>1?2:1):0),
        static_cast<unsigned int>(state),d.overflow_count);
#else
    (void)now_us; (void)source_ms; (void)message_ms; (void)raw;
    (void)hacc; (void)hdop; (void)satellites;
#endif
}

void update(bool armed) {
    const uint64_t now_us=AP_HAL::micros64();
    const AP_GPS &gps=AP::gps();
    if (armed) { set_gate(ARMED); return; }
    if (gps.num_sensors()!=1) { check_gate(now_us); return; }
    const Vector3f &w=AP::ins().get_gyro(0);
    const bool imu_healthy=AP::ins().get_gyro_health(0);
    estimator.update_imu(now_us,w.x,w.y,w.z,imu_healthy);
#if HAL_LOGGING_ENABLED
    AP::logger().Write("S3IM","TimeUS,GX,GY,GZ,Healthy","QfffB",now_us,
        static_cast<double>(w.x),static_cast<double>(w.y),static_cast<double>(w.z),
        static_cast<unsigned int>(imu_healthy));
#endif
    check_gate(now_us);
    if (state==STALE || gps.status(0)<AP_GPS::GPS_OK_FIX_3D) { return; }
    const uint32_t source_ms=gps.last_fix_time_ms(0);
    if (have_seen_fix && source_ms==seen_fix_ms) { return; }
    // Source fix time is exactly the timestamp used by GPS_RAW_INT upstream.
    const uint32_t age_ms=static_cast<uint32_t>(now_us/1000U)-source_ms;
    if (age_ms>2000U || now_us<static_cast<uint64_t>(age_ms)*1000U) { state=STALE; return; }
    const uint64_t source_us=(now_us/1000U-age_ms)*1000U;
    have_seen_fix=true; seen_fix_ms=source_ms;
    float horizontal_accuracy=0;
    if (!gps.horizontal_accuracy(0,horizontal_accuracy)) { horizontal_accuracy=0; }
    const double hdop=static_cast<double>(gps.get_hdop(0))/100.0;
    const Location &raw=gps.location(0);
    const uint64_t begin=AP_HAL::micros64();
    const auto result=estimator.update_gps(source_us,raw.lat,raw.lng,
        static_cast<uint8_t>(gps.status(0)),horizontal_accuracy,hdop,gps.num_sats(0));
    calculation_us=static_cast<uint32_t>(AP_HAL::micros64()-begin);
    if (calculation_us>500U) { ++budget_overruns; }
    if (estimator.diagnostics().segment_count!=previous_core_segments) {
        ++segment_generation; previous_core_segments=estimator.diagnostics().segment_count;
    }
    if (calculation_us>maximum_us) { maximum_us=calculation_us; }
    if (result==staticpos_v3::UPDATE_OK) {
        state=VALID; used_message_ms=gps.last_message_time_ms(0);
        log_epoch(now_us,source_ms,gps.last_message_time_ms(0),raw,
                  static_cast<double>(horizontal_accuracy),hdop,gps.num_sats(0));
    } else {
        check_gate(AP_HAL::micros64());
#if HAL_LOGGING_ENABLED
        AP::logger().Write("S3ER","TimeUS,GPSms,Result,Lost","QIBI",
            now_us,source_ms,static_cast<unsigned int>(result),estimator.diagnostics().overflow_count);
#endif
    }
}

void capture(uint32_t now_ms) {
    snapshot.size=0; snapshot.next=0; snapshot.capture_ms=now_ms;
    snapshot.active=true; ++sequence;
    captured_state=state;
    capture_interval_ms=have_capture?now_ms-previous_capture_ms:0;
    previous_capture_ms=now_ms; have_capture=true;
    const auto &d=estimator.diagnostics();
    snapshot.add_i("ST_VER",3);
    snapshot.add_i("ST_STATE",state);
    snapshot.add_i("ST_SEQ",wire_u32(sequence));
    const uint32_t source_ms=state==VALID?
        static_cast<uint32_t>(estimator.last_sample_us()/1000U):AP::gps().last_fix_time_ms(0);
    snapshot.add_i("ST_GPSMS",wire_u32(source_ms));
    snapshot.add_i("ST_MSGMS",wire_u32(state==VALID?used_message_ms:AP::gps().last_message_time_ms(0)));
    snapshot.add_i("ST_COUNT",wire_u32(d.update_count));
    snapshot.add_i("ST_SEG",wire_u32(segment_generation));
    const uint32_t age=now_ms-source_ms;
    snapshot.add_f("ST_AGE",d.update_count?static_cast<float>(age)*.001f:-1.0f);
    snapshot.add_f("ST_US",static_cast<float>(calculation_us));
    snapshot.add_f("ST_MAXUS",static_cast<float>(maximum_us));
    snapshot.add_f("ST_DT",static_cast<float>(capture_interval_ms)*.001f);
    snapshot.add_i("ST_OVR",wire_u32(budget_overruns));
    if (state==VALID) {
    snapshot.add_i("ST_LAT",estimator.latitude_e7());
    snapshot.add_i("ST_LON",estimator.longitude_e7());
    snapshot.add_f("ST_E",static_cast<float>(estimator.east_m()));
    snapshot.add_f("ST_N",static_cast<float>(estimator.north_m()));
    snapshot.add_i("ST_NWIN",static_cast<int32_t>(d.window_count));
    snapshot.add_f("ST_SIG",static_cast<float>(d.sigma_m));
    snapshot.add_f("ST_LAM",static_cast<float>(d.lambda));
    snapshot.add_f("ST_HUB",static_cast<float>(d.robust_factor));
    snapshot.add_f("ST_WGT",static_cast<float>(d.newest_weight));
    snapshot.add_f("ST_NEFF",static_cast<float>(d.effective_n));
    snapshot.add_f("ST_SPAN",static_cast<float>(d.window_span_s));
    snapshot.add_i("ST_IMU",d.imu_valid?(d.lambda>1?2:1):0);
    Location original;
    const bool original_ok=AP::ahrs().get_location(original);
    snapshot.add_i("ST_FCOK",original_ok?1:0);
    if (original_ok) {
        snapshot.add_i("ST_FCLAT",original.lat);
        snapshot.add_i("ST_FCLON",original.lng);
    }
    }
#if HAL_LOGGING_ENABLED
    AP::logger().Write("S3TX","TimeUS,BatchMS,Seq,GPSms,DtMS,State,Fields",
        "QIIIIBB",AP_HAL::micros64(),now_ms,sequence,source_ms,
        capture_interval_ms,static_cast<unsigned int>(state),static_cast<unsigned int>(snapshot.size));
#endif
}
} // namespace

#ifdef USERHOOK_INIT
void Copter::userhook_init() {
    estimator.reset(); state=WAIT_GPS; have_seen_fix=false;
    sequence=0; segment_generation=0; previous_core_segments=0;
    calculation_us=0; maximum_us=0; captured_state=WAIT_GPS;
    budget_overruns=0;
    have_capture=false; snapshot.active=false;
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"STPOS V3 qrw_w90_k3: STATIC ONLY");
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop() { update(motors->armed()); }
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz() {
    const uint64_t now_us=AP_HAL::micros64();
    if (motors->armed()) { set_gate(ARMED); }
    else { check_gate(now_us); }
    const uint32_t now_ms=static_cast<uint32_t>(now_us/1000U);
    // Safety/invalid transitions preempt an unfinished valid batch. Receivers
    // discard the old batch as soon as the new ST_STATE arrives.
    const bool state_transition=state!=captured_state;
    if (!have_capture || state_transition ||
        (!snapshot.active && now_ms-previous_capture_ms>=1000U)) { capture(now_ms); }
    if (snapshot.active) {
        send_field(snapshot.fields[snapshot.next++],snapshot.capture_ms);
        if (snapshot.next>=snapshot.size) { snapshot.active=false; }
    }
}
#endif
