/* SPDX-License-Identifier: GPL-3.0-or-later
 * Streaming qrw_w90_k3. No heap allocation, RTK input, or navigation writeback.
 */
#ifndef STATICPOS_V3_ROBUST_WINDOW_ESTIMATOR_H
#define STATICPOS_V3_ROBUST_WINDOW_ESTIMATOR_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace staticpos_v3 {

// Extend one monotonically sampled uint32 millisecond clock. Backward jumps
// larger than an ordinary wrap are rejected; reset explicitly after reboot.
class MillisClock {
public:
    MillisClock() : _have(false), _last(0), _us(0) {}
    void reset() { _have=false; _last=0; _us=0; }
    bool extend(uint32_t ms, uint64_t &us) {
        if (!_have) { _have=true; _last=ms; _us=static_cast<uint64_t>(ms)*1000U; }
        else {
            const uint32_t delta=ms-_last;
            if (delta>UINT32_C(0x7fffffff)) return false;
            _us+=static_cast<uint64_t>(delta)*1000U;
            _last=ms;
        }
        us=_us;
        return true;
    }
private:
    bool _have;
    uint32_t _last;
    uint64_t _us;
};

enum UpdateResult {
    UPDATE_OK=0,
    UPDATE_DUPLICATE=1,
    UPDATE_OUT_OF_ORDER=2,
    UPDATE_INVALID=3,
    UPDATE_OVERFLOW=4,
    UPDATE_NUMERICAL_ERROR=5
};

struct Diagnostics {
    double sigma_m;
    double lambda;
    double robust_factor;
    double newest_weight; // Last IRLS weight divided by all last IRLS weights.
    double effective_n;   // Kish support, not independent samples or accuracy.
    double window_span_s;
    uint32_t window_count;
    uint32_t update_count; // Unique valid GPS epochs, including capacity losses.
    uint32_t segment_count;
    uint32_t overflow_count;
    uint8_t iterations;
    bool imu_valid;
    bool history_complete;
    Diagnostics() : sigma_m(0),lambda(1),robust_factor(1),newest_weight(0),
        effective_n(0),window_span_s(0),window_count(0),update_count(0),
        segment_count(0),overflow_count(0),iterations(0),imu_valid(false),
        history_complete(true) {}
};

class RobustWindowEstimator {
public:
    enum { CAPACITY=1024, IMU_HISTORY_CAPACITY=256 };
    static const uint64_t WINDOW_US=UINT64_C(90000000);
    static const uint64_t GAP_US=UINT64_C(10000000);
    static const uint64_t TURN_HOLD_US=UINT64_C(2000000);

    RobustWindowEstimator() { reset(); }

    void reset() {
        _head=0; _size=0; _imu_head=0; _imu_size=0;
        _have_origin=false; _have_gps=false; _output_valid=false;
        _have_missing=false; _last_missing_us=0; _last_sample_us=0;
        _origin_lat=0; _origin_lon=0; _east_scale=0; _north_scale=0;
        _east=0; _north=0; _diag=Diagnostics(); _last_result=UPDATE_INVALID;
    }

    // Gyro SI units (rad/s), source timestamp on the same extended boot clock
    // as GPS. Feed IMU in nondecreasing source-time order. Recent future IMU
    // entries are buffered but never used by an earlier GPS epoch.
    bool update_imu(uint64_t sample_us,double gx,double gy,double gz,
                    bool healthy=true) {
        if (_imu_size && sample_us<imu_at(_imu_size-1).time_us) return false;
        if (_imu_size && sample_us==imu_at(_imu_size-1).time_us) return false;
        const bool good=healthy && finite(gx) && finite(gy) && finite(gz);
        const double norm=good ? std::sqrt(gx*gx+gy*gy+gz*gz) : 0.0;
        ImuPoint point={sample_us,good && finite(norm),good && norm>static_cast<double>(0.08L)};
        if (_imu_size==IMU_HISTORY_CAPACITY) {
            _imu_head=(_imu_head+1U)%IMU_HISTORY_CAPACITY; --_imu_size;
        }
        _imu[(_imu_head+_imu_size)%IMU_HISTORY_CAPACITY]=point;
        ++_imu_size;
        return point.healthy;
    }

    // hacc_m is metres; HDOP is dimensionless (MAVLink eph / 100).
    // Invalid input invalidates publication but does not insert/reset history.
    // Duplicates and old epochs preserve the last solution and its source age.
    UpdateResult update_gps(uint64_t sample_us,int32_t lat_e7,int32_t lon_e7,
                            uint8_t fix,double hacc_m,double hdop,
                            double satellites) {
        if (!sample_us || fix<3U || lat_e7<INT32_C(-900000000) ||
            lat_e7>INT32_C(900000000) || lon_e7<INT32_C(-1800000000) ||
            lon_e7>INT32_C(1800000000) || (lat_e7==0 && lon_e7==0)) {
            _output_valid=false; return result(UPDATE_INVALID);
        }
        if (_have_gps && sample_us==_last_sample_us) return result(UPDATE_DUPLICATE);
        if (_have_gps && sample_us<_last_sample_us) return result(UPDATE_OUT_OF_ORDER);
        const double lat=static_cast<double>(lat_e7)/1e7;
        const double lon=static_cast<double>(lon_e7)/1e7;
        if (!_have_origin && !initialize_origin(lat,lon)) {
            _output_valid=false; return result(UPDATE_NUMERICAL_ERROR);
        }
        if (!_have_gps || sample_us-_last_sample_us>GAP_US) {
            _head=0; _size=0; _have_missing=false; ++_diag.segment_count;
        }
        _have_gps=true; _last_sample_us=sample_us; ++_diag.update_count;
        while (_size && sample_us-point_at(0).time_us>WINDOW_US) {
            _head=(_head+1U)%CAPACITY; --_size;
        }
        if (_have_missing && sample_us-_last_missing_us>WINDOW_US) _have_missing=false;
        _diag.sigma_m=quality_sigma(hacc_m,hdop,satellites);
        _diag.lambda=motion_lambda(sample_us,_diag.imu_valid);
        _diag.iterations=0;
        if (_size==CAPACITY) {
            _have_missing=true; _last_missing_us=sample_us;
            ++_diag.overflow_count; _output_valid=false;
            update_window_diagnostics();
            return result(UPDATE_OVERFLOW);
        }
        const double variance=_diag.sigma_m*_diag.sigma_m*_diag.lambda;
        const Point point={(lon-_origin_lon)*_east_scale,
            (lat-_origin_lat)*_north_scale,variance,1.0/variance,sample_us};
        _points[(_head+_size)%CAPACITY]=point; ++_size;
        update_window_diagnostics();
        if (_have_missing) { _output_valid=false; return result(UPDATE_OVERFLOW); }
        if (!compute()) { _output_valid=false; return result(UPDATE_NUMERICAL_ERROR); }
        _output_valid=true;
        return result(UPDATE_OK);
    }

    bool valid(uint64_t now_us,uint64_t max_age_us=UINT64_C(2000000)) const {
        return _output_valid && _have_gps && now_us>=_last_sample_us &&
            now_us-_last_sample_us<=max_age_us;
    }
    bool has_origin() const { return _have_origin; }
    uint64_t last_sample_us() const { return _last_sample_us; }
    UpdateResult last_result() const { return _last_result; }
    const Diagnostics &diagnostics() const { return _diag; }
    double east_m() const { return _east; }
    double north_m() const { return _north; }
    double latitude_deg() const { return _have_origin?_origin_lat+_north/_north_scale:0.0; }
    double longitude_deg() const { return _have_origin?_origin_lon+_east/_east_scale:0.0; }
    int32_t latitude_e7() const { return geodetic_e7(latitude_deg()); }
    int32_t longitude_e7() const { return geodetic_e7(longitude_deg()); }

    static double quality_sigma(double hacc_m,double hdop,double satellites) {
        if (!finite(hacc_m) || hacc_m<=0 || hacc_m>=1000) hacc_m=2.0;
        if (!finite(hdop) || hdop<=0 || hdop>=static_cast<double>(655.35L)) hdop=2.0;
        if (!finite(satellites) || satellites<=0 || satellites>=255) satellites=6.0;
        return clip(maximum(hacc_m,static_cast<double>(0.6L))*
            clip(hdop/static_cast<double>(1.2L),static_cast<double>(0.7L),3.0)*
            clip(std::sqrt(10.0/satellites),static_cast<double>(0.85L),
                 static_cast<double>(1.6L)),static_cast<double>(0.6L),8.0);
    }

private:
    struct Point { double east,north,variance,inverse_variance; uint64_t time_us; };
    struct ImuPoint { uint64_t time_us; bool healthy,turning; };
    Point _points[CAPACITY];
    ImuPoint _imu[IMU_HISTORY_CAPACITY];
    uint32_t _head,_size,_imu_head,_imu_size;
    bool _have_origin,_have_gps,_output_valid,_have_missing;
    uint64_t _last_missing_us,_last_sample_us;
    double _origin_lat,_origin_lon,_east_scale,_north_scale,_east,_north;
    Diagnostics _diag;
    UpdateResult _last_result;

    static bool finite(double x) { return std::isfinite(x); }
    static double maximum(double a,double b) { return a>b?a:b; }
    static double clip(double x,double lo,double hi) { return x<lo?lo:(x>hi?hi:x); }
    static int32_t geodetic_e7(double degrees) {
        if (!finite(degrees) || degrees*1e7<std::numeric_limits<int32_t>::min() ||
            degrees*1e7>std::numeric_limits<int32_t>::max()) return 0;
        return static_cast<int32_t>(std::llround(degrees*1e7));
    }
    UpdateResult result(UpdateResult r) { _last_result=r; return r; }
    const Point &point_at(uint32_t i) const { return _points[(_head+i)%CAPACITY]; }
    const ImuPoint &imu_at(uint32_t i) const { return _imu[(_imu_head+i)%IMU_HISTORY_CAPACITY]; }
    bool initialize_origin(double lat,double lon) {
        // ArduPilot compiles with -fsingle-precision-constant. Long literals
        // explicitly converted to double retain the same constants on target.
        const double pi=static_cast<double>(3.141592653589793238462643383279502884L);
        const double e2=static_cast<double>(6.6943799901413165e-3L);
        const double p=lat*pi/180.0;
        const double s=std::sin(p),d=1.0-e2*s*s;
        _east_scale=pi/180.0*6378137.0/std::sqrt(d)*std::cos(p);
        _north_scale=pi/180.0*6378137.0*(1.0-e2)/std::pow(d,1.5);
        if (!finite(_east_scale) || !finite(_north_scale) || _east_scale<=0 ||
            _north_scale<=0) return false;
        _origin_lat=lat; _origin_lon=lon; _have_origin=true;
        return true;
    }
    double motion_lambda(uint64_t sample_us,bool &imu_valid) const {
        bool have_recent=false,turning=false;
        imu_valid=false;
        for (uint32_t n=_imu_size;n>0;--n) {
            const ImuPoint &p=imu_at(n-1);
            if (p.time_us>sample_us) continue;
            const uint64_t age=sample_us-p.time_us;
            if (!have_recent) { imu_valid=p.healthy && age<=UINT64_C(1000000); have_recent=true; }
            if (age>TURN_HOLD_US) break;
            if (p.healthy && p.turning) turning=true;
        }
        return turning?4.0:1.0;
    }
    void update_window_diagnostics() {
        _diag.window_count=_size;
        _diag.window_span_s=_size ? static_cast<double>(
            point_at(_size-1).time_us-point_at(0).time_us)/1e6 : 0.0;
        _diag.history_complete=!_have_missing;
    }
    bool compute() {
        double sum_w=0,e=0,n=0;
        for (uint32_t i=0;i<_size;++i) {
            const Point &p=point_at(i); const double w=p.inverse_variance;
            sum_w+=w; e+=p.east*w; n+=p.north*w;
        }
        if (!(sum_w>0) || !finite(sum_w)) return false;
        e/=sum_w; n/=sum_w;
        double final_sum=0,final_sq=0,latest_w=0,latest_robust=1;
        for (uint8_t iteration=0;iteration<4;++iteration) {
            double se=0,sn=0; final_sum=0; final_sq=0;
            for (uint32_t i=0;i<_size;++i) {
                const Point &p=point_at(i);
                const double de=p.east-e,dn=p.north-n;
                const double distance2=de*de+dn*dn;
                // Inliers have Huber weight 1 exactly: avoid software-double
                // sqrt/division on the normal branch. Variance is >=0.6^2.
                double robust=1.0;
                if (distance2>9.0*p.variance) {
                    const double r=std::sqrt(distance2)/maximum(std::sqrt(p.variance),
                        static_cast<double>(0.6L));
                    robust=clip(3.0/maximum(r,static_cast<double>(1e-12L)),0.0,1.0);
                }
                const double w=p.inverse_variance*robust;
                final_sum+=w; final_sq+=w*w; se+=p.east*w; sn+=p.north*w;
                if (i+1==_size) { latest_w=w; latest_robust=robust; }
            }
            if (!(final_sum>0) || !finite(final_sum)) return false;
            const double next_e=se/final_sum,next_n=sn/final_sum;
            const double de=next_e-e,dn=next_n-n;
            e=next_e; n=next_n; _diag.iterations=iteration+1;
            if (std::sqrt(de*de+dn*dn)<static_cast<double>(1e-7L)) break;
        }
        if (!finite(e) || !finite(n) || !(final_sq>0)) return false;
        _east=e; _north=n; _diag.robust_factor=latest_robust;
        _diag.newest_weight=latest_w/final_sum;
        _diag.effective_n=final_sum*final_sum/final_sq;
        return finite(_diag.effective_n);
    }
};

} // namespace staticpos_v3
#endif
