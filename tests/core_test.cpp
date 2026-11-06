/* SPDX-License-Identifier: GPL-3.0-or-later
 * All coordinates are generated around the synthetic 10 deg / 20 deg origin.
 */
#include "RobustWindowEstimator.h"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>

using staticpos_v3::RobustWindowEstimator;
using namespace staticpos_v3;
static unsigned assertions=0;
#define CHECK(condition) do { ++assertions; if (!(condition)) { \
    std::cerr << "FAIL " << __LINE__ << ": " << #condition << '\n'; std::exit(1); } } while (false)

static UpdateResult gps(RobustWindowEstimator &e,uint64_t t,int32_t dlon=0,
                        double h=1.0,double hd=1.2,double sats=10.0) {
    return e.update_gps(t,100000000,200000000+dlon,3,h,hd,sats);
}
static bool close(double a,double b,double tol=1e-10) { return std::abs(a-b)<=tol; }

static void test_initialization_and_input_contract() {
    RobustWindowEstimator e;
    CHECK(!e.valid(1000)); CHECK(!e.has_origin());
    CHECK(e.latitude_e7()==0 && e.longitude_e7()==0);
    CHECK(e.diagnostics().update_count==0 && e.diagnostics().window_count==0);
    CHECK(gps(e,1000)==UPDATE_OK);
    CHECK(e.has_origin() && e.valid(1000));
    CHECK(e.latitude_e7()==100000000 && e.longitude_e7()==200000000);
    CHECK(close(e.east_m(),0) && close(e.north_m(),0));
    CHECK(e.diagnostics().window_count==1 && e.diagnostics().segment_count==1);
    CHECK(close(e.diagnostics().newest_weight,1) && close(e.diagnostics().effective_n,1));
    CHECK(e.valid(2001000) && !e.valid(2001001));
    CHECK(!e.valid(999));
    CHECK(gps(e,1000,1000)==UPDATE_DUPLICATE);
    CHECK(gps(e,999,1000)==UPDATE_OUT_OF_ORDER);
    CHECK(e.diagnostics().update_count==1 && close(e.east_m(),0));
    CHECK(e.update_gps(2000,0,0,3,1,1.2,10)==UPDATE_INVALID);
    CHECK(!e.valid(2000)); CHECK(e.diagnostics().update_count==1);
    CHECK(e.update_gps(2000,100000000,200000000,2,1,1.2,10)==UPDATE_INVALID);
    CHECK(e.update_gps(2000,900000001,200000000,3,1,1.2,10)==UPDATE_INVALID);
    CHECK(e.update_gps(2000,100000000,1800000001,3,1,1.2,10)==UPDATE_INVALID);
    CHECK(gps(e,2000)==UPDATE_OK && e.valid(2000));
    CHECK(e.diagnostics().update_count==2);
    e.reset();
    CHECK(!e.valid(2000) && !e.has_origin());
    CHECK(e.diagnostics().window_count==0 && e.diagnostics().update_count==0);
    CHECK(gps(e,2000)==UPDATE_OK && e.valid(2000));
}

static void test_quality_formula() {
    CHECK(close(RobustWindowEstimator::quality_sigma(1,1.2,10),1));
    CHECK(close(RobustWindowEstimator::quality_sigma(0.01,0.01,254),0.6));
    CHECK(close(RobustWindowEstimator::quality_sigma(999,10,1),8));
    const double fallback=2.0*(2.0/1.2)*std::sqrt(10.0/6.0);
    CHECK(close(RobustWindowEstimator::quality_sigma(0,0,255),fallback));
    const double nan=std::numeric_limits<double>::quiet_NaN();
    CHECK(close(RobustWindowEstimator::quality_sigma(nan,nan,nan),fallback));
    CHECK(close(RobustWindowEstimator::quality_sigma(1000,655.35,255),fallback));
    CHECK(close(RobustWindowEstimator::quality_sigma(-1,-1,-1),fallback));
}

static void test_window_and_segments() {
    RobustWindowEstimator e;
    for (uint64_t t=1000000;t<=91000000;t+=10000000) CHECK(gps(e,t)==UPDATE_OK);
    CHECK(e.diagnostics().window_count==10);
    CHECK(close(e.diagnostics().window_span_s,90));
    CHECK(gps(e,91000001)==UPDATE_OK);
    CHECK(e.diagnostics().window_count==10);
    CHECK(e.diagnostics().window_span_s<90);
    CHECK(gps(e,101000001,100)==UPDATE_OK); // Exactly ten seconds retains segment.
    CHECK(e.diagnostics().segment_count==1 && e.diagnostics().window_count>1);
    CHECK(gps(e,111000002,200)==UPDATE_OK); // >ten seconds cuts history only.
    CHECK(e.diagnostics().window_count==1 && e.diagnostics().segment_count==2);
    CHECK(e.diagnostics().update_count==13);
    CHECK(e.east_m()>1.0); // Origin remains the original first GPS, not this segment.
    CHECK(e.longitude_e7()==200000200);
}

static void test_turn_hold_and_causality() {
    RobustWindowEstimator e;
    CHECK(e.update_imu(1000000,0.08,0,0));
    CHECK(gps(e,1000000)==UPDATE_OK && close(e.diagnostics().lambda,1));
    CHECK(e.update_imu(1100000,0.081,0,0));
    CHECK(gps(e,1100000)==UPDATE_OK && close(e.diagnostics().lambda,4));
    CHECK(e.update_imu(1200000,0,0,0));
    CHECK(gps(e,3100000)==UPDATE_OK && close(e.diagnostics().lambda,4));
    CHECK(gps(e,3100001)==UPDATE_OK && close(e.diagnostics().lambda,1));
    CHECK(e.diagnostics().window_count==4); // No reset at rotation.
    CHECK(e.diagnostics().newest_weight>0);
    CHECK(e.update_imu(5000000,1,0,0)); // Future turn must not change earlier GPS.
    CHECK(gps(e,4000000)==UPDATE_OK && close(e.diagnostics().lambda,1));
    CHECK(gps(e,5000000)==UPDATE_OK && close(e.diagnostics().lambda,4));
    CHECK(!e.update_imu(4999999,0,0,0));
    CHECK(!e.update_imu(5000000,0,0,0));
    CHECK(!e.update_imu(5100000,std::numeric_limits<double>::quiet_NaN(),0,0));
    CHECK(gps(e,5100000)==UPDATE_OK && !e.diagnostics().imu_valid);
    CHECK(close(e.diagnostics().lambda,4)); // Existing two-second hold persists.
    CHECK(gps(e,7000001)==UPDATE_OK && close(e.diagnostics().lambda,1));
    e.reset();
    CHECK(gps(e,8000000)==UPDATE_OK && close(e.diagnostics().lambda,1));
}

static void test_weighting_outliers_and_recovery() {
    RobustWindowEstimator e;
    CHECK(gps(e,1000000,0,1)==UPDATE_OK);
    CHECK(gps(e,2000000,100,2)==UPDATE_OK);
    const double weighted=e.east_m();
    RobustWindowEstimator one;
    CHECK(gps(one,1000000)==UPDATE_OK);
    CHECK(gps(one,12000001,100)==UPDATE_OK);
    CHECK(close(weighted,one.east_m()/5.0,1e-9));
    CHECK(close(e.diagnostics().newest_weight,0.2));
    CHECK(close(e.diagnostics().effective_n,25.0/17.0));
    e.reset();
    for (uint64_t i=0;i<200;++i) CHECK(gps(e,1000000+i*200000)==UPDATE_OK);
    CHECK(gps(e,41000000,11560)==UPDATE_OK); // Large generated position spike.
    CHECK(e.diagnostics().robust_factor<0.04 && e.east_m()<0.1);
    CHECK(e.diagnostics().iterations<=4 && e.diagnostics().iterations>=1);
    for (uint64_t t=41200000;t<=160000000;t+=200000) CHECK(gps(e,t,231)==UPDATE_OK);
    CHECK(e.longitude_e7()==200000231); // Persistent change replaces old history.
    CHECK(e.diagnostics().window_span_s<=90 && e.diagnostics().window_count<=451);
}

static void test_capacity_and_recovery() {
    RobustWindowEstimator e;
    for (uint64_t i=0;i<RobustWindowEstimator::CAPACITY;++i) CHECK(gps(e,1000+i*1000)==UPDATE_OK);
    CHECK(e.diagnostics().window_count==RobustWindowEstimator::CAPACITY);
    CHECK(gps(e,1025000)==UPDATE_OVERFLOW);
    CHECK(!e.valid(1025000) && !e.diagnostics().history_complete);
    CHECK(e.diagnostics().overflow_count==1);
    uint64_t t=1025000;
    while (t<181025000) {
        t+=5000000;
        const UpdateResult r=gps(e,t);
        CHECK(r==UPDATE_OVERFLOW || r==UPDATE_OK);
        CHECK(e.diagnostics().segment_count==1);
    }
    CHECK(e.valid(t) && e.diagnostics().history_complete);
    CHECK(e.diagnostics().overflow_count>1);
    e.reset(); CHECK(gps(e,t+1000)==UPDATE_OK);
    CHECK(e.diagnostics().overflow_count==0 && e.diagnostics().history_complete);
}

static void test_clock_wrap_and_long_session() {
    MillisClock clock;
    uint64_t t=0;
    CHECK(clock.extend(UINT32_C(0xfffffff0),t));
    const uint64_t before=t;
    CHECK(clock.extend(16,t) && t-before==32000);
    CHECK(clock.extend(16,t));
    CHECK(!clock.extend(15,t));
    CHECK(clock.extend(116,t) && t-before==132000);
    RobustWindowEstimator e;
    CHECK(gps(e,before)==UPDATE_OK);
    CHECK(gps(e,t)==UPDATE_OK && e.diagnostics().update_count==2);
    CHECK(e.valid(t));
    clock.reset(); CHECK(clock.extend(1,t) && t==1000);
    e.reset(); CHECK(!e.valid(t)); CHECK(gps(e,t)==UPDATE_OK);
    const uint64_t far=UINT64_C(1730000000000000); // No uint32-us truncation.
    e.reset(); CHECK(gps(e,far)==UPDATE_OK);
    CHECK(gps(e,far+200000)==UPDATE_OK && e.diagnostics().window_count==2);
}

int main() {
    test_initialization_and_input_contract();
    test_quality_formula();
    test_window_and_segments();
    test_turn_hold_and_causality();
    test_weighting_outliers_and_recovery();
    test_capacity_and_recovery();
    test_clock_wrap_and_long_session();
    CHECK(sizeof(RobustWindowEstimator)<65536);
    std::cout << "PASS core: " << assertions << " assertions; object_bytes="
              << sizeof(RobustWindowEstimator) << '\n';
    return 0;
}
