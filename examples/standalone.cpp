// SPDX-License-Identifier: GPL-3.0-or-later
// Synthetic coordinates near 10 degrees N, 20 degrees E; no device access.
#include "../core/RobustWindowEstimator.h"
#include <iomanip>
#include <iostream>

static staticpos_v3::RobustWindowEstimator estimator;

int main() {
    for (unsigned i=0; i<20; ++i) {
        const uint64_t time_us=UINT64_C(1000000)+static_cast<uint64_t>(i)*200000U;
        estimator.update_imu(time_us-1000U,0.0,0.0,i==10?0.2:0.0,true);
        const int32_t latitude=100000000+static_cast<int32_t>(i%5U);
        const int32_t longitude=200000000+static_cast<int32_t>(i%3U);
        const auto result=estimator.update_gps(time_us,latitude,longitude,3,1.0,1.0,12);
        if (result!=staticpos_v3::UPDATE_OK || !estimator.valid(time_us)) {
            std::cerr<<"Synthetic example failed to produce a valid result\n";
            return 1;
        }
    }
    const auto &d=estimator.diagnostics();
    std::cout<<std::fixed<<std::setprecision(7)
        <<"Synthetic latitude: "<<estimator.latitude_deg()<<"\n"
        <<"Synthetic longitude: "<<estimator.longitude_deg()<<"\n"
        <<"Window points: "<<d.window_count<<"\n"
        <<"Turn multiplier: "<<d.lambda<<"\n";
    return 0;
}
