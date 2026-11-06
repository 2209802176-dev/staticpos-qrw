/* SPDX-License-Identifier: GPL-3.0-or-later
 * Tiny stdin replay protocol; no RTK or derived labels are accepted.
 * R
 * I time_us gx_rad_s gy_rad_s gz_rad_s healthy
 * G time_us lat_e7 lon_e7 fix h_acc_m hdop nsat
 */
#include "RobustWindowEstimator.h"
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>

int main() {
    staticpos_v3::RobustWindowEstimator estimator;
    std::cout << std::setprecision(17);
    std::string type;
    while (std::cin>>type) {
        if (type=="R") { estimator.reset(); continue; }
        uint64_t time_us;
        if (type=="I") {
            double x,y,z; int healthy;
            if (!(std::cin>>time_us>>x>>y>>z>>healthy)) return 2;
            estimator.update_imu(time_us,x,y,z,healthy!=0);
        } else if (type=="G") {
            int32_t lat,lon; unsigned fix; double hacc,hdop,sats;
            if (!(std::cin>>time_us>>lat>>lon>>fix>>hacc>>hdop>>sats)) return 2;
            const staticpos_v3::UpdateResult result=estimator.update_gps(
                time_us,lat,lon,static_cast<uint8_t>(fix),hacc,hdop,sats);
            const staticpos_v3::Diagnostics &d=estimator.diagnostics();
            std::cout << time_us << ',' << static_cast<unsigned>(result) << ','
                << estimator.east_m() << ',' << estimator.north_m() << ','
                << estimator.latitude_deg() << ',' << estimator.longitude_deg() << ','
                << d.sigma_m << ',' << d.lambda << ',' << d.robust_factor << ','
                << d.newest_weight << ',' << d.effective_n << ','
                << d.window_count << ',' << d.window_span_s << ','
                << d.update_count << ',' << d.segment_count << ','
                << d.overflow_count << ',' << static_cast<unsigned>(d.iterations)
                << '\n';
        } else return 3;
    }
    return std::cin.eof()?0:4;
}
