/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "DiagPrinter.h"

#include <cstdio>

namespace ov_msckf {

void DiagPrinter::print(const DiagMetrics &m) {
  // Line 1 — identification
  const char *stage = m.initialized ? (m.aligned ? "INIT_OK+aligned" : "INIT_OK") : "waiting_init";
  printf("[DIAG] t=%.2f frame=%d stage=%s mode=%s\n",
         m.t, m.frame_id, stage,
         m.mode_str.empty() ? "-" : m.mode_str.c_str());

  // Line 2 — feature tracking
  printf("       feat: klt=%d track=%d acc=%d msckf=%d slam=%d\n",
         m.klt_raw, m.tracked, m.n_acc, m.msckf_in, m.slam_count);

  // Line 3 — speed / scale
  if (m.gps_speed >= 0) {
    double ratio = (m.gps_speed > 0.01) ? m.vio_speed / m.gps_speed : -1.0;
    if (ratio >= 0)
      printf("       scale: vio_v=%.2f gps_v=%.2f ratio=%.2f vio_path=%.1f gps_path=%.1f\n",
             m.vio_speed, m.gps_speed, ratio, m.vio_path_len, m.gps_path_len);
    else
      printf("       scale: vio_v=%.2f gps_v=%.2f ratio=n/a vio_path=%.1f gps_path=%.1f\n",
             m.vio_speed, m.gps_speed, m.vio_path_len, m.gps_path_len);
  } else {
    printf("       scale: vio_v=%.2f gps_v=n/a ratio=n/a vio_path=%.1f gps_path=n/a\n",
           m.vio_speed, m.vio_path_len);
  }

  // Line 4 — backend
  if (m.depth_med >= 0)
    printf("       backend: chi2_acc=%d chi2_rej=%d depth_med=%.1f depth_max=%.1f\n",
           m.chi2_acc, m.chi2_rej, m.depth_med, m.depth_max);
  else
    printf("       backend: chi2_acc=%d chi2_rej=%d depth_med=n/a depth_max=n/a\n",
           m.chi2_acc, m.chi2_rej);

  // Line 5 — state
  printf("       state: ba=%.3f bg=%.4f toff=%+.5f dist=%.1f\n",
         m.ba_norm, m.bg_norm, m.cam_toff, m.vio_dist);

  fflush(stdout);
}

} // namespace ov_msckf
