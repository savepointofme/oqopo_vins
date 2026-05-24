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

// DiagPrinter — prints a compact multi-line [DIAG] block to stdout.
// No ANSI cursor-control codes.  Output is safe to redirect to a file,
// pipe through tmux, or read in ROS launch logs.
//
// Usage: call print(m) when you want output.  Rate-limiting (every N frames)
// is the caller's responsibility; DiagPrinter itself is stateless.

#ifndef OV_MSCKF_ROS_FREE_DIAG_PRINTER_H
#define OV_MSCKF_ROS_FREE_DIAG_PRINTER_H

#include "DiagMetrics.h"

namespace ov_msckf {

class DiagPrinter {
public:
  // Print one [DIAG] block to stdout.  Format:
  //
  //   [DIAG] t=934.20 frame=12345 stage=INIT_OK mode=FC+gpsAlt
  //          feat: klt=322 track=301 acc=26 msckf=38 slam=12
  //          scale: vio_v=38.70 gps_v=39.40 ratio=0.98 vio_path=1234.5 gps_path=1240.1
  //          backend: chi2_acc=24 chi2_rej=0 depth_med=198.0 depth_max=286.0
  //          state: ba=0.420 bg=0.0030 toff=+0.00370 dist=142.0
  //
  static void print(const DiagMetrics &m);
};

} // namespace ov_msckf

#endif // OV_MSCKF_ROS_FREE_DIAG_PRINTER_H
