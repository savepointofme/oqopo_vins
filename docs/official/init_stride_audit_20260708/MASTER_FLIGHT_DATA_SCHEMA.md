# Master Flight Data Schema

This file is generated from `analysis/master_flight_data.py::SCHEMA`; do not edit it by hand.
Parquet is the authoritative master-table output. CSV and XLSX are inspection copies.
GPS-derived evaluation fields use `reference_*` names and are not estimator truth or online inputs.

Field count: 268

| # | name | dtype | group | source |
| ---: | --- | --- | --- | --- |
| 1 | `run_id` | `string` | `index` | run directory or manifest |
| 2 | `flight_id` | `string` | `index` | metadata/inferred |
| 3 | `config_id` | `string` | `index` | metadata/inferred |
| 4 | `frame_id` | `int64` | `index` | diag.csv or row number |
| 5 | `timestamp_master` | `float64` | `time` | camera/VIO output time |
| 6 | `camera_timestamp_raw` | `float64` | `time` | diag.csv t |
| 7 | `vio_state_timestamp` | `float64` | `time` | traj_raw.txt t |
| 8 | `fc_timestamp_raw` | `float64` | `time` | FC source, optional |
| 9 | `fc_timestamp_aligned` | `float64` | `time` | FC source, optional |
| 10 | `fc_interp_dt_s` | `float64` | `time` | FC raw interpolation nearest endpoint delta |
| 11 | `fc_interpolated` | `bool` | `time` | FC raw interpolation status |
| 12 | `fc_out_of_range` | `bool` | `time` | FC raw interpolation validity |
| 13 | `gps_timestamp_raw` | `float64` | `time` | analysis/GPS source |
| 14 | `gps_timestamp_aligned` | `float64` | `time` | analysis/GPS source |
| 15 | `vio_sample_delay_ms` | `float64` | `time` | analysis data |
| 16 | `dataset_first_imu_timestamp` | `float64` | `time` | nav_frame_metadata.json startup contract |
| 17 | `requested_start_offset_s` | `float64` | `time` | nav_frame_metadata.json startup contract |
| 18 | `trim_boundary_timestamp` | `float64` | `time` | nav_frame_metadata.json startup contract |
| 19 | `seed_camera_timestamp` | `float64` | `time` | nav_frame_metadata.json startup contract |
| 20 | `first_emitted_timestamp` | `float64` | `time` | nav_frame_metadata.json startup contract |
| 21 | `selected_fc_init_timestamp` | `float64` | `time` | nav_frame_metadata.json selected_fc_timestamp |
| 22 | `fc_init_source_dt_s` | `float64` | `time` | nav_frame_metadata.json fc_time_offset |
| 23 | `seed_position_x` | `float64` | `startup` | nav_frame_metadata.json p_I_seed_in_W0_m |
| 24 | `seed_position_y` | `float64` | `startup` | nav_frame_metadata.json p_I_seed_in_W0_m |
| 25 | `seed_position_z` | `float64` | `startup` | nav_frame_metadata.json p_I_seed_in_W0_m |
| 26 | `first_emitted_position_x` | `float64` | `startup` | nav_frame_metadata.json p_I_first_emitted_in_W0_m |
| 27 | `first_emitted_position_y` | `float64` | `startup` | nav_frame_metadata.json p_I_first_emitted_in_W0_m |
| 28 | `first_emitted_position_z` | `float64` | `startup` | nav_frame_metadata.json p_I_first_emitted_in_W0_m |
| 29 | `first_emitted_velocity_x` | `float64` | `startup` | nav_frame_metadata.json v_I_first_emitted_in_W0_mps |
| 30 | `first_emitted_velocity_y` | `float64` | `startup` | nav_frame_metadata.json v_I_first_emitted_in_W0_mps |
| 31 | `first_emitted_velocity_z` | `float64` | `startup` | nav_frame_metadata.json v_I_first_emitted_in_W0_mps |
| 32 | `first_emitted_q_x` | `float64` | `startup` | nav_frame_metadata.json q_ItoW0_first_emitted_xyzw |
| 33 | `first_emitted_q_y` | `float64` | `startup` | nav_frame_metadata.json q_ItoW0_first_emitted_xyzw |
| 34 | `first_emitted_q_z` | `float64` | `startup` | nav_frame_metadata.json q_ItoW0_first_emitted_xyzw |
| 35 | `first_emitted_q_w` | `float64` | `startup` | nav_frame_metadata.json q_ItoW0_first_emitted_xyzw |
| 36 | `fc_init_selection_method` | `string` | `startup` | nav_frame_metadata.json |
| 37 | `nav_metadata_schema_version` | `string` | `provenance` | nav_frame_metadata.json schema_version |
| 38 | `segment_id` | `string` | `index` | analysis segment index |
| 39 | `init_status` | `string` | `index` | diag.csv initialized |
| 40 | `stride_input` | `int64` | `stride` | run metadata or command |
| 41 | `stride_visual_update` | `int64` | `stride` | camera stride audit or command |
| 42 | `stride_analysis_sample` | `int64` | `stride` | analysis metadata |
| 43 | `overlap_theory_along` | `float64` | `stride` | future footprint model |
| 44 | `overlap_polygon_area` | `float64` | `stride` | future footprint model |
| 45 | `overlap_valid` | `bool` | `stride` | future footprint model |
| 46 | `fc_q_x` | `float64` | `fc` | FC raw converted to OpenVINS q_GtoI JPL, SLERP |
| 47 | `fc_q_y` | `float64` | `fc` | FC raw converted to OpenVINS q_GtoI JPL, SLERP |
| 48 | `fc_q_z` | `float64` | `fc` | FC raw converted to OpenVINS q_GtoI JPL, SLERP |
| 49 | `fc_q_w` | `float64` | `fc` | FC raw converted to OpenVINS q_GtoI JPL, SLERP |
| 50 | `fc_roll_raw` | `float64` | `fc` | FC source, optional |
| 51 | `fc_pitch_raw` | `float64` | `fc` | FC source, optional |
| 52 | `fc_yaw_raw` | `float64` | `fc` | FC source, optional |
| 53 | `fc_roll_calibrated` | `float64` | `fc` | future FC calibration |
| 54 | `fc_pitch_calibrated` | `float64` | `fc` | future FC calibration |
| 55 | `fc_yaw_calibrated` | `float64` | `fc` | future FC calibration |
| 56 | `fc_velocity_e` | `float64` | `fc` | FC source, optional |
| 57 | `fc_velocity_n` | `float64` | `fc` | FC source, optional |
| 58 | `fc_velocity_u` | `float64` | `fc` | FC source, optional |
| 59 | `fc_position_e` | `float64` | `fc` | FC raw WGS84 converted to local ENU |
| 60 | `fc_position_n` | `float64` | `fc` | FC raw WGS84 converted to local ENU |
| 61 | `fc_position_u` | `float64` | `fc` | FC raw WGS84 converted to local ENU |
| 62 | `fc_source_path` | `string` | `fc` | FC raw CSV path |
| 63 | `vio_raw_q_x` | `float64` | `vio_raw` | traj_raw.txt |
| 64 | `vio_raw_q_y` | `float64` | `vio_raw` | traj_raw.txt |
| 65 | `vio_raw_q_z` | `float64` | `vio_raw` | traj_raw.txt |
| 66 | `vio_raw_q_w` | `float64` | `vio_raw` | traj_raw.txt |
| 67 | `vio_raw_position_x` | `float64` | `vio_raw` | traj_raw.txt |
| 68 | `vio_raw_position_y` | `float64` | `vio_raw` | traj_raw.txt |
| 69 | `vio_raw_position_z` | `float64` | `vio_raw` | traj_raw.txt |
| 70 | `vio_raw_velocity_x` | `float64` | `vio_raw` | traj.txt.bias |
| 71 | `vio_raw_velocity_y` | `float64` | `vio_raw` | traj.txt.bias |
| 72 | `vio_raw_velocity_z` | `float64` | `vio_raw` | traj.txt.bias |
| 73 | `vio_gyro_bias_x` | `float64` | `vio_raw` | traj.txt.bias or diag.csv |
| 74 | `vio_gyro_bias_y` | `float64` | `vio_raw` | traj.txt.bias or diag.csv |
| 75 | `vio_gyro_bias_z` | `float64` | `vio_raw` | traj.txt.bias or diag.csv |
| 76 | `vio_accel_bias_x` | `float64` | `vio_raw` | traj.txt.bias or diag.csv |
| 77 | `vio_accel_bias_y` | `float64` | `vio_raw` | traj.txt.bias or diag.csv |
| 78 | `vio_accel_bias_z` | `float64` | `vio_raw` | traj.txt.bias or diag.csv |
| 79 | `vio_nav_q_x` | `float64` | `vio_nav` | traj_nav.txt |
| 80 | `vio_nav_q_y` | `float64` | `vio_nav` | traj_nav.txt |
| 81 | `vio_nav_q_z` | `float64` | `vio_nav` | traj_nav.txt |
| 82 | `vio_nav_q_w` | `float64` | `vio_nav` | traj_nav.txt |
| 83 | `vio_nav_position_e` | `float64` | `vio_nav` | traj_nav.txt |
| 84 | `vio_nav_position_n` | `float64` | `vio_nav` | traj_nav.txt |
| 85 | `vio_nav_position_u` | `float64` | `vio_nav` | traj_nav.txt |
| 86 | `vio_nav_velocity_e` | `float64` | `vio_nav` | traj_nav.txt |
| 87 | `vio_nav_velocity_n` | `float64` | `vio_nav` | traj_nav.txt |
| 88 | `vio_nav_velocity_u` | `float64` | `vio_nav` | traj_nav.txt |
| 89 | `nav_transform_source_path` | `string` | `vio_nav` | nav_frame_metadata.json |
| 90 | `nav_transform_version` | `string` | `vio_nav` | nav_frame_metadata.json |
| 91 | `nav_transform_future_data_used` | `bool` | `vio_nav` | nav_frame_metadata.json |
| 92 | `nav_transform_valid` | `bool` | `vio_nav` | frame_contract_validation.json pass/hash gate |
| 93 | `vio_dashboard_aligned_q_x` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 94 | `vio_dashboard_aligned_q_y` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 95 | `vio_dashboard_aligned_q_z` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 96 | `vio_dashboard_aligned_q_w` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 97 | `vio_dashboard_aligned_position_e` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 98 | `vio_dashboard_aligned_position_n` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 99 | `vio_dashboard_aligned_position_u` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 100 | `vio_dashboard_aligned_velocity_e` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 101 | `vio_dashboard_aligned_velocity_n` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 102 | `vio_dashboard_aligned_velocity_u` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 103 | `dashboard_alignment_id` | `string` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 104 | `dashboard_alignment_method` | `string` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 105 | `dashboard_alignment_rotation_yaw_deg` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 106 | `dashboard_alignment_translation_e` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 107 | `dashboard_alignment_translation_n` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 108 | `dashboard_alignment_translation_u` | `float64` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 109 | `dashboard_alignment_source_path` | `string` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 110 | `dashboard_alignment_evaluation_only` | `bool` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 111 | `dashboard_alignment_uses_future_data` | `bool` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 112 | `dashboard_alignment_applied` | `bool` | `vio_dashboard_aligned` | dashboard alignment metadata |
| 113 | `vio_eval_aligned_position_e` | `float64` | `vio_eval_aligned` | analysis/data/gps_time_aligned_samples.csv |
| 114 | `vio_eval_aligned_position_n` | `float64` | `vio_eval_aligned` | analysis/data/gps_time_aligned_samples.csv |
| 115 | `vio_eval_aligned_position_u` | `float64` | `vio_eval_aligned` | analysis/data/gps_time_aligned_samples.csv |
| 116 | `vio_eval_aligned_velocity_e` | `float64` | `vio_eval_aligned` | analysis/data/gps_time_aligned_samples.csv |
| 117 | `vio_eval_aligned_velocity_n` | `float64` | `vio_eval_aligned` | analysis/data/gps_time_aligned_samples.csv |
| 118 | `vio_eval_aligned_velocity_u` | `float64` | `vio_eval_aligned` | analysis/data/gps_time_aligned_samples.csv |
| 119 | `alignment_id` | `string` | `alignment` | analysis metadata |
| 120 | `alignment_mode` | `string` | `alignment` | analysis metadata |
| 121 | `alignment_rotation_yaw_deg` | `float64` | `alignment` | analysis metadata |
| 122 | `alignment_translation_e` | `float64` | `alignment` | analysis metadata |
| 123 | `alignment_translation_n` | `float64` | `alignment` | analysis metadata |
| 124 | `alignment_translation_u` | `float64` | `alignment` | analysis metadata |
| 125 | `alignment_applied` | `bool` | `alignment` | analysis data present |
| 126 | `reference_position_e` | `float64` | `reference` | GPS/reference from analysis data (evaluation-only) |
| 127 | `reference_position_n` | `float64` | `reference` | GPS/reference from analysis data (evaluation-only) |
| 128 | `reference_position_u` | `float64` | `reference` | GPS/reference from analysis data (evaluation-only) |
| 129 | `reference_velocity_e` | `float64` | `reference` | GPS/reference from analysis data (evaluation-only) |
| 130 | `reference_velocity_n` | `float64` | `reference` | GPS/reference from analysis data (evaluation-only) |
| 131 | `reference_velocity_u` | `float64` | `reference` | GPS/reference from analysis data (evaluation-only) |
| 132 | `reference_course_deg` | `float64` | `reference` | GPS/reference from analysis data (evaluation-only) |
| 133 | `gps_quality` | `string` | `reference` | GPS/reference source, optional |
| 134 | `gps_velocity_source` | `string` | `reference` | GPS/reference velocity provenance from analysis data |
| 135 | `err_e` | `float64` | `error` | analysis data |
| 136 | `err_n` | `float64` | `error` | analysis data |
| 137 | `err_u` | `float64` | `error` | analysis data |
| 138 | `err_xy` | `float64` | `error` | analysis data |
| 139 | `err_3d` | `float64` | `error` | analysis data |
| 140 | `err_along` | `float64` | `error` | analysis data |
| 141 | `err_cross` | `float64` | `error` | analysis data |
| 142 | `err_vertical` | `float64` | `error` | analysis data |
| 143 | `err_v_e` | `float64` | `error` | analysis data |
| 144 | `err_v_n` | `float64` | `error` | analysis data |
| 145 | `err_v_u` | `float64` | `error` | analysis data |
| 146 | `err_v_along` | `float64` | `error` | analysis data |
| 147 | `err_v_cross` | `float64` | `error` | analysis data |
| 148 | `err_v_vertical` | `float64` | `error` | analysis data |
| 149 | `course_error_deg` | `float64` | `error` | analysis data |
| 150 | `feature_klt_raw` | `int64` | `diagnostic` | diag.csv |
| 151 | `feature_tracked` | `int64` | `diagnostic` | diag.csv |
| 152 | `feature_msckf_accepted` | `int64` | `diagnostic` | diag.csv |
| 153 | `feature_slam_count` | `int64` | `diagnostic` | diag.csv |
| 154 | `mean_track_length` | `float64` | `diagnostic` | future tracker summary |
| 155 | `visual_update_executed` | `bool` | `diagnostic` | future per-frame update log |
| 156 | `data_missing_flags` | `string` | `diagnostic` | builder |
| 157 | `frame_contract_version` | `string` | `provenance` | run/nav metadata |
| 158 | `frame_validation_path` | `string` | `provenance` | frame contract validation report |
| 159 | `frame_validation_sha256` | `string` | `provenance` | frame contract validation report bytes |
| 160 | `frame_validation_schema` | `string` | `provenance` | frame contract validation report schema_version |
| 161 | `frame_validation_status` | `string` | `provenance` | frame contract validation gate |
| 162 | `provenance_path` | `string` | `provenance` | run provenance |
| 163 | `provenance_sha256` | `string` | `provenance` | run provenance file bytes |
| 164 | `provenance_status` | `string` | `provenance` | run provenance capture_status |
| 165 | `provenance_schema` | `string` | `provenance` | run provenance schema |
| 166 | `repo_commit` | `string` | `provenance` | run provenance repository.head |
| 167 | `repo_branch` | `string` | `provenance` | run provenance repository.branch |
| 168 | `dirty_worktree` | `bool` | `provenance` | run provenance tracked/untracked state |
| 169 | `dirty_patch_hash` | `string` | `provenance` | run provenance dirty.patch SHA256 |
| 170 | `untracked_manifest_hash` | `string` | `provenance` | run provenance untracked manifest SHA256 |
| 171 | `runner_path` | `string` | `provenance` | run metadata/provenance |
| 172 | `runner_hash` | `string` | `provenance` | run provenance runner SHA256 |
| 173 | `config_path` | `string` | `provenance` | run metadata |
| 174 | `config_hash` | `string` | `provenance` | first run provenance config SHA256 |
| 175 | `config_hashes_json` | `string` | `provenance` | all run provenance config records |
| 176 | `dataset_root_hash` | `string` | `provenance` | run provenance dataset root SHA256 |
| 177 | `gps_reference_hash` | `string` | `provenance` | run provenance GPS/reference SHA256 |
| 178 | `fc_init_hash` | `string` | `provenance` | run provenance FC-init SHA256 |
| 179 | `calibration_id` | `string` | `provenance` | config snapshot |
| 180 | `command_path` | `string` | `provenance` | run directory |
| 181 | `analysis_source` | `string` | `provenance` | analysis directory |
| 182 | `record_type` | `string` | `index` | master builder row contract |
| 183 | `evaluation_contract` | `string` | `evaluation_contract` | absolute_navigation, relative_drift, legacy_diagnostic, or not_applicable |
| 184 | `evaluation_alignment_mode` | `string` | `evaluation_contract` | evaluation artifact metadata |
| 185 | `absolute_navigation_available` | `bool` | `evaluation_contract` | builder contract gate |
| 186 | `relative_drift_available` | `bool` | `evaluation_contract` | builder contract gate |
| 187 | `absolute_position_error_e_m` | `float64` | `absolute_navigation` | unaligned canonical VIO minus FC/GPS navigation position |
| 188 | `absolute_position_error_n_m` | `float64` | `absolute_navigation` | unaligned canonical VIO minus FC/GPS navigation position |
| 189 | `absolute_position_error_u_m` | `float64` | `absolute_navigation` | unaligned canonical VIO minus FC/GPS navigation position |
| 190 | `absolute_position_error_xy_m` | `float64` | `absolute_navigation` | unaligned canonical VIO minus FC/GPS navigation position |
| 191 | `absolute_position_error_3d_m` | `float64` | `absolute_navigation` | unaligned canonical VIO minus FC/GPS navigation position |
| 192 | `absolute_velocity_error_e_mps` | `float64` | `absolute_navigation` | unaligned canonical VIO minus FC velocity |
| 193 | `absolute_velocity_error_n_mps` | `float64` | `absolute_navigation` | unaligned canonical VIO minus FC velocity |
| 194 | `absolute_velocity_error_u_mps` | `float64` | `absolute_navigation` | unaligned canonical VIO minus FC velocity |
| 195 | `absolute_attitude_error_roll_deg` | `float64` | `absolute_navigation` | unaligned canonical VIO attitude minus FC attitude |
| 196 | `absolute_attitude_error_pitch_deg` | `float64` | `absolute_navigation` | unaligned canonical VIO attitude minus FC attitude |
| 197 | `absolute_attitude_error_yaw_deg` | `float64` | `absolute_navigation` | unaligned canonical VIO attitude minus FC attitude |
| 198 | `relative_drift_error_e_m` | `float64` | `relative_drift` | once-origin-relative evaluation |
| 199 | `relative_drift_error_n_m` | `float64` | `relative_drift` | once-origin-relative evaluation |
| 200 | `relative_drift_error_u_m` | `float64` | `relative_drift` | once-origin-relative evaluation |
| 201 | `relative_drift_error_xy_m` | `float64` | `relative_drift` | once-origin-relative evaluation |
| 202 | `relative_drift_error_3d_m` | `float64` | `relative_drift` | once-origin-relative evaluation |
| 203 | `relative_drift_velocity_error_e_mps` | `float64` | `relative_drift` | once-origin-relative evaluation |
| 204 | `relative_drift_velocity_error_n_mps` | `float64` | `relative_drift` | once-origin-relative evaluation |
| 205 | `relative_drift_velocity_error_u_mps` | `float64` | `relative_drift` | once-origin-relative evaluation |
| 206 | `relative_drift_course_error_deg` | `float64` | `relative_drift` | once-origin-relative evaluation |
| 207 | `calibration_source_id` | `string` | `calibration` | global baseline lineage or run snapshot |
| 208 | `calibration_source_path` | `string` | `calibration` | global baseline lineage or run snapshot |
| 209 | `calibration_version` | `string` | `calibration` | calibration manifest/hash |
| 210 | `calibration_lineage_status` | `string` | `calibration` | global baseline lineage audit |
| 211 | `camera_imu_extrinsic_contract` | `string` | `calibration` | fixed T_C_I audit contract |
| 212 | `camera_imu_rotation_matrix_json` | `string` | `calibration` | fixed T_C_I lineage artifact |
| 213 | `camera_imu_translation_m_json` | `string` | `calibration` | fixed T_C_I lineage artifact |
| 214 | `online_calibration_intrinsics_enabled` | `bool` | `online_calibration` | run config/log |
| 215 | `online_calibration_extrinsics_enabled` | `bool` | `online_calibration` | run config/log |
| 216 | `online_calibration_timeoffset_enabled` | `bool` | `online_calibration` | run config/log |
| 217 | `online_parameter_covariance_status` | `string` | `online_calibration` | online calibration trace audit |
| 218 | `calib_camera_time_offset_s` | `float64` | `online_calibration` | state trace or locked config |
| 219 | `calib_fx` | `float64` | `online_calibration` | state trace or locked config |
| 220 | `calib_fy` | `float64` | `online_calibration` | state trace or locked config |
| 221 | `calib_cx` | `float64` | `online_calibration` | state trace or locked config |
| 222 | `calib_cy` | `float64` | `online_calibration` | state trace or locked config |
| 223 | `calib_k1` | `float64` | `online_calibration` | state trace or locked config |
| 224 | `calib_k2` | `float64` | `online_calibration` | state trace or locked config |
| 225 | `calib_p1` | `float64` | `online_calibration` | state trace or locked config |
| 226 | `calib_p2` | `float64` | `online_calibration` | state trace or locked config |
| 227 | `calib_extr_qx` | `float64` | `online_calibration` | state trace or locked config |
| 228 | `calib_extr_qy` | `float64` | `online_calibration` | state trace or locked config |
| 229 | `calib_extr_qz` | `float64` | `online_calibration` | state trace or locked config |
| 230 | `calib_extr_qw` | `float64` | `online_calibration` | state trace or locked config |
| 231 | `calib_extr_tx_m` | `float64` | `online_calibration` | state trace or locked config |
| 232 | `calib_extr_ty_m` | `float64` | `online_calibration` | state trace or locked config |
| 233 | `calib_extr_tz_m` | `float64` | `online_calibration` | state trace or locked config |
| 234 | `online_parameter_std_json` | `string` | `online_calibration` | state covariance trace; explicit null/status when absent |
| 235 | `online_parameter_increment_json` | `string` | `online_calibration` | per-state increments from online calibration trace |
| 236 | `fc_angular_rate_x_rad_s` | `float64` | `fc_board` | raw FC rate or attitude-derived rate |
| 237 | `fc_angular_rate_y_rad_s` | `float64` | `fc_board` | raw FC rate or attitude-derived rate |
| 238 | `fc_angular_rate_z_rad_s` | `float64` | `fc_board` | raw FC rate or attitude-derived rate |
| 239 | `fc_angular_rate_source` | `string` | `fc_board` | source inventory |
| 240 | `fc_latitude_deg` | `float64` | `fc` | raw FC WGS84 latitude |
| 241 | `fc_longitude_deg` | `float64` | `fc` | raw FC WGS84 longitude |
| 242 | `fc_altitude_m` | `float64` | `fc` | raw FC altitude |
| 243 | `board_imu_angular_rate_x_rad_s` | `float64` | `fc_board` | raw imu0 gyro averaged over FC interval |
| 244 | `board_imu_angular_rate_y_rad_s` | `float64` | `fc_board` | raw imu0 gyro averaged over FC interval |
| 245 | `board_imu_angular_rate_z_rad_s` | `float64` | `fc_board` | raw imu0 gyro averaged over FC interval |
| 246 | `fc_board_nominal_rotation_id` | `string` | `fc_board` | NOMINAL_FC_BOARD_ROTATION_CANDIDATE.yaml |
| 247 | `fc_board_nominal_rotation_matrix_json` | `string` | `fc_board` | NOMINAL_FC_BOARD_ROTATION_CANDIDATE.yaml |
| 248 | `fc_board_rotation_status` | `string` | `fc_board` | NOMINAL_FC_BOARD_ROTATION_CANDIDATE.yaml |
| 249 | `fc_board_residual_omega_x_rad_s` | `float64` | `fc_board` | raw FC/board mount diagnostic |
| 250 | `fc_board_residual_omega_y_rad_s` | `float64` | `fc_board` | raw FC/board mount diagnostic |
| 251 | `fc_board_residual_omega_z_rad_s` | `float64` | `fc_board` | raw FC/board mount diagnostic |
| 252 | `fc_board_relative_rotation_residual_deg` | `float64` | `fc_board` | 1 s integrated relative-motion residual |
| 253 | `fc_board_event_delta_angle_deg` | `float64` | `fc_board` | turn-local integrated relative-motion residual |
| 254 | `flex_event_id` | `string` | `fc_board` | turn mount audit |
| 255 | `flex_event_start_s` | `float64` | `fc_board` | turn mount audit event boundary |
| 256 | `flex_event_end_s` | `float64` | `fc_board` | turn mount audit event boundary |
| 257 | `flex_event_duration_s` | `float64` | `fc_board` | turn mount audit event boundary |
| 258 | `flex_event_candidate` | `bool` | `fc_board` | turn mount audit gated candidate |
| 259 | `flex_event_diagnostic_class` | `string` | `fc_board` | turn mount audit |
| 260 | `flex_event_returns_to_nominal` | `bool` | `fc_board` | turn mount audit recovery gate |
| 261 | `flex_event_recovery_time_s` | `float64` | `fc_board` | turn mount audit recovery gate |
| 262 | `turn_direction` | `string` | `flight_geometry` | FC/analysis geometry |
| 263 | `bank_angle_deg` | `float64` | `flight_geometry` | raw FC roll |
| 264 | `initialization_mode` | `string` | `startup` | run command/init artifact |
| 265 | `initialization_artifact_path` | `string` | `startup` | run command/init artifact |
| 266 | `initialization_lineage_status` | `string` | `startup` | global baseline reconstruction audit |
| 267 | `result_lineage_status` | `string` | `provenance` | global baseline lineage audit |
| 268 | `contamination_flags` | `string` | `provenance` | BASELINE_CONTAMINATION_MATRIX.csv |

## Regeneration and drift check

```powershell
python analysis/master_flight_data.py schema --out-dir docs/official/init_stride_audit_20260708
python analysis/master_flight_data.py schema --check --out-dir docs/official/init_stride_audit_20260708
```
