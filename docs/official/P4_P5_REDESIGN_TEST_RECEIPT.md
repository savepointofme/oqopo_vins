# P4/P5 redesign test receipt

Date: 2026-07-14  
Build tree: `D:/vscode_dir/open_vins/build_ov_msckf`

Command:

```bash
cmake --build build_ov_msckf \
  --target test_online_alignment_initializer test_adaptive_stride \
           run_serial_msckf_ros_free -j2
build_ov_msckf/test_online_alignment_initializer
build_ov_msckf/test_adaptive_stride
```

Result:

```text
Built target test_online_alignment_initializer
Built target test_adaptive_stride
Built target run_serial_msckf_ros_free
online alignment initializer tests passed
P4/P5 visual scheduling tests 1-21 passed
```

Raw logs:

- `C:/Users/baloney/Desktop/实验目录/P4_P5_redesign_20260713/delivery/tests/build.log`;
- `C:/Users/baloney/Desktop/实验目录/P4_P5_redesign_20260713/delivery/tests/test_online_alignment_initializer.log`;
- `C:/Users/baloney/Desktop/实验目录/P4_P5_redesign_20260713/delivery/tests/test_adaptive_stride.log`.

The P4 test includes persistent collection beyond 60 s with bounded buffers, stale visual-tail pruning, duplicate-fingerprint suppression, candidate validation/retry, one release, and zero post-release solve behavior. The P5 test includes rotation-compensated motion, target-parallax hysteresis, polygon footprint overlap, information/latency backend triggers, alignment-frame selection, and exact-time tracking-only cleanup.
