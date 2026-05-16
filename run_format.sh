#!/bin/bash

# sudo apt install clang-format
find ov_core/ -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;
find ov_eval/ -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;
find ov_init/ -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;
find ov_msckf/ -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;

# sudo apt install libc++-dev clang-tidy
#find . -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -exec clang-tidy {} \;


./run_serial_msckf_ros_free   --stereo   --config /mnt/d/vscode_dir/open_vins/config/user_drone_stereo_jc82/estimator_config.yaml   --dataset /mnt/d/vscode_dir/open_vins/20260509_fly1/mav0   --gps /mnt/d/vscode_dir/open_vins/20260509_fly1/result/gps_tum_time_alignment/aligned_gps_cam_time.csv   --gps-time-offset 0   --align-seconds 100   --start-time 200   --output /mnt/d/vscode_dir/open_vins/20260509_fly1/result/pr19_stereo_tum.txt