PKG_PATH=$(ros2 pkg prefix mbf_bringup)/share/mbf_bringup
CONFIG_FILE="$PKG_PATH/config/ctrl_param.yaml"

ros2 run wbr_control wbr_control \
    --ros-args \
    --params-file "$CONFIG_FILE" \
    -r imu:=imu/data