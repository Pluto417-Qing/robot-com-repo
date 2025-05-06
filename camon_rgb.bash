# camon_rgb.bash
ros2 launch camera_test stereo_camera.py >/dev/null 2>&1 &
sleep 5
ros2 lifecycle set /stereo_camera configure
ros2 lifecycle set /stereo_camera activate