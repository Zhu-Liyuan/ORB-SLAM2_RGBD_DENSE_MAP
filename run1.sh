
gnome-terminal -x bash -c  "roslaunch realsense2_camera rs_camera.launch align_depth:=true"

sleep 5

gnome-terminal -x bash -c  "cd /home/lenovo/PACKAGES/ORB-SLAM2_RGBD_DENSE_MAP; rosrun ORB_SLAM21 RGBD Vocabulary/ORBvoc.bin D435.yaml"


