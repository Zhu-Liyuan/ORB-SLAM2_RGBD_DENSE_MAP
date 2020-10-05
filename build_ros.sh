echo "Building ROS nodes"
sudo rm -rf Examples/ROS/ORB_SLAM21/build
cd Examples/ROS/ORB_SLAM21
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j4
