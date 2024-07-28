echo "Building ROS nodes"

cd Examples_old/ROS/ORB_SLAM3
. /opt/ros/melodic/setup.sh
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/root/catkin_ws/src/ORB_SLAM3_fixed/Examples_old/ROS/ORB_SLAM3
# rosdep init
# rosdep update
# sed -i 's/++11/++14/g' CMakeLists.txt

mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j8
