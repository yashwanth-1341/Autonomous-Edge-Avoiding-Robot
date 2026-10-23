FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y \
build-essential git \
python3-colcon-common-extensions python3-rosdep \
ros-dev-tools \
ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py \
# Simulation / control / visualization dependencies of the workspace packages
ros-jazzy-xacro ros-jazzy-robot-state-publisher \
ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
ros-jazzy-rviz2 ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-controller-manager \
ros-jazzy-gz-ros2-control \
# Browser-viewable virtual desktop (noVNC on port 6080) for Gazebo and RViz
tigervnc-standalone-server novnc websockify fluxbox xterm \
&& rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
CMD ["bash"]
