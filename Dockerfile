FROM ros:noetic-ros-base-focal AS builder

COPY py_talker /catkin_ws/src/py_talker
WORKDIR /catkin_ws
RUN /ros_entrypoint.sh catkin_make install -DCMAKE_INSTALL_PREFIX=/tmp/ros/noetic

# Final Stage
FROM ros:noetic-ros-base-focal

COPY --from=builder /tmp/ros/noetic /opt/ros/noetic

ENTRYPOINT /ros_entrypoint.sh rosrun py_talker runme.py