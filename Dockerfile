FROM ros:humble
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=humble

ENV SOURCE_DIR=/app/driving
ENV ROBOT_TYPE=duckiebot
ENV ROS2_SOURCE=/opt/ros/${ROS_DISTRO}/setup.sh
ENV ROBOT_HARDWARE=jetson_nano
ENV ROBOT_CONFIGURATION=DB21J
ENV VEHICLE_NAME=example_bot
ENV PROJECT_NAME=example_project
ENV DEFAULT_LAUNCH=/app/driving/launchers/default.sh
ENV ROS_DOMAIN_ID=0
ENV PARALLEL_WORKERS=4

RUN apt-get update && apt update

RUN apt install python3-pip -y

RUN rosdep update --rosdistro $ROS_DISTRO

RUN apt update && apt install python3-opencv -y
RUN pip3 install numpy

COPY ./packages /app/driving/packages
COPY ./launchers /app/driving/launchers

WORKDIR /app

RUN rosdep install -q -y -r --from-path . --rosdistro $ROS_DISTRO

RUN apt install nano -y

RUN source ${ROS2_SOURCE} && colcon build --parallel-workers ${PARALLEL_WORKERS}

COPY ./scripts /app/
ENTRYPOINT ["./entrypoint.sh"]