FROM ros:kinetic
MAINTAINER Ryota Sakamoto <saka_ro@yahoo.co.jp>

ENV PYTHONUNBUFFERED 1

ARG MSGS_NAME="fiware_ros_msgs"
ARG MSGS_GIT_REPO="https://github.com/ryo-ma/fiware_ros_msgs.git"
ARG MSGS_GIT_REV="master"

COPY ./kube_entrypoint.sh /opt/kube_entrypoint.sh
COPY . /opt/ros_ws/src/fiware_ros_bridge
WORKDIR /opt/ros_ws

RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh
RUN apt update && apt upgrade -y && \
    apt install python-pip -y && \
    git clone ${MSGS_GIT_REPO} src/${MSGS_NAME} && cd src/${MSGS_NAME} && git checkout ${MSGS_GIT_REV} && cd ../.. && \
    source /opt/ros/kinetic/setup.bash && \
    /opt/ros/kinetic/bin/catkin_make && \
    source /opt/ros_ws/devel/setup.bash && \
    pip install -r /opt/ros_ws/src/fiware_ros_bridge/requirements/common.txt && \
    /opt/ros/kinetic/bin/catkin_make && \
    echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros_ws/devel/setup.bash" >> /root/.bashrc
RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh
