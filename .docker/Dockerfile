FROM osrf/ros:humble-desktop

ARG USERNAME=humble_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME


RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-venv \
    ros-humble-xacro \
    gedit \
    git \
    patchelf\
    libosmesa6-dev \
    libgl1-mesa-glx \
    libgl1-mesa-dev \
    libglew-dev \
    python3.10-dev


#Installing MuJoCo 2.10
RUN mkdir -p /home/${USERNAME}/.mujoco && \
    curl -sSL https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz \
    | tar -xzf - -C /home/${USERNAME}/.mujoco

RUN  apt-get install -y libglfw3-dev

RUN cat <<'EOF' >> /home/${USERNAME}/.bashrc
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/humble_user/.mujoco/mujoco210/bin:/usr/lib/nvidia
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
EOF

RUN git clone https://github.com/openai/mujoco-py.git /home/${USERNAME}/.mujoco/mujoco-py

RUN pip3 install -y mujoco-py "cython<3" ipdb

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${USERNAME}/.bashrc


ENV TERM=xterm-256color
USER $USERNAME

# Optional, for customising bashrc and running a custom script after build
RUN --mount=type=bind,source=.,target=/tmp/context \
    if [ -f /tmp/context/.docker/scripts/custom.sh ]; then \
        cp /tmp/context/.docker/scripts/custom.sh /home/${USERNAME}/custom.sh && \
        chmod +x /home/${USERNAME}/custom.sh && \
        /home/${USERNAME}/custom.sh; \
    fi &&     if [ -f /tmp/context/.docker/scripts/bashrc_preferences.sh ]; then \
        cat /tmp/context/.docker/scripts/bashrc_preferences.sh >> /home/${USERNAME}/.bashrc; \
    fi

RUN mkdir -p /home/${USERNAME}/sim_ws/src
 

WORKDIR /home/${USERNAME}/sim_ws/
