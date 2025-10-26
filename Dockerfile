FROM ros:jazzy
ARG USERNAME=cvdoc
ARG USER_UID=1000
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash
ENV ROS_DISTRO=jazzy

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

RUN rm -f /etc/apt/sources.list.d/ubuntu.sources && \
    { \
    echo 'Types: deb'; \
    echo 'URIs: https://mirrors.tuna.tsinghua.edu.cn/ubuntu'; \
    echo 'Suites: noble noble-updates noble-backports'; \
    echo 'Components: main restricted universe multiverse'; \
    echo 'Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg'; \
    echo ''; \
    echo 'Types: deb'; \
    echo 'URIs: https://mirrors.tuna.tsinghua.edu.cn/ubuntu'; \
    echo 'Suites: noble-security'; \
    echo 'Components: main restricted universe multiverse'; \
    echo 'Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg'; \
    } > /etc/apt/sources.list.d/ubuntu.sources
RUN curl -sSL https://ghproxy.cn/https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    ros-jazzy-camera-info-manager \
    ros-jazzy-image-transport \
    ros-jazzy-serial-driver \
    ros-jazzy-cv-bridge \
    ros-jazzy-asio-cmake-module \
    ros-jazzy-angles \
    ros-jazzy-nav2-common \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-rviz2 \
    screen \
    tini \
    iproute2 \
    net-tools
RUN apt-get install -y python3-pip wget vim htop
RUN echo 'export PATH=$PATH:/home/ws/.script' >> /home/$USERNAME/.bashrc
RUN echo 'alias wsi="source /opt/ros/jazzy/setup.bash"' >> /home/$USERNAME/.bashrc
RUN echo 'alias ini="source install/setup.bash"' >> /home/$USERNAME/.bashrc

USER $USERNAME

RUN sudo mkdir /home/ws && sudo chown $USERNAME:$USERNAME /home/ws
RUN sudo curl -o /etc/ros/rosdep/sources.list.d/20-default.list -L https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
RUN export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml && rosdep update || true

RUN --mount=type=bind,target=/home/ws,source=.,readonly=false cd /home/ws \
    && sudo cp /home/ws/.script/atrm-service /etc/init.d/atrm || true \ 
    && sudo cp /home/ws/.script/entrypoint /entrypoint.sh \
    && sudo chown $USERNAME:$USERNAME /entrypoint.sh && sudo chmod +x /entrypoint.sh \
    && sudo chown root:root /etc/init.d/atrm \
    && sudo chmod +x /etc/init.d/atrm \
    && sudo rosdep install --from-paths src --ignore-src -y || true \
    && source /home/ws/.script/envinit.bash

RUN pip install -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple xmacro --break-system-packages

# ENTRYPOINT [""]
CMD ["/entrypoint.sh"]