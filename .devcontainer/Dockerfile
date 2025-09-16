FROM ros:humble
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

RUN rm -f /etc/apt/sources.list && \
    { \
    echo 'deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy main restricted universe multiverse'; \
    echo 'deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse'; \
    echo 'deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse'; \
    echo 'deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-security main restricted universe multiverse'; \
    } > /etc/apt/sources.list

# 创建密钥目录并添加 sagernet GPG 密钥
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://board.nyanc.cc/sbgpg.key -o /etc/apt/keyrings/sagernet.asc && \
    chmod a+r /etc/apt/keyrings/sagernet.asc

# 添加 sagernet 软件源
RUN echo 'Types: deb\nURIs: https://deb.sagernet.org/\nSuites: *\nComponents: *\nEnabled: yes\nSigned-By: /etc/apt/keyrings/sagernet.asc' > /etc/apt/sources.list.d/sagernet.sources

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -y \
    iproute2 \
    net-tools
RUN apt-get install -y python3-pip
RUN curl -fsSL https://board.nyanc.cc/sbinst.sh | sh -s -- --version 1.11.0

RUN echo '#!/bin/bash\n\
cd /home/ws/.proxy && \
eth0_ip=$(ip addr show eth0 | grep "inet " | awk "{print \$2}" | cut -d"/" -f1 | sed "s/\\.[0-9]*\$/.1/") && \
echo $eth0_ip && \
sudo ip route del default || true && \
sudo ip route add default via $eth0_ip dev eth0 || true && \
nohup sudo sing-box run -c /home/ws/.proxy/config.json > /dev/null 2>&1 & \
sleep 10\n\
exec "$@"' > /usr/local/bin/startup.sh && \
chmod +x /usr/local/bin/startup.sh

ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

ENTRYPOINT ["/usr/local/bin/startup.sh"]
CMD ["/bin/bash"]