ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}
ARG USERNAME=username
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user with the same UID
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Set up user and sudo
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    wget \
    vim \
    tmux \
    bash-completion \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL=/bin/bash

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/$USERNAME/.bashrc
USER $USERNAME
CMD ["/bin/bash"]