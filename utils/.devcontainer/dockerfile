FROM osrf/ros:noetic-desktop-full

#########################################################################################################
# Create a non-root user:                                                                               #
# - https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user.  #
#########################################################################################################
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m $USERNAME && \
    apt-get update && \
    apt-get install -y sudo && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# Activate the previously created user.
USER ${USERNAME}
# Set our working directory.
WORKDIR "/workspace"
# Ensure we can read/write to the working directory.
RUN sudo chmod a+rwx .

#########################################################################################################
# Update and install required apt and pip dependencies.                                                 #
#########################################################################################################
RUN sudo apt-get update && sudo apt-get upgrade -y && \
    sudo apt-get install -y --no-install-recommends \
        curl \
        wget \
        python3-catkin-tools \
        python3-pip \
        git \
        nano \
        clang-format \
        bash-completion \
        ninja-build && \
    # Perform cleanup.
    rm -rf "/var/lib/apt/lists/*" && \
    sudo apt-get clean

RUN pip3 install \
    packaging \
    tqdm \
    pandas \
    pyyaml 

RUN echo "source /opt/ros/noetic/setup.bash" >> "/home/${USERNAME}/.bashrc"