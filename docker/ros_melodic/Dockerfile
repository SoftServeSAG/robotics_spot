FROM lyubomyrd/ros_melodic:latest

WORKDIR /root

RUN apt-get update && apt-get install -y \    
    openssh-server \    
    gdb \
    gdbserver \
    libnss3-dev \
    ros-melodic-joint-state-publisher-gui \
    ros-melodic-interactive-marker-twist-server \ 
    ros-melodic-robot-state-publisher \
    python-pip \
    && pip install simple-pid \
    && rm -rf /var/lib/apt/lists/* 

RUN mkdir /var/run/sshd
RUN echo 'root:root' | chpasswd
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

RUN mkdir -p deps_ws/src && \
    cd deps_ws && \
    catkin config \
        --profile release \
        --extend /opt/ros/melodic \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin profile set release

RUN echo "PS1='\[\e[33m\][MrHandy]\[\e[m\] ${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc

RUN echo "source ~/ws/devel/setup.bash" >> ~/.bashrc

RUN echo "export XV_NAME=hand_0" >> ~/.bashrc
