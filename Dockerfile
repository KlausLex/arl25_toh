# Use Ubuntu 20.04 as the base image
FROM ubuntu:20.04

# Set up the environment to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Set CXXFLAGS to disable treating warnings as errors
ENV CXXFLAGS="${CXXFLAGS} -Wno-error"

# Update the package list and install essential packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    lsb-release \
    gnupg2 \
    curl \
    wget \
    ca-certificates \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libx11-dev \
    libxrender-dev \
    libxrandr-dev \
    build-essential \
    udev \
    usbutils \
    libusb-1.0-0-dev \
    xserver-xorg \
    x11-xserver-utils \
    x11-utils \
    mesa-utils \
    pkg-config \
    libxau-dev \
    libxdmcp-dev \
    libxcb1-dev \
    libxext-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Fix Error "Invoking "make -j12 -l12" failed"
RUN apt-get update && apt-get install -y libsdl1.2-dev && \
    rm -rf /var/lib/apt/lists/*

# Set up NVIDIA drivers
RUN apt-get update && apt-get install -y \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libgles2 \
    && rm -rf /var/lib/apt/lists/*

# Create required directories for NVIDIA
RUN mkdir -p /usr/local/share/glvnd/egl_vendor.d/
RUN echo '{"file_format_version" : "1.0.0", "ICD" : {"library_path" : "libEGL_nvidia.so.0"}}' > /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Install ROS Noetic
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Install Python tools and additional ROS packages
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rviz \
    ros-noetic-ros-controllers \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Install Gazebo 11
RUN apt-get update && apt-get install -y \
    gazebo11 \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS packages for OpenMANIPULATOR-X (without joystick-drivers)
RUN apt-get update && apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-moveit-ros-perception \
    ros-noetic-industrial-core \
    ros-noetic-dynamixel-sdk \
    ros-noetic-dynamixel-workbench \
    ros-noetic-robotis-manipulator \
    ros-noetic-pcl-ros \
    ros-noetic-joy \
    ros-noetic-control-toolbox \
    ros-noetic-controller-interface \
    ros-noetic-controller-manager \
    ros-noetic-joint-state-controller \
    ros-noetic-moveit-visual-tools \
    libboost-dev \
    libeigen3-dev \
    libtinyxml-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for USB device access
RUN apt-get update && apt-get install -y \
    usb-modeswitch \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS workspace
# RUN apt-get update && apt-get install -y ros-noetic-moveit-visual-tools    
# RUN apt-get update && apt-get install -y python3-pip

# # Install requirements.txt
# COPY requirements.txt /tmp/requirements.txt
# RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Setup ROS workspace and clone noetic packages
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws/src && \
    git clone https://github.com/KlausLex/ARL_25_noetic_packages.git && \
    cd ARL_25_noetic_packages && \
    git submodule update --init --recursive && \
    cd .. && \
    cp -r ARL_25_noetic_packages/* . && \
    rm -rf ARL_25_noetic_packages

# Copy recording dir
# COPY requirements /root/catkin_ws/src/recordings

# Add scripts and recording files
RUN cd /root/catkin_ws/src && \
    git clone https://github.com/KlausLex/arl25_toh.git && \
    cp -r arl25_toh/* . && \
    rm -rf arl25_toh

# Add YOLO and SAM2 models to the assignment 2 directory
RUN cd /root/catkin_ws/src/my_scripts/assignment_2 && \
    curl -L -o yolo11m.pt https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11m.pt && \
    curl -L -o yolo11n.pt https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt && \
    curl -L -o yolov8n.pt https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt && \
    curl -L -o sam2.1_b.pt https://github.com/ultralytics/assets/releases/download/v8.3.0/sam2_t.pt && \
    curl -L -o sam2_b.pt https://github.com/ultralytics/assets/releases/download/v8.3.0/sam2_b.pt

# Fix: Source ROS setup before building
WORKDIR /root/catkin_ws

# Install any missing dependencies
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y

# Install requirements.txt
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_VERBOSE_MAKEFILE=ON"

# Add user for accessing USB devices
RUN groupadd -r docker && usermod -aG docker root

# # Try building everything except problematic packages
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
#     source /root/catkin_ws/devel/setup.bash && \
#     catkin_make"

# Set up environment for running GUI applications
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0
RUN apt-get update && apt-get install -y x11-apps && rm -rf /var/lib/apt/lists/*

RUN wget https://github.com/IntelRealSense/librealsense/raw/master/scripts/libuvc_installation.sh
RUN chmod +x ./libuvc_installation.sh
RUN ./libuvc_installation.sh

RUN apt-get install -y ros-noetic-realsense2-camera

# # Install requirements.txt
# COPY requirements.txt /tmp/requirements.txt
# RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Source the ROS environment by default
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc


# Set the default command
CMD ["bash"]