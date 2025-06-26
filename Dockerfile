# Use Ubuntu 20.04 as the base image
FROM ubuntu:20.04

ARG ROS_DISTRO=noetic

# Set up the environment to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

ENV ROS_WS /root/catkin_ws
RUN mkdir -p ${ROS_WS}/src 



# Update the package list and install essential packages
RUN apt-get update && apt-get install -y \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    gnupg2 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libusb-1.0-0-dev \
    libx11-dev \
    libxau-dev \
    libxcb1-dev \
    libxdmcp-dev \
    libxext-dev \
    libxrender-dev \
    libxrandr-dev \
    lsb-release \
    mesa-utils \
    pkg-config \
    python3-pip \
    python3-wstool \
    udev \
    usbutils \
    wget \
    x11-xserver-utils \
    x11-utils \
    xserver-xorg \
    && rm -rf /var/lib/apt/lists/*

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

# Setup ROS Install
RUN apt-get update && apt-get install -y && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get update && apt-get install -y \
    python3-osrf-pycommon \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*


# Install ROS ${ROS_DISTRO}
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Source workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Setup rosdep
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && \
    rosdep update --rosdistro ${ROS_DISTRO}
    
# Install Python tools and additional ROS packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-ros-controllers \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS packages for OpenMANIPULATOR-X (without joystick-drivers)
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-perception \
    ros-${ROS_DISTRO}-industrial-core \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-dynamixel-workbench \
    ros-${ROS_DISTRO}-robotis-manipulator \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-controller \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    libboost-dev \
    libeigen3-dev \
    libtinyxml-dev \
    libpcl-dev \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-resource-retriever \
    ros-${ROS_DISTRO}-control-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for USB device access
RUN apt-get update && apt-get install -y \
    usb-modeswitch \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS workspace and clone ${ROS_DISTRO} packages
RUN cd ${ROS_WS}/src && \
    git clone https://github.com/KlausLex/ARL_25_noetic_packages.git && \
    cd ARL_25_noetic_packages && \
    git submodule update --init --recursive && \
    cd .. && \
    cp -r ARL_25_noetic_packages/* . && \
    rm -rf ARL_25_noetic_packages

# Add scripts and recording files
RUN cd ${ROS_WS}/src && \
    git clone https://github.com/KlausLex/arl25_toh.git && \
    cp -r arl25_toh/* . && \
    cp arl25_toh/my_scripts/toh_solver/toh_setup.launch om_position_controller/launch/ && \
    cp arl25_toh/my_scripts/toh_solver/* my_scripts/assignment_3/Docker_volume/ && \
    cp arl25_toh/my_scripts/toh_solver/llm_env.yaml /tmp/llm_env.yaml && \
    rm -rf arl25_toh

# Add YOLO and SAM2 models to the assignment 2 directory
RUN cd ${ROS_WS}/src/my_scripts/assignment_2 && \
    curl -L -o yolo11m.pt https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11m.pt && \
    curl -L -o yolo11n.pt https://github.com/ultralytics/assets/download/v8.3.0/yolo11n.pt && \
    curl -L -o yolov8n.pt https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt && \
    curl -L -o sam2.1_b.pt https://github.com/ultralytics/assets/releases/download/v8.3.0/sam2_t.pt && \
    curl -L -o sam2_b.pt https://github.com/ultralytics/assets/releases/download/v8.3.0/sam2_b.pt

# Fix: Source ROS setup before building
WORKDIR ${ROS_WS}

# Install any missing dependencies
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build workspace
RUN /bin/bash -c "set -e && source /opt/ros/noetic/setup.bash && catkin_make"

# Fix ChromaDB protobuf crash before installing langchain
RUN pip3 install --no-cache-dir protobuf==3.20.3

# Fix chromadb/protobuf according to: https://github.com/chroma-core/chroma/issues/2571
RUN pip install opentelemetry-exporter-otlp-proto-grpc==1.25.0

# Install requirements.txt
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Add user for accessing USB devices
RUN groupadd -r docker && usermod -aG docker root

# Set up environment for running GUI applications
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0
RUN apt-get update && apt-get install -y x11-apps && rm -rf /var/lib/apt/lists/*

RUN wget https://github.com/IntelRealSense/librealsense/raw/master/scripts/libuvc_installation.sh
RUN chmod +x ./libuvc_installation.sh
RUN ./libuvc_installation.sh

RUN apt-get install -y ros-${ROS_DISTRO}-realsense2-camera

# Install dependencies for conda installer
RUN apt-get update && apt-get install -y wget bzip2 && rm -rf /var/lib/apt/lists/*

# Setup miniconda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    /bin/bash /tmp/miniconda.sh -b -p /opt/miniconda && rm /tmp/miniconda.sh

ENV PATH=/opt/miniconda/bin:$PATH

RUN conda install -y conda=25.3.1 && conda clean -afy

RUN conda env create -f /tmp/llm_env.yaml && conda clean -afy

ENV CONDA_DEFAULT_ENV=llm_env

# Set up bashrc for auto-activation of conda env & ROS
RUN echo "source /opt/miniconda/etc/profile.d/conda.sh" >> /root/.bashrc && \
    echo "conda activate llm_env" >> /root/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python" >> /root/.bashrc

# Fix for protobuf / chromadb?
ENV PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

# Install Ollama
RUN curl -fsSL https://ollama.com/install.sh | sh

# Copy and set entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]