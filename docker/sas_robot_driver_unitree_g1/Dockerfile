FROM murilomarinho/sas:jazzy
ARG ROS_DOMAIN_ID=77
ARG BRANCH=main
ENV ROS_DOMAIN_ID=$ROS_DOMAIN_ID
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"

# The base image already has ROS 2 Jazzy, the SAS packages, DQ Robotics, and marinholab::sas::core.

# Update the SAS packages of the base image. Only those already installed. The debug symbols are removed first,
# as they pin the old versions.
RUN apt-get update && \
    apt-get purge -y 'ros-jazzy-sas-*-dbgsym' && \
    apt-get install -y --only-upgrade 'ros-jazzy-sas-*' && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install the Unitree SDK2, and register its vendored CycloneDDS in the dynamic linker
RUN mkdir -p ~/git
RUN cd ~/git/ && \
    git clone https://github.com/unitreerobotics/unitree_sdk2.git --recursive && \
    cd ~/git/unitree_sdk2/ && \
    cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics && \
    cmake --build build -j$(nproc) && \
    cmake --install build
RUN echo "/opt/unitree_robotics/lib" > /etc/ld.so.conf.d/unitree_robotics.conf && ldconfig

# Install unitree_drivers
RUN cd ~/git/ && \
    git clone https://github.com/Adorno-Lab/unitree_drivers.git && \
    cd ~/git/unitree_drivers/ && \
    cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_PREFIX_PATH=/opt/unitree_robotics && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    ldconfig

# Clone sas_tools and the driver
RUN mkdir -p ~/ros2_ws/src
RUN cd ~/ros2_ws/src && \
    git clone https://github.com/Adorno-Lab/sas_tools.git && \
    git clone -b $BRANCH https://github.com/Adorno-Lab/sas_robot_driver_unitree_g1

# Build the driver. sas_legged_client_window (sas_tools) is not built, as it needs Qt.
RUN source /opt/ros/jazzy/setup.bash && \
    export CMAKE_PREFIX_PATH=/opt/unitree_robotics:$CMAKE_PREFIX_PATH && \
    cd ~/ros2_ws && \
    colcon build --packages-up-to sas_robot_driver_unitree_g1

# ALIASES
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash_env
RUN echo "source ~/ros2_ws/install/setup.bash" >> /etc/bash_env
RUN echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> /etc/bash_env
RUN echo "export CMAKE_PREFIX_PATH=/opt/unitree_robotics:\$CMAKE_PREFIX_PATH" >> /etc/bash_env
RUN echo "alias cdros2='cd ~/ros2_ws/' " >> /etc/bash_env
RUN echo "alias buildros2='cdros2 && colcon build --packages-up-to sas_robot_driver_unitree_g1 && source install/setup.bash' " >> /etc/bash_env
RUN echo "alias launch_ROS2_driver='ros2 launch sas_robot_driver_unitree_g1 start_high_level_driver.py' " >> /etc/bash_env
RUN echo "source /etc/bash_env" >> ~/.bashrc
