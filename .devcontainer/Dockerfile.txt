# 1. Use the Official ROS 2 Humble Image (The "Gold Standard")
FROM osrf/ros:humble-desktop-full

# 2. Install PX4 Dependencies & Tools
# We install git, pip, and the tools needed to build the bridge
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    cmake \
    build-essential \
    iputils-ping \
    net-tools \
    wget \
    # Install Python dependencies for PX4
    && pip3 install --no-cache-dir \
    kconfiglib \
    jsonschema \
    jinja2 \
    pyros-genmsg \
    packaging \
    toml \
    numpy \
    future

# 3. Install Micro-XRCE-DDS Agent (The Bridge)
# This builds it from source to ensure it works with Humble
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
    && cd /tmp/Micro-XRCE-DDS-Agent \
    && mkdir build && cd build \
    && cmake .. \
    && make \
    && make install \
    && ldconfig /usr/local/lib/ \
    && rm -rf /tmp/Micro-XRCE-DDS-Agent

# 4. Set Up Workspace & User
USER root
WORKDIR /root/workspace

# 5. Add ROS 2 setup to bashrc so you don't have to source it every time
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc