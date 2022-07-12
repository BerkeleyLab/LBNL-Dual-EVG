FROM debian:bullseye-slim as xilinx_env

# Vivado needs libtinfo5, at least for Artix?
# libz-dev required for Verilator FST support
RUN apt-get update && \
    apt-get install -y \
    git \
    time \
    iverilog \
    libz-dev \
    libbsd-dev \
    xc3sprog \
    build-essential \
    libtinfo5 \
    wget \
    iputils-ping \
    iproute2 \
    bsdmainutils \
    curl \
    flake8 \
    x11-utils \
    lib32z1 \
    libgtk-3-dev \
    xvfb \
    dbus-x11 \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    gcc-riscv64-unknown-elf \
    picolibc-riscv64-unknown-elf \
    cmake \
    libidn11 \
    libftdi1-dev \
    libusb-dev \
    verilator \
    openocd \
    pkg-config && \
    rm -rf /var/lib/apt/lists/* && \
    python3 -c "import numpy; print('Test %f' % numpy.pi)" && \
    pip3 --version

RUN pip3 install pyyaml==5.1.2 nmigen==0.2 pyserial==3.4
