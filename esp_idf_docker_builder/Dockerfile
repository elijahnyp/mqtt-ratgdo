FROM ubuntu:22.04

ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update; \
    apt-get install -y \
        git \
        wget \
        flex \
        bison \
        gperf \
        python3 \
        python3-pip \
        python3-setuptools \
        python3-venv\
        cmake \
        ninja-build \
        ccache \
        libffi-dev \
        libssl-dev \
        dfu-util \
        libusb-1.0-0

RUN git clone --recursive https://github.com/espressif/esp-idf.git && \
    cd esp-idf; git checkout 4d0db7045d && \
    git submodule update --init --recursive
ENV IDF_TOOLS_PATH=/idf_tools
RUN cd /esp-idf/ && ./install.sh && \
    mkdir /.cache && \
    chmod 777 /.cache;

RUN pip3 install requests

RUN useradd -m -u 1000 -o -s /bin/sh jenkins
