FROM ros:humble
MAINTAINER Gabriele Abbate "gabriele.abbate@idsia.ch"
RUN apt-get update && apt-get install -y \
    locales \
    uuid-dev \
    libsdl2-dev \
    usbutils \
    libusb-1.0-0-dev \
    python3-pip \
    curl \
    software-properties-common\
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    locales \
    alsa-base \
    alsa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN sed -i 's+archive.ubuntu.com/ubuntu/ jammy universe+archive.ubuntu.com/ubuntu/ focal universe+' /etc/apt/sources.list
RUN sed -i 's+archive.ubuntu.com/ubuntu/ jammy-updates universe+archive.ubuntu.com/ubuntu/ focal-updates universe+' /etc/apt/sources.list

RUN apt-get update && apt-get install -y \
    libsoundio1 \
    libsoundio-dev \
    && rm -rf /var/lib/apt/lists/*

RUN sed -i 's+archive.ubuntu.com/ubuntu/ focal universe+archive.ubuntu.com/ubuntu/ jammy universe+' /etc/apt/sources.list
RUN sed -i 's+archive.ubuntu.com/ubuntu/ focal-updates universe+archive.ubuntu.com/ubuntu/ jammy-updates universe+' /etc/apt/sources.list

RUN curl -L https://packages.microsoft.com/keys/microsoft.asc | apt-key add -
RUN echo "deb https://packages.microsoft.com/ubuntu/18.04/prod bionic main" >> /etc/apt/sources.list.d/archive_uri-https_packages_microsoft_com_ubuntu_18_04_prod-bionic.list

#RUN cat /etc/apt/sources.list.d/archive_uri-https_packages_microsoft_com_ubuntu_20_04_prod-focal.list

ARG DEBIAN_FRONTEND=noninteractive
RUN echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections
RUN echo 'libk4a1.4 libk4a1.4/accept-eula boolean true' | debconf-set-selections
RUN apt-get update && apt-get install -y \
    k4a-tools \
    && rm -rf /var/lib/apt/lists/*
COPY ./99-k4a.rules /etc/udev/rules.d/
RUN chmod a+rwx /etc/udev/rules.d