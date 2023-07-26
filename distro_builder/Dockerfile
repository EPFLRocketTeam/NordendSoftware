FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Zurich



RUN apt-get update
RUN apt-get install -y locales
RUN apt-get install -y bsdmainutils build-essential chrpath cpio debianutils diffstat gawk gcc-multilib git iputils-ping libegl1-mesa libgmp-dev libmpc-dev libsdl1.2-dev libssl-dev pylint python3 python3-git python3-jinja2 python3-pexpect python3-pip socat texinfo unzip wget xterm xz-utils

RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    locale-gen
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en  
ENV LC_ALL en_US.UTF-8  

RUN rm /bin/sh && ln -s bash /bin/sh

RUN groupadd -g 1000 iacopo
RUN useradd -g 1000 -m -s /bin/bash -u 1000 iacopo
    
USER iacopo
    
    
    
    
    
    