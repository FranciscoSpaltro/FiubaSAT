FROM ubuntu:24.04

RUN mkdir -p /usr/src/
WORKDIR /usr/src/

RUN apt-get update && apt-get install -y \
    git \
    python3 \
    usbutils \
    openocd \
    make \
    wget \
    xz-utils \
    nano

RUN git clone https://github.com/fran855/FiubaSAT.git

# Verifica que el archivo exista antes de copiarlo
RUN ls -l /usr/src/FiubaSAT/Drivers/stm32f1x.cfg && \
    cp -f /usr/src/FiubaSAT/Drivers/stm32f1x.cfg /usr/share/openocd/scripts/target/stm32f1x.cfg

RUN mkdir -p /usr/src/Descargas && \
    wget -P /usr/src/Descargas https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz

RUN cd /opt && \
    tar xJf /usr/src/Descargas/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz && \
    mv arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi gcc-arm

RUN echo "export PATH="/opt/gcc-arm/bin:$PATH"" >> ~/.bashrc

CMD ["bash", "-c", "source ~/.bashrc && /bin/bash"]
