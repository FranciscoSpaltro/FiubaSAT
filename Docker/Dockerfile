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
RUN git clone https://github.com/fran855/FiubaSAT_testing.git

RUN cd FiubaSAT_testing/lib
RUN git clone https://github.com/libopencm3/libopencm3.git
RUN cd ../..

# Verifica que el archivo exista antes de copiarlo
RUN ls -l /usr/src/FiubaSAT/Drivers/stm32f1x.cfg && \
    cp -f /usr/src/FiubaSAT/Drivers/stm32f1x.cfg /usr/share/openocd/scripts/target/stm32f1x.cfg

RUN mkdir -p /usr/src/Descargas && \
    wget -P /usr/src/Descargas https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz

RUN cd /opt && \
    tar xJf /usr/src/Descargas/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz && \
    mv arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi gcc-arm

RUN echo "export PATH="/opt/gcc-arm/bin:$PATH"" >> ~/.bashrc

RUN echo "program_stm32() { \
    if [ \"\$#\" -ne 2 ]; then \
        echo \"Usage: program_stm32 <file> <address>\"; \
        return 1; \
    fi; \
    local file=\"\$1\"; \
    local address=\"\$2\"; \
    openocd -f /usr/share/openocd/scripts/interface/stlink.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg -c \"program \$file \$address\" -c \"shutdown\"; \
}" >> ~/.bashrc


CMD ["bash", "-c", "source ~/.bashrc && /bin/bash"]
