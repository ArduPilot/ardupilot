#########################################################################################
# Micro XRCE-DDS Client Docker
#########################################################################################

# Build stage
FROM ubuntu AS build
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root

# Essentials
RUN apt-get update
RUN apt-get install -y \
            software-properties-common \
            build-essential \
            cmake \
            git

# Prepare Micro XRCE-DDS Client workspace
RUN mkdir -p /client/build
ADD . /client/

# Build Micro XRCE-DDS Client and install
RUN cd /client/build && \
    cmake \
        -DCMAKE_INSTALL_PREFIX=../install \
        -DUCLIENT_BUILD_EXAMPLES=ON \
        -DUCLIENT_INSTALL_EXAMPLES=ON \
        -DUCLIENT_ISOLATED_INSTALL=OFF \
        .. &&\
    make && make install

# Prepare Micro XRCE-DDS Client artifacts
RUN cd /client && \
    tar -czvf install.tar.gz  -C install .

# Final user image
FROM ubuntu
WORKDIR /root

# Copy Micro XRCE-DDS Client build artifacts
COPY --from=build /client/install.tar.gz  /usr/local/
RUN tar -xzvf /usr/local/install.tar.gz -C /usr/local/ &&\
    rm /usr/local/install.tar.gz

RUN ldconfig