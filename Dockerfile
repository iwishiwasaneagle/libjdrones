FROM ubuntu:mantic

RUN apt-get update \
    && apt-get install -y \
      git build-essential cmake \
      python3.11 python3-pip \
      libeigen3-dev

WORKDIR /opt
RUN git clone --depth 1 --branch v3.4.0 https://github.com/catchorg/Catch2.git \
    && cd Catch2 \
    && cmake -Bbuild -H. -DBUILD_TESTING=OFF \
    && cmake --build build/ --target install

RUN pip install "pybind11[global]>=2.12.0" --break-system-packages

WORKDIR /src
COPY . .
RUN pip install .[test] --break-system-package
