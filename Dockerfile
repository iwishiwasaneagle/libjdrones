FROM ubuntu:mantic

RUN apt-get update \
    && apt-get install -y \
      git build-essential cmake \
      python3.11 python3-pip \
      libeigen3-dev

WORKDIR /opt

RUN pip install "pybind11[global]>=2.12.0" --break-system-packages

WORKDIR /src
COPY . .
RUN cmake -DCMAKE_BUILD_TYPE=Release -S . -B "/var/build" \
    && cmake --build "/var/build" -j $(nproc) --config Release --target install \
    && pip install .[test] --break-system-package
