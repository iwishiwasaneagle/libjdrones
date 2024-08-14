# libjdrones

[![CI](https://github.com/iwishiwasaneagle/libjdrones/actions/workflows/CI.yml/badge.svg)](https://github.com/iwishiwasaneagle/libjdrones/actions/workflows/CI.yml)

A fast quadcopter support library written in C++ for [jdrones](https://github.com/iwishiwasaneagle/jdrones).

## Installation

If you're simply looking for python, run `pip install libjdrones`.

Else, to install the C++ library run

```bash
git clone  https://github.com/iwishiwasaneagle/libjdrones
cd libjdrones
cmake \
    -B build
    -DCMAKE_BUILD_TYPE=Release \
    .
cmake \
    --build build \
    --target install
```

With conda, be sure to add `-DCMAKE_INSTALL_PREFIX=${CONDA_PREFIX} -DCMAKE_PREFIX_PATH=${CONDA_PREFIX}` to ensure that
it is correctly installed in the env.


### Dependencies


#### Eigen3

Please see the [official documentation](https://eigen.tuxfamily.org/dox/GettingStarted.html).

#### pybind

The minimum version of pybind11 depends on the version of python. It is advised to either install from source or
using pip (`pip install "pybind11[global]>=2.11.0"`).

**WARNING**: `Ubuntu 22.04` only ships with `pybind11@v2.9.1`.

| Python | [`pybind11-dev`](https://pybind11.readthedocs.io/en/stable/changelog.html) |
|--------|----------------------------------------------------------------------------|
| 3.10   | 2.9.0                                                                      |
| 3.11   | 2.10.0                                                                     |
| 3.12   | 2.11.0                                                                     |

## Run tests

 `Catch2>=v3.4.0` is required (see [CI.yml](.github/workflows/CI.yml))

```bash
cmake -B build \
  -DJDRONES_ENABLE_UNIT_TESTING=ON \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j $(nproc)
ctest --test-dir build
```

### With coverage

Add `-DJDRONES_ENABLE_CODE_COVERAGE=ON` to the initial cmake call.

```bash
lcov --capture --directory build --output-file coverage.info
lcov --remove coverage.info --output-file coverage.info '/usr/*' '*/test/*' '*/extern/*' '*/geos/*'
genhtml coverage.info -o build/html
firefox build/html/index.html
```
