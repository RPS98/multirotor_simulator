# Multirotor Simulator in C++

The purpose of this repository is to provide a multirotor simulator, implemented in C++.

## API

The documentation of the implementation can be found in [docs subdirectory](docs/README.md).

This repository make use of the [PID library](https://github.com/RPS98/pid_controller.git) for the PID controller implementation.

For code documentation, use [Doxygen](https://www.doxygen.nl/index.html). The documentation files have to be built locally:

1. Install doxygen `sudo apt install doxygen`
2. Run doxygen in the root folder `multirotor_simulator/` by the `doxygen` command.
3. Open the documentation in by clicking `multirotor_simulator/doxygen/html/index.html`

## Usage
An implementation example can be found in [Aerostack2](https://github.com/aerostack2/aerostack2), specifically in the [as2 multirotor simulator platform](https://github.com/aerostack2/as2_multirotor_simulator) package.

## Build
Dependencies: [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page), [gtest](https://github.com/google/googletest) (only if set `BUILD_TESTING` to ON).

The project can be built using CMake. The following commands can be used to build the project:

```bash
mkdir build
cd build
cmake ..
make
```

## Development
Build tests using `BUILD_DEVELOPER_TESTS` flag:

Dependencies: [cpplint](https://github.com/cpplint/cpplint), [benchmark](https://github.com/google/benchmark), [lcov](https://github.com/linux-test-project/lcov).

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_DEVELOPER_TESTS=true ..
make
```

Run tests using ctest from the build folder:

```bash
cd build
ctest
```

View code coverage opening the `index.html` file in `build/tests/coverage/index.html` with a browser.

## Examples
Check the [examples](examples/README.md) folder for more information.