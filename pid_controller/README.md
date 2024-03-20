# PID Controller in C++

This repository contains a C++ implementation of a PID controller.

## API
For documentation, use [Doxygen](https://www.doxygen.nl/index.html). The documentation files have to be built locally:

1. Install doxygen `sudo apt install doxygen`
2. Run doxygen in the root folder `geometric_controller/` by simply issuing the `doxygen` command.
3. Open the documentation in by clicking `geometric_controller/doxygen/html/index.html`

## Development
Build tests using `BUILD_DEVELOPER_TESTS` flag:

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