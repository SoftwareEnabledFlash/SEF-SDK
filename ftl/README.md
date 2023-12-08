# SEF SDK

In order to provide a starting point for host application development,
the SEF SDK was built in conjunction with the SEF API. The SEF SDK
includes sample drivers, libraries, supporting documentation, and
an implementation of a fully functional starter FTL that can be extended
or modified as appropriate.

The library implements an asynchronous user mode block API using the
SEF library. The implementation serves as an example of how to use
the SEF library and demonstrates the following:
- Managing a flash translation layer (FTL) with multiple placement IDs
- Persisting and recovering FTL metadata
- Background garbage collection using I/O priority control
- Handling and accounting for device wear


## Build and Install

You can get started using the SEF SDK library by building and installing it using the `cmake` and `make` tool set.

Before getting started, please ensure the following prerequisites have been installed:
- pthread

To build and install the library please use the following commands:

```
./build.sh
pushd bin
sudo make install
popd
```

## Usage

The SEF SDK builds on the SEF library. The API is defined by sef-block-module.h.
It is designed for user mode and exposes a C11 API. It has been tested
with CentOS 8 on x86/AMD 64-bit processes. The SDK requires the pthread
header and library. Applications accessing a SEF unit through the
SEF library or SEF SDK must currently run as root.

More information about the offered APIs can be found with the enclosed pdf.

## Test

The SEF SDK was tested using the following test methods:

- Simulator
  - unit tests
  - google load tests
- Emulator
  - google load tests
