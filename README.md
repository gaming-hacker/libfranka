# Cmake flags

To build for OSX, add -DAPPLE to CXXFLAGS in your cmake arguments.

To build for non-realtime for Linux, add -DNREALTIME to CXXFLAGS in your cmake arguments.

There must be a complier capable of supporting C++17 and cmake version of at least 3.15 so clang 8+, gcc 8+.

The libs were successfully compiled on OSX 10.15.3 and Ubuntu 18LTS.  Although it should be noted that on this version of Ubuntu you will have to upgrade the compilers and cmake to compatible or latest versions.

The release version of the library will be libfranka.dylib and debug versions libfrankad.dylib, note the additional d in the debug library name.

The library supports the FCI which must have firmware supporting version 3.

## Caveats

On OSX, the functionality is limited.  The examples of motion with force or control will not work, any program requiring active joint control will not function and will return "libfranka error: model not found".  libfranka was not intended to work on OSX and inside the code, the hooks are for ARM, Linux, Windows and generic UNIX.  OSX is not typical UNIX.  With more time and effect this could be resolved.

The non-realtime library has NOT been tested on Linux.

# libfranka: C++ library for Franka Emika research robots

[![Build Status][travis-status]][travis]
[![codecov][codecov-status]][codecov]

With this library, you can control research versions of Franka Emika robots. See the [Franka Control Interface (FCI) documentation][fci-docs] for more information about what `libfranka` can do and how to set it up. The [generated API documentation][api-docs] also gives an overview of its capabilities.

## License

`libfranka` is licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[api-docs]: https://frankaemika.github.io/libfranka
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/libfranka.svg?branch=master
[travis]: https://travis-ci.org/frankaemika/libfranka
[codecov-status]: https://codecov.io/gh/frankaemika/libfranka/branch/master/graph/badge.svg
[codecov]: https://codecov.io/gh/frankaemika/libfranka
