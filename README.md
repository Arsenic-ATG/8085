<!-- TODO: add a banner here -->
# 8085

The Intel 8085 is a classic 8-bit microprocessor that was widely used in the 1970s and 1980s in a variety of computer systems. This project is a software emulator for the 8085 processor using C++ programming language. The emulator allows users to run programs written for the 8085 on modern computer systems.

## Prerequisites

### For user ( If you just want to just use this project )

  - Standard C++ compiler ( supporting C++17 or above )
  - [optional] Cmake build tool
  - [optional] Git version control system 

### for developers ( If you want to contribute to this project )

  - A text editor to view/edit code
  - Git version control system
  - Standard C++ compiler ( supporting C++17 or above )
  - Cmake build tools along with GNU Make
  - Google Test framework ( for running testsuite )

## Installation instructions

You can either use Cmake to build the project or compile it manually by hand. 

Caveat: I have only built and tested it on macOS and linux, so have very little idea of how things should work on windows, if you are able to build it on windows then please c

If you find any difficulties while building the project or face any error during the process then please feel free to open a new issue regarding the same with all the necessary information ( including output generated by Cmake if used )

### using Cmake

Simply navigate to the repository ( `$ cd 8085` ) and use `cmake` to gneerate build makefiles and inturn use `make` to generate executables

```shell
$ cmake . && make
```

When done, the library should be built as 'src/cpu.a' ( cmake is currently configured to compile it to a static library ).

### without using Cmake

All the source code is present in the `/src` subdirectoy of the project. The project currently only contains a header file and a source file pair with no additional dependency ( at least none for building ) so compilation command should not be complicated at all.

you can simply use your standard C++ compiler to compile the code and use `ar` ( archiver ) that usually comes bundled with UNIX to create static/dynamic library ( according to your choice ) from the project.

Here is the sequence of instructions that you would be performing ( you can use whatever compiler you have on your machine, I am using gcc here in this example )

``` shell
$ cd src
$ g++ -c cpu.cpp -std=c++2a -o cpu.o
$ ar rcs cpu.o lib8085.a
```

## Contribution

If you want to contribute to the project, then make sure you can build the project properly before proceeding ( not always applicable for small documentation changes ), feel free to use discussion section or the issue section in case you have any difficulties during the process.

There are multiple ways in which you can contribute to the project 

1. Code contribution
2. Testing
3. Documentation
4. Feedback

have a look at CONTRIBUTING.md ( currently work in progress ) for more detailed explanation about contributing to the project.

## The API

WIP

## Feedback

You can star the project, use discussion section, issue section or even mail me your feedback about what all you like and what all can be improved in future versions of the software

## Support the project

I currently don't take any monetary support so the only way to support this project currently is to give a feedback which can also be as simple as giving this repository a star on GitHub.

## License

[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](./LICENSE)

- **[MIT license](./LICENSE)**
- Copyright 2023 © <a href="https://github.com/Arsenic-ATG" target="_blank">Ankur Saini</a>.
