# Environment Configuration

## Google OR-Tools (ortools)
1. Install ortools (Ubuntu 20.04 LTS 64-bit (x86_64)) from [binary](https://developers.google.com/optimization/install/cpp/binary_linux). The download link may be found [here](https://github.com/google/or-tools/releases/download/v9.5/or-tools_amd64_ubuntu-20.04_cpp_v9.5.2237.tar.gz).
2. Open a terminal, run
```
sudo apt update
sudo apt install -y build-essential cmake lsb-release
```
3. In the terminal, enter the root directory of ortools' installation, run: 
```
make test
```
4. You may follow the [Get started tutorial](https://developers.google.com/optimization/introduction/cpp) to build and run a simple demo. But we recommand to build the source code with Bazel, following the instructions of the next section.

## GCC
OR-Tools Ubuntu applies ISO C++20 standard. To install GCC version 9, run:
```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-9
sudo apt install g++-9
```

## Bazel
1. Bazel can be installed through [Bazelisk](https://github.com/bazelbuild/bazelisk):
```
npm install -g @bazel/bazelisk
```
2. You may change the bazel version to 6.0.0 by running in the terminal:
```
USE_BAZEL_VERSION=6.0.0 bazelisk versio
```
3. List the directory as below:
```
ROMP/
|1---->| inp/
|1---->| out/
|1---->| init_solver/
|2-------->| WORKSPACE
|2-------->| main/
|3------------>| BUILD
|3------------>| point.h
|3------------>| point.cpp
|3------------>|...
```
4. Note that [WORKSPACE](src/offline_scheme/WORKSPACE) and [BUILD](src/offline_scheme/init_solver/main/BUILD) are **compulsory** files for compiling ortools. 
Please check [Google's guidance](https://github.com/google/or-tools/blob/stable/bazel/README.md) for updating. Now we can build source code with:
```
cd init_solver/
sudo bazel build --cxxopt="-std=c++2a" //main:all
```
5. The binary executable file can be found in `init_solver/bazel-bin/main/cpp_ortools`.

## MPICH
1. The MPICH can be installed by running:
```
sudo apt-get update
sudo apt-get -y install mpich
```
2. List the directory as below:
```
ROMP/
|1---->| inp/
|1---->| out/
|1---->| opt_solver/
|2-------->| point.h
|2-------->| point.cpp
|2-------->| ...
```
3. Build the code with `mpicxx`:
```
cd opt_solver/main/
mpicxx -std=c++2a *.cpp -o cpp_bha
```
4. The binary executable file can be found in `opt_solver/main/cpp_bha`.
