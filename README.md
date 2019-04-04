# splash
Splash is a Work-In-Progress physical simulation using the APIC Material Point Method. 

This program implements the material point method described in the [2016 SIGGRAPH Course Notes](https://www.seas.upenn.edu/~cffjiang/research/mpmcourse/mpmcourse.pdf) by Jiang  et al.
It also draws from details elaborated in [Stomakhin 2013](http://alexey.stomakhin.com/research/snow.html) and [Jiang 2015](https://www.seas.upenn.edu/~cffjiang/research/apic/paper.pdf).

The purpose of this project is to first learn the core principles of the Material Point Method. 
The second goal of this project is to learn and implement various parallel programming methods. 

Splash uses [libigl](https://libigl.github.io/) for rendering.

## Features/TODO
* [x] Serial Explicit MPM
* [ ] Serial Implicit MPM
* [ ] OpenMP Implicit MPM
* [ ] MPI Implicit MPM

## Compiling
Dependencies
- [libigl](https://libigl.github.io/) and its dependencies (imgui, opengl)


### MacOS & Linux
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```
I tested compiling with GCC 7.3 on Ubuntu 18.

### Windows
Untested. 

## Usage
From the binary directory (build/bin/)
```
./splash
```
