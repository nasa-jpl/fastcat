# fastcat

C++ library for Control of EtherCat Devices

| Primary Dev    | Secondary Dev | Date    |
| -------------- | ------------- | ------- |
| Joseph Bowkett | Davis Born    | 12/2020 |

# Building and Installing
## Dependencies

Tested on Ubuntu 16.04.

### YamlCpp, ReadLine, DoxyGen

```
$ sudo apt install libyaml-cpp-dev libreadline-dev doxygen
```

### Python yaml, cogapp, graphviz, ipython

```
$ sudo apt install python3-pip
$ sudo pip3 install pyaml cogapp graphviz ipython==7.9
```

## Building

build fastcat from source
```
$ git clone git@fornat1.jpl.nasa.gov:ethercat/fastcat.git
$ cd fastcat
$ mkdir build
$ cd build
$ cmake ..
$ make
```
