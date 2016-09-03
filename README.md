cpp-atlas
================

This is the C++11 code for the ATLAS Localization Server.

## Dependencies

* GCC 4.9 and later
* CMake 2.6 and later
* LibYAML (C++)
* Boost (Currently, due to LibYAML dependency)
* Armadillo (C++)


## Installing

On Debian or similar:

```Shell
sudo apt-get install git cmake libyaml-cpp-dev libboost-dev libarmadillo-dev
```

## Building

Building should be straight forward, as CMake is used.

```Shell
cmake .
make
```

## Configuration

The `config.yaml` is used to configure the server. 
Since all anchors are connected to the server via USB, a unique symlink is created for each anchor to differentiate between them. 
This is done using the `99-usb-uwb.rules` udev rules file.


## License

See the [LICENSE](LICENSE.md) file for license rights and limitations (MIT).