cpp-atlas
================

**This code is deprecated**

This is the C++11 code for the ATLAS Localization Server.

[![DOI](https://zenodo.org/badge/20743/tudo-cni-atlas/cpp-atlas.svg)](https://zenodo.org/badge/latestdoi/20743/tudo-cni-atlas/cpp-atlas)

## Publications

If you use this work in an academic context, please cite the following publication(s):

* J. Tiemann, F. Eckermann, C. Wietfeld, **ATLAS - An Open-Source TDOA-based Ultra-Wideband Localization System**, In 2016 International Conference on Indoor Positioning and Indoor Navigation (IPIN), Alcalá de Henares, Madrid, Spain, October 2016. ([PDF](https://doi.org/10.1109/IPIN.2016.7743707))

        @inproceedings{Tiemann2016,
            author = {J. Tiemann and F. Eckermann and C. Wietfeld},
            booktitle = {2016 International Conference on Indoor Positioning and Indoor Navigation (IPIN)},
            title = {{ATLAS - An Open-Source TDOA-based Ultra-Wideband Localization System}},
            publisher = {IEEE},
            year = {2016}
        }


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
