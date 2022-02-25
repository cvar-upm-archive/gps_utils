# gps_utils

## Dependencies
```
sudo apt update
sudo apt install geographiclib-tools
sudo apt install libgeographic-dev
```

- Linking problem [[1]](https://stackoverflow.com/questions/48169653/finding-geographiclib-in-cmake-on-debian):
```
sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-*/Modules/
sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/local/share/cmake-*/Modules/
```
