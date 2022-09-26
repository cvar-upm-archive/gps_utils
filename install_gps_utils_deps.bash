sudo apt update
# Install GeographicLib
sudo apt install libgeographic-dev geographiclib-tools -y &&\
sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-*/Modules/ &&\


