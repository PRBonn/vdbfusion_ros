# ROS-VDBFusion: Flexible and Efficient TSDF Integration

A ROS C++ wrapper to the [vdbfusion](https://github.com/PRBonn/vdbfusion) library for flexible and efficient TSDF Integration

## Installation
- OpenVDB
    ```sh
    # Install OpenVDB dependencies
    sudo apt-get update && apt-get install --no-install-recommends -y \
        libblosc-dev \
        libboost-iostreams-dev \
        libboost-system-dev \
        libboost-system-dev \
        libeigen3-dev

    # Install OpenVDB from source
    git clone --depth 1 https://github.com/nachovizzo/openvdb.git -b nacho/vdbfusion
    cd openvdb
    mkdir build && cd build
    cmake  -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DUSE_ZLIB=OFF ..
    sudo make -j$(nproc) all install
    cd ../
    ```

- VDBfusion
    ```sh
    git clone --depth 1 https://github.com/PRBonn/vdbfusion.git
    cd vdbfusion
    mkdir build && cd build
    cmake ..
    make -j$(nproc) all install
    cd ../
    ```
- [ROS Noetic](http://wiki.ros.org/noetic) - Follow the installation instructions [here](http://wiki.ros.org/noetic/Installation/Ubuntu) if not installed already and create a ros workspace as below
    ```sh
    mkdir -p vdbfusion_ros_ws/src
    cd vdbfusion_ros_ws/
    catkin init
    cd ../
    ```

- Clone the [vdbfusion_ros](https://github.com/saurabh1002/vdbfusion_ros.git) repository in your ros workspace
    ```sh
    cd vdbfusion_ros_ws/src/
    git clone https://github.com/saurabh1002/vdbfusion_ros.git
    # Install dependencies
    sudo ./vdbfusion_ros/install_deps.sh
    catkin build
    ```

## Usage
1. Create a config file compatible with your dataset and desired tsdf integration parameters using this [template configuration](/config/template.yaml)

## Dataset Examples

## Citation

If you use this library for any academic work, please cite the original [paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/vizzo2022sensors.pdf).

```bibtex
@article{vizzo2022sensors,
  author         = {Vizzo, Ignacio and Guadagnino, Tiziano and Behley, Jens and Stachniss, Cyrill},
  title          = {VDBFusion: Flexible and Efficient TSDF Integration of Range Sensor Data},
  journal        = {Sensors},
  volume         = {22},
  year           = {2022},
  number         = {3},
  article-number = {1296},
  url            = {https://www.mdpi.com/1424-8220/22/3/1296},
  issn           = {1424-8220},
  doi            = {10.3390/s22031296}
}
```
