# Workspace Setup

Before starting the setup process, it may not be a bad idea to get yourself a beverage :coffee: since the setup will take quite some time :grin:.

## Carla Workspace

Although it is not compulsory, we would recommend to have all dependencies collected under one ROS workspace. In the following of this documentation, we would assume this workspace is `~/carla_ws`.

```
cd
mkdir -p carla_ws/src
catkin init --workspace carla_ws
```

Make sure that the `carla_ws` does not extending any other ROS workspaces (not even the default ROS workspace `/opt/ros/melodic/`).

## Boost 1.69.0

Download Boost 1.69.0 and install it to `carla_ws`. The details of compiling and installing Boost libraries can be found [here](https://www.boost.org/doc/libs/1_69_0/more/getting_started/unix-variants.html). For simplicity, they are summarized as follows.

In the unzipped Boost folder, do the following,
```
./bootstrap.sh --prefix=/home/$USER/carla_ws/boost_1_69_0
./b2 install
```
After these steps, there should be two directories in `/home/$USER/carla_ws/boost_1_69_0`, `include` and `lib`. Make sure that `libboost_python27.*` is in the `lib` directory, since we will be using python2.7 with ROS melodic (compile ROS with python3 is beyond the scope of this documentation). In case `libboost_python27.*` is missing, see [this](https://www.boost.org/doc/libs/1_69_0/libs/python/doc/html/building.html) for building Boost.Python. Especially, check [this](https://www.boost.org/doc/libs/1_69_0/libs/python/doc/html/building/configuring_boost_build.html) for setting up the correct python version.

## PCL 1.9.1

The details of compiling PCL from source can be found [here](https://pcl-tutorials.readthedocs.io/en/latest/building_pcl.html#). The following commands are related to using PCL in this project. In the unzipped PCL directory, do the following.

```
mkdir build && cd build
cmake -DBOOST_ROOT=/home/$USER/carla_ws/boost_1_69_0 \
      -DCMAKE_INSTALL_PREFIX=/home/$USER/carla_ws/pcl_1_9_1 \
      -DCMAKE_BUILD_TYPE=Release ..
make install
```

After these steps, there should be `bin`, `include`, `lib`, and `share` subdirectories in `/home/$USER/carla_ws/pcl_1_9_1`.

## ROS Melodic

Follow the instructions [here](http://wiki.ros.org/melodic/Installation/Source) to compile ROS melodic from source. Related instructions are listed as follows.

```
cd ~/carla_ws
rosinstall_generator desktop_full --rosdistro melodic --deps --tar > melodic-desktop-full.rosinstall
wstool init -j8 src melodic-desktop-full.rosinstall
catkin config -DCMAKE_BUILD_TYPE=Release \
              -DBOOST_ROOT=/home/${USER}/carla_ws/boost_1_69_0 \
              -DBoost_NO_SYSTEM_PATHS=ON \
              -DBOOST_INCLUDEDIR=/home/${USER}/carla_ws/boost_1_69_0/include \
              -DBOOST_LIBRARYDIR=/home/${USER}/carla_ws/boost_1_69_0/lib \
              -DPCL_DIR=/home/${USER}/carla_ws/pcl_1_9_1/share/pcl-1.9
```

Of course, ROS will not be compiled successfully at the first pass, since Boost 1.69.0 has renamed and removed some libraries compared to Boost 1.65.0. Some touches of `CMakeLists.txt` in a few packages are required. The required modifications (maybe not all) are summarized below.

* `tf2`: remove `signals` in Boost components.
* `cv_bridge`: replace `python` with `python27`.
* `camera_calibration_parsers`: replace `python` with `python27`.
* `gazebo_ros_packages`: add a `CATKIN_IGNORE` in this directory since gazebo won't be used.

## Carla 0.9.6

**Carla is still under heavy development. Although we will try to, we may not always use the latest version of Carla.**

Follow the steps [here](https://carla.readthedocs.io/en/0.9.6/how_to_build_on_linux/) to build Carla from source. A few things to note,

* Clone the Carla reponsitory into `~/carla_ws`.
* Use the 0.9.6 tag instead of the master branch.
* Once the head of the repository is switched to 0.9.6, apply the [patch](scripts/road_map.patch) to unlock a hidden map interface, which the simulator uses to find more road information. The patch can be applied with `git apply road_map.patch` at the root of carla repository.
*  Test Carla as given [here](https://carla.readthedocs.io/en/0.9.6/getting_started/) to make sure it has been correctly built.

## Conformal Lattice Planner Workspace

Before setting up the conformal lattice planner workspace, we have to add a few lines into `.bashrc` to make sure the previously built libraries can be found and linked against.

```
if [ -d /home/$USER/carla_ws ]; then
  # You may already have this line while compling Carla.
  export UE4_ROOT=/home/ke/Downloads/UnrealEngine

  # Set up the Carla directories.
  export Carla_ROOT=/home/$USER/carla_ws/carla
  export Carla_DIST=${Carla_ROOT}/Dist/CARLA_Shipping_0.9.6-16-gd1d9174a-dirty/LinuxNoEditor
  export Carla_INCLUDE_DIRS=${Carla_ROOT}/Examples/CppClient/libcarla-install/include
  export Carla_LIB_DIRS=${Carla_ROOT}/Examples/CppClient/libcarla-install/lib

  # Python and Cpp libraries.
  export PYTHONPATH="$PYTHONPATH:${Carla_DIST}/PythonAPI/carla/dist/carla-0.9.6-py2.7-linux-x86_64.egg"
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${Carla_LIB_DIRS}:/home/$USER/carla_ws/boost_1_69_0/lib:/home/$USER/carla_ws/pcl_1_9_1/lib"
fi
```

Finally we are able to set up the conformal lattice planner workspace. Instead of cloning this code repository into `carla_ws`, we will create another workspace, `self_driving_ws`, extending `carla_ws`. Hence, `carla_ws` will contain dependency files only.

```
mkdir -p ~/self_driving_ws/src
catkin init --workspace self_driving_ws
cd self_driving_ws
catkin config --extend /home/$USER/carla_ws/devel
catkin config -DBOOST_ROOT=/home/$USER/carla_ws/boost_1_69_0 \
              -DBoost_NO_SYSTEM_PATHS=ON \
              -DBOOST_INCLUDEDIR=/home/$USER/carla_ws/boost_1_69_0/include \
              -DBOOST_LIBRARYDIR=/home/$USER/carla_ws/boost_1_69_0/lib \
              -DCMAKE_BUILD_TYPE=Release \
              -DPCL_DIR=/home/$USER/carla_ws/pcl_1_9_1/share/pcl-1.9
cd src
git clone link_to_this_repository
catkin build
```




