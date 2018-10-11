# Update for unpacked ROB_SLAM with pcl view repo

## build:

### build the whole project ( inclouding binary loading tools ):

Before all the cmd, **DONOT** forget to download the Vocabulary form the [origin repo](https://github.com/raulmur/ORB_SLAM2) and place it into dir ./Vocabulary

```bash
     cd YourDirectory/ORBSLAM2_with_pointcloud_map
     chmod +x build.sh
     ./build.sh
```

### only build the ORB_SLAM2 mode with pcl

```bash
    cd YourDirectory/ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified
    mkdir build
    cd build
    cmake ..
    make -j
```

## Run:

```bash
    ./run/rgbd_tum Vocabulary/ORBvoc.bin path_to_settings path_to_sequence path_to_association
```

# What are modified:

* 1. 以二进制方式载入特征词典

* 2. 添加一个点云可视化器(增加一个可视化线程 viewer thread) 

* 3. 编译方式修改，修改 CMakeLists.txt 所以的可执行文件放入 ./bin

# 修改细节



