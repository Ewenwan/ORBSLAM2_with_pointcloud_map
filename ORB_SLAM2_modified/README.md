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

> 1. 参数配置文件多了一个 点云地图精度参数

     PointCloudMapping.Resolution: 0.01
> 2. 二进制方式载入特征词典函数

     /Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h  1450-1460行

     二进制方式载入特征词典 bool TemplatedVocabulary<TDescriptor,F>::loadFromBinaryFile(const std::string &filename) {}
     二进制方式保存特征词典 void TemplatedVocabulary<TDescriptor,F>::saveToBinaryFile(const std::string &filename) const {}

> 3. g2o版本问题提示，需要 存储矩阵为列优先存储的版本

    /Thirdparty/g2o/config.h

> 4. 



