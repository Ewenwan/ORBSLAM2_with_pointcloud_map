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

> 4. 点云可视化器 类 线程 等 头文件修改
```c
    a. ORB_SLAM2-pc/include/System.h  39行  和 180行
          // for point cloud viewing     39行
          #include "pointcloudmapping.h" // 包含 构建点云地图 头文件
          class PointCloudMapping;       // 声明 构建点云地图类
          
          // point cloud mapping        180行
          shared_ptr<PointCloudMapping> mpPointCloudMapping; // 构建点云地图类的一个共享指针 shared_ptr
          
    b. ORB_SLAM2-pc/include/Tracking.h  42行 和 
          // for pointcloud mapping and viewing   42 行
          #include "pointcloudmapping.h" // 包含 构建点云地图 头文件
          class PointCloudMapping;       // 声明 构建点云地图类
          
          // Tracking线程类 构造函数接口修改  63行
          Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
                  KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
          >>>> 修改成 
          Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,     
                  shared_ptr<PointCloudMapping> pPointCloud,
                  KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
                  // 多添加了一个 构建点云地图类的一个共享指针 shared_ptr 作为参数
         
         // public 类型的 成员变量 多了一个 用于点云可视化  106行
         cv::Mat mImDepth; // adding mImDepth member to realize pointcloud view
         
         // protected 类型的 成员变量 多了一个 构建点云地图类的一个共享指针  228行
         // for point cloud viewing
         shared_ptr<PointCloudMapping> mpPointCloudMapping;
         
    c. 构建点云地图类 头文件
         ORB_SLAM2-pc/include/pointcloudmapping.h

> 5. System.cc 源文件修改


> 6. Tracking.cc 源文件修改



> 7. pointcloudmapping.cc 源文件分析



```
    
    
    
