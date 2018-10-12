# ORBSLAM2_with_pointcloud_map
This is a modified ORB_SLAM2 (from https://github.com/raulmur/ORB_SLAM2, thanks for Raul's great work!) with a online point cloud map module running in RGB-D mode. You can visualize your point cloud map during the SLAM process. 

# How to Install
Unzip the file you will find two directories. First compile the modified g2o:

```
  cd g2o_with_orbslam2
  mkdir build
  cd build
  cmake ..
  make 
```

Following the instructions from the original g2o library: [https://github.com/RainerKuemmerle/g2o] if you have dependency problems. I just add the extra vertecies and edges provided in ORB_SLAM2 into g2o. 

Then compile the ORB_SLAM2. You need firstly to compile the DBoW2 in ORB_SLAM2_modified/Thirdpary, and then the Pangolin module (https://github.com/stevenlovegrove/Pangolin). Finally, build ORB_SLAM2:

```
cd ORB_SLAM2_modified
mkdir build
cd build
cmake ..
make
```

To run the program you also need to download the ORB vocabulary (which is a large file so I don't upload it) in the original ORB_SLAM2 repository.

# Run examples
Prepare a RGBD camera or dataset, give the correct parameters and you can get a ORB SLAM with point cloud maps like the example.jpg in this repo.

# Build the unpacked modified repo 

please see this [README](./ORB_SLAM2_modified/README.md)

# What are modified:

* 1. 以二进制方式载入特征词典

* 2. 添加一个点云可视化器(增加一个可视化线程 viewer thread) ，只用关键帧来建立点云地图

* 3. 编译方式修改，修改 CMakeLists.txt 所以的可执行文件放入 ./bin

# 修改细节

> 1. 参数配置文件多了一个 点云地图精度参数

     PointCloudMapping.Resolution: 0.01        // 体素格滤波对象 参数 体素格子大小
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
         
         // public 类型的 成员变量 多了一个 深度图（原来的深度图为一个局部变量）  106行
         cv::Mat mImDepth; // adding mImDepth member to realize pointcloud view
         
         // protected 类型的 成员变量 多了一个 构建点云地图类的一个共享指针  228行
         // for point cloud viewing
         shared_ptr<PointCloudMapping> mpPointCloudMapping;
         
    c. 构建点云地图类 头文件
         ORB_SLAM2-pc/include/pointcloudmapping.h
```
> 5. System.cc 源文件修改
```c
    // 后缀查找   29行
     #include <time.h>
     // 找后缀，str中是否有 suffix 字符串 可以获取文件的类型
     bool has_suffix(const std::string &str, const std::string &suffix) 
     {
          std::size_t index = str.find(suffix, str.size() - suffix.size());
          return (index != std::string::npos);
     }
     
    // 读取 点云地图精度配置参数      68行
     // for point cloud resolution
     float resolution = fsSettings["PointCloudMapping.Resolution"]; // 配置文件中的参数 0.01
    
    // 两种方式读取 特征字典文件      74行
     if (has_suffix(strVocFile, ".txt"))
          bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);  // txt类型的字典文件
     else
          bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);// bin二进制类型的文件
   
    // 初始化  构建点云地图类 共享智能指针   99行
     // Initialize pointcloud mapping
     mpPointCloudMapping = make_shared<PointCloudMapping>( resolution );
     
    // Tracking 跟踪类 初始化 有变化，多传入一个 构建点云地图类 共享智能指针 mpPointCloudMapping  104行
     mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
         mpMap, mpPointCloudMapping, mpKeyFrameDatabase, strSettingsFile, mSensor);
     
    void System::Shutdown()  // 类退出(关闭)函数
    {
        ...
         mpPointCloudMapping->shutdown(); // 添加一个 构建点云地图类 的 关闭函数
        ...
    }

```
> 6. Tracking.cc 文件修改分析
```c
          // Tracking线程类 构造函数接口修改 46行
          Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
                  KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
                  
          >>>> 修改成 
          Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,     
                  shared_ptr<PointCloudMapping> pPointCloud,
                  KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
                  // 多添加了一个 构建点云地图类的一个共享指针 shared_ptr 作为参数
          
          // public 类型的 成员变量 多了一个 深度图（原来的深度图为一个局部变量） 210行
          cv::Mat imDepth = imD;// 原来的深度图为一个局部变量
          // 改成一个类内部 成员变量
          mImDepth = imD;
          
          // 后面有关的 imDepth 都变成 mImDepth
          if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
               imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);// 转换深度图精度
               
          // 创建当前帧
          mCurrentFrame =    
          Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);


void Tracking::CreateNewKeyFrame() // 创建关键帧 函数
{
     ...
     // insert Key Frame into point cloud viewer
     mpPointCloudMapping->insertKeyFrame( pKF, this->mImGray, this->mImDepth );// 在点云地图里插入该关键帧
     ...       
}
```


> 7. pointcloudmapping 类分析
```c
// pointcloudmapping.h 类头文件
using namespace ORB_SLAM2;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT; // 点类型 xyzrgba 点+颜色+透明度
    typedef pcl::PointCloud<PointT> PointCloud;// 点云类型

    PointCloudMapping( double resolution_ );// 类初始化(构造)函数

    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );
    void shutdown();// 相当于类析构函数
    void viewer();  // 可是化点云函数

protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);// 生成点云

    PointCloud::Ptr globalMap; // 点云地图指针
    shared_ptr<thread>  viewerThread;// 点云可视化线程

    bool    shutDownFlag    = false;// 关闭标志
    mutex   shutDownMutex;          // 关闭 线程互斥锁

    condition_variable  keyFrameUpdated;    // 关键帧更新 <condition_variable> 头文件主要包含了与条件变量相关的类和函数。
                                            // 全局条件变量. 用于多线程之间的 相互等待！！！！！！！
    // condition_variable 类 参考 https://msdn.microsoft.com/zh-cn/magazine/hh874752(v=vs.120)
    mutex               keyFrameUpdateMutex;// 关键帧更新  互斥锁

    // data to generate point clouds
    vector<KeyFrame*>       keyframes;  // 关键帧指针 数组
    vector<cv::Mat>         colorImgs;  // 灰度图    数组
    vector<cv::Mat>         depthImgs;  // 深度图    数组
    mutex                   keyframeMutex; // 关键帧 互斥锁
    uint16_t                lastKeyframeSize =0;

    double resolution = 0.04;      // 默认点云地图精度    用于设置体素格子的边长大小
    pcl::VoxelGrid<PointT>  voxel; // 点对应的 体素格滤波对象
};
 

// pointcloudmapping.cc 类实现函数
PointCloudMapping::PointCloudMapping(double resolution_)   // 类构造函数=
{
    this->resolution = resolution_;// 地图精度
    voxel.setLeafSize( resolution, resolution, resolution);// 设置体素格子的边长大小
    globalMap = boost::make_shared< PointCloud >( );       // 点云地图 共享指针

    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );// 可视化 线程 指针 绑定viewer()函数 
    
}

void PointCloudMapping::shutdown() // 类关闭函数，类似 类析构函数
{
    {
        unique_lock<mutex> lck(shutDownMutex);// 执行关闭线程
        shutDownFlag = true;
        keyFrameUpdated.notify_one();// 将等待 keyFrameUpdated 条件变量对象的其中一个线程解除阻塞
    }
    viewerThread->join();// 等待 可视化线程 结束后返回
}

// 地图中插入一个关键帧
void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);// 对关键帧上锁
    keyframes.push_back( kf );            // 关键帧数组 加入一个关键帧
    colorImgs.push_back( color.clone() ); // 图像数组  加入一个 图像  深拷贝
    depthImgs.push_back( depth.clone() ); // 深度数据数组 加入   深拷贝

    keyFrameUpdated.notify_one();         // 关键字更新线程 解除一个阻塞的，来进行关键帧更新
}

// 根据 关键帧中的相机参数 和 像素图(rgb 三色) 和 深度图来计算一帧的点云( 已经变换到 世界坐标系下)========
pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );// 一帧 点云 共享指针
    // point cloud is null ptr  这里需要判断 指针不为 null
    for ( int m=0; m<depth.rows; m+=3 ) // 行   y  (每次读取3个值(rgb三色))
    {
        for ( int n=0; n<depth.cols; n+=3 ) // 列  x
        {
            float d = depth.ptr<float>(m)[n];// 对应深度图处的 深度值
            if (d < 0.01 || d>10) // 跳过合理范围外的深度值
                continue;
            PointT p;
            p.z = d;// z坐标 
            p.x = ( n - kf->cx) * p.z / kf->fx; // (x-cx)*z/fx
            p.y = ( m - kf->cy) * p.z / kf->fy; // (y-cy)*z/fy

            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(p);// 无序点云，未设置 点云长和宽，直接push进入点云地图
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );// 当前关键帧 位姿 四元素表示
    
    PointCloud::Ptr cloud(new PointCloud); // 变换到世界坐标系的点云 
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());// 当前帧下的点云 变换到 世界坐标系下
    cloud->is_dense = false;// 非稠密点云，会有不好的点 nan值等

    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}

// 可视化所有保存的 关键帧形成的 点云
void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer"); // pcl 点云可视化器
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex ); // 关闭锁
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex ); // 关键帧更新锁
            keyFrameUpdated.wait( lck_keyframeUpdated );// 阻塞 关键帧更新锁
            // 需要等待 insertKeyFrame() 函数中完成 添加 关键帧 后，执行后面的!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );// 关键帧锁
            N = keyframes.size();// 当前 保存的 关键帧数量
        }

        for ( size_t i=lastKeyframeSize; i<N ; i++ )// 从上一次已经可视化的关键帧开始 继续向地图中添加点云
        {
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );// 生成新一帧的点云
            *globalMap += *p;// 加入到 总的 点云地图中
        }
        
        PointCloud::Ptr tmp(new PointCloud());// 体素格滤波后的点云
        
        voxel.setInputCloud( globalMap );     // 体素格滤波器 输入原点云
        voxel.filter( *tmp );                 // 滤波后的点云
        globalMap->swap( *tmp );              // 全局点云地图 替换为 体素格滤波后的点云
        
        viewer.showCloud( globalMap );        // 显示 点云
        cout << "show global map, size=" << globalMap->points.size() << endl;
        lastKeyframeSize = N;                 // 迭代更新上次已经更新到的 关键帧id
    }
}


```

> 8. CMakeLists.txt 修改
```c
     SET(CMAKE_EXPORT_COMPILE_COMMANDS "ON") // 使用此选项，cmake会生成一个JSON文件，其中包含包含路径
                                             // 生成一个JSON编译数据库。
                                             // 编译数据库JSON文件是在 cmake 执行时生成的，而不是在 make 编译 时生成。
     
     # adding for point cloud viewer and mapper
     find_package( PCL 1.7 REQUIRED )        // 添加pcl 库
     include_directories(
     ${PROJECT_SOURCE_DIR}
     ${PROJECT_SOURCE_DIR}/include
     ${EIGEN3_INCLUDE_DIR}
     ${Pangolin_INCLUDE_DIRS}
     ${PCL_INCLUDE_DIRS}                     // 库文件 
     )

     add_definitions( ${PCL_DEFINITIONS} )   // 添加 pcl 定义
     link_directories( ${PCL_LIBRARY_DIRS} ) // 添加 pcl 库依赖

     add_library(${PROJECT_NAME} SHARED      // shared 动态链接库
     src/System.cc
     src/Tracking.cc
     src/LocalMapping.cc
     src/LoopClosing.cc
     src/ORBextractor.cc
     src/ORBmatcher.cc
     src/FrameDrawer.cc
     src/Converter.cc
     src/MapPoint.cc
     src/KeyFrame.cc
     src/Map.cc
     src/MapDrawer.cc
     src/Optimizer.cc
     src/PnPsolver.cc
     src/Frame.cc
     src/KeyFrameDatabase.cc
     src/Sim3Solver.cc
     src/Initializer.cc
     src/Viewer.cc
     src/pointcloudmapping.cc               // 添加一个文件  
     )

     target_link_libraries(${PROJECT_NAME}  // 动态库添加 链接
     ${OpenCV_LIBS}
     ${EIGEN3_LIBS}
     ${Pangolin_LIBRARIES}
     ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
     ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
     ${PCL_LIBRARIES}                       // 点云库链接
     )
     
     set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) // 编译后的可执行文件 存放的目录

     # Build tools
     set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)  // 工具 可执行文件存放目录
     add_executable(bin_vocabulary
     tools/bin_vocabulary.cc)
     target_link_libraries(bin_vocabulary ${PROJECT_NAME}) // 链接
```
    
    
    
