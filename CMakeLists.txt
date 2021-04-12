# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )
# 声明一个 cmake 工程
project( delaunary )
# 设置编译模式
set( CMAKE_BUILD_TYPE "Release" )
#如果不需要指定OpenCV版本，代码如下
find_package(OpenCV REQUIRED)

#添加OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})

#显示OpenCV_INCLUDE_DIRS的值
message(${OpenCV_INCLUDE_DIRS})

find_package(dlib REQUIRED)
include_directories(${dlib_INCLUDE_DIRS})

set(SOURCES
	Dlib.cpp
    Test.cpp
    main.cpp
)
# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable( delaunary ${SOURCES})

# 将库文件链接到可执行程序上
target_link_libraries(${PROJECT_NAME} ${dlib_LIBRARIES}  ${OpenCV_LIBS})