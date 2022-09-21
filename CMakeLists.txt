##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(mul_t)

## add
set(CMAKE_INCLUDE_CURRENT_DIR ON)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
    qt_build
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
    )
find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

# aux_source_directory会查找指定目录下的所有源文件，并将结果存入指定变量名
# 这里是将当前目录下的所有源文件存进变量SRC_LIST
aux_source_directory(. SRC_LIST)

# 设置ffmpeg依赖库及头文件所在目录，并存进指定变量
set(ffmpeg_libs_DIR /usr/lib/x86_64-linux-gnu/)
set(ffmpeg_headers_DIR /home/myn/libs/include)

#对于find_package找不到的外部依赖库，可以用add_library添加
# SHARED表示添加的是动态库
# IMPORTED表示是引入已经存在的动态库
add_library( avcodec SHARED IMPORTED )

#指定所添加依赖库的导入路径
set_target_properties( avcodec PROPERTIES IMPORTED_LOCATION ${ffmpeg_libs_DIR}/libavcodec.so )
 
add_library( avfilter SHARED IMPORTED )
set_target_properties( avfilter PROPERTIES IMPORTED_LOCATION ${ffmpeg_libs_DIR}/libavfilter.so )
 
add_library( swresample SHARED IMPORTED )
set_target_properties( swresample PROPERTIES IMPORTED_LOCATION ${ffmpeg_libs_DIR}/libswresample.so )
 
add_library( swscale SHARED IMPORTED )
set_target_properties( swscale PROPERTIES IMPORTED_LOCATION ${ffmpeg_libs_DIR}/libswscale.so )
 
add_library( avformat SHARED IMPORTED )
set_target_properties( avformat PROPERTIES IMPORTED_LOCATION ${ffmpeg_libs_DIR}/libavformat.so )
 
add_library( avutil SHARED IMPORTED )
set_target_properties( avutil PROPERTIES IMPORTED_LOCATION ${ffmpeg_libs_DIR}/libavutil.so )

## add
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
# rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/mul_t/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(mul_t ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(mul_t ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
                        avcodec avformat avutil swresample swscale swscale avfilter)
install(TARGETS mul_t RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

