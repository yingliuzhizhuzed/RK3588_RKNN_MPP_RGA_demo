cmake_minimum_required(VERSION 3.5)
project(zlmediakit_demo)
set(CMAKE_CXX_STANDARD 17)
#启动元编译
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
#输出编译指令
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#查找Qt库
find_package(Qt5 COMPONENTS Widgets Gui Core Network REQUIRED)
#设置3rdparty路径
set(3rdparty_DIR                ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty)
set(MK_API_INCLUDE_DIRS         ${3rdparty_DIR}/zlmediakit/include)
set(MK_API_LIBRARIES            ${3rdparty_DIR}/zlmediakit/aarch64/libmk_api.so)
set(ROCKCHIP_RKNN_INCLUDE_DIRS  ${3rdparty_DIR}/rknn/include)
set(ROCKCHIP_RKNN_LIBRARIES     ${3rdparty_DIR}/rknn/aarch64/librknnrt.so)

#设置mpp 和 rga 路径
set(ROCKCHIP_MPP_INCLUDE_DIRS   /usr/include/rockchip)  
set(ROCKCHIP_MPP_LIBRARIES      /usr/lib/aarch64-linux-gnu/librockchip_mpp.so)
set(ROCKCHIP_RGA_INCLUDE_DIRS   /usr/include/rga)
set(ROCKCHIP_RGA_LIBRARIES      /usr/lib/aarch64-linux-gnu/librga.so)


#添加头文件路径
include_directories(    
                        ${MK_API_INCLUDE_DIRS} 
                        ${ROCKCHIP_RKNN_INCLUDE_DIRS}
                        ${ROCKCHIP_RGA_INCLUDE_DIRS}
                        ${ROCKCHIP_MPP_INCLUDE_DIRS}) 
#查找源文件
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

# 如果没有找到文件，显示错误信息并停止配置
if(NOT SRC_FILES)
    message(FATAL_ERROR "No source files found in ${PROJECT_SOURCE_DIR}")
endif()

#生成可执行文件
add_executable(${PROJECT_NAME} ${SRC_FILES})

#链接库文件
target_link_libraries(${PROJECT_NAME} ${MK_API_LIBRARIES} 
                                        ${ROCKCHIP_MPP_LIBRARIES}
                                        ${ROCKCHIP_RGA_LIBRARIES} 
                                        pthread
                                        Qt5::Core Qt5::Widgets Qt5::Gui
                                        ${ROCKCHIP_RKNN_LIBRARIES})