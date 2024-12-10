# RK3588_RKNN_MPP_RGA_demo
## 在RK3588上使用进行目标识别+跟踪  
目标识别使用官方修改的yolov5模型，ByteTrack跟踪器参考https://github.com/Handsome36233/bytetrack_cpp.git  
解码使用Mpp，图形预处理使用rga，界面显示用Qt5，rtsp拉流使用Zlmediakit,后处理实在cpu上实现  
## 可以改进的地方：  
### 解码：  
目前通过dma_buf共享dma32_heap内存，如果使用DRM显存，import phy_addr速度可能会更快，可以实现零拷贝显示  
### 推理：  
目前是单线程推理，可修改为多线程推理，提高NPU利用率  
### 显示：  
使用OpenGL ES提高效率，利用RK3588的Mali GPU  
### 跟踪：  
ByteTrack源码也许可以再优化，提高效率
