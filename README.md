# RK3588_RKNN_MPP_RGA_demo
## 在RK3588上进行目标识别+跟踪  
模型是官方的yolov5s.rknn，ByteTrack跟踪器参考https://github.com/Handsome36233/bytetrack_cpp.git  
解码使用Mpp，图形预处理使用rga，界面显示用Qt5，rtsp拉流使用Zlmediakit,后处理在cpu上实现  

## 依赖  
1.MPP for decode  
2.RGA for preprocecss  
3.RKNN Runtime for inference   
4.Qt5 for GUI   

## 可以改进的地方：    
1.将单线程推理修改为多线程推理，提高NPU利用率  
2.使用NPU零拷贝提高效率  
