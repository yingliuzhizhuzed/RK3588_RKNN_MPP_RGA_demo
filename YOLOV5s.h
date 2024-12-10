#ifndef __YOLOV5S_H
#define __YOLOV5S_H
#include <QObject>
#include <QThread>
#include <QImage>
#include <QMutex>
#include <QQueue>
#include <vector>
#include "rknn_api.h"
#include "im2d.hpp"
#include "rga.h"
#include "RgaUtils.h"
#include "im2d_type.h"
#include "dma_alloc.h"
#include "postprocess.h"
#include "ByteTracker.h"
#include "MppDecoder.h"
typedef struct{
    rknn_context            rknn_ctx;
    rknn_sdk_version        sdk_ver;
    rknn_input_output_num   io_num;
    rknn_input              inputs;
    rknn_tensor_attr*        input_attrs;
    rknn_tensor_attr*        output_attrs;
    rknn_output             outputs;
    int model_width           = 0;
    int model_height          = 0;
    int model_channel         = 0;
    BYTETracker*              bytetracker;
}RKNN_Context;

class YOLOV5s:public QThread
{
    Q_OBJECT
public:
    explicit YOLOV5s(QObject* parent = nullptr);
    ~YOLOV5s();
signals:
    void UpdateImage_signal(QImage);
public slots:
    int init_rknn(RKNN_Context* app_ctx,std::string modelpath,std::string cocolist);
    unsigned char* read_file_data(const char* filename, int* model_size);
    void dump_tensor_attr(rknn_tensor_attr* attr);
    unsigned char* load_data(FILE* fp, size_t ofst, size_t sz);
    void run();
    void inference_model(RKNN_Context* app_ctx); 
    std::vector<Detection> get_frame_detection(detect_result_group_t* dect_result);
    void UpdateImage(QImage image);


public:
    QQueue<QImage> img_list;
    QImage last_image;
    QMutex mutex;
    MppDecoder* mpp_dec;
};
#endif