#include "YOLOV5s.h"
#include <string>
#include <cstring>
#include <cstdlib>
#include <stdio.h>
#include <QPainter>
#include <QPen>
extern bool Running;
bool Waiting = false;
YOLOV5s::YOLOV5s(QObject* parent):QThread(parent)
{
    mpp_dec = new MppDecoder(this);
    connect(mpp_dec,&MppDecoder::UpdateImage_signal,this,&YOLOV5s::UpdateImage);
}
YOLOV5s::~YOLOV5s()
{
    mpp_dec->quit();
    mpp_dec->wait();

}
int YOLOV5s::init_rknn(RKNN_Context* app_ctx,std::string modelpath,std::string cocolist)
{
    int ret;
    rknn_context ctx;
    printf("Loading mode...\n");
    int model_data_size = 0;
    //加载模型
    unsigned char* model_data = read_file_data(modelpath.c_str(), &model_data_size);
    if (model_data == NULL) 
    {
        return -1;
    }
    //初始化rknn_context 
    ret = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
    if (ret < 0) {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }

    if (model_data) {
        free(model_data);
    }
    //设置NPU核心数
    ret = rknn_set_core_mask(ctx,RKNN_NPU_CORE_0_1_2);
    if(ret<0)
    {
        printf("rknn_set_core_mask failed\n");
        return -1;
    }
    //查询SDK 版本
    rknn_sdk_version version;
    ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
    if (ret < 0) {
        printf("rknn_query RKNN_QUERY_SDK_VERSION error ret=%d\n", ret);
        return -1;
    }
    printf("sdk version: %s driver version: %s\n", version.api_version, version.drv_version);
    //查询模型输入输出信息
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &app_ctx->io_num, sizeof(rknn_input_output_num));
    if (ret < 0) {
        printf("rknn_query RKNN_QUERY_IN_OUT_NUM error ret=%d\n", ret);
        return -1;
    }
    printf("model input num: %d, output num: %d\n", app_ctx->io_num.n_input, app_ctx->io_num.n_output);

    rknn_tensor_attr* input_attrs = (rknn_tensor_attr*)malloc(app_ctx->io_num.n_input * sizeof(rknn_tensor_attr));
    memset(input_attrs, 0, sizeof(input_attrs));
    for (int i = 0; i < app_ctx->io_num.n_input; i++) {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret < 0) {
        printf("rknn_query RKNN_QUERY_INPUT_ATTR error ret=%d\n", ret);
        return -1;
        }
        dump_tensor_attr(&(input_attrs[i]));
    }

    rknn_tensor_attr* output_attrs = (rknn_tensor_attr*)malloc(app_ctx->io_num.n_output * sizeof(rknn_tensor_attr));
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < app_ctx->io_num.n_output; i++) {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret < 0) {
        printf("rknn_query RKNN_QUERY_OUTPUT_ATTR error ret=%d\n", ret);
        return -1;
        }
        dump_tensor_attr(&(output_attrs[i]));
    }

    app_ctx->input_attrs = input_attrs;
    app_ctx->output_attrs = output_attrs;
    app_ctx->rknn_ctx = ctx;

    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
        printf("model is NCHW input fmt\n");
        app_ctx->model_channel = input_attrs[0].dims[1];
        app_ctx->model_height  = input_attrs[0].dims[2];
        app_ctx->model_width   = input_attrs[0].dims[3];
    } else {
        printf("model is NHWC input fmt\n");
        app_ctx->model_height  = input_attrs[0].dims[1];
        app_ctx->model_width   = input_attrs[0].dims[2];
        app_ctx->model_channel = input_attrs[0].dims[3];
    }
    printf("model input height=%d, width=%d, channel=%d\n", app_ctx->model_height, app_ctx->model_width, app_ctx->model_channel);
        float low_conf_threshold = 0.2;
    float high_conf_threshold = 0.5;
    float iouThreshold = 0.25;
    int img_width = 640;
    int img_height = 640;
    int frame_rate = 25;
    //初始化跟踪器
    app_ctx->bytetracker = new BYTETracker(frame_rate, low_conf_threshold, high_conf_threshold);
    return 0;
}
void YOLOV5s::run()
{
    mpp_dec->start();
    RKNN_Context app_ctx;
    memset(&app_ctx,0,sizeof(app_ctx));
    std::string model_path = "../3rdparty/rknn/model/yolov5s-640-640.rknn";
    std::string coco_list = "../3rdparty/rknn/model/coco_80_labels_list.txt";
    int ret = 0;
    ret = init_rknn(&app_ctx,model_path,coco_list);
    if(ret<0)
    {
        printf("init_rknn failed\n");
        return;
    }
    while(Running)
    {
        inference_model(&app_ctx);
    }
    ret = rknn_destroy(app_ctx.rknn_ctx);
    if(ret<0)
    {
        printf("rknn_destroy failed\n");
        return;
    }
    free(app_ctx.input_attrs);
    free(app_ctx.output_attrs);
    delete app_ctx.bytetracker;
}

unsigned char* YOLOV5s::read_file_data(const char* filename, int* model_size)
{
  FILE* fp;
  unsigned char* data;

  fp = fopen(filename, "rb");
  if (NULL == fp) {
    printf("Open file %s failed.\n", filename);
    return NULL;
  }

  fseek(fp, 0, SEEK_END);
  int size = ftell(fp);

  data = load_data(fp, 0, size);

  fclose(fp);

  *model_size = size;
  return data;
}

void YOLOV5s::dump_tensor_attr(rknn_tensor_attr* attr)
{
  printf("index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
         "zp=%d, scale=%f\n",
         attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
         attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
         get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

unsigned char* YOLOV5s::load_data(FILE* fp, size_t ofst, size_t sz)
{
  unsigned char* data;
  int ret;

  data = NULL;

  if (NULL == fp) {
    return NULL;
  }

  ret = fseek(fp, ofst, SEEK_SET);
  if (ret != 0) {
    printf("blob seek failure.\n");
    return NULL;
  }

  data = (unsigned char*)malloc(sz);
  if (data == NULL) {
    printf("buffer malloc failure.\n");
    return NULL;
  }
  ret = fread(data, 1, sz, fp);
  return data;
}

void YOLOV5s::inference_model(RKNN_Context* app_ctx)
{
    if(!img_list.isEmpty())
    {
        mutex.lock();
        QImage img = img_list.dequeue();
        mutex.unlock();
        rknn_context ctx = app_ctx->rknn_ctx;
        int model_width = app_ctx->model_width;
        int model_height = app_ctx->model_height;
        int model_channel = app_ctx->model_channel;
        const float    nms_threshold      = NMS_THRESH;
        const float    box_conf_threshold = BOX_THRESH;
        
        float scale_w = (float)model_width / img.width();
        float scale_h = (float)model_height / img.height();

        rknn_input inputs[1];
        memset(inputs, 0, sizeof(inputs));
        inputs[0].index        = 0;
        inputs[0].type         = RKNN_TENSOR_UINT8;
        inputs[0].size         = model_width * model_height * model_channel;
        inputs[0].fmt          = RKNN_TENSOR_NHWC;
        inputs[0].pass_through = 0;

        inputs[0].buf = img.bits();


        int ret = rknn_inputs_set(ctx, app_ctx->io_num.n_input, inputs);
        if(ret<0)
        {
            printf("rknn_inputs_set error:%d\n",ret);
            return;
        }
        rknn_output outputs[app_ctx->io_num.n_output];
        memset(outputs, 0, sizeof(outputs));
        for (int i = 0; i < app_ctx->io_num.n_output; i++) {
            outputs[i].want_float = 0;
        }

        ret = rknn_run(ctx, NULL);
        if(ret<0)
        {
            printf("rknn_run error:%d\n",ret);
            return;
        }
        ret = rknn_outputs_get(ctx, app_ctx->io_num.n_output, outputs, NULL);
        if(ret<0)
        {
            printf("rknn_output_get error:%d\n",ret);
            return;
        }
        //printf("post process config: box_conf_threshold = %.2f, nms_threshold = %.2f\n", box_conf_threshold, nms_threshold);
        std::vector<float> out_scales;
        std::vector<int32_t> out_zps;
        for (int i = 0; i < app_ctx->io_num.n_output; ++i) {
            out_scales.push_back(app_ctx->output_attrs[i].scale);
            out_zps.push_back(app_ctx->output_attrs[i].zp);
        }
        detect_result_group_t detect_result;
        //printf("outputs[0] size:%d ,outputs[1] size:%d ,outputs[2] size:%d\n",outputs[0].size,outputs[1].size,outputs[2].size);
        post_process((int8_t*)outputs[0].buf, (int8_t*)outputs[1].buf, (int8_t*)outputs[2].buf, model_height, model_width,
                    box_conf_threshold, nms_threshold, scale_w, scale_h, out_zps, out_scales, &detect_result);
        ret = rknn_outputs_release(ctx, app_ctx->io_num.n_output, outputs);  
        if(ret<0)
        {
            printf("rknn_outputs_release error:%d\n",ret);
            return;
        }
        if(1)
        {
            // gen vector<Detection>
            std::vector<Detection> detection;
            detection = get_frame_detection(&detect_result);
            std::vector<STrack> tracked_stracks = app_ctx->bytetracker->update(detection);
            for (auto& track : tracked_stracks) {
                int x1 = (int)track.tlwh()[0];
                int y1 = (int)track.tlwh()[1];
                int x2 = (int)(x1 + track.tlwh()[2]);
                int y2 = (int)(y1 + track.tlwh()[3]);
                QPainter painter(&img);
                painter.setPen(QColor(Qt::red));
                painter.drawRect(x1,y1,x2-x1,y2-y1);
                painter.setPen(QColor(Qt::white));
                QString qtext = QString::number(track.track_id);
                painter.drawText(x1,y1,qtext);
            }
        }
        else
        {
            char text[256];
            for (int i = 0; i < detect_result.count; i++) {
                detect_result_t* det_result = &(detect_result.results[i]);
                printf("%s @ (%d %d %d %d) %f\n", det_result->name,det_result->box.left, det_result->box.top,
                    det_result->box.right, det_result->box.bottom, det_result->prop);
                sprintf(text, "%s %.1f%%", det_result->name, det_result->prop * 100);
                int x1 = det_result->box.left;
                int y1 = det_result->box.top;
                int x2 = det_result->box.right;
                int y2 = det_result->box.bottom;
                
                QPainter painter(&img);
                painter.setPen(QColor(Qt::red));
                painter.drawRect(x1,y1,x2-x1,y2-y1);
                painter.setPen(QColor(Qt::white));
                QString qtext = QString(text);
                painter.drawText(x1,y1,qtext);
            }
        }
        emit UpdateImage_signal(img);
        return ;
    }
}

std::vector<Detection> YOLOV5s::get_frame_detection(detect_result_group_t* detect_result)
{
    std::vector<Detection> dets;
    for (int i = 0; i < detect_result->count; i++) {
            detect_result_t* det_result = &(detect_result->results[i]);
            printf("%s @ (%d %d %d %d) %f\n", det_result->name,det_result->box.left, det_result->box.top,
                det_result->box.right, det_result->box.bottom, det_result->prop);
            //sprintf(text, "%s %.1f%%", det_result->name, det_result->prop * 100);
            int x1 = det_result->box.left;
            int y1 = det_result->box.top;
            int x2 = det_result->box.right;
            int y2 = det_result->box.bottom;
            Detection detection(x1,y1,x2-x1,y2-y1,det_result->class_id,det_result->prop);
            dets.push_back(detection);
        }

    return dets;
}

void YOLOV5s::UpdateImage(QImage image)
{
    if(mutex.tryLock())
    {
        if(!image.isNull())
        {
            QImage background_img(640,640,image.format());
            background_img.fill(QColor(Qt::black));
            QPainter painter(&background_img);
            const int offsetX = 0;
            const int offsetY = (640-360)/2;
            painter.drawImage(offsetX,offsetY,image);
            img_list.enqueue(background_img);
            //printf("img list size:%d\n",img_list.size());
            if(img_list.size()>200)
            {
                Waiting = true;
            }
            else
            {
                Waiting = false;
            }
        }
        mutex.unlock();
    }
    
}