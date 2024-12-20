#include "MppDecoder.h"
#include <stdio.h>
#include <cstring>
#include <unistd.h>
#include <chrono>
extern bool Running;
extern bool Waiting ;
void Decode(DECLoopData* loopdata,u_int8_t* data,size_t pkt_size)
{
    RK_U32 pkt_done = 0;
    RK_U32 err_info = 0;
    MPP_RET ret = MPP_OK;
    MppCtx ctx  = loopdata->ctx;
    MppApi *mpi = loopdata->mpi;
    size_t packet_size = loopdata->packet_size;
    int pkt_eos = 0;
    //loopdata->eos = 0;
    MppFrame  frame  = NULL;
    if(data == NULL && pkt_size ==0)
    {
        size_t read_size = fread(loopdata->buf, 1, loopdata->packet_size, loopdata->fp_input);
        if (read_size != loopdata->packet_size && feof(loopdata->fp_input))
        {
            printf("found last packet\n");
            loopdata->eos = pkt_eos = 1;
        }
        mpp_packet_write(loopdata->packet, 0, loopdata->buf, read_size);
        mpp_packet_set_pos(loopdata->packet, loopdata->buf);
        mpp_packet_set_length(loopdata->packet, read_size);
        if (pkt_eos)
        {
            mpp_packet_set_eos(loopdata->packet);
        }
        if(Waiting)
        {
            
        }
    }
    else{
        mpp_packet_write(loopdata->packet, 0, data, pkt_size);
        mpp_packet_set_pos(loopdata->packet, data);
        mpp_packet_set_length(loopdata->packet, pkt_size);
    }

    do{
        RK_U32 frm_eos = 0;
        if (!pkt_done)
        {
            ret = loopdata->mpi->decode_put_packet(loopdata->ctx, loopdata->packet);
            if (MPP_OK == ret)
            {
                pkt_done = 1;
                //printf("%s , %d\n",__FUNCTION__,__LINE__);
            }
        }
        do{
            RK_S32 get_frm = 0;
            RK_U32 frm_eos = 0;
            ret = mpi->decode_get_frame(ctx, &frame);
            if(frame)
            {
                if(mpp_frame_get_info_change(frame))
                {
                    RK_U32 width = mpp_frame_get_width(frame);
                    RK_U32 height = mpp_frame_get_height(frame);
                    RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
                    RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
                    RK_U32 buf_size = mpp_frame_get_buf_size(frame);   
                    printf("decoder require buffer w:h [%d:%d] stride [%d:%d] buf_size %d\n", width, height, hor_stride, ver_stride, buf_size);       
                    loopdata->src_width = hor_stride;
                    loopdata->src_height = ver_stride;
                    if (NULL == loopdata->frm_grp)
                    {                 
                         //纯外部分配模式    分配DMA_HEAP_DMA32_UNCACHED_PATH  内存
                        ret = mpp_buffer_group_get_external(&loopdata->frm_grp, MPP_BUFFER_TYPE_DMA_HEAP);
                        if (ret)
                        {
                            printf("get mpp buffer group failed ret %d\n", ret);
                            break;
                        }   
                        int drm_ret = init_dma_buffer(loopdata,buf_size);
                        if(drm_ret<0)
                        {
                            printf("alloc dma_buffer failed!\n");
                            return;
                        }             
                        ret = loopdata->mpi->control(loopdata->ctx, MPP_DEC_SET_EXT_BUF_GROUP, loopdata->frm_grp);
                        if (ret)
                        {
                            printf("set buffer group failed ret %d\n", ret);
                            break;
                        }
                    }
                    else
                    {                     
                        ret = mpp_buffer_group_clear(loopdata->frm_grp);
                        if (ret)
                        {
                            printf("clear buffer group failed ret %d\n", ret);
                            break;
                        }
                    }
                     ret = mpp_buffer_group_limit_config(loopdata->frm_grp, buf_size, 24);
                    if (ret)
                    {
                        printf("limit buffer group failed ret %d\n", ret);
                        break;
                    }                  
                    ret = loopdata->mpi->control(loopdata->ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
                    if (ret)
                    {
                        printf("info change ready failed ret %d\n", ret);
                        break;
                    }

                }
                else
                {
                    err_info = mpp_frame_get_errinfo(frame) | mpp_frame_get_discard(frame);
                    if (err_info)
                    {
                        printf("decoder_get_frame get err info:%d discard:%d.\n", mpp_frame_get_errinfo(frame), mpp_frame_get_discard(frame));
                    }
                    loopdata->frame_count++;
                    //printf("decode_get_frame get frame %d\n", loopdata->frame_count);
                    if (!err_info)
                    {
                        cvt_YUV_to_RGB(frame,loopdata);

                    }
                }
                frm_eos = mpp_frame_get_eos(frame);
                mpp_frame_deinit(&frame);
                frame = NULL;
                get_frm = 1;
            }
            if (pkt_eos&& pkt_done && !frm_eos)
            {
                usleep(1000);
                //printf("%s,%d\n",__FUNCTION__,__LINE__);
                continue;
            }
            if (frm_eos)
            {
                printf("found last frame\n");
                break;
            }
            if ((loopdata->frame_num > 0 && (loopdata->frame_count >= loopdata->frame_num)) ||
            ((loopdata->frame_num == 0) && frm_eos)) 
            {
                loopdata->eos = 1;
                break;
            }
            if (get_frm)
            {
                continue;
            }
            break;
            
        }while(1);
         if ((loopdata->frame_num > 0 && (loopdata->frame_count >= loopdata->frame_num)) ||
            ((loopdata->frame_num == 0) && frm_eos)) 
        {
            loopdata->eos = 1;
            printf("reach max frame number %d\n", loopdata->frame_count);
            break;
        }
        if (pkt_done)
        {
            break;
        }
    }while(1);
}

void cvt_YUV_to_RGB(MppFrame frame,DECLoopData* data)
{

    int    ret      = 0; 
    MppBuffer buffer = mpp_frame_get_buffer(frame);
    int src_fd = mpp_buffer_get_fd(buffer);
    External_Buffer ex_buf = data->_this->buffer_map[src_fd];
    ret = imcvtcolor(ex_buf.src, data->dst_img, data->src_format, data->dst_format);
    ret = imresize(data->dst_img,data->resize_img);
    if (ret == IM_STATUS_SUCCESS) {
        QImage image = QImage((uchar*)data->resize_buf,data->resize_width,data->resize_height,data->resize_width*3,QImage::Format_RGB888);
        emit data->_this->UpdateImage_signal(image);
    } else {
        printf("running failed, %s\n", imStrError((IM_STATUS)ret));
        data->eos = 1;
    }
}
void API_CALL on_track_frame_out(void* user_data, mk_frame frame) {
    DECLoopData* loopdata = static_cast<DECLoopData*>(user_data);
    const char* data = mk_frame_get_data(frame);
    size_t size = mk_frame_get_data_size(frame);
    if (data) {
        Decode(loopdata,(uint8_t*)data,size);
    }
}
void API_CALL on_mk_play_event_func(void *user_data, int err_code, const char *err_msg, mk_track tracks[],int track_count)
{
    DECLoopData *loopdata = (DECLoopData*)user_data;
    if (err_code == 0) {
      //success
      printf("play success------------------------!\n");
      int i;
      for (i = 0; i < track_count; ++i) {
          if (mk_track_is_video(tracks[i])) {
              log_info("got video track: %s", mk_track_codec_name(tracks[i]));
              //监听track数据回调
              mk_track_add_delegate(tracks[i], on_track_frame_out, user_data);
          }
      }
  } else {
      printf("play failed: %d %s", err_code, err_msg);
  }
}
void API_CALL on_mk_shutdown_func(void *user_data, int err_code, const char *err_msg, mk_track tracks[], int track_count)
{
    printf("play interrupted: %d %s", err_code, err_msg);
}

MppDecoder::MppDecoder(QObject* parent):QThread(parent)
{

}
void MppDecoder::run()
{
    //Mpp init
    DECLoopData loopdata;
    memset(&loopdata, 0, sizeof(loopdata));
    loopdata._this = this;
    RK_U32 pkt_done = 0;
    RK_U32 err_info = 0;
    MPP_RET ret = MPP_OK;
    loopdata.type = MPP_VIDEO_CodingAVC;
    loopdata.packet_size = MPI_DEC_STREAM_SIZE;
    loopdata.src_format = RK_FORMAT_YCbCr_420_SP;
    loopdata.dst_format = RK_FORMAT_RGB_888;
    loopdata.resize_width = 640;
    loopdata.resize_height = 360;
    loopdata.frame_num = 0;
    MppParam param  = NULL;
    RK_U32 need_split   = 1;


    //Mpp settings
    ret = mpp_create(&loopdata.ctx, &loopdata.mpi);
    param = &need_split;
    ret = loopdata.mpi->control(loopdata.ctx, MPP_DEC_SET_PARSER_SPLIT_MODE, param);
    if (MPP_OK != ret)
    {
        printf("mpi->control failed\n");
        ReleaseAll(&loopdata);
        return;
    }
    
    ret = mpp_init(loopdata.ctx, MPP_CTX_DEC,loopdata.type);
    if (MPP_OK != ret)
    {
        printf("mpp_init failed\n");
        ReleaseAll(&loopdata);
        return;
    }
    

    loopdata.buf = malloc(loopdata.packet_size);
    if(!loopdata.buf)
    {
        printf("malloc failed\n");
        ReleaseAll(&loopdata);
        return;
    }
    ret = mpp_packet_init(&loopdata.packet, loopdata.buf, loopdata.packet_size);
        if (MPP_OK != ret)
    {
        printf("MppPacket_init failed\n");
        ReleaseAll(&loopdata);
        return;
    }
    if(source_file.contains(".txt"))
    {
        //zlmediakit settings
        mk_config config;
        memset(&config, 0, sizeof(mk_config));
        config.log_mask = LOG_CONSOLE;
        mk_env_init(&config);
        loopdata.player = mk_player_create();

        //读取rtsp地址
        std::string filename_s = source_file.toStdString();
        FILE* f_rtsp = fopen(filename_s.c_str(), "rb");
        if (NULL == f_rtsp)
        {
            printf("failed to open input file %s\n",filename_s.c_str() );
            ReleaseAll(&loopdata);
        }

        char read_buffer[1024];
        while (fgets(read_buffer, 1024, f_rtsp) != NULL)
        {
            printf("rtsp addr:%s\n", read_buffer);
        }      
        fclose(f_rtsp);
        mk_player_set_on_result(loopdata.player, on_mk_play_event_func,&loopdata);
        mk_player_set_on_shutdown(loopdata.player, on_mk_shutdown_func,&loopdata);
        mk_player_play(loopdata.player, read_buffer);
        while(Running);
        ReleaseAll(&loopdata);
    }
    else if(source_file.contains(".h264"))
    {
        std::string file_path =source_file.toStdString();
        loopdata.fp_input = fopen(file_path.c_str(),"rb");
        if(!loopdata.fp_input)
        {
            printf("open file error!\n");
            ReleaseAll(&loopdata);
            return;
        }
        while(!loopdata.eos)
        {
            Decode(&loopdata,NULL,0);
        }
    }
    else
    {
        printf("format error!\n");
        ReleaseAll(&loopdata);
    }
}
MppDecoder::~MppDecoder()
{

}

void MppDecoder::ReleaseAll(DECLoopData* loopdata)
{
    if(loopdata->fp_input)
    {
        fclose(loopdata->fp_input);
    }

    if(loopdata->player)
    {
        mk_player_release(loopdata->player);
    }
    if (loopdata->packet)
    {
        mpp_packet_deinit(&loopdata->packet);
        loopdata->packet = NULL;
    }
    if (loopdata->ctx)
    {
        mpp_destroy(loopdata->ctx);
        loopdata->ctx = NULL;
    }
    if (loopdata->buf)
    {
        free(loopdata->buf);
        loopdata->buf = NULL;
    } 
    if (loopdata->frm_grp)
    {
        mpp_buffer_group_put(loopdata->frm_grp);
        loopdata->frm_grp = NULL;
    }
    if (loopdata->dst_handle)
        releasebuffer_handle(loopdata->dst_handle);
    if (loopdata->resize_handle)
        releasebuffer_handle(loopdata->resize_handle);

    dma_buf_free(loopdata->dst_buf_size, &loopdata->dst_dma_fd, loopdata->dst_buf);
    dma_buf_free(loopdata->resize_buf_size, &loopdata->resize_dma_fd, loopdata->resize_buf);
    for (const auto& pair : buffer_map) 
    {
        External_Buffer ext_buf = pair.second;
        if(ext_buf.src_handle>0)
            releasebuffer_handle(ext_buf.src_handle);
        dma_buf_free(ext_buf.buf_size, &ext_buf.src_dma_fd, ext_buf.src_buf);
    }
    printf("release all\n");
}

int init_dma_buffer(DECLoopData* data,size_t buf_size)
{
    int ret = 0;
    RK_U32 i;
    MppBufferInfo commit;
    data->dst_buf_size = data->src_width * data->src_height * get_bpp_from_format(data->dst_format);
    data->resize_buf_size = data->resize_width*data->resize_height*get_bpp_from_format(data->dst_format);
    ret = dma_buf_alloc(DMA_HEAP_DMA32_UNCACHED_PATH, data->dst_buf_size, &data->dst_dma_fd, (void **)&data->dst_buf);
    if(ret<0)
    {
        printf("can not alloc dma_buf\n");
        return -1;
    }
    ret = dma_buf_alloc(DMA_HEAP_DMA32_UNCACHED_PATH, data->resize_buf_size, &data->resize_dma_fd, (void **)&data->resize_buf);
    if(ret<0)
    {
        printf("can not alloc dma_buf\n");
        return -1;
    }
    data->dst_handle = importbuffer_fd(data->dst_dma_fd,data->dst_buf_size);
    data->resize_handle = importbuffer_fd(data->resize_dma_fd,data->resize_buf_size);
    if (data->dst_handle == 0 || data->resize_handle ==0) 
    {
        printf("import dma_fd error!\n");
        return -1;
    }
    data->dst_img = wrapbuffer_handle(data->dst_handle, data->src_width, data->src_height, data->dst_format);
    data->resize_img = wrapbuffer_handle(data->resize_handle, data->resize_width, data->resize_height, data->dst_format);
    for(int i = 0;i<24;i++)
    {
        External_Buffer ext_buffer;
        memset(&ext_buffer,0,sizeof(External_Buffer));
        ext_buffer.buf_size  = buf_size;
        ret = dma_buf_alloc(DMA_HEAP_DMA32_UNCACHED_PATH, ext_buffer.buf_size, &ext_buffer.src_dma_fd, (void **)&ext_buffer.src_buf);
        if(ret<0)
        {
            printf("can not alloc dma_buf\n");
            return -1;
        }
        ext_buffer.src_handle = importbuffer_fd(ext_buffer.src_dma_fd, ext_buffer.buf_size);
        if (ext_buffer.src_handle == 0) {
            printf("import dma_fd error!\n");
            return -1;
        }
        ext_buffer.src = wrapbuffer_handle(ext_buffer.src_handle, data->src_width, data->src_height, data->src_format);
        commit.type = MPP_BUFFER_TYPE_DMA_HEAP;
        commit.size = ext_buffer.buf_size;
        commit.fd = ext_buffer.src_dma_fd;
        commit.ptr = ext_buffer.src_buf;


        ret = mpp_buffer_commit(data->frm_grp, &commit);
        if (ret) {
            printf("external buffer commit failed ret %d\n", ret);
            return -1;
        }
        
        //printf("dma_buf alloced dam_fd:%d, buf_size:%d , commited fd:%d\n",ext_buffer.src_dma_fd,ext_buffer.buf_size,commit.fd);
        data->_this->buffer_map[commit.fd] = ext_buffer;
    }
}
