#ifndef __MPPDECODER_H
#define __MPPDECODER_H
#include <cstring>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <QObject>
#include <QThread>
#include <QImage>
#include "im2d.hpp"
#include "rga.h"
#include "RgaUtils.h"
#include "im2d_type.h"
#include "rk_mpi.h"
#include "mpp_buffer.h"
#include "mpp_packet.h"
#include "mpp_frame.h"
#include "dma_alloc.h"
#include "mk_mediakit.h"
#include "rknn_api.h"
#define MAX_FILE_NAME_LENGTH        256
#define MPI_DEC_STREAM_SIZE         1024*600
#define MPI_DEC_LOOP_COUNT          4 
class MppDecoder;
typedef struct{
    char *src_buf;
    int src_dma_fd;
    rga_buffer_handle_t src_handle;
    size_t buf_size;
    rga_buffer_t src;
}External_Buffer;

typedef struct{
    MppCtx          ctx;
    MppApi          *mpi;
    RK_U32          eos;
    void            *buf;
    MppBufferGroup  frm_grp;
    MppPacket       packet;
    size_t          packet_size;
    int             src_width,src_height;
    int             resize_width,resize_height;
    RK_S32          frame_count;
    RK_S32          frame_num;
    RK_S32          ret = 0;
    RK_S32          max_usage;
    FILE*           fp_input;

    MppCodingType   type;
    MppFrameFormat  format;
    int             src_format;
    int             dst_format;
    int             dst_dma_fd;
    int             resize_dma_fd;
    char*           dst_buf;
    char*           resize_buf;
    int             dst_buf_size;
    int             resize_buf_size;
    rga_buffer_t    dst_img;
    rga_buffer_t    resize_img;
    rga_buffer_handle_t dst_handle;
    rga_buffer_handle_t resize_handle;
    MppDecoder*             _this;
    mk_player        player;
}DECLoopData;

void Decode(DECLoopData* loopdata,u_int8_t* data,size_t size);
void cvt_YUV_to_RGB(MppFrame frame,DECLoopData* data);
int init_dma_buffer(DECLoopData* data,size_t buf_size); 
class MppDecoder:public QThread
{
    Q_OBJECT
public:
    MppDecoder(QObject* parent = nullptr);
    ~MppDecoder();
    void run();
    void ReleaseAll(DECLoopData* loopdata);   
    
    QString source_file;
    std::map<int,External_Buffer> buffer_map;
    
signals:
    void UpdateImage_signal(QImage);
};

#endif