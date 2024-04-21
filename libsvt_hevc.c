/*
* Scalable Video Technology for HEVC encoder library plugin
*
* Copyright (c) 2018 Intel Corporation
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*/

//需要在顶层父目录的CMakeLists:             
//          1.              55行 SET(FFMPEG_FLAG 处添加 --enable-libsvthevc 
//          2.              (--enable-hevctile指令用不了，只能参考代码在libsvthevc上改吧)
//          3.              83行 COMMAND export PKG_CONFIG_PATH 添加 /usr/local/lib64/pkgconfig                   

// 项目自带的代码没法结束程序，换成了svt仓库里提供的代码（两个不一样）

#include "EbApi.h"

#include "libavutil/common.h"
#include "libavutil/frame.h"
#include "libavutil/opt.h"
#include "libavutil/fifo.h"

#include "internal.h"
#include "avcodec.h"

#include "../360SCVP/360SCVPTiledstreamAPI.h"
#include <pthread.h>
#include <unistd.h>

#include <sys/time.h>

#include <math.h>

// #define MAX_TILES 256
// #define FIFO_SIZE 8024

#define MAX_TILES 16
#define FIFO_SIZE 1024

// 需要手动补充头部的VPS, SPS, PPS, SEI信息，以满足omef-packing的流要求

//!
//! \struct: Headers
//! \brief:  headers of output stream
//! \details: VPS, SPS, PPS, SEI information of output stream
//!
typedef struct HEADERS
{
    unsigned char *headerData;  //uint8_t 
    long int  headerSize;   //int64_t
}Headers;

// 投影格式,已经包含在360SCVPTiledstreamAPI.h内了
/*
typedef enum H265SEIType
{
    E_FRAMEPACKING_ARRANGEMENT = 45,
    E_EQUIRECT_PROJECTION = 150,
    E_CUBEMAP_PROJECTION,
    E_SPHERE_ROTATION = 154,
    E_REGIONWISE_PACKING,
    E_OMNI_VIEWPORT,
    E_NOVEL_VIEW_GENERATION = 182
}H265SEIType;
*/

// NAL第四块的SEI信息封装，150替换为投影格式
int nal4[] = {
    0, 0, 0, 1, 78, 1, 150, 0, 68
};

// 直接复制的distributed_encoder现成头部
int head1[112] = {
    0, 0, 0, 1, 64, 1, 12, 6, 255, 255, 
    2, 32, 0, 0, 3, 0, 128, 0, 0, 3, 
    0, 0, 3, 0, 93, 0, 0, 8, 17, 3, 
    0, 0, 3, 0, 1, 0, 0, 3, 0, 30, 
    80, 0, 0, 0, 1, 66, 1, 6, 2, 32, 
    0, 0, 3, 0, 128, 0, 0, 3, 0, 0, 
    3, 0, 93, 0, 0, 160, 2, 0, 128, 40, 
    22, 52, 32, 70, 73, 27, 9, 192, 0, 252, 
    0, 0, 3, 0, 4, 0, 0, 3, 0, 120, 
    32, 0, 0, 0, 1, 68, 1, 192, 113, 196, 
    148, 228, 128, 0, 0, 0, 1, 78, 1, 150, 
    0, 68
};

int head2[113] = {
    0, 0, 0, 1, 64, 1, 12, 6, 255, 255, 
    2, 32, 0, 0, 3, 0, 128, 0, 0, 3, 
    0, 0, 3, 0, 180, 0, 0, 4, 2, 16, 
    48, 0, 0, 3, 0, 16, 0, 0, 3, 1, 
    229, 0, 0, 0, 1, 66, 1, 6, 2, 32, 
    0, 0, 3, 0, 128, 0, 0, 3, 0, 0, 
    3, 0, 180, 0, 0, 160, 1, 224, 32, 7, 
    129, 99, 65, 0, 134, 73, 27, 9, 192, 0, 
    252, 0, 0, 3, 0, 4, 0, 0, 3, 0, 
    120, 32, 0, 0, 0, 1, 68, 1, 192, 113, 
    196, 48, 211, 146, 0, 0, 0, 1, 78, 1, 
    150, 0, 68
};

struct SvtContext;

// 多线程结构体///////////////////////////////////////////////////////////////////////

typedef enum SLICE_TYPE {
    SLICE_B = 0,
    SLICE_P = 1,
    SLICE_I = 2,
    SLICE_IDR = 3,
    INVALID_SLICE = 4
}slice_type;

typedef int (*ENC_CLOSE)(void*);
typedef int (*ENC_INIT)(void*);
typedef int (*ENC_FRAME)(void*, AVPacket*, const AVFrame*, int*);

typedef struct SvtEncoder {
    EB_H265_ENC_CONFIGURATION           enc_params;
    EB_COMPONENTTYPE                    *svt_handle;
    EB_BUFFERHEADERTYPE                 *in_buf;
    EB_BUFFERHEADERTYPE                 *out_buf;
    int                                  raw_size;
} SvtEncoder;

typedef struct TileInfo{
        int            top;
        int            left;
        int            tWidth;
        int            tHeight;
        int            tBitrate;
        int            tMaxrate;
        AVFifoBuffer*  outpkt_fifo;
        int            proc_idx;
        int            eos;
        struct SvtContext*    enc_ctx;         //多线程中实际存储svt实例信息的位置
        AVPacket*      internal_pkt;
} TileInfo;

typedef struct TileEncoderInfo{
    void          *ctx;
    int           tile_idx;
}TileEncoderInfo;

typedef struct EncoderWrapper{
        AVCodecContext* avctx;

        int             width;
        int             height;
        void*           enc_param;

        bool            uniform_split;
        int             tile_num;
        int             tile_w;
        int             tile_h;
        TileInfo        tile_info[MAX_TILES];

        ENC_CLOSE       enc_close;
        ENC_INIT        enc_init;
        ENC_FRAME       enc_frame;

        TileEncoderInfo *tile_enc_info; 

        pthread_t       *tid0;       //接收线程句柄
        pthread_t       *tid1;       //发送线程句柄

        int             initialized;

        void            *pGen;
        param_gen_tiledStream paramTiledStream;

        pthread_mutex_t mutex0;     //接收线程使用的信号量和互斥锁
        pthread_cond_t cond0;

} EncoderWrapper;

typedef enum eos_status {
    EOS_NOT_REACHED = 0,
    EOS_SENT,
    EOS_RECEIVED
}EOS_STATUS;

typedef struct SvtContext {
    AVClass *class;

    EB_H265_ENC_CONFIGURATION enc_params;
    EB_COMPONENTTYPE *svt_handle;   //svt实例句柄
    EB_BUFFERHEADERTYPE in_buf;
    uint8_t *in_data;
    EOS_STATUS eos_flag;

    // User options.
    int width;
    int height;
    int bit_rate;
    int max_rate;

    int profile;
    int hierarchical_level;
    int enc_mode;
    int tier;
    int level;
    int rc_mode;
    int scd;
    int tune;
    int base_layer_switch_mode;
    int qp;
    int aud;
    int asm_type;
    int forced_idr;
    int la_depth;
    int thread_count;
    int target_socket;
    int high_dynamic_range;
    int unrestricted_motion_vector;

    int instance_row_count;
    int instance_col_count;

    int tile_row_count;
    int tile_col_count;

    int tile_slice_mode;
    int pred_struct;
    int vid_info;

    // 头部补充用
    int frame_number;
    int projInfo;
    const char* proj_type;

    // 多线程上下文
    EncoderWrapper encoder_wrapper;
    SvtEncoder  *svt_enc;

    // 多线程标记发送接收的帧数,以及是否发送了eos帧
    int sendCnt;
    int receiveCnt;

    int send_eos_frame;

    // 标记发送线程是否完成当帧发送
    int sendFlag;                

    pthread_mutex_t mutex1;     //发送线程使用的信号量和互斥锁
    pthread_cond_t cond1;
    AVFrame* tile_pic;          //发送线程传递的帧，需要在线程中释放资源

    // 360SCVP库
    param_360SCVP* pParam360SCVP;
    void* p360SCVPHandle;

    // 计时用
    struct timeval last_frame_start;
    struct timeval last_frame_end;

} SvtContext;

long get_interval(struct timeval end, struct timeval start);
void show_svt_params(EB_H265_ENC_CONFIGURATION *param);
void show_read_in_data(EB_BUFFERHEADERTYPE *headerPtr);
void show_context(SvtContext *svt_enc);

int svt_thread(TileEncoderInfo *tile_enc_info);
int send_tile_thread(TileEncoderInfo *tile_enc_info);
int get_tile_bitrate(EncoderWrapper* wrapper, int idx);
int get_tile_maxrate(EncoderWrapper* wrapper, int idx);
int svt_thread_init(EncoderWrapper* wrapper, int tile_idx);
int svt_multi_thread_encode(void* ctx, AVPacket *pkt, const AVFrame *pic, int *got_packet);
int svt_multi_thread_close(void* ctx);
int tile_stitching(EncoderWrapper* wrapper, AVPacket* outPkt);
int bFifoReady( EncoderWrapper* wrapper );
int get_tile_frame_copy(EncoderWrapper* wrapper, int tile_idx, const AVFrame *pic, AVFrame** tile_pic );
int get_tile_frame_nocopy(EncoderWrapper* wrapper, int tile_idx, const AVFrame *pic, AVFrame** tile_pic );
int receive_packet(EncoderWrapper* wrapper, int tile_idx, AVPacket *pkt);
int svt_thread_close(EncoderWrapper* wrapper, int tile_idx);

/////////////////////////////////////////////////////////////////////////////////



long get_interval(struct timeval end, struct timeval start)
{
    return 1000000 * ( end.tv_sec - start.tv_sec ) + end.tv_usec - start.tv_usec;
}

int SHOW_DEBUG = 0; 
void show_svt_params(EB_H265_ENC_CONFIGURATION *param)
{
    if(SHOW_DEBUG)
    {
        printf("show_svt_params()\n\
        %d %d %d %d %d %d %d %d %d %d \n\
        %d %d %d %d %d %d %d %d %d %d \n\
        %d %d %d %d %d %d %d %d %d %d \n\
        %d %d %d %d %d %d %d %d %d %d \n\
        %d %d %d %d %d %d %d %d %d %d \n\
        %d %d %d %d %d %d %d %d %d %d \n\
        %d %d %d %d %d %d %d %d %d %d \n\
        %d %d %d %d %d %d %d %d \n",

        param->accessUnitDelimiter, 
        param->activeChannelCount, 
        param->asmType, 
        param->baseLayerSwitchMode, 
        param->bitRateReduction, 
        param->bufferingPeriodSEI, 
        param->channelId, 
        param->codeEosNal, 
        param->codeVpsSpsPps, 
        param->compressedTenBitFormat, 

        param->constrainedIntra, 
        param->disableDlfFlag, 
        param->displayPrimaryX, 
        param->displayPrimaryY, 
        param->dolbyVisionProfile, 
        param->enableHmeFlag, 
        param->enableSaoFlag, 
        param->enableTemporalId, 
        param->encMode, 
        param->encoderBitDepth, 

        param->encoderColorFormat, 
        param->fpsInVps, 
        param->frameRate, 
        param->frameRateDenominator, 
        param->frameRateNumerator, 
        param->framesToBeEncoded, 
        param->hierarchicalLevels, 
        param->highDynamicRangeInput, 
        param->hrdFlag, 
        param->improveSharpness, 

        param->injectorFrameRate, 
        param->interlacedVideo, 
        param->intraPeriodLength, 
        param->intraRefreshType, 
        param->latencyMode, 
        param->level, 
        param->logicalProcessors, 
        param->lookAheadDistance, 
        param->maxCLL, 
        param->maxDisplayMasteringLuminance, 

        param->maxFALL, 
        param->maxQpAllowed, 
        param->minDisplayMasteringLuminance, 
        param->minQpAllowed, 
        param->pictureTimingSEI, 
        param->predStructure, 
        param->profile, 
        param->qp, 
        param->rateControlMode, 
        param->reconEnabled, 

        param->recoveryPointSeiFlag, 
        param->registeredUserDataSeiFlag, 
        param->sceneChangeDetection, 
        param->searchAreaHeight, 
        param->searchAreaWidth, 
        param->sourceHeight, 
        param->sourceWidth, 
        param->speedControlFlag, 
        param->switchThreadsToRtPriority, 
        param->targetBitRate, 

        param->targetSocket, 
        param->tier, 
        param->tileColumnCount, 
        param->tileRowCount, 
        param->tileSliceMode, 
        param->tune, 
        param->unregisteredUserDataSeiFlag, 
        param->unrestrictedMotionVector, 
        param->useDefaultMeHme, 
        param->useMasteringDisplayColorVolume, 

        param->useNaluFile, 
        param->useQpFile, 
        param->vbvBufInit, 
        param->vbvBufsize, 
        param->vbvMaxrate, 
        param->videoUsabilityInfo, 
        param->whitePointX, 
        param->whitePointY
        );
    }    
}

void show_read_in_data(EB_BUFFERHEADERTYPE *headerPtr)
{
    if(SHOW_DEBUG)
    {
        EB_H265_ENC_INPUT *in_data = (EB_H265_ENC_INPUT *)headerPtr->pBuffer;

        printf("show_read_in_data()\n\
            %d %d %d %d %d %d %d %d %d %d \n\
            %d %d %d %d  \n",

            headerPtr->dts, 
            headerPtr->nAllocLen,
            //headerPtr->naluBase64Encode,
            headerPtr->naluFound,
            headerPtr->naluNalType,
            headerPtr->naluPayloadType,
            headerPtr->naluPOC,
            headerPtr->naluPrefix,
            headerPtr->nFilledLen,
            headerPtr->nFlags,
            headerPtr->nSize,

            headerPtr->nTickCount,
            //headerPtr->pAppPrivate,
            //headerPtr->pBuffer,
            headerPtr->pts,
            headerPtr->qpValue,
            //headerPtr->segmentOvPtr,
            headerPtr->sliceType
            //headerPtr->wrapperPtr
        );
        
        printf("==============\n");
        for(int i = 0; i < 50; i++)
        {
            printf("%d ",in_data->luma[i]);
        }
        printf("\n");
    }
    
}

void show_context(SvtContext *svt_enc)
{
    if(SHOW_DEBUG)
    {
        printf("show_context()\n\
            %d %d %d %d %d %d %d %d %d %d \n\
            %d %d %d %d %d %d %d %d %d %d \n\
            %d %d %d %d %d %d %d \n",

            svt_enc->asm_type,
            svt_enc->aud,
            svt_enc->base_layer_switch_mode,
            svt_enc->bit_rate,
            svt_enc->enc_mode,
            svt_enc->forced_idr,
            svt_enc->frame_number,
            svt_enc->height,
            svt_enc->width,
            svt_enc->hierarchical_level,

            svt_enc->high_dynamic_range,
            svt_enc->instance_col_count,
            svt_enc->instance_row_count,
            svt_enc->la_depth,
            svt_enc->level,
            svt_enc->max_rate,
            svt_enc->pred_struct,
            svt_enc->profile,
            svt_enc->qp,
            svt_enc->rc_mode,

            svt_enc->scd,
            svt_enc->tier,
            svt_enc->tile_col_count,
            svt_enc->tile_row_count,
            svt_enc->tune,
            svt_enc->unrestricted_motion_vector,
            svt_enc->vid_info
        );
    }
}

static int error_mapping(EB_ERRORTYPE svt_ret)
{
    switch (svt_ret) {
    case EB_ErrorInsufficientResources:
        return AVERROR(ENOMEM);

    case EB_ErrorUndefined:
    case EB_ErrorInvalidComponent:
    case EB_ErrorBadParameter:
        return AVERROR(EINVAL);

    case EB_ErrorDestroyThreadFailed:
    case EB_ErrorSemaphoreUnresponsive:
    case EB_ErrorDestroySemaphoreFailed:
    case EB_ErrorCreateMutexFailed:
    case EB_ErrorMutexUnresponsive:
    case EB_ErrorDestroyMutexFailed:
        return AVERROR_EXTERNAL;

    case EB_NoErrorEmptyQueue:
        return AVERROR(EAGAIN);

    case EB_ErrorNone:
        return 0;

    default:
        return AVERROR_UNKNOWN;
    }
    }

static void free_buffer(SvtContext *svt_enc)
{
    if (svt_enc && svt_enc->in_data) {
        av_freep(&svt_enc->in_data);
        svt_enc->in_data = NULL;
    }
}

static EB_ERRORTYPE alloc_buffer(SvtContext *svt_enc)
{
    EB_BUFFERHEADERTYPE *in_buf = &svt_enc->in_buf;
    EB_H265_ENC_INPUT *in_data = NULL;

    memset(in_buf, 0, sizeof(*in_buf));
    in_buf->nSize = sizeof(*in_buf);
    in_buf->sliceType = EB_INVALID_PICTURE;

    in_data = (EB_H265_ENC_INPUT *)av_mallocz(sizeof(*in_data));
    if (in_data) {
        svt_enc->in_data = in_buf->pBuffer = (uint8_t *)in_data;
        return EB_ErrorNone;
    } else {
        return EB_ErrorInsufficientResources;
    }
}

static int config_enc_params(EB_H265_ENC_CONFIGURATION *param, AVCodecContext *avctx, SvtContext *svt_enc)
{
    //SvtContext *svt_enc = avctx->priv_data;

    //param->latencyMode = 1; //修改延迟模式 已经弃用了
    param->encMode = svt_enc->enc_mode; //编码模式

    param->switchThreadsToRtPriority = 1; //改了SVT库之后突然报错，必须要赋值？
    param->vbvBufInit = 90;                 
    param->hrdFlag = 0;

    param->sourceWidth = svt_enc->width;
    param->sourceHeight = svt_enc->height;
    param->targetBitRate = svt_enc->bit_rate;
    param->vbvMaxrate = svt_enc->max_rate;

    if ((avctx->pix_fmt == AV_PIX_FMT_YUV420P10) ||
        (avctx->pix_fmt == AV_PIX_FMT_YUV422P10) ||
        (avctx->pix_fmt == AV_PIX_FMT_YUV444P10)) {
        av_log(avctx, AV_LOG_DEBUG, "Set 10 bits depth input\n");
        printf("[CPJ] detect 10 bits format input\n");
        param->encoderBitDepth = 10;
    } else {
        av_log(avctx, AV_LOG_DEBUG, "Set 8 bits depth input\n");
        param->encoderBitDepth = 8;
    }

    if ((avctx->pix_fmt == AV_PIX_FMT_YUV420P) ||
        (avctx->pix_fmt == AV_PIX_FMT_YUV420P10))
        param->encoderColorFormat = EB_YUV420;
    else if ((avctx->pix_fmt == AV_PIX_FMT_YUV422P) ||
             (avctx->pix_fmt == AV_PIX_FMT_YUV422P10))
        param->encoderColorFormat = EB_YUV422;
    else
        param->encoderColorFormat = EB_YUV444;

    param->profile = svt_enc->profile;

    if (FF_PROFILE_HEVC_MAIN_STILL_PICTURE == param->profile) {
        av_log(avctx, AV_LOG_ERROR, "Main Still Picture Profile not supported\n");
        return EB_ErrorBadParameter;
    }

    if ((param->encoderColorFormat >= EB_YUV422) &&
        (param->profile != FF_PROFILE_HEVC_REXT)) {
        av_log(avctx, AV_LOG_WARNING, "Rext Profile forced for 422 or 444\n");
        param->profile = FF_PROFILE_HEVC_REXT;
    }

    if ((FF_PROFILE_HEVC_MAIN == param->profile) &&
        (param->encoderBitDepth > 8)) {
        av_log(avctx, AV_LOG_WARNING, "Main10 Profile forced for 10 bits\n");
        param->profile = FF_PROFILE_HEVC_MAIN_10;
    }

    param->vbvBufsize = avctx->rc_buffer_size;

    if (avctx->gop_size > 0)
        param->intraPeriodLength = avctx->gop_size - 1;

    if ((avctx->framerate.num > 0) && (avctx->framerate.den > 0)) {
        param->frameRateNumerator = avctx->framerate.num;
        param->frameRateDenominator =
            avctx->framerate.den * avctx->ticks_per_frame;
    } else {
        param->frameRateNumerator = avctx->time_base.den;
        param->frameRateDenominator =
            avctx->time_base.num * avctx->ticks_per_frame;
    }

    param->hierarchicalLevels = svt_enc->hierarchical_level;
    param->encMode = svt_enc->enc_mode;
    param->tier = svt_enc->tier;
    param->level = svt_enc->level;
    param->rateControlMode = svt_enc->rc_mode;
    param->sceneChangeDetection = svt_enc->scd;
    param->tune = svt_enc->tune;
    param->baseLayerSwitchMode = svt_enc->base_layer_switch_mode;
    param->qp = svt_enc->qp;
    param->accessUnitDelimiter = svt_enc->aud;
    param->asmType = svt_enc->asm_type;
    param->intraRefreshType =  svt_enc->forced_idr;
    param->highDynamicRangeInput = svt_enc->high_dynamic_range;
    param->targetSocket = svt_enc->target_socket;
    if (param->rateControlMode) {
        param->maxQpAllowed = avctx->qmax;
        param->minQpAllowed = avctx->qmin;
    }

    if (svt_enc->la_depth != -1)
        param->lookAheadDistance = svt_enc->la_depth;

    
    if(avctx->gop_size > 0) {       //设置intra_period
        param->intraPeriodLength = avctx->gop_size - 1;
    }

    //不知道为啥缺了一项线程数变量，注释掉也能跑
    // if ((svt_enc->thread_count > 0) &&
    //     (svt_enc->thread_count < (EB_THREAD_COUNT_MIN_CORE * EB_THREAD_COUNT_FACTOR))) {
    //     param->threadCount = EB_THREAD_COUNT_MIN_CORE * EB_THREAD_COUNT_FACTOR;
    //     av_log(avctx, AV_LOG_WARNING, "Thread count is set too small, forced to %"PRId32"\n",
    //            param->threadCount);
    // } else if (svt_enc->thread_count % EB_THREAD_COUNT_MIN_CORE) {
    //     param->threadCount = (svt_enc->thread_count + EB_THREAD_COUNT_MIN_CORE - 1)
    //                          / EB_THREAD_COUNT_MIN_CORE * EB_THREAD_COUNT_MIN_CORE;
    //     av_log(avctx, AV_LOG_DEBUG, "Thread count is rounded to %"PRId32"\n",
    //            param->threadCount);
    // } else {
    //     param->threadCount = svt_enc->thread_count;
    // }
    

    if (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER)
        param->codeVpsSpsPps = 0;
    else
        param->codeVpsSpsPps = 1;

    param->codeEosNal = 1;

    //svt_enc->unrestricted_motion_vector = 0;    // 直接设0了
    if (svt_enc->unrestricted_motion_vector == 0 || svt_enc->unrestricted_motion_vector == 1) {
        param->unrestrictedMotionVector = svt_enc->unrestricted_motion_vector;
    } else {
        av_log(avctx, AV_LOG_ERROR, "Unrestricted Motion Vector should be set 0 or 1\n");
        return EB_ErrorBadParameter;
    }

    if(svt_enc->tile_row_count >= 1 && svt_enc->tile_row_count <= 16) {
        param->tileRowCount = svt_enc->tile_row_count;
    } else {
        av_log(avctx, AV_LOG_ERROR, "Tile Row Count should between 1-16 [%d]\n", svt_enc->tile_row_count);
        return EB_ErrorBadParameter;
    }

    if(svt_enc->tile_col_count >= 1 && svt_enc->tile_col_count <= 16) {
        param->tileColumnCount = svt_enc->tile_col_count;
    } else {
        av_log(avctx, AV_LOG_ERROR, "Tile Column Count should between 1-16 [%d]\n", svt_enc->tile_col_count);
        return EB_ErrorBadParameter;
    }

    if(svt_enc->tile_slice_mode == 0 || svt_enc->tile_slice_mode == 1) {
        param->tileSliceMode = svt_enc->tile_slice_mode;
    } else {
        av_log(avctx, AV_LOG_ERROR, "Tile Slice Mode should be set 0 or 1\n");
        return EB_ErrorBadParameter;
    }

    //svt_enc->pred_struct = 0;    // 直接设0了
    if(svt_enc->pred_struct >= 0 && svt_enc->pred_struct <= 2) {
        param->predStructure = svt_enc->pred_struct;
    } else {
        av_log(avctx, AV_LOG_ERROR, "Pred Structure should between 0-2\n");
        return EB_ErrorBadParameter;
    }

    if(svt_enc->vid_info == 0 || svt_enc->vid_info == 1) {
        param->videoUsabilityInfo = svt_enc->vid_info;
    } else {
        av_log(avctx, AV_LOG_ERROR, "Video Usability Info should be set 0 or 1\n");
        return EB_ErrorBadParameter;
    }
    return EB_ErrorNone;
}

static void read_in_data(EB_H265_ENC_CONFIGURATION *config, const AVFrame *frame, EB_BUFFERHEADERTYPE *header_ptr)
{
    uint8_t is16bit;
    uint64_t frame_size;
    EB_H265_ENC_INPUT *in_data = (EB_H265_ENC_INPUT *)header_ptr->pBuffer;

    is16bit = config->encoderBitDepth > 8;
    frame_size = (uint64_t)(config->sourceWidth * config->sourceHeight) << is16bit;

    in_data->luma = frame->data[0];
    in_data->cb = frame->data[1];
    in_data->cr = frame->data[2];

    in_data->yStride = frame->linesize[0] >> is16bit;
    in_data->cbStride = frame->linesize[1] >> is16bit;
    in_data->crStride = frame->linesize[2] >> is16bit;

    if (config->encoderColorFormat == EB_YUV420)
        frame_size *= 3/2u;
    else if (config->encoderColorFormat == EB_YUV422)
        frame_size *= 2u;
    else
        frame_size *= 3u;

    header_ptr->nFilledLen += frame_size;

    show_read_in_data(header_ptr);
}

static av_cold int eb_enc_init(AVCodecContext *avctx)
{
    printf("eb_enc_init()\n"); //CPJ
    SvtContext *svt_enc = avctx->priv_data;
    EB_ERRORTYPE svt_ret;

    //添加投影格式和帧数信息
    svt_enc->frame_number = 0;


    if(0 == strncmp(svt_enc->proj_type, "ERP", 3))
    {
        svt_enc->projInfo = E_EQUIRECT_PROJECTION;
    }
    else if (0 == strncmp(svt_enc->proj_type, "Cube", 4))
    {
        svt_enc->projInfo = E_CUBEMAP_PROJECTION;
    }
    else if (0 == strncmp(svt_enc->proj_type, "Planar", 6))
    {
        svt_enc->projInfo = -1;
    }
    else
    {
        av_log(avctx, AV_LOG_ERROR,
                "Invalid input source projection type %s \n", svt_enc->proj_type);
        return -1;
    }

    svt_enc->width = avctx->width;
    svt_enc->height = avctx->height;
    svt_enc->bit_rate = avctx->bit_rate;
    svt_enc->max_rate = avctx->rc_max_rate;
    svt_enc->eos_flag = EOS_NOT_REACHED;

    show_context(svt_enc);

    svt_ret = EbInitHandle(&svt_enc->svt_handle, svt_enc, &svt_enc->enc_params);
    if (svt_ret != EB_ErrorNone) {
        av_log(avctx, AV_LOG_ERROR, "Failed to init handle\n");
        return error_mapping(svt_ret);
    }

    svt_ret = config_enc_params(&svt_enc->enc_params, avctx, avctx->priv_data);
    if (svt_ret != EB_ErrorNone) {
        av_log(avctx, AV_LOG_ERROR, "Failed to config parameters\n");
        goto failed_init_handle;
    }

    svt_ret = EbH265EncSetParameter(svt_enc->svt_handle, &svt_enc->enc_params);
    if (svt_ret != EB_ErrorNone) {
        av_log(avctx, AV_LOG_ERROR, "Failed to set parameters\n");
        goto failed_init_handle;
    }

    svt_ret = EbInitEncoder(svt_enc->svt_handle);
    if (svt_ret != EB_ErrorNone) {
        av_log(avctx, AV_LOG_ERROR, "Failed to init encoder\n");
        goto failed_init_handle;
    }

    // 下面的会被调用，但是注释了好像也不影响
    // if (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER) {
    //     

    //     EB_BUFFERHEADERTYPE *header_ptr = NULL;
        
    //     svt_ret = EbH265EncStreamHeader(svt_enc->svt_handle, &header_ptr);
    //     if (svt_ret != EB_ErrorNone) {
    //         av_log(avctx, AV_LOG_ERROR, "Failed to build stream header\n");
    //         goto failed_init_encoder;
    //     }

    //     avctx->extradata_size = header_ptr->nFilledLen;
    //     avctx->extradata = av_malloc(avctx->extradata_size + AV_INPUT_BUFFER_PADDING_SIZE);
    //     if (!avctx->extradata) {
    //         av_log(avctx, AV_LOG_ERROR, "Failed to allocate extradata\n");
    //         svt_ret = EB_ErrorInsufficientResources;
    //         goto failed_init_encoder;
    //     }
    //     memcpy(avctx->extradata, header_ptr->pBuffer, avctx->extradata_size);
    //     memset(avctx->extradata+avctx->extradata_size, 0, AV_INPUT_BUFFER_PADDING_SIZE);
    // }

    svt_ret = alloc_buffer(svt_enc); 
     
    show_svt_params(&svt_enc->enc_params);

    if (svt_ret != EB_ErrorNone) {
        av_log(avctx, AV_LOG_ERROR, "Failed to alloc data buffer\n");
        goto failed_init_encoder;
    }
    return 0;

failed_init_encoder:
    EbDeinitEncoder(svt_enc->svt_handle);
failed_init_handle:
    EbDeinitHandle(svt_enc->svt_handle);
    svt_enc->svt_handle = NULL;
    svt_enc = NULL;
    return error_mapping(svt_ret);
}

int call = 0;
int pkt_num = 0;
static int eb_encode_frame(AVCodecContext *avctx, AVPacket *pkt, const AVFrame *frame, int *got_packet)
{
    //printf("eb_encode_frame[%d]\n", call++);

    SvtContext *svt_enc = avctx->priv_data;
    EB_BUFFERHEADERTYPE *header_ptr = &svt_enc->in_buf;
    EB_ERRORTYPE svt_ret;
    int av_ret;

    if (EOS_RECEIVED == svt_enc->eos_flag) {
        *got_packet = 0;
        return 0;
    }

    if (!frame) {
        if (!svt_enc->eos_flag) {
            svt_enc->eos_flag = EOS_SENT;

            header_ptr->nAllocLen = 0;
            header_ptr->nFilledLen = 0;
            header_ptr->nTickCount = 0;
            header_ptr->nFlags = EB_BUFFERFLAG_EOS; //发送一个EB_BUFFERFLAG_EOS标记的空帧，在收到之后即说明已经全部编完
            header_ptr->pBuffer = NULL;

            EbH265EncSendPicture(svt_enc->svt_handle, header_ptr);

            av_log(avctx, AV_LOG_DEBUG, "Sent EOS\n");
        }
    } else {
        read_in_data(&svt_enc->enc_params, frame, header_ptr);
        header_ptr->pts = frame->pts;

        struct timeval send_tile_timer[2];
        gettimeofday(&send_tile_timer[0], NULL );

        EbH265EncSendPicture(svt_enc->svt_handle, header_ptr);

        gettimeofday(&send_tile_timer[1], NULL );
        printf("send tile time[%ld]\n", get_interval(send_tile_timer[1], send_tile_timer[0]));

        av_log(avctx, AV_LOG_DEBUG, "Sent PTS %"PRId64"\n", header_ptr->pts);
    }

    header_ptr = NULL;
    svt_ret = EbH265GetPacket(svt_enc->svt_handle, &header_ptr, svt_enc->eos_flag);

    if (svt_ret == EB_NoErrorEmptyQueue) {
        *got_packet = 0;
        av_log(avctx, AV_LOG_DEBUG, "Received none\n");
        return 0;
    } else if (svt_ret == EB_ErrorMax) {
        *got_packet = 0;
        av_log(avctx, AV_LOG_ERROR, "Received NULL packet with error code 0x%X\n", header_ptr->nFlags);
        return AVERROR_INVALIDDATA;
    }

    av_log(avctx, AV_LOG_DEBUG, "Received PTS %"PRId64" packet\n", header_ptr->pts);

    av_ret = ff_alloc_packet2(avctx, pkt, header_ptr->nFilledLen, 0);
    if (av_ret) {
        av_log(avctx, AV_LOG_ERROR, "Failed to allocate a packet\n");
        EbH265ReleaseOutBuffer(&header_ptr);
        return av_ret;
    }

    memcpy(pkt->data, header_ptr->pBuffer, header_ptr->nFilledLen);
    pkt->size = header_ptr->nFilledLen;
    pkt->pts  = header_ptr->pts;
    pkt->dts  = header_ptr->dts;

    //printf("pkt->size[%d][%d]\n", ++pkt_num, pkt->size);

    if ((header_ptr->sliceType == EB_IDR_PICTURE) ||
        (header_ptr->sliceType == EB_I_PICTURE))
        pkt->flags |= AV_PKT_FLAG_KEY;
    if (header_ptr->sliceType == EB_NON_REF_PICTURE)
        pkt->flags |= AV_PKT_FLAG_DISPOSABLE;

    EbH265ReleaseOutBuffer(&header_ptr);




    //修改///////////////////////////////////////////////////////////////////////////////////////

    if (svt_enc->frame_number == 0)   // 第一帧有个额外头部
    {
        
        // Headers* header = (Headers*)malloc(sizeof(Headers));
        // if(!header)
        // {
        //      av_log(avctx, AV_LOG_ERROR, "Failed to create header for output .\n");
        //      return AVERROR(ENOMEM);
        // }

        // ret = DistributedEncoder_GetParam(deCxt->handle, Param_Header, &header); // distribute_encoder里的写法

        EB_BUFFERHEADERTYPE *header_buf = NULL;
        EB_ERRORTYPE res = EbH265EncStreamHeader (svt_enc->svt_handle, &header_buf);

        if (res != EB_ErrorNone || header_buf == NULL) {
            printf("Failed to get header\n");
            return AVERROR(ENOMEM);
        }
        else 
        {

            pkt->side_data = (AVPacketSideData*)malloc(sizeof(AVPacketSideData));
            if(!pkt->side_data)
            {
                //free(header);
                free(header_buf);
                return AVERROR(EINVAL);
            }
            //pkt->side_data->size = header->headerSize;
            pkt->side_data->size = header_buf->nFilledLen + 9;
            
            pkt->side_data->data = av_malloc(pkt->side_data->size + AV_INPUT_BUFFER_PADDING_SIZE);
            if (!(pkt->side_data->data))
            {
                av_log(avctx, AV_LOG_ERROR,
                    "Cannot allocate HEVC header of size %d. \n", pkt->side_data->size);
                //free(header);
                free(header_buf);
                return AVERROR(ENOMEM);
            }
            memcpy(pkt->side_data->data, header_buf->pBuffer, pkt->side_data->size);
            //memcpy(pkt->side_data->data + header_buf->nFilledLen, &(svt_enc->projInfo), sizeof(int));

            for(int i = 0; i < 9; i++)
            {
                memcpy(pkt->side_data->data + header_buf->nFilledLen + i, &(nal4[i]), sizeof(int));
            }
            memcpy(pkt->side_data->data + header_buf->nFilledLen + 6, &(svt_enc->projInfo), sizeof(int));  //150替换为配置的投影类型

            /*
            for(int i = 0; i < pkt->side_data->size; i++)
            {
                printf("%d ",pkt->side_data->data[i]);
            }
            printf("\n---------------------------------------------\n");
            */

            // 先拿distributed的测试
            /*
            if(svt_enc->tile_col_count <= 2) pkt->side_data->size = 112 ;
            else pkt->side_data->size = 113 ;
            pkt->side_data->data = av_malloc(pkt->side_data->size);

            for(int i = 0; i < pkt->side_data->size; i++)
            {
                if(svt_enc->tile_col_count <= 2)
                {
                    memcpy(pkt->side_data->data + i, &head1[i], sizeof(int));
                }
                else
                {
                    memcpy(pkt->side_data->data + i, &head2[i], sizeof(int));
                }
            }


            for(int i = 0; i < pkt->side_data->size; i++)
            {
                printf("%d ",pkt->side_data->data[i]);
            }
            printf("\n==============================================\n");
            */
            //free(header);
            //header = NULL;
            //free(header_buf);
            
            pkt->side_data->type = AV_PKT_DATA_NEW_EXTRADATA;
            pkt->side_data_elems = 1;
        }
    
        printf("[first frame]------------------------------------------------------\n");
        for(int i = 0; i < 200; i++)
        {
            printf("%d ", pkt->data[i]);
        }
        printf("\n[first frame]------------------------------------------------------\n");
        for(int i = 0; i < pkt->side_data->size; i++)
        {
            printf("%d ", pkt->side_data->data[i]);
        }
        printf("\n[first frame]------------------------------------------------------\n");
    }

    svt_enc->frame_number++;

    /////////////////////////////////////////////////////////////////////////////////////////


    *got_packet = 1;

    if (EB_BUFFERFLAG_EOS == header_ptr->nFlags)
       svt_enc->eos_flag = EOS_RECEIVED;

    return 0;
}

static av_cold int eb_enc_close(AVCodecContext *avctx)
{
    SvtContext *svt_enc = avctx->priv_data;

    if (svt_enc) {
        free_buffer(svt_enc);

        if (svt_enc->svt_handle) {
            EbDeinitEncoder(svt_enc->svt_handle);
            EbDeinitHandle(svt_enc->svt_handle);
            svt_enc->svt_handle = NULL;
        }
    }

    return 0;
}

// 多线程函数////////////////////////////////////////////////////////////////////////////////

// 实际执行编码的多线程函数，子线程只负责接收
int svt_thread(TileEncoderInfo *tile_enc_info)
{
    //printf("svt_thread()\n"); //CPJ FINAL
    int ret = 0;
    int received_packet = 0;    // 已经收到的包数量

    EncoderWrapper *wrapper = (EncoderWrapper*)tile_enc_info->ctx;
    int            tile_idx = tile_enc_info->tile_idx;

    while(1)
    {
        if(wrapper->initialized)    //在线程中等待直到svt_enc_init()执行完成
            break;  
    }

    SvtContext          *q         = (SvtContext *)wrapper->tile_info[tile_idx].enc_ctx;
    SvtEncoder          *svt_enc   = q->svt_enc;
    //EB_BUFFERHEADERTYPE *headerPtr = svt_enc->out_buf;

    while(!wrapper->tile_info[tile_idx].eos)
    {
        // Wait until next frame is sent
        /*
        if(!q->eos_flag)
        {
            printf("waiting[%d]\n", tile_idx);
            pthread_cond_wait(&(wrapper->cond0),&(wrapper->mutex0));
            printf("running[%d]\n", tile_idx);
        }
        */
        //printf("waiting[%d]\n", tile_idx); //CPJ FINAL
        pthread_cond_wait(&(wrapper->cond0),&(wrapper->mutex0));
        //printf("running[%d]\n", tile_idx); //CPJ FINAL
        // if(q->eos_flag)
        // {
        //     printf("eos_flag received[%d]\n", tile_idx);
        //     wrapper->tile_info[tile_idx].eos = 1;
        //     break;
        // }

        AVPacket tile_pkts = {0};
        ret = receive_packet(wrapper, tile_idx, &tile_pkts);    // 子线程会在接收过程阻塞，循环直至收到

        //if(ret == -5) continue;


        //对于各个流的第一帧似乎需要PPS等信息,还得直接放进流里不能放在side_data里//////////////////////////////////////////////////

        if (received_packet == 0 )   // 第一帧有个额外头部
        {

            //printf("making header PPS ……\n");  //CPJ FINAL
            SvtContext* q = (SvtContext *)wrapper->tile_info[tile_idx].enc_ctx;
            SvtEncoder* tileEnc = q->svt_enc;

            EB_BUFFERHEADERTYPE *header_buf = NULL;
            EB_ERRORTYPE res = EbH265EncStreamHeader (tileEnc->svt_handle, &header_buf);

            if (res != EB_ErrorNone || header_buf == NULL) {
                printf("Failed to get header\n");
                return AVERROR(ENOMEM);
            }
            else 
            {
                AVPacket pkt = {0};
                pkt.data = (uint8_t *)malloc(tile_pkts.size + header_buf->nFilledLen);
                if(!pkt.data)
                {
                    printf("malloc failed [0]\n");
                    return AVERROR(ENOMEM);
                }

                pkt.size = tile_pkts.size + header_buf->nFilledLen;

                pkt.pts = tile_pkts.pts;
                pkt.dts = tile_pkts.dts;

                memcpy(pkt.data, header_buf->pBuffer, header_buf->nFilledLen);
                memcpy(pkt.data + header_buf->nFilledLen, tile_pkts.data, tile_pkts.size);
                av_fifo_generic_write( wrapper->tile_info[tile_idx].outpkt_fifo, &pkt, sizeof(AVPacket), NULL);
                //free(pkt.data);

                // printf("[av_fifo_generic_write]------------------------------------------------------\n");
                // for(int i = 0; i < 200; i++)
                // {
                //     //printf("%d ", outPkt->data[i]);
                //     printf("%d ", pkt.data[i]);
                // }
                // printf("\n[av_fifo_generic_write]------------------------------------------------------\n");

                // tile_pkts.side_data = (AVPacketSideData*)malloc(sizeof(AVPacketSideData));
                // if(!tile_pkts.side_data)
                // {
                //     //free(header);
                //     free(header_buf);
                //     return AVERROR(EINVAL);
                // }
                // tile_pkts.side_data->size = header_buf->nFilledLen ;
                
                // tile_pkts.side_data->data = av_malloc(tile_pkts.side_data->size + AV_INPUT_BUFFER_PADDING_SIZE);
                // if (!(tile_pkts.side_data->data))
                // {
                //     //free(header);
                //     free(header_buf);
                //     return AVERROR(ENOMEM);
                // }
                // memcpy(tile_pkts.side_data->data, header_buf->pBuffer, tile_pkts.side_data->size);
                // //memcpy(pkt->side_data->data + header_buf->nFilledLen, &(svt_enc->projInfo), sizeof(int));
                // tile_pkts.side_data->type = AV_PKT_DATA_NEW_EXTRADATA;
                // tile_pkts.side_data_elems = 1;
            }
        }
        else
        {
            av_fifo_generic_write( wrapper->tile_info[tile_idx].outpkt_fifo, &tile_pkts, sizeof(AVPacket), NULL);
        }

        received_packet++;

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        
        //printf("tile id = %d begin to eb_receive_packet!!!\n", tile_idx);
        if( 0 == ret || AVERROR_EOF == ret ){
            //printf("**********tile id = %d eb_receive_packet got packet, packet size = %d, packet addr=%p!!!\n", tile_idx, tile_pkts.size, tile_pkts.data);
            // printf("tile_pkts.size[%d][%d]", tile_idx, tile_pkts.size);
            // av_fifo_generic_write( wrapper->tile_info[tile_idx].outpkt_fifo, &tile_pkts, sizeof(AVPacket), NULL);

            if( AVERROR_EOF == ret ){
                //printf("tile id = %d EOS!!!\n", tile_idx); //CPJ FINAL
                wrapper->tile_info[tile_idx].eos = 1;
            }
        }else{
            //av_packet_unref(&tile_pkts);
            //free(tile_pkts.data);
        }

    }

    //printf("wait to exit……[%d]\n", tile_idx); //CPJ FINAL
    // Wait until all tiles are done
    while(AVERROR_EOF!=bFifoReady(wrapper))
    {
        pthread_cond_wait(&(wrapper->cond0),&(wrapper->mutex0));
        usleep(10000);
    }
    
    return ret;
}

// 尝试将发送过程也用线程实现，检测到发送过程用时过长 
int send_tile_thread(TileEncoderInfo *tile_enc_info)
{
    //printf("svt_thread()\n"); //CPJ FINAL
    int ret = 0;

    EncoderWrapper *wrapper = (EncoderWrapper*)tile_enc_info->ctx;
    int            tile_idx = tile_enc_info->tile_idx;

    while(1)
    {
        if(wrapper->initialized)    //在线程中等待直到svt_enc_init()执行完成
            break;  
    }

    SvtContext          *q         = (SvtContext *)wrapper->tile_info[tile_idx].enc_ctx;
    SvtEncoder          *svt_enc   = q->svt_enc;
    EB_BUFFERHEADERTYPE  *headerPtr = svt_enc->in_buf;


    while(!wrapper->tile_info[tile_idx].eos)
    {
        //需要主线程中设置q->sendFlag = false;
        //主线程完成headerPtr的赋值
        pthread_cond_wait(&(q->cond1), &(q->mutex1));
        
        ret = EbH265EncSendPicture(svt_enc->svt_handle, headerPtr);

        if(NULL!= q->tile_pic) av_frame_free(&q->tile_pic); 

        q->sendFlag = true;
    }

    return ret;
}

// 释放SvtEncoder结构体资源
static void free_encoder(SvtEncoder *svt_enc)
{
    //printf("free_buffer()\n"); //CPJ FINAL
    if (svt_enc->in_buf) {
        EB_H265_ENC_INPUT *in_data = (EB_H265_ENC_INPUT* )svt_enc->in_buf->pBuffer;
        if (in_data) {
            av_freep(&in_data);
        }
        av_freep(&svt_enc->in_buf);
    }
    av_freep(&svt_enc->out_buf);
}

// 计算tile的尺寸
static int assign_tiles_avg( SvtContext* ctx )
{
    EncoderWrapper* wrapper = &(ctx->encoder_wrapper);

    wrapper->tile_num = ctx->instance_col_count * ctx->instance_row_count;
    wrapper->tile_w = ctx->instance_col_count;
    wrapper->tile_h = ctx->instance_row_count;

#define LCU_SIZE 64

    // Width and Height should be divisible by LCU_SIZE
    int width_in_lcu = wrapper->width / LCU_SIZE;
    int height_in_lcu = wrapper->height / LCU_SIZE;

    // (6.5.1) in Rec. ITU-T H.265 v5 (02/2018)
    int *tiles_col_width, *tiles_row_height;
    tiles_col_width = (int *)malloc(ctx->instance_col_count * sizeof(int));
    tiles_row_height = (int *)malloc(ctx->instance_row_count * sizeof(int));
    if(!tiles_col_width || !tiles_row_height)
    {
        printf("malloc failed [1]\n");
        return -1;
    }
    for( int i=0; i<ctx->instance_col_count; i++)
    {
        tiles_col_width[i] = (i+1) * width_in_lcu / ctx->instance_col_count - i * width_in_lcu / ctx->instance_col_count;
    }
    for( int i=0; i<ctx->instance_row_count; i++)
    {
        tiles_row_height[i] = (i+1) * height_in_lcu / ctx->instance_row_count - i * height_in_lcu / ctx->instance_row_count;

    }

    for(int i = 0; i < ctx->instance_row_count; i++)
    {
        for(int j = 0; j < ctx->instance_col_count; j++)
        {
            int idx = i * ctx->instance_col_count + j;
            wrapper->tile_info[idx].left    = (j == 0) ? 0 : wrapper->tile_info[idx - 1].left + tiles_col_width[j-1] * LCU_SIZE;
            wrapper->tile_info[idx].top     = (i == 0) ? 0 : wrapper->tile_info[(i-1)*ctx->instance_col_count + j].top + tiles_row_height[i-1] * LCU_SIZE;
            wrapper->tile_info[idx].tHeight = tiles_row_height[i] * LCU_SIZE;
            wrapper->tile_info[idx].tWidth  = tiles_col_width[j] * LCU_SIZE;
        }
    }

    if(tiles_col_width)
    {
        free(tiles_col_width);
        tiles_col_width = NULL;
    }
    if(tiles_row_height)
    {
        free(tiles_row_height);
        tiles_row_height = NULL;
    }

    return 0;
}

static EB_ERRORTYPE tile_alloc_buffer(EB_H265_ENC_CONFIGURATION *config, SvtEncoder *svt_enc)
{
    //printf("tile_alloc_buffer()\n"); //CPJ FINAL
    EB_ERRORTYPE       ret       = EB_ErrorNone;

    const int    pack_mode_10bit   = (config->encoderBitDepth > 8) && (config->compressedTenBitFormat == 0) ? 1 : 0;
    const size_t luma_size_8bit    = config->sourceWidth * config->sourceHeight * (1 << pack_mode_10bit);
    const size_t luma_size_10bit   = (config->encoderBitDepth > 8 && pack_mode_10bit == 0) ? luma_size_8bit : 0;

    svt_enc->raw_size = (luma_size_8bit + luma_size_10bit) * 3 / 2;

    // allocate buffer for in and out
    svt_enc->in_buf           = (EB_BUFFERHEADERTYPE *)malloc(sizeof(EB_BUFFERHEADERTYPE));
    svt_enc->out_buf          = (EB_BUFFERHEADERTYPE *)malloc(sizeof(EB_BUFFERHEADERTYPE));
    if (!svt_enc->in_buf || !svt_enc->out_buf)
        goto failed;

    svt_enc->in_buf->pBuffer  = malloc(sizeof(EB_H265_ENC_INPUT));
    if (!svt_enc->in_buf->pBuffer)
        goto failed;

    svt_enc->in_buf->nSize        = sizeof(EB_BUFFERHEADERTYPE);
    svt_enc->in_buf->pAppPrivate  = NULL;
    svt_enc->in_buf->sliceType    = EB_INVALID_PICTURE;
    svt_enc->out_buf->nSize       = sizeof(EB_BUFFERHEADERTYPE);
    svt_enc->out_buf->nAllocLen   = svt_enc->raw_size;
    svt_enc->out_buf->pAppPrivate = NULL;

    return ret;

failed:
    free_encoder(svt_enc);
    return AVERROR(ENOMEM);


}

// 把tile绑到对应的headerPtr上准备发送
static void tile_read_in_data(EB_H265_ENC_CONFIGURATION *config, const AVFrame* frame, EB_BUFFERHEADERTYPE *headerPtr)
{
    //printf("tile_read_in_data()\n"); //CPJ
    unsigned int is16bit = config->encoderBitDepth > 8;
    unsigned long long lumaReadSize = (unsigned long long)(config->sourceWidth * 
                                        config->sourceHeight) << is16bit;
    EB_H265_ENC_INPUT *in_data = (EB_H265_ENC_INPUT*)headerPtr->pBuffer;

    
    // support yuv420p and yuv420p010
    in_data->luma = frame->data[0];
    in_data->cb   = frame->data[1];
    in_data->cr   = frame->data[2];

	// stride info
    in_data->yStride  = frame->linesize[0] >> is16bit;
    in_data->cbStride = frame->linesize[1] >> is16bit;
    in_data->crStride = frame->linesize[2] >> is16bit;

    //headerPtr->nFilledLen   += lumaReadSize * 3/2u;

    if (config->encoderColorFormat == EB_YUV420)
        lumaReadSize *= 3/2u;
    else if (config->encoderColorFormat == EB_YUV422)
        lumaReadSize *= 2u;
    else
        lumaReadSize *= 3u;

    headerPtr->nFilledLen += lumaReadSize;

    show_read_in_data(headerPtr);
}

/// 计算tile分配码率
int get_tile_bitrate(EncoderWrapper* wrapper, int idx)
{
    int bit_rate = wrapper->avctx->bit_rate;
    double percent = 0.0;

    if( 0==bit_rate ) bit_rate = wrapper->avctx->bit_rate_tolerance;

    ///FIXME if there is more suitable way to calculate bit rate for each tile
    percent = (double)( wrapper->tile_info[idx].tHeight * wrapper->tile_info[idx].tWidth ) / (double)(wrapper->width * wrapper->height);

    return (int) (bit_rate * percent);

 }

int get_tile_maxrate(EncoderWrapper* wrapper, int idx)
{
    int max_rate = wrapper->avctx->rc_max_rate;

    ///FIXME if there is more suitable way to calculate bit rate for each tile
    double percent = (double)( wrapper->tile_info[idx].tHeight * wrapper->tile_info[idx].tWidth ) / (double)(wrapper->width * wrapper->height);

    return (int) (max_rate * percent);

}

// 给单个svt实例送视频帧的函数，在其中完成帧的tile内容分割和传递
static int send_tile(EncoderWrapper* wrapper, int tile_idx, const AVFrame *frame)
{
    //printf("send_tile() [%d]\n", tile_idx); //CPJ FINAL
    SvtContext *q       = (SvtContext *)wrapper->tile_info[tile_idx].enc_ctx;
    SvtEncoder           *svt_enc = q->svt_enc;
    EB_BUFFERHEADERTYPE  *headerPtr = svt_enc->in_buf;

    AVFrame* tile_pic = NULL;
    int                  ret = 0;
    
    struct timeval send_tile_timer[5];
    gettimeofday(&send_tile_timer[0], NULL );

    if (!frame) {
        if(q->send_eos_frame == 0)
        {
            q->send_eos_frame++;    //只送一帧，不然SVT会段错误 艹

            EB_BUFFERHEADERTYPE headerPtrLast;
            headerPtrLast.nAllocLen = 0;
            headerPtrLast.nFilledLen = 0;
            headerPtrLast.nTickCount = 0;
            headerPtrLast.pAppPrivate = NULL;
            //headerPtrLast.nOffset = 0;
            //headerPtrLast.nTimeStamp = 0;
            headerPtrLast.nFlags = EB_BUFFERFLAG_EOS;
            headerPtrLast.pBuffer = NULL;

            ret = EbH265EncSendPicture(svt_enc->svt_handle, &headerPtrLast);
            //q->eos_flag = 1;    // 修改，使用返回值通知编码结束，在主线程中控制子线程结束
            //printf("Finish sending frames!!!\n"); //CPJ FINAL
        }

        return -1;
    }
    gettimeofday(&send_tile_timer[1], NULL );

    get_tile_frame_nocopy(wrapper, tile_idx, frame, &tile_pic);
    //get_tile_frame_copy(wrapper, tile_idx, frame, &tile_pic);

    //printf("------tile id = %d start frame address: y=%p, u=%p, v=%p!!!\n",
    //                                      tile_idx, tile_pic->data[0], tile_pic->data[1], tile_pic->data[2]);
    gettimeofday(&send_tile_timer[2], NULL );

    tile_read_in_data(&svt_enc->enc_params, tile_pic, headerPtr);

    gettimeofday(&send_tile_timer[3], NULL );
    //headerPtr->nOffset    = 0;
    
    //headerPtr->nFlags     = 0;
    //headerPtr->pAppPrivate = NULL;
    headerPtr->pts        = frame->pts;
    //headerPtr->nFlags     = 0;
    //headerPtr->nTimeStamp = 0;
    //headerPtr->sliceType  = INVALID_SLICE;
    //q->i += 1;    //TODO ????????????
    //av_log(wrapper->avctx, AV_LOG_DEBUG, "tile id = %d start to send frame, times = %d!!!\n", tile_idx, q->i);

    //printf("send tile size[%d][%d][%d][%d]\n", headerPtr->pts, tile_idx, ++pkt_num, headerPtr->nFilledLen); //CPJ FINAL


    //使用子线程发送///////////////////////////////////////////////////////////////////////////////////////////////////////

    q->sendFlag = false;
    q->tile_pic = tile_pic;
    pthread_cond_broadcast(&(q->cond1)); //这个信号是全局的，改成单个svt实例的
    //ret = EbH265EncSendPicture(svt_enc->svt_handle, headerPtr);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    gettimeofday(&send_tile_timer[4], NULL );

    //if(NULL!= tile_pic) av_frame_free(&tile_pic);  // 换在子线程中释放

    //printf("send tile time[%d][%d]\n", get_interval(send_tile_timer[4], send_tile_timer[3]), tile_idx);

    return ret;
}

// 获取单个实例的返回
int receive_packet(EncoderWrapper* wrapper, int tile_idx, AVPacket *pkt)
{
    //printf("receive_packet() [%d]\n", tile_idx); //CPJ FINAL
    SvtContext *q        = (SvtContext *)wrapper->tile_info[tile_idx].enc_ctx;
    SvtEncoder  *svt_enc = q->svt_enc;
    EB_BUFFERHEADERTYPE   *headerPtr = svt_enc->out_buf;
    //EB_BUFFERHEADERTYPE   *headerPtr = svt_enc->in_buf;
    EB_ERRORTYPE          stream_status = EB_ErrorNone;
    
    int ret = 0;

    //if ((ret = ff_alloc_packet2(wrapper->avctx, pkt, svt_enc->raw_size, 0)) < 0){
    //    av_log(wrapper->avctx, AV_LOG_ERROR, "tile id = %d ff_alloc_packet2 ret = %d!!!\n", tile_idx, ret);
    //    return ret;
    //}
    pkt->data = (uint8_t *)malloc(svt_enc->raw_size);
    pkt->size = svt_enc->raw_size;

    //headerPtr->pBuffer = pkt->data;
    headerPtr->pBuffer = (uint8_t *)malloc(svt_enc->raw_size);
    
    if(!pkt->data || !headerPtr->pBuffer)
    {
        printf("malloc failed [2]\n");
        return -1;
    }
    
    stream_status = EbH265GetPacket(svt_enc->svt_handle, &headerPtr, q->eos_flag);
    // 偶尔有收不到包的情况，循环等待获取
    while ((stream_status == EB_NoErrorEmptyQueue)){
        //return -5;

        usleep(10000);
        stream_status = EbH265GetPacket(svt_enc->svt_handle, &headerPtr, q->eos_flag);
    }
    //printf("packet received\n"); //CPJ FINAL
    pkt->size = headerPtr->nFilledLen;
    pkt->pts = headerPtr->pts;
    pkt->dts = headerPtr->dts;
    ret = (headerPtr->nFlags & EB_BUFFERFLAG_EOS) ? AVERROR_EOF : 0;
    
    //printf("receive packet size[%d][%d][%d][%d][%d]\n", pkt->pts, pkt->dts, tile_idx, pkt_num, pkt->size); //CPJ FINAL

    // 原来的代码根本没做内容复制，光有长度没有内容 艹
    memcpy(pkt->data, headerPtr->pBuffer, headerPtr->nFilledLen);

    // for(unsigned int k = 0; k < 200; k++)
    // {
    //     printf("%d ", pkt->data[k]);
    // }
    // printf("\n");

     // 需要释放SVT资源队列对象
    EbH265ReleaseOutBuffer(&headerPtr);

    return ret;
}

// 流合并
int tile_stitching(EncoderWrapper* wrapper, AVPacket* outPkt)
{
    int ret = 0;
    AVPacket pkt[MAX_TILES];
    int bReady = bFifoReady(wrapper);
    int totalsize=0;
    uint8_t* dst = NULL;

    if( AVERROR_EOF == bReady ) return AVERROR_EOF;

    //printf("tile_stitching [%d]\n",bReady); //CPJ FINAL
    if( 1 == bReady ){
        for(int i = 0; i < wrapper->tile_num; i++){
            av_fifo_generic_read( wrapper->tile_info[i].outpkt_fifo, &pkt[i],  sizeof(AVPacket),  NULL);
            totalsize += pkt[i].size;
        }
        //printf("tile_stitching size[%d]\n",totalsize); //CPJ FINAL

        // Sometimes the size of output is larger than size of input,
        // so we alloc 2 times larger size packet.
        ret = ff_alloc_packet2(wrapper->avctx, outPkt, 2*totalsize, 2*totalsize);
        if( ret < 0)
        {
            printf("???????????????\n");
            return -2;
        } 

        dst = outPkt->data;

        SvtContext *svt_enc = wrapper->avctx->priv_data;

        svt_enc->pParam360SCVP->inputBitstreamLen = totalsize;
        svt_enc->pParam360SCVP->outputBitstreamLen = 2*totalsize; //TODO 确认输出大小
        svt_enc->pParam360SCVP->pOutputBitstream = dst;

        int height = svt_enc->pParam360SCVP->paramPicInfo.tileHeightNum;
        int width = svt_enc->pParam360SCVP->paramPicInfo.tileWidthNum;
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                param_oneStream_info* oneStream = svt_enc->pParam360SCVP->paramStitchInfo.pTiledBitstream[i*width + j];
                oneStream->pTiledBitstreamBuffer = pkt[i*width + j].data;
                oneStream->inputBufferLen = pkt[i*width + j].size;

                oneStream->tilesHeightCount = svt_enc->tile_row_count;    //TODO 确认这三个参数怎么设置
                oneStream->tilesWidthCount = svt_enc->tile_col_count;
                oneStream->tilesIdx = i*width + j; 
            }
        }


        // memcpy(outPkt->data, pkt[0].data, pkt[0].size);   //测试输出单码流，正常
        // outPkt->size = pkt[0].size;

        //memset(outPkt->data, 0, outPkt->size);


        struct timeval start, end;
        gettimeofday(&start, NULL );

        ret = I360SCVP_process(svt_enc->pParam360SCVP, svt_enc->p360SCVPHandle);
        
        gettimeofday(&end, NULL );
        long timeuse = 1000000 * ( end.tv_sec - start.tv_sec ) + end.tv_usec - start.tv_usec;
        //printf("tile_stitching time [%ld]\n", timeuse);   //微秒





        outPkt->size = svt_enc->pParam360SCVP->outputBitstreamLen; //输出流的具体长度没有赋值 艹
        outPkt->pts = pkt[0].pts;   // 还要把pts dts赋值上
        outPkt->dts= pkt[0].dts;

        // printf("===========================\n");
        // for(int i = 0; i < 100; i++)
        // {
        //     if(outPkt->data[i] != pkt[0].data[i])
        //     {
        //         printf("different found[%d]\n", i);
        //         break;
        //     }
        // }
        // for(int i = 0; i < 100; i++)
        // {
        //     printf("%d ", outPkt->data[i]);
        // }
        // printf("\n--------------------\n");
        // for(int i = 0; i < 100; i++)
        // {
        //     printf("%d ", pkt[0].data[i]);
        // }
        // printf("\n===========================\n");


        int gop = wrapper->avctx->gop_size; //分段，会让最后的码流按gop划分文件
        if(outPkt->pts % gop == 0)
        {
            outPkt->flags |= AV_PKT_FLAG_KEY;
        }

        //printf("new 360scvp call [%d, %d, %d, %d, %d]\n", pkt[0].pts, pkt[0].dts, ret, outPkt->size, totalsize); //CPJ FINAL

        //最后再补上第一帧的额外信息////////////////////////////////////////////////////////////


        if (svt_enc->frame_number == 0)   // 第一帧有个额外头部
        {
        
            Param_BSHeader* bsHeader = (Param_BSHeader*)malloc(sizeof(Param_BSHeader));
            if(!bsHeader)
            {
                printf("malloc failed [3]\n");
                return AVERROR(EINVAL);
            }
            I360SCVP_GetParameter(svt_enc->p360SCVPHandle, ID_SCVP_BITSTREAMS_HEADER, &bsHeader);
            
            outPkt->side_data = (AVPacketSideData*)malloc(sizeof(AVPacketSideData));
            if(!outPkt->side_data)
            {
                printf("malloc failed [4]\n");
                free(bsHeader);
                return AVERROR(EINVAL);
            }
            //pkt->side_data->size = header->headerSize;
            outPkt->side_data->size = bsHeader->size;   // ??? 360SCVP库好像自动解决了投影SEI的部分，不需要自己复制了 艹
            
            outPkt->side_data->data = (uint8_t *)malloc(outPkt->side_data->size + AV_INPUT_BUFFER_PADDING_SIZE);
            if (!(outPkt->side_data->data))
            {
                printf("malloc failed [5]\n");
                free(bsHeader);
                return AVERROR(ENOMEM);
            }
            memcpy(outPkt->side_data->data, bsHeader->data, bsHeader->size);

            outPkt->side_data->type = AV_PKT_DATA_NEW_EXTRADATA;
            outPkt->side_data_elems = 1;

        }
        // else //其他帧保持side_data为空指针就行
        // {
        //     outPkt->side_data = (AVPacketSideData*)malloc(sizeof(AVPacketSideData));
        //     outPkt->size = 0;
        //     outPkt->side_data->data = av_malloc(sizeof(int));
        // }
        
        svt_enc->frame_number++;

        //////////////////////////////////////////////////////////////////////////////////////

        // printf("[new frame]------------------------------------------------------\n");
        // for(int i = 0; i < 100; i++)
        // {
        //     printf("%d ", outPkt->data[i]);
        //     //printf("%d ", pkt[0].data[i]);
        // }
        // printf("\n[new frame]------------------------------------------------------\n");


        ///unref the packet read from fifo
        for(int i = 0; i < wrapper->tile_num; i++){
            free(pkt[i].data);
            av_packet_unref(&pkt[i]);
        }

        return 0;
    }

    return -1;  // 没有可合并的流，返回-1
}

int bFifoReady( EncoderWrapper* wrapper )
{
    int eos = 0;
    int ready = 0;
    for(int i=0; i<wrapper->tile_num; i++){
        if( wrapper->tile_info[i].outpkt_fifo ){
            if( av_fifo_size(wrapper->tile_info[i].outpkt_fifo) ){
                ready++;
            }else{
                if(wrapper->tile_info[i].eos) eos++;
            }
        }
    }
    if( ready == wrapper->tile_num ) return 1;
    if( eos == wrapper->tile_num ) return AVERROR_EOF;

    return 0;
}

// tile拆分
int get_tile_frame_copy(EncoderWrapper* wrapper, int tile_idx, const AVFrame *pic, AVFrame** tile_pic )
{
    int ret = 0;
    uint8_t* src = NULL;
    uint8_t* dst = NULL;
    int factor = 1;
    AVFrame* frame = NULL;

    if( NULL == *tile_pic ){
        *tile_pic = av_frame_alloc();
        if (!*tile_pic) {
            av_freep(*tile_pic);
            return AVERROR(ENOMEM);
        }
    }

    frame = *tile_pic;
    frame->height = wrapper->tile_info[tile_idx].tHeight;
    frame->width  = wrapper->tile_info[tile_idx].tWidth;

    frame->format = pic->format;

    if (!frame->data[0]) {
        ret = av_frame_get_buffer(frame, 32);
        if (ret < 0){
            av_freep(*tile_pic);
            return ret;
        }
    }

    ///current copy is based on YUV420p format
    for( int planner=0; planner<3; planner++ ){
        if( planner > 0 ){
            factor = 2;
        }
        src = pic->data[planner] + pic->linesize[planner]*(wrapper->tile_info[tile_idx].top / factor) + wrapper->tile_info[tile_idx].left / factor;
        dst = frame->data[planner];
        for( int i = 0; i < frame->height / factor; i++ ){
            src += pic->linesize[planner];
            dst += frame->linesize[planner];
            memcpy( dst, src, frame->width / factor );
        }
    }

    return ret;
}

int get_tile_frame_nocopy(EncoderWrapper* wrapper, int tile_idx, const AVFrame *pic, AVFrame** tile_pic )
{
    AVFrame* frame = NULL;
    int factor = 1;

    if( NULL == *tile_pic ){
        *tile_pic = av_frame_alloc();
        if (!*tile_pic) {
            av_freep(*tile_pic);
            return AVERROR(ENOMEM);
        }
    }

    frame = *tile_pic;
    frame->height = wrapper->tile_info[tile_idx].tHeight;
    frame->width = wrapper->tile_info[tile_idx].tWidth;
    frame->format = pic->format;

    for( int i = 0; i < 4; i++ ){
        if( i > 0 ){
            factor = 2;
        }
        frame->data[i] = pic->data[i] + pic->linesize[i]*(wrapper->tile_info[tile_idx].top / factor) + wrapper->tile_info[tile_idx].left / factor;
        frame->linesize[i] = pic->linesize[i];
    }

    return 0;
}

static int svt_multi_thread_init(void* ctx)
{
    //printf("svt_multi_thread_init()\n"); //CPJ FINAL

    // 先按照单线程的一样复制顶层参数添加投影格式和帧数信息
    EncoderWrapper* wrapper = (EncoderWrapper*)ctx;
    SvtContext *svt_enc = wrapper->avctx->priv_data;
    EB_ERRORTYPE svt_ret;
    svt_enc->frame_number = 0;


    if(0 == strncmp(svt_enc->proj_type, "ERP", 3))
    {
        svt_enc->projInfo = E_EQUIRECT_PROJECTION;
    }
    else if (0 == strncmp(svt_enc->proj_type, "Cube", 4))
    {
        svt_enc->projInfo = E_CUBEMAP_PROJECTION;
    }
    else if (0 == strncmp(svt_enc->proj_type, "Planar", 6))
    {
        svt_enc->projInfo = -1;
    }
    else
    {
        return -1;
    }

    svt_enc->width = wrapper->avctx->width;
    svt_enc->height = wrapper->avctx->height;
    svt_enc->bit_rate = wrapper->avctx->bit_rate;
    svt_enc->max_rate = wrapper->avctx->rc_max_rate;
    svt_enc->eos_flag = EOS_NOT_REACHED;

    //svt_ret = EbInitHandle(&svt_enc->svt_handle, svt_enc, &svt_enc->enc_params);
    svt_ret = config_enc_params(&svt_enc->enc_params, wrapper->avctx, wrapper->avctx->priv_data);

    printf("main context=======\n");
    show_context(svt_enc);


    // 再做每个线程的参数设置
    SvtContext* svt_ctx = NULL;
    int ret = 0;

    for(int i=0; i<wrapper->tile_num; i++){     //开启多个svt实例，每个由eb_enc_init()函数启动，相关信息存储在wrapper->tile_info[i].enc_ctx中
        svt_ctx = (SvtContext *)malloc(sizeof(SvtContext));
        if(!svt_ctx)
        {
            printf("malloc failed [6]\n");
            return -1;
        }

        svt_ctx->tile_row_count = svt_enc->tile_row_count;    // 一个实例包含一个或多个tile，
        svt_ctx->tile_col_count = svt_enc->tile_col_count;
        svt_ctx->tile_slice_mode = (svt_enc->tile_row_count > 1 || svt_enc->tile_col_count > 1) ? 1 : 0;

        svt_ctx->width     = wrapper->tile_info[i].tWidth;
        svt_ctx->height    = wrapper->tile_info[i].tHeight;
        svt_ctx->bit_rate = wrapper->tile_info[i].tBitrate;
        svt_ctx->max_rate = wrapper->tile_info[i].tMaxrate;
        svt_enc->eos_flag = EOS_NOT_REACHED;
        svt_ctx->tier = svt_enc->tier;
        svt_ctx->level = svt_enc->level;
        svt_ctx->aud = svt_enc->aud;
        svt_ctx->asm_type = svt_enc->asm_type;
        svt_ctx->high_dynamic_range = svt_enc->high_dynamic_range;

        //svt_ctx->target_socket = svt_enc->target_socket;
        
        if(svt_enc->target_socket < -1)
        {
            //svt_ctx->target_socket = i >= wrapper->tile_num/2 ? 0 : 1;  //设置运行的socket

            svt_ctx->target_socket = i % (-svt_enc->target_socket);  //设置运行的socket
        }
        else
        {
            svt_ctx->target_socket = svt_enc->target_socket;
        }

        svt_ctx->unrestricted_motion_vector = svt_enc->unrestricted_motion_vector;
        svt_ctx->pred_struct = svt_enc->pred_struct;

        svt_ctx->hierarchical_level = svt_enc->hierarchical_level;
        svt_ctx->enc_mode = svt_enc->enc_mode;
        svt_ctx->forced_idr = svt_enc->forced_idr;
        svt_ctx->profile = svt_enc->profile;
        svt_ctx->rc_mode = svt_enc->rc_mode;//0-CQP, 1-VBR
        svt_ctx->qp = svt_enc->qp;
        svt_ctx->scd = svt_enc->scd;
        svt_ctx->tune = svt_enc->tune;
        //svt_ctx->intra_period = 5;  //avctx->gop_size // 在set_enc_params中设置

        svt_ctx->base_layer_switch_mode = svt_enc->base_layer_switch_mode;
        svt_ctx->vid_info = svt_enc->vid_info;  
        svt_ctx->la_depth = svt_enc->la_depth;
        wrapper->avctx->bit_rate = wrapper->tile_info[i].tBitrate;  // 每个实例初始化都用到了avctx->bit_rate，实际码率由tBitrate独立决定
        wrapper->tile_info[i].enc_ctx = svt_ctx;


        ret = svt_thread_init(wrapper, i);
        if( 0 != ret ) return ret;
    }
    wrapper->initialized = 1;
    return svt_ret;
}

int svt_thread_init(EncoderWrapper* wrapper, int tile_idx)
{
    //printf("svt_thread_init()\n"); //CPJ FINAL
    SvtContext* ctx = wrapper->tile_info[tile_idx].enc_ctx;

    EB_ERRORTYPE ret = EB_ErrorNone;
    SvtEncoder* svt_enc = NULL;

    pthread_mutex_init(&(ctx->mutex1), NULL);   // 给各个SVT实例对应的互斥锁和信号量
    pthread_cond_init(&(ctx->cond1), NULL);

    ctx->svt_enc  = (SvtEncoder *)malloc(sizeof(*ctx->svt_enc));
    if (!ctx->svt_enc)
        return AVERROR(ENOMEM);

    svt_enc = ctx->svt_enc;

    ctx->eos_flag = 0;
    ctx->send_eos_frame = 0;

    show_context(ctx);

    ret = EbInitHandle(&svt_enc->svt_handle, ctx, &svt_enc->enc_params);
    if (ret != EB_ErrorNone)
        goto failed_init;

    ret = config_enc_params(&svt_enc->enc_params, wrapper->avctx, ctx);
    if (ret != EB_ErrorNone)
        goto failed_init;

    ret = EbH265EncSetParameter(svt_enc->svt_handle, &svt_enc->enc_params);
    if (ret != EB_ErrorNone)
        goto failed_init;

    ret = EbInitEncoder(svt_enc->svt_handle);
    if (ret != EB_ErrorNone)
        goto failed_init;

    ret = tile_alloc_buffer(&svt_enc->enc_params, ctx->svt_enc);

    show_svt_params(&svt_enc->enc_params);

    return ret;

failed_init:
    return error_mapping(ret);
}

int svt_multi_thread_encode(void* ctx, AVPacket *pkt, const AVFrame *pic, int *got_packet)
{
    //printf("svt_multi_thread_encode()\n"); //CPJ FINAL
    EncoderWrapper* wrapper = (EncoderWrapper*)ctx;
    SvtContext *svt_enc = wrapper->avctx->priv_data;    // 主线程上下文

    SvtContext *q = NULL;   // 子线程上下文

    struct timeval current_frame_start, current_frame_end, send_tile_start, send_tile_end;
    gettimeofday(&current_frame_start, NULL );
    if (svt_enc->frame_number == 0)
    {
        gettimeofday(&(svt_enc->last_frame_start), NULL );
        gettimeofday(&(svt_enc->last_frame_end), NULL );
    }
    

    *got_packet = 0;
    int ret = 0;

    gettimeofday(&send_tile_start, NULL );
    for(int i = 0; i < wrapper->tile_num; i++){
        q = (SvtContext *)wrapper->tile_info[i].enc_ctx;
        if( wrapper->tile_info[i].eos ) 
            continue;
        ret = send_tile( wrapper, i, pic );  // 发送tile帧
    }

    while(1)
    {
        int i = 0;
        for(; i < wrapper->tile_num; i++)
        {
            q = (SvtContext *)wrapper->tile_info[i].enc_ctx;
            if(!q->sendFlag)    //还没发送完
            {
                break;
            }
        }
        if(i == wrapper->tile_num)
        {
            break;
        }
        else
        {
            usleep(1000);
        }
    }

    ret = !pic ? -1 : 0;    //换用多线程就不用返回值了,其实就是没帧可送了返回-1

    if(ret == 0)
    {
        svt_enc->sendCnt++;
    }
    gettimeofday(&send_tile_end, NULL );

    // Wake up all receive tile threads
    //if(!q->eos_flag) 
    if(ret == 0)  
    {
        pthread_cond_broadcast(&(wrapper->cond0));   // 通知执行线程启动
    }
    else
    {
        //printf("wait to exit……[main]\n"); //CPJ FINAL
        // 先把所有剩余没合并的帧处理好,每次调用处理一帧
        if(svt_enc->receiveCnt < svt_enc->sendCnt)
        {
            pthread_cond_broadcast(&(wrapper->cond0)); 

            ret = tile_stitching(wrapper, pkt);  // 合并流

            while (ret != 0)    //循环直至有效
            {
                usleep(1000);
                pthread_cond_broadcast(&(wrapper->cond0));
                ret = tile_stitching(wrapper, pkt); 
            }

            svt_enc->receiveCnt++;  
            *got_packet = 1;
            printf("[%d, %d]\n", svt_enc->receiveCnt, svt_enc->sendCnt); //CPJ FINAL
                  
        }
        
        // 最后检查是否已经完成了所有帧
        if(svt_enc->receiveCnt == svt_enc->sendCnt)
        {
            // Wait until all tiles are ready
            while(0 == bFifoReady(wrapper))
            {
                //printf("broadcasting…[main]\n"); //CPJ FINAL
                pthread_cond_broadcast(&(wrapper->cond0));
                usleep(10000);
            }
        }
        //printf("one round finished\n"); //CPJ FINAL
        return 0;
    }

    //FIXME, suppose all encoder has the rhythm to get packet, so there is no buffer in the first time

    ret = tile_stitching(wrapper, pkt);  // 合并流


    gettimeofday(&current_frame_end, NULL );
    long start_interval = get_interval(current_frame_start, svt_enc->last_frame_start);
    long idle_interval = get_interval(current_frame_start, svt_enc->last_frame_end);
    long run_interval = get_interval(current_frame_end, current_frame_start);
    long send_tile_interval = get_interval(send_tile_end, send_tile_start);

    svt_enc->last_frame_start.tv_sec = current_frame_start.tv_sec;
    svt_enc->last_frame_start.tv_usec = current_frame_start.tv_usec;
    svt_enc->last_frame_end.tv_sec = current_frame_end.tv_sec;
    svt_enc->last_frame_end.tv_usec = current_frame_end.tv_usec;

    //printf("runtime: [%ld, %ld, %ld][%d][%d]\n", run_interval, idle_interval, start_interval, send_tile_interval, ret);   //微秒

    if( AVERROR_EOF == ret )
    {
        *got_packet = 0;
        return AVERROR_EOF;
    }
    else if(0 == ret) 
    {
        *got_packet = 1;
        svt_enc->receiveCnt++; 
        //printf("[%d, %d, %d]\n", svt_enc->receiveCnt, svt_enc->sendCnt, pkt->size); //CPJ FINAL
    }
    else
    {
        *got_packet = 0;
        return 0;
    }  

    return 0;
}

int svt_multi_thread_close(void* ctx)
{
    //printf("svt_multi_thread_close()\n"); //CPJ FINAL
    EncoderWrapper* wrapper = (EncoderWrapper*)ctx;
    SvtContext* svt_ctx = NULL;

    for(int i=0; i<wrapper->tile_num; i++){
        svt_ctx = (SvtContext*)wrapper->tile_info[i].enc_ctx;
        if( NULL != svt_ctx){
            svt_thread_close(wrapper, i);
            free(svt_ctx);
        }
        wrapper->tile_info[i].enc_ctx = NULL;
    }

    return 0;
}

int svt_thread_close(EncoderWrapper* wrapper, int tile_idx)
{
    //printf("svt_thread_close()\n"); //CPJ FINAL
    SvtContext *q         = (SvtContext *)wrapper->tile_info[tile_idx].enc_ctx;
    SvtEncoder *svt_enc   = q->svt_enc;

    EbDeinitEncoder(svt_enc->svt_handle);
    EbDeinitHandle(svt_enc->svt_handle);

    free_encoder(svt_enc);
    av_freep(&svt_enc);

    return 0;
}

// 顶层初始化函数
static av_cold int multi_init(AVCodecContext *avctx)
{
    SvtContext *svt_enc = avctx->priv_data;
    EncoderWrapper* wrapper = &(svt_enc->encoder_wrapper);
    int ret = 0;

    wrapper->width = avctx->coded_width;
    wrapper->height = avctx->coded_height;

    wrapper->avctx = avctx;

    // 尺寸参数设置，直接用计算的tile尺寸，不用固定尺寸
    wrapper->uniform_split = true;
    assign_tiles_avg( svt_enc );

    // 接口设置
    wrapper->enc_close = svt_multi_thread_close;
    wrapper->enc_frame = svt_multi_thread_encode;
    wrapper->enc_init  = svt_multi_thread_init;

    // 线程创建
    pthread_mutex_init(&(wrapper->mutex0), NULL);
    pthread_cond_init(&(wrapper->cond0), NULL);
    wrapper->tid0 = (pthread_t *)malloc(wrapper->tile_num * sizeof(pthread_t));
    wrapper->tid1 = (pthread_t *)malloc(wrapper->tile_num * sizeof(pthread_t));
    wrapper->tile_enc_info = (TileEncoderInfo *)malloc(wrapper->tile_num * sizeof(TileEncoderInfo));
    if(!wrapper->tid0 || !wrapper->tid1 || !wrapper->tile_enc_info)
    {
        printf("malloc failed [7]\n");
        return -1;
    }
    for(int i=0; i<wrapper->tile_num; i++){
        wrapper->tile_info[i].tBitrate = get_tile_bitrate(wrapper, i);  // 线程上下文为wrapper->tile_info[i]
        wrapper->tile_info[i].tMaxrate = get_tile_maxrate(wrapper, i);
        wrapper->tile_info[i].eos = 0;
        wrapper->tile_info[i].outpkt_fifo = av_fifo_alloc( FIFO_SIZE * sizeof(AVPacket));

        wrapper->tile_enc_info[i].ctx      = wrapper;
        wrapper->tile_enc_info[i].tile_idx = i;

        ret = pthread_create(&wrapper->tid0[i], NULL, svt_thread, &(wrapper->tile_enc_info[i]));   // 为每一个tile开启线程，线程头部函数svt_thread()
        ret = pthread_create(&wrapper->tid1[i], NULL, send_tile_thread, &(wrapper->tile_enc_info[i]));   // 为每一个tile开启线程，线程头部函数send_tile_thread()
        if(0 != ret)
        {
            av_log(avctx, AV_LOG_ERROR, "Cannot create thread!\n");
            return ret;
        }
    }

    ret = svt_multi_thread_init(wrapper);
    if( 0 != ret ) return ret;
    // if( NULL != svt_enc->encoder_wrapper.enc_init ){
    //     ret = wrapper->enc_init(wrapper);   // 初始化编码器
    //     if( 0 != ret ) return ret;
    // }

    // 自己建360SCVP库来用？ //////////////////////////////////////////////////////////////

        svt_enc->pParam360SCVP = (param_360SCVP*)malloc(sizeof(param_360SCVP));
        if(!svt_enc->pParam360SCVP)
        {
            printf("malloc failed [8]\n");
            return -1;
        }
        svt_enc->pParam360SCVP->usedType = E_STREAM_STITCH_ONLY;

        //PicInfo
        svt_enc->pParam360SCVP->paramPicInfo.maxCUWidth = 0;
        svt_enc->pParam360SCVP->paramPicInfo.picHeight = wrapper->height;
        svt_enc->pParam360SCVP->paramPicInfo.picWidth = wrapper->width;
        svt_enc->pParam360SCVP->paramPicInfo.tileHeightNum = wrapper->tile_h;
        svt_enc->pParam360SCVP->paramPicInfo.tileWidthNum = wrapper->tile_w;
        svt_enc->pParam360SCVP->paramPicInfo.tileIsUniform = 1;

        //StitchInfo
        svt_enc->pParam360SCVP->paramStitchInfo.pTiledBitstream = (param_oneStream_info**)malloc(wrapper->tile_h * wrapper->tile_w * sizeof(param_oneStream_info *));
        svt_enc->pParam360SCVP->paramStitchInfo.AUD_enable = 0;
        svt_enc->pParam360SCVP->paramStitchInfo.VUI_enable = 1;
        svt_enc->pParam360SCVP->paramStitchInfo.sliceType = 0;
        svt_enc->pParam360SCVP->paramStitchInfo.pts = 0;
        if(!svt_enc->pParam360SCVP->paramStitchInfo.pTiledBitstream)
        {
            printf("malloc failed [9]\n");
            return -1;
        }

        for (int i = 0; i < wrapper->tile_h; i++)
        {
            for (int j = 0; j < wrapper->tile_w; j++)
            {
                svt_enc->pParam360SCVP->paramStitchInfo.pTiledBitstream[i*wrapper->tile_w + j] = (param_oneStream_info*)malloc(sizeof(param_oneStream_info));
                if(!svt_enc->pParam360SCVP->paramStitchInfo.pTiledBitstream[i*wrapper->tile_w + j])
                {
                    printf("malloc failed [10]\n");
                    return -1;
                }
            }
        }

        svt_enc->p360SCVPHandle = I360SCVP_Init(svt_enc->pParam360SCVP);
        I360SCVP_SetParameter(svt_enc->p360SCVPHandle, ID_SCVP_PARAM_SEI_PROJECTION, &(svt_enc->projInfo));    // TODO 确认具体是哪个投影

    //////////////////////////////////////////////////////////////////////////////////////

    // 记录发送接收帧数
    svt_enc->sendCnt = 0;
    svt_enc->receiveCnt = 0;

    return 0;
}

// 顶层编码函数
static int multi_encode(AVCodecContext *avctx, AVPacket *pkt, const AVFrame *pic, int *got_packet)
{
    SvtContext *ctx = avctx->priv_data;
    if( NULL != ctx->encoder_wrapper.enc_frame )
        ctx->encoder_wrapper.enc_frame(&(ctx->encoder_wrapper), pkt, pic, got_packet);

    return 0;
}

/// 顶层结束函数
static av_cold int multi_close(AVCodecContext *avctx)
{
    SvtContext *ctx = avctx->priv_data;
    EncoderWrapper *wrapper = &(ctx->encoder_wrapper);
    AVFifoBuffer* fifo = NULL;

    if(wrapper->pGen)
    {
        genTiledStream_unInit(wrapper->pGen);
    }

    if (wrapper->paramTiledStream.pTiledBitstream)
    {
        for (int i = 0; i < wrapper->paramTiledStream.tilesHeightCount; i++)
        {
            for (int j = 0; j < wrapper->paramTiledStream.tilesWidthCount; j++)
            {
                free(wrapper->paramTiledStream.pTiledBitstream[i*wrapper->paramTiledStream.tilesWidthCount + j]);
                wrapper->paramTiledStream.pTiledBitstream[i*wrapper->paramTiledStream.tilesWidthCount + j] = NULL;
            }
        }
        free(wrapper->paramTiledStream.pTiledBitstream);
        wrapper->paramTiledStream.pTiledBitstream = NULL;
    }
    if(avctx->extradata)
    {
        free(avctx->extradata);
        avctx->extradata = NULL;
    }

    if(wrapper->tid0)
    {
        free(wrapper->tid0);
        wrapper->tid0 = NULL;
    }
    if(wrapper->tile_enc_info)
    {
        free(wrapper->tile_enc_info);
        wrapper->tile_enc_info = NULL;
    }

    if( NULL != ctx->encoder_wrapper.enc_close )
        ctx->encoder_wrapper.enc_close(&(ctx->encoder_wrapper));

    for( int i=0; i < ctx->encoder_wrapper.tile_num; i++ ){

        fifo = ctx->encoder_wrapper.tile_info[i].outpkt_fifo;
        while ( fifo && av_fifo_size(fifo)) {
            AVPacket pkt;
            av_fifo_generic_read(fifo, &pkt,  sizeof(pkt),  NULL);
            free(pkt.data);
            av_packet_unref(&pkt);
        }
        av_fifo_free(fifo);
        fifo = NULL;
    }

    float fuck = cos(20);

    return 0;
}


// 换用.send_frame和.receive_packet接口处理///////////////////////////////////////////////////

// static int multi_send_frame(AVCodecContext *avctx, const AVFrame *frame)
// {
// }

// static int multi_receive_packet(AVCodecContext *avctx, AVPacket *pkt)
// {
// }


////////////////////////////////////////////////////////////////////////////////////////////




#define OFFSET(x) offsetof(SvtContext, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {

    // 添加投影格式的配置
    { "proj_type", "input source projection type, ERP or Cubemap", OFFSET(proj_type),
      AV_OPT_TYPE_STRING, { .str = "ERP" }, 0, 0, VE },

    // 添加关于tile和svt实例划分的配置
    // 单个svt实例内的tile划分
    { "tile_row_cnt", "tile count in the row", OFFSET(tile_row_count), AV_OPT_TYPE_INT, { .i64 = 1 }, 1, 16, VE },
    { "tile_col_cnt", "tile count in the column", OFFSET(tile_col_count), AV_OPT_TYPE_INT, { .i64 = 1 }, 1, 16, VE },
    //{ "tile_slice_mode", "per slice per tile, only valid for multi-tile", OFFSET(tile_slice_mode),
    //  AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },

    //svt实例数目
    { "instance_row_cnt", "svt instance count in the row", OFFSET(instance_row_count), AV_OPT_TYPE_INT, { .i64 = 1 }, 1, 8, VE },  
    { "instance_col_cnt", "svt instance count in the col", OFFSET(instance_col_count), AV_OPT_TYPE_INT, { .i64 = 1 }, 1, 8, VE },  

    { "asm_type", "Assembly instruction set type [0: C Only, 1: Auto]", OFFSET(asm_type),
      AV_OPT_TYPE_BOOL, { .i64 = 1 }, 0, 1, VE },

    { "aud", "Include Access Unit Delimiter", OFFSET(aud),
      AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },

    { "bl_mode", "Random Access Prediction Structure type setting", OFFSET(base_layer_switch_mode),
      AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },

    //和distrubuted_encoder保持一致
    //{ "forced-idr", "If forcing keyframes, force them as IDR frames.", OFFSET(forced_idr),
    //  AV_OPT_TYPE_INT,   { .i64 = -1 }, -1, INT_MAX, VE },

    { "forced-idr", "If forcing keyframes, force them as IDR frames.", OFFSET(forced_idr),
      AV_OPT_TYPE_BOOL,   { .i64 = 1 }, 0, 1, VE },

    { "hielevel", "Hierarchical prediction levels setting", OFFSET(hierarchical_level),
      AV_OPT_TYPE_INT, { .i64 = 3 }, 0, 3, VE , "hielevel"},
        { "flat",   NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 0 },  INT_MIN, INT_MAX, VE, "hielevel" },
        { "1 level", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 1 },  INT_MIN, INT_MAX, VE, "hielevel" },
        { "2 level", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 2 },  INT_MIN, INT_MAX, VE, "hielevel" },
        { "3 level", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 3 },  INT_MIN, INT_MAX, VE, "hielevel" },

    { "la_depth", "Look ahead distance [0, 256]", OFFSET(la_depth),
      AV_OPT_TYPE_INT, { .i64 = -1 }, -1, 256, VE },

    { "level", "Set level (level_idc)", OFFSET(level),
      AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 0xff, VE, "level" },

    { "preset", "Encoding preset [0, 12]",
      OFFSET(enc_mode), AV_OPT_TYPE_INT, { .i64 = 9 }, 0, 12, VE },

    { "profile", "Profile setting, Main Still Picture Profile not supported", OFFSET(profile),
      AV_OPT_TYPE_INT, { .i64 = FF_PROFILE_HEVC_MAIN_10 }, FF_PROFILE_HEVC_MAIN, FF_PROFILE_HEVC_REXT, VE, "profile"},

    { "qp", "QP value for intra frames", OFFSET(qp),
      AV_OPT_TYPE_INT, { .i64 = 32 }, 0, 51, VE },

    { "rc", "Bit rate control mode", OFFSET(rc_mode),
      AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 1, VE , "rc"},
        { "cqp", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 0 },  INT_MIN, INT_MAX, VE, "rc" },
        { "vbr", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 1 },  INT_MIN, INT_MAX, VE, "rc" },

    //和distrubuted_encoder保持一致
    //{ "sc_detection", "Scene change detection", OFFSET(scd),
    //  AV_OPT_TYPE_BOOL, { .i64 = 1 }, 0, 1, VE },

    { "sc_detection", "Scene change detection", OFFSET(scd),
      AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },

    { "socket", "Target CPU socket to use.  -1 use all available", OFFSET(target_socket),
      AV_OPT_TYPE_INT, {.i64 = -1 }, -4, 3, VE },

    { "thread_count", "Number of threads [0: Auto, 96: Min]", OFFSET(thread_count),
      AV_OPT_TYPE_INT, {.i64 = 0}, 0, INT_MAX, VE },

    { "tier", "Set tier (general_tier_flag)", OFFSET(tier),
      AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 1, VE, "tier" },
        { "main", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 0 }, 0, 0, VE, "tier" },
        { "high", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 1 }, 0, 0, VE, "tier" },

    { "tune", "Quality tuning mode", OFFSET(tune), AV_OPT_TYPE_INT, { .i64 = 1 }, 0, 2, VE, "tune" },
        { "sq", "Visually optimized mode", 0,
          AV_OPT_TYPE_CONST, { .i64 = 0 },  INT_MIN, INT_MAX, VE, "tune" },
        { "oq",  "PSNR / SSIM optimized mode",  0,
          AV_OPT_TYPE_CONST, { .i64 = 1 },  INT_MIN, INT_MAX, VE, "tune" },
        { "vmaf", "VMAF optimized mode", 0,
          AV_OPT_TYPE_CONST, { .i64 = 2 },  INT_MIN, INT_MAX, VE, "tune" },
    { "hdr", "High dynamic range input (HDR10)", OFFSET(high_dynamic_range), AV_OPT_TYPE_INT, { .i64 = 0}, 0, 1, VE, "hdr" },
    { "umv", "Enables or disables unrestricted motion vectors", OFFSET(unrestricted_motion_vector),
      AV_OPT_TYPE_BOOL, { .i64 = 1 }, 0, 1, VE },

    //和distrubuted_encoder保持一致
    { "pred_struct", "The prediction structure", OFFSET(pred_struct), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 2, VE },
    //{ "pred_struct", "The prediction structure", OFFSET(pred_struct), AV_OPT_TYPE_INT, { .i64 = 2 }, 0, 2, VE },

    // { "pred_struct", "Prediction structure used to construct GOP", OFFSET(gop_pred_structure),
    //   AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 2, VE, "pred_struct" },
    //     { "IPPP", "P is low delay P", 0, AV_OPT_TYPE_CONST, { .i64 = 0 }, INT_MIN, INT_MAX, VE, "pred_struct" },
    //     { "Ibbb", "b is low delay B", 1, AV_OPT_TYPE_CONST, { .i64 = 1 }, INT_MIN, INT_MAX, VE, "pred_struct" },
    //     { "IBBB", "B is normal bi-directional B", 2, AV_OPT_TYPE_CONST, { .i64 = 2 }, INT_MIN, INT_MAX, VE, "pred_struct" },

    { "vid_info", "Enables or disables sending a vui structure in the HEVC Elementary bitstream.", OFFSET(vid_info),
      AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },
    {NULL},
};

static const AVClass class = {
    .class_name = "libsvt_hevc",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

/*
static const AVCodecDefault eb_enc_defaults[] = {
    { "b",         "7M"    },
    { "qmin",      "10"    },
    { "qmax",      "48"    },
    { "g",         "-2"    },
    { NULL },
};
*/

static const AVCodecDefault eb_enc_defaults[] = {
    { "b",         "7M"    },
    { "flags",     "+cgop" },
    { "qmin",      "10"    },
    { "qmax",      "48"    },
    { "g",         "-2"    },
    { NULL },
};

AVCodec ff_libsvt_hevc_encoder = {
    .name           = "libsvt_hevc",
    .long_name      = NULL_IF_CONFIG_SMALL("SVT-HEVC(Scalable Video Technology for HEVC) encoder"),
    .priv_data_size = sizeof(SvtContext),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_HEVC,
    
    // 单线程版
    // .init           = eb_enc_init,
    // .encode2        = eb_encode_frame,
    // .close          = eb_enc_close,

    // 多线程版
    .init           = multi_init,
    .encode2        = multi_encode,
    .close          = multi_close,
    // .send_frame     = multi_send_frame,
    // .receive_packet = multi_receive_packet,

    .capabilities   = AV_CODEC_CAP_DELAY,
    .pix_fmts       = (const enum AVPixelFormat[]){ AV_PIX_FMT_YUV420P,
                                                    AV_PIX_FMT_YUV420P10,
                                                    AV_PIX_FMT_YUV422P,
                                                    AV_PIX_FMT_YUV422P10,
                                                    AV_PIX_FMT_YUV444P,
                                                    AV_PIX_FMT_YUV444P10,
                                                    AV_PIX_FMT_NONE },
    .priv_class     = &class,
    .defaults       = eb_enc_defaults,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .wrapper_name   = "libsvt_hevc",
};


