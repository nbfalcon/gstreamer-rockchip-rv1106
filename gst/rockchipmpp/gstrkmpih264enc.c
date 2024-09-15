/*
 * Copyright 2018 Rockchip Electronics Co. LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <gst/gst.h>
#include <gst/video/gstvideoencoder.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rk_mpi_mb.h"
#include "rk_mpi_venc.h"
#include "rk_mpi_sys.h"

static uint64_t monotonic_millis() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)(ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);
}

#define GST_CAT_DEFAULT gstrkpmpih264
GST_DEBUG_CATEGORY(GST_CAT_DEFAULT);

typedef struct _GstRKMPIH264Enc GstRKMPIH264Enc;
typedef struct _GstRKMPIH264EncClass GstRKMPIH264EncClass;

struct _GstRKMPIH264Enc {
  GstVideoEncoder parent;

  GstVideoCodecState *input_state;
  GstVideoInfo info;
  VENC_CHN_ATTR_S stAttr;

  MB_POOL imagePool;

  RK_S32 chnId;
  RK_U32 frameCounter;
};

struct _GstRKMPIH264EncClass {
  GstVideoEncoderClass parent_class;
};

G_DEFINE_TYPE(GstRKMPIH264Enc, gst_rkmpi_h264enc, GST_TYPE_VIDEO_ENCODER)
#define GST_TYPE_RKMPIH264ENC (gst_rkmpi_h264enc_get_type())
#define GST_RKMPIH264ENC(obj)                                                  \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_RKMPIH264ENC, GstRKMPIH264Enc))
#define GST_RKMPIH264ENC_CLASS(klass)                                          \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_RKMPIH264ENC,                     \
                           GstRKMPIH264EncClass))
#define GST_IS_RKMPIH264ENC(obj)                                               \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_RKMPIH264ENC))
#define GST_IS_RKMPIH264ENC_CLASS(klass)                                       \
  (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_RKMPIH264ENC))
#define GST_RKMPIH264ENC_GET_CLASS(obj)                                        \
  (G_TYPE_INSTANCE_GET_CLASS((obj), GST_TYPE_RKMPIH264ENC,                     \
                             GstRKMPIH264EncClass))

static GstFlowReturn gst_rkmpi_h264_enc_handle_frame(GstVideoEncoder *encoder,
                                                     GstVideoCodecFrame *frame);
static gboolean gst_rkmpi_h264_enc_start(GstVideoEncoder *encoder);
static gboolean gst_rkmpi_h264_enc_stop(GstVideoEncoder *encoder);
static gboolean gst_rkmpi_h264_enc_set_format(GstVideoEncoder *encoder,
                                              GstVideoCodecState *state);
static GstFlowReturn gst_rkmpi_h264_handle_frame(GstVideoEncoder *self,
                                                 GstVideoCodecFrame *frame);

#define GST_RKMPI_H264ENC_SIZE_CAPS                                            \
  "width  = (int) [ 96, MAX ], height = (int) [ 64, MAX ]"
static GstStaticPadTemplate gst_rkmpi_h264enc_src_template =
    GST_STATIC_PAD_TEMPLATE(
        "src", GST_PAD_SRC, GST_PAD_ALWAYS,
        GST_STATIC_CAPS("video/x-h264, " GST_RKMPI_H264ENC_SIZE_CAPS ","
                        "stream-format = (string) { byte-stream }, "
                        "alignment = (string) { au }, "
                        "profile = (string) { baseline, main, high }"));
static GstStaticPadTemplate gst_rkmpih264_enc_sink_template =
    GST_STATIC_PAD_TEMPLATE(
        "sink", GST_PAD_SINK, GST_PAD_ALWAYS,
        GST_STATIC_CAPS(
            "video/x-raw,"
            "format = (string) { NV24, Y444 }, " GST_RKMPI_H264ENC_SIZE_CAPS));

static void gst_rkmpi_h264enc_init(GstRKMPIH264Enc *element) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(element);

  self->input_state = NULL;
  self->imagePool = 0;
  self->chnId = 0;
  self->frameCounter = 0;
}

static void gst_rkmpi_h264enc_class_init(GstRKMPIH264EncClass *klass) {
  GstVideoEncoderClass *video_encoder = GST_VIDEO_ENCODER_CLASS(klass);
  video_encoder->start = gst_rkmpi_h264_enc_start;
  video_encoder->stop = gst_rkmpi_h264_enc_stop;
  video_encoder->set_format = gst_rkmpi_h264_enc_set_format;
  video_encoder->handle_frame = gst_rkmpi_h264_handle_frame;

  GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
  gst_element_class_add_pad_template(
      element_class,
      gst_static_pad_template_get(&gst_rkmpi_h264enc_src_template));
  gst_element_class_add_pad_template(
      element_class,
      gst_static_pad_template_get(&gst_rkmpih264_enc_sink_template));
  gst_element_class_set_static_metadata(
      element_class, "Rockchip Rockcit H264 Encoder", "Codec/Encoder/Video",
      "Encode video streams via Rockchip rockit/RKMPI",
      "Nikita <nikblos@outlook.com>");
}

static struct gst_rkmpi_format {
  GstVideoFormat gst_format;
  PIXEL_FORMAT_E rkmpi_format;
} GST_RKMPI_FORMAT_ALIST[] = {
    {GST_VIDEO_FORMAT_NV12, RK_FMT_YUV420SP},
    {GST_VIDEO_FORMAT_NV12_10LE32, RK_FMT_YUV420SP_10BIT},
    {GST_VIDEO_FORMAT_NV16, RK_FMT_YUV422SP},
    {GST_VIDEO_FORMAT_P010_10LE, RK_FMT_YUV422SP_10BIT},
    {GST_VIDEO_FORMAT_I420, RK_FMT_YUV420P},
    {GST_VIDEO_FORMAT_YV12, RK_FMT_YUV420P_VU},
    {GST_VIDEO_FORMAT_NV21, RK_FMT_YUV420SP_VU},
    {GST_VIDEO_FORMAT_Y42B, RK_FMT_YUV422P},
    {GST_VIDEO_FORMAT_VYUY, RK_FMT_YUV422SP_VU},
    {GST_VIDEO_FORMAT_YUY2, RK_FMT_YUV422_YUYV},
    {GST_VIDEO_FORMAT_UYVY, RK_FMT_YUV422_UYVY},
    {GST_VIDEO_FORMAT_GRAY8, RK_FMT_YUV400SP},
    {GST_VIDEO_FORMAT_Y444, RK_FMT_YUV444},
    {GST_VIDEO_FORMAT_RGB16, RK_FMT_RGB565},
    {GST_VIDEO_FORMAT_BGR16, RK_FMT_BGR565},
    {GST_VIDEO_FORMAT_RGB15, RK_FMT_RGB555},
    {GST_VIDEO_FORMAT_BGR15, RK_FMT_BGR555},
    {GST_VIDEO_FORMAT_RGB, RK_FMT_RGB888},
    {GST_VIDEO_FORMAT_BGR, RK_FMT_BGR888},
    {GST_VIDEO_FORMAT_ARGB, RK_FMT_ARGB8888},
    {GST_VIDEO_FORMAT_ABGR, RK_FMT_ABGR8888},
    {GST_VIDEO_FORMAT_BGRA, RK_FMT_BGRA8888},
    {GST_VIDEO_FORMAT_RGBA, RK_FMT_RGBA8888},
    {GST_VIDEO_FORMAT_YVYU, RK_FMT_YUV422_YVYU},
    {GST_VIDEO_FORMAT_VYUY, RK_FMT_YUV422_VYUY},
    {GST_VIDEO_FORMAT_NV16, RK_FMT_YUV422SP},
    {GST_VIDEO_FORMAT_NV61, RK_FMT_YUV422SP_VU},
    {GST_VIDEO_FORMAT_NV24, RK_FMT_YUV444SP},
    {GST_VIDEO_FORMAT_RGB16, RK_FMT_RGB565},
    {GST_VIDEO_FORMAT_BGR16, RK_FMT_BGR565},
    // FIXME: there are some missing entries. Also, some of these might be
    // ChatGPT hallucinated
    // FIXME: Bayer formats
};

#define COUNTOF(x) (sizeof(x) / sizeof((x)[0]))
#define RK_MPI_ERROR_CHECK(name) if (rkret != RK_SUCCESS) { fprintf(stderr, "rockit MPI: %s failed (%d)\n", #name, rkret); return GST_FLOW_ERROR; }
#define RK_MPI_ERROR_CHECK2(name) if (rkret != RK_SUCCESS) { fprintf(stderr, "rockit MPI: %s failed (%d)\n", #name, rkret); return FALSE; }

static gboolean gst_rkmpi_h264_enc_set_format(GstVideoEncoder *encoder,
                                              GstVideoCodecState *state) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);

  self->input_state = gst_video_codec_state_ref(state);
  self->info = state->info;

  VENC_CHN_ATTR_S *stAttr = &self->stAttr;
  memset(stAttr, 0, sizeof(VENC_CHN_ATTR_S));
  // Encoding
  // FIXME: settings
  stAttr->stVencAttr.enType = RK_VIDEO_ID_AVC;
  stAttr->stVencAttr.u32Profile = H264E_PROFILE_MAIN;
  stAttr->stVencAttr.enMirror = MIRROR_NONE;
  stAttr->stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
  stAttr->stRcAttr.stH264Cbr.u32BitRate = 3 * 1024;
  stAttr->stRcAttr.stH264Cbr.u32Gop = 1; // FIXME: this is BS
  // Correlate format
  stAttr->stVencAttr.enPixelFormat = RK_FMT_BUTT;
  GstVideoFormat gstFormat = GST_VIDEO_INFO_FORMAT(&state->info);
  for (int i = 0; i < COUNTOF(GST_RKMPI_FORMAT_ALIST); i++) {
    if (GST_RKMPI_FORMAT_ALIST[i].gst_format == gstFormat) {
      stAttr->stVencAttr.enPixelFormat = GST_RKMPI_FORMAT_ALIST[i].rkmpi_format;
      break;
    }
  }
  if (stAttr->stVencAttr.enPixelFormat == RK_FMT_BUTT) {
    // TODO: Reason not negotiated
    return FALSE;
  }
  // Width, height, stride et al
  RK_U32 width = GST_VIDEO_INFO_WIDTH(&self->info),
         height = GST_VIDEO_INFO_HEIGHT(&self->info);
  // stAttr.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
  stAttr->stVencAttr.u32PicWidth = width;
  stAttr->stVencAttr.u32PicHeight = height;
  // FIXME: proper stride
  stAttr->stVencAttr.u32VirWidth = width;
  stAttr->stVencAttr.u32VirHeight = height;
  stAttr->stVencAttr.u32StreamBufCnt = 2;
  stAttr->stVencAttr.u32BufSize = self->info.size;

  return TRUE;
}

static gboolean gst_rkmpi_h264_enc_start(GstVideoEncoder *encoder) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);
  
  RK_S32 rkret;
  // FIXME: shuffle this to ->prepare() (or whatever the READY callback is called)
  rkret = RK_MPI_SYS_Init();
  RK_MPI_ERROR_CHECK2(RK_MPI_SYS_Init)

  self->frameCounter = 0;
  gst_video_info_init(&self->info);

  MB_POOL_CONFIG_S PoolCfg;
  memset(&PoolCfg, 0, sizeof(MB_POOL_CONFIG_S));
  PoolCfg.u64MBSize = 1024 * 1024 * 16;
  PoolCfg.u32MBCnt = 1;
  PoolCfg.enAllocType = MB_ALLOC_TYPE_DMA;
  // PoolCfg.bPreAlloc = RK_FALSE;
  self->imagePool = RK_MPI_MB_CreatePool(&PoolCfg);

  RK_MPI_VENC_CreateChn(self->chnId, &self->stAttr);
  VENC_RECV_PIC_PARAM_S stRecvParam;
  memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
  stRecvParam.s32RecvPicNum = -1;
  RK_MPI_VENC_StartRecvFrame(self->chnId, &stRecvParam);

  return TRUE;
}

static gboolean gst_rkmpi_h264_enc_stop(GstVideoEncoder *encoder) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);

  RK_MPI_VENC_StopRecvFrame(self->chnId);
  RK_MPI_VENC_DestroyChn(self->chnId);

  RK_MPI_MB_DestroyPool(self->imagePool);

  return TRUE;
}

static GstFlowReturn gst_rkmpi_h264_handle_frame(GstVideoEncoder *encoder,
                                                 GstVideoCodecFrame *gstFrame) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);
  RK_S32 rkret = 0;

  GstMapInfo inputMapInfo;
  if (!gst_buffer_map(gstFrame->input_buffer, &inputMapInfo, GST_MAP_READ))
    return GST_FLOW_ERROR; // FIXME: log error
  MB_BLK src_Blk = RK_MPI_MB_GetMB(self->imagePool, inputMapInfo.size, RK_TRUE);
  if (!src_Blk)
    return GST_FLOW_ERROR; // FIXME: probably just flush now.
  // FIXME: investigate RK_MPI_MMZ_Fd2Handle. Maybe we do get zero copy...
  void *block = RK_MPI_MB_Handle2VirAddr(src_Blk);
  memcpy(block, inputMapInfo.data, inputMapInfo.size);
  // FIXME: sync
  gst_buffer_unmap(gstFrame->input_buffer, &inputMapInfo);

  VIDEO_FRAME_INFO_S frame;
  memset(&frame, 0, sizeof(VIDEO_FRAME_INFO_S));
  frame.stVFrame.enField = VIDEO_FIELD_FRAME;
  frame.stVFrame.enVideoFormat = VIDEO_FORMAT_LINEAR;
  frame.stVFrame.enPixelFormat = RK_FMT_RGB565;
  frame.stVFrame.enCompressMode = COMPRESS_MODE_NONE;
  frame.stVFrame.enDynamicRange = DYNAMIC_RANGE_SDR8;
  frame.stVFrame.enColorGamut = COLOR_GAMUT_BT601;
  frame.stVFrame.pVirAddr[0] = src_Blk;
  frame.stVFrame.pVirAddr[1] = NULL;

  frame.stVFrame.u32TimeRef = self->frameCounter++;
  frame.stVFrame.u64PTS = monotonic_millis();

  frame.stVFrame.u64PrivateData = 0;
  frame.stVFrame.u32FrameFlag = 0;
  // FIXME: error check
  rkret = RK_MPI_VENC_SendFrame(self->chnId, &frame, -1);
  RK_MPI_ERROR_CHECK(RK_MPI_VENC_SendFrame)

  // Get response bitstream
  VENC_STREAM_S response;
  response.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S));
  rkret = RK_MPI_VENC_GetStream(self->chnId, &response, -1);
  RK_MPI_ERROR_CHECK(RK_MPI_VENC_GetStream)

  // Output to new buffer
  if (GST_FLOW_OK != gst_video_encoder_allocate_output_frame(encoder, gstFrame, response.pstPack->u32Len))
    return FALSE; // FIXME: unmap, error logging
  GstMapInfo outputMapInfo;
  if (!gst_buffer_map(gstFrame->output_buffer, &outputMapInfo, GST_MAP_WRITE)) 
    return FALSE; // FIXME: error check
  void *response_data = RK_MPI_MB_Handle2VirAddr(response.pstPack->pMbBlk);
  memcpy(outputMapInfo.data, response_data, response.pstPack->u32Len);
  // FIXME: dma_buf_sync??
  gst_buffer_unmap(gstFrame->output_buffer, &outputMapInfo);
  if (GST_FLOW_OK != gst_video_encoder_finish_frame(encoder, gstFrame))
    return FALSE; // FIXME: error check

  return TRUE;
}

gboolean gst_rkmpih264_plugin_register(GstPlugin *plugin) {
  return gst_element_register(plugin, "rkmpih264enc", GST_RANK_PRIMARY,
                              GST_TYPE_RKMPIH264ENC);
}