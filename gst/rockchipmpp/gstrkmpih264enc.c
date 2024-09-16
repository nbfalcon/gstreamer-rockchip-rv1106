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
#include "rk_mpi_sys.h"
#include "rk_mpi_venc.h"

static uint64_t monotonic_micros() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)(ts.tv_sec * 1000 * 1000) + (ts.tv_nsec / 1000);
}

#define GST_CAT_DEFAULT gstrkpmpih264
GST_DEBUG_CATEGORY(GST_CAT_DEFAULT);

typedef struct _GstRKMPIH264Enc GstRKMPIH264Enc;
typedef struct _GstRKMPIH264EncClass GstRKMPIH264EncClass;

#define chnId 0

struct _GstRKMPIH264Enc {
  GstVideoEncoder parent;

  VENC_STREAM_S stFrame;
  MB_POOL src_Pool;
  MB_BLK src_Blk;
  void *src_BlkMMAP;

  RK_U32 frameCounter;

  GstVideoCodecState *state;
  GstVideoInfo info;
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
        GST_STATIC_CAPS("video/x-raw,"
                        "format = (string) { RGB, NV24, Y444 "
                        "}, " GST_RKMPI_H264ENC_SIZE_CAPS));

static void gst_rkmpi_h264enc_init(GstRKMPIH264Enc *element) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(element);
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
#define RK_MPI_ERROR_CHECK(name)                                               \
  if (rkret != RK_SUCCESS) {                                                   \
    fprintf(stderr, "rockit MPI: %s failed (%d)\n", #name, rkret);             \
    return GST_FLOW_ERROR;                                                     \
  }
#define RK_MPI_ERROR_CHECK2(name)                                              \
  if (rkret != RK_SUCCESS) {                                                   \
    fprintf(stderr, "rockit MPI: %s failed (%d)\n", #name, rkret);             \
    return FALSE;                                                              \
  }
#define RK_MPI_ERROR_CHECK_NULL(v, name)                                       \
  if (v) {                                                                     \
    fprintf(stderr, "rockit MPI: %s returned NULL!\n", #name);                 \
    return FALSE;                                                              \
  }

static gboolean gst_rkmpi_h264_enc_start(GstVideoEncoder *encoder) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);

  RK_S32 rkret;
  // FIXME: shuffle this to ->prepare() (or whatever the READY callback is
  // called)
  rkret = RK_MPI_SYS_Init();
  RK_MPI_ERROR_CHECK2(RK_MPI_SYS_Init)

  gst_video_info_init(&self->info);

  return TRUE;
}

static gboolean gst_gst2rkmpi_format(PIXEL_FORMAT_E *outFormat,
                                     GstVideoFormat inFormat) {
  for (int i = 0; i < COUNTOF(GST_RKMPI_FORMAT_ALIST); i++) {
    if (GST_RKMPI_FORMAT_ALIST[i].gst_format == inFormat) {
      *outFormat = GST_RKMPI_FORMAT_ALIST[i].rkmpi_format;
      return TRUE;
    }
  }
  gst_printerrln("Cannot handle video format '%d' (RKMPI does not support it)",
                 inFormat);
  return FALSE;
}

static gboolean gst_rkmpi_enc_set_src_caps(GstVideoEncoder *encoder,
                                           const char *media_type) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);
  GstVideoInfo *info = &self->info;
  GstVideoCodecState *output_state;

  GstCaps *caps = gst_caps_new_empty_simple(media_type);
  gst_caps_set_simple(caps, "stream-format", G_TYPE_STRING, "byte-stream",
                      "alignment", G_TYPE_STRING, "nalu", "width", G_TYPE_INT,
                      GST_VIDEO_INFO_WIDTH(info), "height", G_TYPE_INT,
                      GST_VIDEO_INFO_HEIGHT(info), NULL);

  GST_DEBUG_OBJECT(self, "output caps: %" GST_PTR_FORMAT, caps);

  output_state = gst_video_encoder_set_output_state(encoder, caps, self->state);

  GST_VIDEO_INFO_WIDTH(&output_state->info) = GST_VIDEO_INFO_WIDTH(info);
  GST_VIDEO_INFO_HEIGHT(&output_state->info) = GST_VIDEO_INFO_HEIGHT(info);
  gst_video_codec_state_unref(output_state);

  return gst_video_encoder_negotiate(encoder);
}

static gboolean gst_rkmpi_h264_enc_set_format(GstVideoEncoder *encoder,
                                              GstVideoCodecState *state) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);

  self->state = gst_video_codec_state_ref(state);
  self->info = state->info;
  self->frameCounter = 0;

  const RK_U32 width = GST_VIDEO_INFO_WIDTH(&self->info),
               height = GST_VIDEO_INFO_HEIGHT(&self->info);

  MB_POOL_CONFIG_S pool_cfg;
  memset(&pool_cfg, 0, sizeof(MB_POOL_CONFIG_S));
  pool_cfg.u64MBSize = width * height * 3;
  pool_cfg.u32MBCnt = 1;
  pool_cfg.enAllocType = MB_ALLOC_TYPE_DMA;
  // PoolCfg.bPreAlloc = RK_FALSE;
  self->src_Pool = RK_MPI_MB_CreatePool(&pool_cfg);
  RK_MPI_ERROR_CHECK_NULL(self->src_Pool, RK_MPI_MB_CreatePool)
  self->src_Blk = RK_MPI_MB_GetMB(self->src_Pool, width * height * 3, RK_TRUE);
  self->src_BlkMMAP = RK_MPI_MB_Handle2VirAddr(self->src_Blk);

  VENC_CHN_ATTR_S stAttr;
  memset(&stAttr, 0, sizeof(VENC_CHN_ATTR_S));
  stAttr.stVencAttr.enType = RK_VIDEO_ID_AVC;
  if (!gst_gst2rkmpi_format(&stAttr.stVencAttr.enPixelFormat,
                            GST_VIDEO_INFO_FORMAT(&self->info)))
    return FALSE;
  stAttr.stVencAttr.u32Profile = H264E_PROFILE_MAIN;
  stAttr.stVencAttr.u32PicWidth = width;
  stAttr.stVencAttr.u32PicHeight = height;
  stAttr.stVencAttr.u32VirWidth = width;
  stAttr.stVencAttr.u32VirHeight = height;
  stAttr.stVencAttr.u32StreamBufCnt = 2;
  stAttr.stVencAttr.u32BufSize = width * height * 3 / 2;
  stAttr.stVencAttr.enMirror = MIRROR_NONE;

  stAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
  stAttr.stRcAttr.stH264Cbr.u32BitRate = 3 * 1024;
  stAttr.stRcAttr.stH264Cbr.u32Gop = 10;
  RK_MPI_VENC_CreateChn(chnId, &stAttr);

  VENC_RECV_PIC_PARAM_S stRecvParam;
  memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
  stRecvParam.s32RecvPicNum = -1;
  RK_MPI_VENC_StartRecvFrame(chnId, &stRecvParam);

  self->frameCounter = 0;
  self->state = gst_video_codec_state_ref(state);
  self->info = state->info;

  return gst_rkmpi_enc_set_src_caps(encoder, "video/x-h264");
}

static gboolean gst_rkmpi_h264_enc_stop(GstVideoEncoder *encoder) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);

  // FIXME: error check?
  RK_MPI_VENC_StopRecvFrame(chnId);
  RK_MPI_VENC_DestroyChn(chnId);

  RK_MPI_MB_ReleaseMB(self->src_Blk);
  RK_MPI_MB_DestroyPool(self->src_Pool);

  RK_MPI_SYS_Exit();

  gst_video_codec_state_unref(self->state);
  GST_DEBUG_OBJECT(self, "stopped");

  return TRUE;
}

static GstFlowReturn gst_rkmpi_h264_handle_frame(GstVideoEncoder *encoder,
                                                 GstVideoCodecFrame *gstFrame) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);
  RK_S32 rkret = 0;

  GstMapInfo inputMapInfo;
  if (!gst_buffer_map(gstFrame->input_buffer, &inputMapInfo, GST_MAP_READ))
    return GST_FLOW_ERROR; // FIXME: log error
  memcpy(self->src_BlkMMAP, inputMapInfo.data, inputMapInfo.size);
  rkret = RK_MPI_SYS_MmzFlushCache(self->src_Blk, RK_FALSE);
  RK_MPI_ERROR_CHECK(RK_MPI_SYS_MmzFlushCache)
  gst_buffer_unmap(gstFrame->input_buffer, &inputMapInfo);

  RK_U32 width = GST_VIDEO_INFO_WIDTH(&self->info),
         height = GST_VIDEO_INFO_HEIGHT(&self->info);
  VIDEO_FRAME_INFO_S h264_frame;
  h264_frame.stVFrame.u32Width = width;
  h264_frame.stVFrame.u32Height = height;
  h264_frame.stVFrame.u32VirWidth = width;
  h264_frame.stVFrame.u32VirHeight = height;
  if (!gst_gst2rkmpi_format(&h264_frame.stVFrame.enPixelFormat,
                            GST_VIDEO_INFO_FORMAT(&self->info)))
    return FALSE;
  h264_frame.stVFrame.u32FrameFlag = 160;
  h264_frame.stVFrame.pMbBlk = self->src_Blk;
  h264_frame.stVFrame.u32TimeRef = self->frameCounter++;
  h264_frame.stVFrame.u64PTS = monotonic_micros();

  rkret = RK_MPI_VENC_SendFrame(chnId, &h264_frame, -1);
  RK_MPI_ERROR_CHECK(RK_MPI_VENC_SendFrame)

  // Get response bitstream
  VENC_STREAM_S stFrame;
  stFrame.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S));
  rkret = RK_MPI_VENC_GetStream(0, &stFrame, -1);
  RK_MPI_ERROR_CHECK(RK_MPI_VENC_GetStream)

  // Output to new buffer
  if (GST_FLOW_OK != gst_video_encoder_allocate_output_frame(
                         encoder, gstFrame, stFrame.pstPack->u32Len))
    return FALSE; // FIXME: unmap, error logging
  GstMapInfo outputMapInfo;
  if (!gst_buffer_map(gstFrame->output_buffer, &outputMapInfo, GST_MAP_WRITE))
    return FALSE; // FIXME: error check
  void *response_data = RK_MPI_MB_Handle2VirAddr(stFrame.pstPack->pMbBlk);
  memcpy(outputMapInfo.data, response_data, stFrame.pstPack->u32Len);
  // FIXME: dma_buf_sync??
  rkret = RK_MPI_VENC_ReleaseStream(chnId, &stFrame);
  RK_MPI_ERROR_CHECK(RK_MPI_VENC_ReleaseStream)
  gst_buffer_unmap(gstFrame->output_buffer, &outputMapInfo);
  if (GST_FLOW_OK != gst_video_encoder_finish_frame(encoder, gstFrame))
    return GST_FLOW_ERROR; // FIXME: error check

  return GST_FLOW_OK;
}

gboolean gst_rkmpih264_plugin_register(GstPlugin *plugin) {
  return gst_element_register(plugin, "rkmpih264enc", GST_RANK_PRIMARY,
                              GST_TYPE_RKMPIH264ENC);
}