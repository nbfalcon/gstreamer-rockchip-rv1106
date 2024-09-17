// License: GPLv3
#include <gst/gst.h>
#include <gst/video/gstvideoencoder.h>
#include <glib/gqueue.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rk_mpi_mb.h"
#include "rk_mpi_mmz.h"
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

// We need to do this dance because g_async_queue_push cannot accept NULL pointers
struct QueuedGstFrame {
  GstVideoCodecFrame *frame_or_null;
  uint32_t u32SeqNo;
};

static struct QueuedGstFrame *queued_gst_frame_new(GstVideoCodecFrame *frame, uint32_t seqno) {
  struct QueuedGstFrame *ret = g_malloc(sizeof(struct QueuedGstFrame));
  ret->frame_or_null = frame;
  ret->u32SeqNo = seqno;
  return ret;
}

struct _GstRKMPIH264Enc {
  // Gstreamer
  GstVideoEncoder parent;
  GstVideoCodecState *state;
  GstVideoInfo info;

  // State
  _Atomic uint32_t input_frame_counter;
  // What buffer do we expect to be dequeued next?
  _Atomic uint32_t output_frame_counter;
  /// Type: QueuedGstFrame
  GAsyncQueue *gstframe_queue;

  // Source image (1)
  MB_POOL src_Pool;
  MB_BLK src_Blk;
  void *src_BlkMMAP;
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

#define GST_RK_ALIST_XMACRO(X, XLAST) \
  X(NV12, RK_FMT_YUV420SP) \
  X(NV12_10LE32, RK_FMT_YUV420SP_10BIT) \
  X(NV16, RK_FMT_YUV422SP) \
  X(P010_10LE, RK_FMT_YUV422SP_10BIT) \
  X(I420, RK_FMT_YUV420P) \
  X(YV12, RK_FMT_YUV420P_VU) \
  X(NV21, RK_FMT_YUV420SP_VU) \
  X(Y42B, RK_FMT_YUV422P) \
  X(VYUY, RK_FMT_YUV422SP_VU) \
  X(YUY2, RK_FMT_YUV422_YUYV) \
  X(UYVY, RK_FMT_YUV422_UYVY) \
  X(GRAY8, RK_FMT_YUV400SP) \
  X(Y444, RK_FMT_YUV444) \
  X(RGB16, RK_FMT_RGB565) \
  X(BGR16, RK_FMT_BGR565) \
  X(RGB15, RK_FMT_RGB555) \
  X(BGR15, RK_FMT_BGR555) \
  X(RGB, RK_FMT_RGB888) \
  X(BGR, RK_FMT_BGR888) \
  X(ARGB, RK_FMT_ARGB8888) \
  X(ABGR, RK_FMT_ABGR8888) \
  X(BGRA, RK_FMT_BGRA8888) \
  X(RGBA, RK_FMT_RGBA8888) \
  X(YVYU, RK_FMT_YUV422_YVYU) \
  X(VYUY, RK_FMT_YUV422_VYUY) \
  X(NV16, RK_FMT_YUV422SP) \
  X(NV61, RK_FMT_YUV422SP_VU) \
  X(NV24, RK_FMT_YUV444SP) \
  X(RGB16, RK_FMT_RGB565) \
  XLAST(BGR16, RK_FMT_BGR565)
// FIXME: there are some missing entries. Also, some of these might be
// ChatGPT hallucinated
// FIXME: Bayer formats

#define GST_RK_ALIST_E(gst, rk) { GST_VIDEO_FORMAT_ ## gst, rk },
static struct gst_rkmpi_format {
  GstVideoFormat gst_format;
  PIXEL_FORMAT_E rkmpi_format;
  const char *gst_string;
} GST_RKMPI_FORMAT_ALIST[] = {
    GST_RK_ALIST_XMACRO(GST_RK_ALIST_E, GST_RK_ALIST_E)
};


#define GST_RKMPI_H264ENC_SIZE_CAPS                                            \
  "width  = (int) [ 96, MAX ], height = (int) [ 64, MAX ]"
static GstStaticPadTemplate gst_rkmpi_h264enc_src_template =
    GST_STATIC_PAD_TEMPLATE(
        "src", GST_PAD_SRC, GST_PAD_ALWAYS,
        GST_STATIC_CAPS("video/x-h264, " GST_RKMPI_H264ENC_SIZE_CAPS ","
                                                                     "stream-format = (string) { byte-stream }, "
                                                                     "alignment = (string) { au }, "
                                                                     "profile = (string) { baseline, main, high }"));
#define GST_RK_ALIST_CAPS_E(gst, rk) #gst ", "
#define GST_RK_ALIST_CAPS_L(gst, rk) #gst
static GstStaticPadTemplate gst_rkmpih264enc_sink_template =
    GST_STATIC_PAD_TEMPLATE(
        "sink", GST_PAD_SINK, GST_PAD_ALWAYS,
        GST_STATIC_CAPS("video/x-raw, "
                        "format = (string) { " GST_RK_ALIST_XMACRO(GST_RK_ALIST_CAPS_E, GST_RK_ALIST_CAPS_L) " }, "
                        GST_RKMPI_H264ENC_SIZE_CAPS));

static void gst_rkmpi_h264enc_init(GstRKMPIH264Enc *element) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(element);
}

static gboolean gst_rkmpi_h264enc_start(GstVideoEncoder *encoder);
static gboolean gst_rkmpi_h264enc_stop(GstVideoEncoder *encoder);
static gboolean gst_rkmpi_h264enc_set_format(GstVideoEncoder *encoder,
                                             GstVideoCodecState *state);
static GstFlowReturn gst_rkmpi_h264enc_finish(GstVideoEncoder *encoder);
static GstFlowReturn gst_rkmpi_h264enc_handle_frame(GstVideoEncoder *self,
                                                    GstVideoCodecFrame *frame);
static void gst_rkmpi_h264enc_class_init(GstRKMPIH264EncClass *klass) {
  GstVideoEncoderClass *video_encoder = GST_VIDEO_ENCODER_CLASS(klass);
  video_encoder->start = gst_rkmpi_h264enc_start;
  video_encoder->stop = gst_rkmpi_h264enc_stop;
  video_encoder->finish = gst_rkmpi_h264enc_finish; // FIXME: maybe implement flush?
  video_encoder->set_format = gst_rkmpi_h264enc_set_format;
  video_encoder->handle_frame = gst_rkmpi_h264enc_handle_frame;

  GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
  gst_element_class_add_pad_template(
      element_class,
      gst_static_pad_template_get(&gst_rkmpi_h264enc_src_template));
  gst_element_class_add_pad_template(
      element_class,
      gst_static_pad_template_get(&gst_rkmpih264enc_sink_template));
  gst_element_class_set_static_metadata(
      element_class, "Rockchip Rockcit H264 Encoder", "Codec/Encoder/Video",
      "Encode video streams via Rockchip rockit/RKMPI",
      "Nikita <nikblos@outlook.com>");
}

#define COUNTOF(x) (sizeof(x) / sizeof((x)[0]))
#define RK_MPI_ERROR_CHECK(name)                                               \
  if (rkret != RK_SUCCESS) {                                                   \
    fprintf(stderr, "rockit MPI: %s failed (%d)\n", #name, rkret);             \
    return GST_FLOW_ERROR;                                                     \
  }
#define RK_MPI_ERROR_CHECKV(name)                                               \
  if (rkret != RK_SUCCESS) {                                                   \
    fprintf(stderr, "rockit MPI: %s failed (%d)\n", #name, rkret);             \
    return;                                                     \
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

static void gstvideocodecframe_unref2(void *frame) {
  // We can push null
  if (frame) {
    gst_video_codec_frame_unref((GstVideoCodecFrame *) frame);
  }
}

static gboolean gst_rkmpi_h264enc_start(GstVideoEncoder *encoder) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);

  RK_S32 rkret;
  // FIXME: shuffle this to ->prepare() (or whatever the READY callback is
  // called)
  rkret = RK_MPI_SYS_Init();
  RK_MPI_ERROR_CHECK2(RK_MPI_SYS_Init)

  gst_video_info_init(&self->info);
  // FIXME: is this type of cast legal?
  self->gstframe_queue = g_async_queue_new_full(gstvideocodecframe_unref2);
  // FIXME: NULLCHECK queue alloc

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

static void gst_rkmpi_buffer_loop(gpointer encoder) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);

  // Next frame we expect
  struct QueuedGstFrame *gst_frame_w = g_async_queue_pop(self->gstframe_queue); // FIXME: this can block forever
  GstVideoCodecFrame *gst_frame = gst_frame_w->frame_or_null;
  uint32_t frame_seqno = gst_frame_w->u32SeqNo;
  g_free(gst_frame_w);
  if (!gst_frame)
    return; // Poison pill

  // Get response bitstream
  RK_S32 rkret;
  VENC_PACK_S pack;
  VENC_STREAM_S stFrame;
  stFrame.pstPack = &pack; // This is actually an array, but we have size 1
  stFrame.u32PackCount = 1;
  stFrame.u32Seq = self->output_frame_counter++;
  rkret = RK_MPI_VENC_GetStream(0, &stFrame, -1);
  RK_MPI_ERROR_CHECKV(RK_MPI_VENC_GetStream)

  gst_println("rkmpi: successfully dequeued stream packet %d (expect %d, length %d)",
              stFrame.u32Seq,
              frame_seqno,
              stFrame.pstPack->u32Len);

  // Output to new buffer
  if (GST_FLOW_OK != gst_video_encoder_allocate_output_frame(
      encoder, gst_frame, stFrame.pstPack->u32Len))
    return; // FIXME: unmap, error logging
  GstMapInfo outputMapInfo;
  if (!gst_buffer_map(gst_frame->output_buffer, &outputMapInfo, GST_MAP_WRITE))
    return; // FIXME: error check
  void *response_data = RK_MPI_MB_Handle2VirAddr(stFrame.pstPack->pMbBlk);
  memcpy(outputMapInfo.data, response_data, stFrame.pstPack->u32Len);
  rkret = RK_MPI_VENC_ReleaseStream(chnId, &stFrame);
  RK_MPI_ERROR_CHECKV(RK_MPI_VENC_ReleaseStream)
  gst_buffer_unmap(gst_frame->output_buffer, &outputMapInfo);
  if (GST_FLOW_OK != gst_video_encoder_finish_frame(encoder, gst_frame))
    return; // FIXME: error check;
}

static gboolean gst_rkmpi_h264enc_set_format(GstVideoEncoder *encoder,
                                             GstVideoCodecState *state) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);

  self->state = gst_video_codec_state_ref(state);
  self->info = state->info;
  self->output_frame_counter = self->input_frame_counter = 0;

  const RK_U32 width = GST_VIDEO_INFO_WIDTH(&self->info),
      height = GST_VIDEO_INFO_HEIGHT(&self->info);
  const RK_U32 size = GST_VIDEO_INFO_SIZE(&self->info);

  MB_POOL_CONFIG_S pool_cfg;
  memset(&pool_cfg, 0, sizeof(MB_POOL_CONFIG_S));
  pool_cfg.u64MBSize = size;
  pool_cfg.u32MBCnt = 1;
  pool_cfg.enAllocType = MB_ALLOC_TYPE_DMA;
  // PoolCfg.bPreAlloc = RK_FALSE;
  self->src_Pool = RK_MPI_MB_CreatePool(&pool_cfg);
  RK_MPI_ERROR_CHECK_NULL(self->src_Pool, RK_MPI_MB_CreatePool)
  self->src_Blk = RK_MPI_MB_GetMB(self->src_Pool, size, RK_TRUE);
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
  stAttr.stVencAttr.u32BufSize = 0; // Doesn't actually matter for some reason
  stAttr.stVencAttr.enMirror = MIRROR_NONE;

  stAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
  stAttr.stRcAttr.stH264Cbr.u32BitRate = 3 * 1024;
  stAttr.stRcAttr.stH264Cbr.u32Gop = 4000;
  RK_MPI_VENC_CreateChn(chnId, &stAttr);

  VENC_RECV_PIC_PARAM_S stRecvParam;
  memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
  stRecvParam.s32RecvPicNum = -1;
  RK_MPI_VENC_StartRecvFrame(chnId, &stRecvParam);

  gst_pad_start_task(encoder->srcpad, gst_rkmpi_buffer_loop, self, NULL);

  return gst_rkmpi_enc_set_src_caps(encoder, "video/x-h264");
}

static GstFlowReturn gst_rkmpi_h264enc_finish(GstVideoEncoder *encoder) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);

  RK_S32 rkret;
  rkret = RK_MPI_VENC_StopRecvFrame(chnId);
  // Hey, please generate the EOS bitstream
  RK_MPI_ERROR_CHECK(RK_MPI_VENC_StopRecvFrame);

  if (gst_pad_get_task_state(encoder->srcpad) == GST_TASK_STARTED) {
    g_async_queue_push(self->gstframe_queue, queued_gst_frame_new(NULL, -1));
    // NOTE: Wait what, UNLOCK then LOCK??? This looks horrible, but just seems to be how you do things in Gstreamer
    // https://gitlab.freedesktop.org/gstreamer/gst-plugins-good/-/blob/6525abfc63917e3f92e359f68a02cf65c8fda7d8/sys/v4l2/gstv4l2videoenc.c#L271
    GST_VIDEO_ENCODER_STREAM_UNLOCK(encoder);
    if (!gst_pad_stop_task(encoder->srcpad))
      return GST_FLOW_ERROR;
    GST_VIDEO_ENCODER_STREAM_LOCK(encoder);
  }

  return GST_FLOW_OK;
}

static gboolean gst_rkmpi_h264enc_stop(GstVideoEncoder *encoder) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);

  RK_MPI_VENC_DestroyChn(chnId);

  RK_MPI_MB_ReleaseMB(self->src_Blk);
  RK_MPI_MB_DestroyPool(self->src_Pool);

  RK_MPI_SYS_Exit();

  gst_video_codec_state_unref(self->state);
  GST_DEBUG_OBJECT(self, "stopped");

  return TRUE;
}

static GstFlowReturn gst_rkmpi_h264enc_handle_frame(GstVideoEncoder *encoder,
                                                    GstVideoCodecFrame *frame) {
  GstRKMPIH264Enc *self = GST_RKMPIH264ENC(encoder);
  RK_S32 rkret = 0;

  GstMapInfo inputMapInfo;
  if (!gst_buffer_map(frame->input_buffer, &inputMapInfo, GST_MAP_READ))
    return GST_FLOW_ERROR; // FIXME: log error
  memcpy(self->src_BlkMMAP, inputMapInfo.data, inputMapInfo.size);
  rkret = RK_MPI_SYS_MmzFlushCache(self->src_Blk, RK_FALSE);
  RK_MPI_ERROR_CHECK(RK_MPI_SYS_MmzFlushCache)
  gst_buffer_unmap(frame->input_buffer, &inputMapInfo);

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
  h264_frame.stVFrame.u32FrameFlag = 0;
  h264_frame.stVFrame.pMbBlk = self->src_Blk;
  uint32_t next_frame_counter = self->input_frame_counter++;
  h264_frame.stVFrame.u32TimeRef = self->input_frame_counter;
  h264_frame.stVFrame.u64PTS = monotonic_micros(); // FIXME: frame->pts

  g_async_queue_push(self->gstframe_queue, queued_gst_frame_new(frame, next_frame_counter));
  rkret = RK_MPI_VENC_SendFrame(chnId, &h264_frame, -1);
  RK_MPI_ERROR_CHECK(RK_MPI_VENC_SendFrame)

  return GST_FLOW_OK;
}

gboolean gst_rkmpih264_plugin_register(GstPlugin *plugin) {
  return gst_element_register(plugin, "rkmpih264enc", GST_RANK_PRIMARY,
                              GST_TYPE_RKMPIH264ENC);
}