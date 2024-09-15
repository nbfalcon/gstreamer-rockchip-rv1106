#include <gst/gst.h>
#include <rkaiq/common/rk_aiq.h>
#include <rkaiq/common/rk_aiq_types.h>
#include <rkaiq/uAPI2/rk_aiq_user_api2_sysctl.h>
#include <rkaiq/xcore/base/xcam_common.h>
#include <stdio.h>

typedef struct _GstRkaiqSrc {
  GstBin parent;       // The parent bin
  GstElement *v4l2src; // Element that will be populated with v4l2src

  rk_aiq_sys_ctx_t *rkaiq_ctx;
} GstRkaiqSrc;

typedef struct _GstRkaiqSrcClass {
  GstBinClass parent_class;
} GstRkaiqSrcClass;

#define GST_TYPE_RKAIQSRC (gst_rkaiqsrc_get_type())
#define GST_RKAIQSRC(obj)                                                      \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_RKAIQSRC, GstRkaiqSrc))
#define GST_RKAIQSRC_CLASS(klass)                                              \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_RKAIQSRC, GstRkaiqSrcClass))
#define GST_IS_RKAIQSRC(obj)                                                   \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_RKAIQSRC))
#define GST_IS_RKAIQSRC_CLASS(klass)                                           \
  (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_RKAIQSRC))

G_DEFINE_TYPE(GstRkaiqSrc, gst_rkaiqsrc, GST_TYPE_BIN);

// Forward declarations
static void gst_rkaiqsrc_dispose(GObject *object);
static void gst_rkaiqsrc_finalize(GObject *object);
static gboolean gst_rkaiqsrc_create_elements(GstRkaiqSrc *src);
static GstStateChangeReturn
gst_rkaiqsrc_change_state(GstElement *element, GstStateChange transition);

// Initialization function for the instance
static void gst_rkaiqsrc_init(GstRkaiqSrc *self) {
  // Create the v4l2src element
  self->v4l2src = gst_element_factory_make("v4l2src", "v4l2src");
  if (!self->v4l2src) {
    g_printerr("Failed to create v4l2src element\n");
  }
  gst_bin_add(GST_BIN(self), self->v4l2src);

  GstPad *v4l2src_src = gst_element_get_static_pad(self->v4l2src, "src");
  GstPad *mainpath_ghost = gst_ghost_pad_new("src_mainpath", v4l2src_src);
  g_object_unref(v4l2src_src);
  gst_element_add_pad(GST_ELEMENT(&self->parent), mainpath_ghost);
}

// Class initialization function
static void gst_rkaiqsrc_class_init(GstRkaiqSrcClass *klass) {
  GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
  element_class->change_state = gst_rkaiqsrc_change_state;
}

static int find_rkisp_mainpath() {
  int rkisp_index = -1;
  for (int i = 0; i < 64; i++) {
    char path[256];
    sprintf(path, "/sys/class/video4linux/video%d/name", i);

    FILE *fp = fopen(path, "rb");
    if (!fp)
      continue;

    char line[32];
    fgets(line, 32, fp);

    fclose(fp);

    if (strncmp(line, "rkisp_mainpath", 14) == 0) {
      rkisp_index = i;
      break;
    }
  }

  if (rkisp_index == -1) {
    fprintf(stderr, "cannot find v4l device with name rkisp_mainpath\n");
    return -1;
  }

  return rkisp_index;
}

#define RKAIQ_ERROR_CHECK(name) if (ret != XCAM_RETURN_NO_ERROR) { fprintf (stderr, "rkaiqsrc: %s failed: %d\n", #name, ret); return false; }

static _Bool rkaiqsrc_ready_setup(GstRkaiqSrc *self) {
  int rkisp_num = find_rkisp_mainpath();
  if (rkisp_num < 0) {
    return false;
  }

  char path[256];
  snprintf(path, sizeof(path), "/dev/video%d", rkisp_num);
  g_object_set(self->v4l2src, "device", path, NULL);

  // rkaiq setup
  XCamReturn ret;

  rk_aiq_static_info_t aiq_static_info;
  ret = rk_aiq_uapi2_sysctl_enumStaticMetasByPhyId(0, &aiq_static_info);
  RKAIQ_ERROR_CHECK(rk_aiq_uapi2_sysctl_enumStaticMetasByPhyId)

  const char *sns_entity_name = aiq_static_info.sensor_info.sensor_name;
  ret = rk_aiq_uapi2_sysctl_preInit_devBufCnt(sns_entity_name, "rkraw_rx", 2);
  RKAIQ_ERROR_CHECK(rk_aiq_uapi2_sysctl_preInit_devBufCnt)

  ret = rk_aiq_uapi2_sysctl_preInit_scene(sns_entity_name, "normal", "day");
  RKAIQ_ERROR_CHECK(rk_aiq_uapi2_sysctl_preInit_scene)

  rk_aiq_sys_ctx_t *aiq_ctx = rk_aiq_uapi2_sysctl_init(
      sns_entity_name, "/oem/usr/share/iqfiles", NULL, NULL);
  if (!aiq_ctx) {
    fprintf(stderr, "rkaiqsrc: rk_aiq_uapi2_sysctl_init() failed");
    return false;
  }
  self->rkaiq_ctx = aiq_ctx;

  // FIXME: auto-discover width-height?
  // https://www.gophotonics.com/products/cmos-image-sensors/smartsens-technology/21-1025-sc3336
  ret = rk_aiq_uapi2_sysctl_prepare(aiq_ctx, 1304, 2312,
                                    RK_AIQ_WORKING_MODE_NORMAL);
  RKAIQ_ERROR_CHECK(rk_aiq_uapi2_sysctl_prepare)

  return true;
}

static _Bool rkaiqsrc_start(GstRkaiqSrc *self) {
  // rkaiq setup
  XCamReturn ret;

  ret = rk_aiq_uapi2_sysctl_start(self->rkaiq_ctx);
  RKAIQ_ERROR_CHECK(rk_aiq_uapi2_sysctl_start)

  return true;
}

static _Bool rkaiqsrc_pause(GstRkaiqSrc *self) {
  XCamReturn ret;
  ret = rk_aiq_uapi2_sysctl_stop(self->rkaiq_ctx, false);
  RKAIQ_ERROR_CHECK(rk_aiq_uapi2_sysctl_stop)
  
  return true;
}

static _Bool rkaiqsrc_null(GstRkaiqSrc *self) {
  rk_aiq_uapi2_sysctl_deinit(self->rkaiq_ctx);
  return true;
}

static GstStateChangeReturn
gst_rkaiqsrc_change_state(GstElement *element, GstStateChange transition) {
  GstStateChangeReturn ret;
  GstRkaiqSrc *self = GST_RKAIQSRC(element);

  bool result = true;
  switch (transition) {
  case GST_STATE_CHANGE_NULL_TO_READY:
    result = rkaiqsrc_ready_setup(self);
    break;
  case GST_STATE_CHANGE_READY_TO_PAUSED:
    break;
  case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
    result = rkaiqsrc_start(self);
    break;
  case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    result = rkaiqsrc_pause(self);
    break;
  case GST_STATE_CHANGE_PAUSED_TO_READY:
    // Nothing to do, stream is already paused.
    break;
  case GST_STATE_CHANGE_READY_TO_NULL:
    result = rkaiqsrc_null(self);
    break;
  case GST_STATE_CHANGE_NULL_TO_NULL:
  case GST_STATE_CHANGE_READY_TO_READY:
  case GST_STATE_CHANGE_PAUSED_TO_PAUSED:
  case GST_STATE_CHANGE_PLAYING_TO_PLAYING:
    // Nothing to do at all
    break;
  }
  if (!result) {
    return GST_STATE_CHANGE_FAILURE;
  }

  // FIXME: [   38.967360] rkisp-vir0: MIPI drop frame
  // Maybe we need to stop the v4l2 stream before going to PAUSED in the rkaiq?

  // Chain up to the parent class's state change handler
  ret = GST_ELEMENT_CLASS(gst_rkaiqsrc_parent_class)
            ->change_state(element, transition);

  return ret;
}

gboolean rkaiqsrc_plugin_register(GstPlugin *plugin) {
  return gst_element_register(plugin, "rkaiqsrc", GST_RANK_PRIMARY,
                              gst_rkaiqsrc_get_type());
}

// GST_PLUGIN_DEFINE(GST_VERSION_MAJOR, GST_VERSION_MINOR, rkaiqsrc,
//                   "Rockchip rkaiq ISP source element",
//                   rkaiqsrc_plugin_register, VERSION, "MIT", "GStreamer",
//                   "https://gstreamer.freedesktop.org/" // FIXME
// )