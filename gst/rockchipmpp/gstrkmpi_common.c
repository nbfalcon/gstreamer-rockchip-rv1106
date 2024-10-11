#include "rk_mpi_sys.h"
#include <glib.h>
#include <gst/gst.h>

static GMutex gst_rkmpi_init_mutex;
static int gst_rkmpi_init_recount = 0;

gboolean gst_rkmpi_init() {
  gboolean success = TRUE;
  g_mutex_lock(&gst_rkmpi_init_mutex);
  if (gst_rkmpi_init_recount++ == 0) {
    RK_S32 rkret = RK_MPI_SYS_Init();
    if (RK_SUCCESS != rkret) {
      gst_printerr("RK_MPI_SYS_INIT failed: %d", rkret);
      success = FALSE;
    }
  }
  g_mutex_unlock(&gst_rkmpi_init_mutex);
  return success;
}

void gst_rkmpi_exit() {
  g_mutex_lock(&gst_rkmpi_init_mutex);
  if (--gst_rkmpi_init_recount == 0) {
    RK_S32 rkret = RK_MPI_SYS_Exit();
    if (rkret != RK_SUCCESS) {
      gst_printerr("RK_MPI_SYS_EXIT failed: %d", rkret);
    }
  }
  g_mutex_unlock(&gst_rkmpi_init_mutex);
}