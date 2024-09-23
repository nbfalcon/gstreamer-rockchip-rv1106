#include <gst/allocators/allocators.h>
#include <gst/gst.h>
#include <gst/gstmemory.h>

#include "rk_mpi_mb.h"
#include "rk_mpi_sys.h"
#include "rk_mpi_vi.h"

typedef struct _GstRkmpiAllocator {
  GstAllocator parent;

  MB_POOL pool;
  MB_POOL_CONFIG_S pool_config;
} GstRkmpiAllocator;

typedef struct _GstRkmpiAllocatorClass {
  GstAllocatorClass parent_class;
} GstRkmpiAllocatorClass;

#define GST_TYPE_RKMPI_ALLOCATOR (gst_rkmpi_allocator_get_type())
#define GST_RKMPI_ALLOCATOR(obj)                                               \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_RKMPI_ALLOCATOR,                 \
                              GstRkmpiAllocator))
#define GST_RKMPI_ALLOCATOR_CLASS(klass)                                       \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_RKMPI_ALLOCATOR,                  \
                           GstRkmpiAllocatorClass))
#define GST_IS_RKMPI_ALLOCATOR(obj)                                            \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_RKMPI_ALLOCATOR))
#define GST_IS_RKMPI_ALLOCATOR_CLASS(klass)                                    \
  (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_RKMPI_ALLOCATOR))

#define gst_rkmpi_allocator_parent_class parent_class
G_DEFINE_TYPE(GstRkmpiAllocator, gst_rkmpi_allocator, GST_TYPE_ALLOCATOR)

struct GstRkmpiMemory {
  GstMemory parent;
  // The MB_BLK we got from the MB_POOL.
  MB_BLK blk;

  // Optional: did we get the frame from ViGetChnFrame? Then we story the info,
  // so we can call ViChnFree.
  gboolean haveViInfo;
  VI_CHN viChn;
  VI_PIPE viPipe;
  VIDEO_FRAME_INFO_S viInfo;
};

#define GST_RKMPI_MEMORY(mem) (struct GstRkmpiMemory *)mem

static gpointer gst_rkmpi_allocator_mem_map(GstMemory *mem, gsize maxsize,
                                            GstMapFlags flags) {
  struct GstRkmpiMemory *self = GST_RKMPI_MEMORY(mem);

  return RK_MPI_MB_Handle2VirAddr(self->blk);
}

static void gst_rkmpi_allocator_mem_unmap(GstMemory *mem) {
  struct GstRkmpiMemory *self = GST_RKMPI_MEMORY(mem);

  RK_MPI_SYS_MmzFlushCache(self->blk, RK_FALSE);
}

static void gst_rkmpi_allocator_free(GstAllocator *allocator, GstMemory *mem) {
  GstRkmpiAllocator *self = GST_RKMPI_ALLOCATOR(self);
  struct GstRkmpiMemory *ctx = GST_RKMPI_MEMORY(mem);

  if (ctx->haveViInfo) {
    RK_MPI_VI_ReleaseChnFrame(ctx->viPipe, ctx->viChn, &ctx->viInfo);
  } else {
    RK_MPI_MB_ReleaseMB(ctx->blk);
  }
  GST_ALLOCATOR_CLASS(parent_class)->free(allocator, mem);
}

static GstMemory *gst_rkmpi_allocator_make_mem_import(GstRkmpiAllocator *self, MB_BLK blk,
                                            const VIDEO_FRAME_INFO_S *vInfo,
                                            gsize size) {
  // NOTE: How do we set a custom memory type? We don't. gst_memory_new_wrapped
  // doesn't either, and it does have a custom allocator.

  struct GstRkmpiMemory *xmem = g_malloc0(sizeof(struct GstRkmpiMemory));
  gst_memory_init(&xmem->parent, 0, GST_ALLOCATOR(self), NULL,
                  self->pool_config.u64MBSize, 4096, 0, size);

  xmem->blk = blk;
  xmem->haveViInfo = FALSE;
  if (vInfo) {
    xmem->haveViInfo = TRUE;
    xmem->viInfo = *vInfo;
  }

  return (GstMemory *)xmem;
}

// RKMPI memory allocation function
static GstMemory *gst_rkmpi_allocator_alloc(GstAllocator *allocator, gsize size,
                                            GstAllocationParams *params) {
  GstRkmpiAllocator *self = GST_RKMPI_ALLOCATOR(allocator);

  if (!self->pool) {
    gst_printerr("gst_rkmpi_allocator_alloc(): this allocator is configured "
                 "for import only!");
    return NULL;
  }

  MB_BLK blk = RK_MPI_MB_GetMB(self->pool, size, RK_FALSE);
  if (!blk)
    return NULL;
  return gst_rkmpi_allocator_make_mem_import(self, blk, NULL, size);
}

static void gst_rkmpi_allocator_finalize(GObject *obj) {
  GstRkmpiAllocator *self = GST_RKMPI_ALLOCATOR(obj);

  if (self->pool) {
    RK_MPI_MB_DestroyPool(self->pool);
  }
  G_OBJECT_CLASS(parent_class)->finalize(obj);
}

static void gst_rkmpi_allocator_class_init(GstRkmpiAllocatorClass *klass) {
  GstAllocatorClass *allocator_class = GST_ALLOCATOR_CLASS(klass);
  allocator_class->alloc = gst_rkmpi_allocator_alloc;
  allocator_class->free = gst_rkmpi_allocator_free;

  GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
  gobject_class->finalize = gst_rkmpi_allocator_finalize;
}

static void gst_rkmpi_allocator_init(GstRkmpiAllocator *self) {
  GstAllocator *allocator = GST_ALLOCATOR_CAST(self);
  allocator->mem_map = gst_rkmpi_allocator_mem_map;
  allocator->mem_unmap = gst_rkmpi_allocator_mem_unmap;

  self->pool = 0; // FIXME: is this ok
}

GstMemory *gst_rkmpi_allocator_import_mb(GstRkmpiAllocator *self, MB_BLK blk) {
  return gst_rkmpi_allocator_make_mem_import(self, blk, NULL, RK_MPI_MB_GetSize(blk));
}

GstMemory *gst_rkmpi_allocator_import_viframe(GstRkmpiAllocator *self,
                                              const VIDEO_FRAME_INFO_S *info) {
  MB_BLK blk = info->stVFrame.pMbBlk;
  return gst_rkmpi_allocator_make_mem_import(self, blk, info, RK_MPI_MB_GetSize(blk));
}

/// An allocator that only supports _import()
GstRkmpiAllocator *gst_rkmpi_allocator_new_empty() {
  return g_object_new(GST_TYPE_RKMPI_ALLOCATOR, NULL);
}

GstRkmpiAllocator *gst_rkmpi_allocator_new_pool(gsize size, gsize count) {
  MB_POOL_CONFIG_S pool_config = {
      .bPreAlloc = RK_FALSE,
      .enAllocType = MB_ALLOC_TYPE_DMA,
      .enDmaType = MB_DMA_TYPE_CMA,
      .enRemapMode = MB_REMAP_MODE_CACHED,
      .u32MBCnt = count,
      .u64MBSize = size,
  };
  MB_POOL pool = RK_MPI_MB_CreatePool(&pool_config);
  if (!pool) {
    gst_print("RK_MPI_MB_CreatePool failed!");
    return NULL;
  }

  GstRkmpiAllocator *alloc = gst_rkmpi_allocator_new_empty();
  alloc->pool = pool;
  alloc->pool_config = pool_config;
  return alloc;
}