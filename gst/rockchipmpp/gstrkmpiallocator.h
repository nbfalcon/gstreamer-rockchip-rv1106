#pragma once

#include "rk_mpi_vi.h"
#include <gst/allocators/allocators.h>

#define GST_RKMPI_ALLOCATOR_NAME "rockchip-rkmpi"

typedef struct _GstRkmpiAllocator GstRkmpiAllocator;

GstMemory *gst_rkmpi_allocator_import_mb(GstRkmpiAllocator *self, MB_BLK blk);
GstMemory *gst_rkmpi_allocator_import_viframe(GstRkmpiAllocator *self,
                                              const VIDEO_FRAME_INFO_S *info);
/// An allocator that only supports _import()
GstRkmpiAllocator *gst_rkmpi_allocator_new_empty();
GstRkmpiAllocator *gst_rkmpi_allocator_new_pool(gsize size, gsize count);

MB_BLK gst_rkmpi_allocator_mem_get_mb(GstMemory *mem);