GST_VERSION = \"1.14.4\"
GST_PACKAGE_NAME = '"GStreamer Rockchip Plug-ins"'

LUCKFOX_PICO_SDK_DIR ?= /home/nikita/src/luckfox-pico
RK_CFLAGS = -I$(LUCKFOX_PICO_SDK_DIR)/media/out/include
# librockchip_mpp.so is found in /oem/lib
RK_LDFLAGS = -Wl,-L$(LUCKFOX_PICO_SDK_DIR)/media/out/lib,-rpath=/oem/usr/lib
PKG_CONFIG ?= pkg-config

# Provided by BUILDROOT
# TOOLCHAIN_PATH ?= $(LUCKFOX_PICO_SDK_DIR)/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/
# TOOLCHAIN_PREFIX ?= arm-rockchip830-linux-uclibcgnueabihf-
# CC = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)gcc
# LD = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)ld

DEFS = -DVERSION=$(GST_VERSION) -DPACKAGE=\"gst-rockchip\" -DPACKAGE_VERSION=$(GST_VERSION) -DPACKAGE_NAME=$(GST_PACKAGE_NAME) -DGST_API_VERSION=$(GST_VERSION) -DGST_LICENSE=\"LGPL\"
DEFS += -DGST_PACKAGE_NAME=$(GST_PACKAGE_NAME) -DGST_PACKAGE_ORIGIN='"Cobbled together for Luckfox SDK"'
DEFS += -DUNUSED='__attribute__((unused))'
DEFS += -DHAVE_RGA=1

OBJECTS = gstmpp.o gstmppallocator.o gstmppalphadecodebin.o gstmppdec.o gstmppenc.o gstmpph264enc.o gstmpph265enc.o gstmppjpegdec.o gstmppjpegenc.o gstmppvideodec.o gstmppvp8enc.o gstmppvpxalphadecodebin.o gstrkaiqsrc.o gstrgaconvert.o gstrkmpih264enc.o gstrkmpi_common.o gstrkmpiallocator.o
DEPS = gstreamer-1.0 gstreamer-base-1.0 gstreamer-allocators-1.0 gstreamer-video-1.0 gstreamer-pbutils-1.0

CFLAGS += -fPIC
CFLAGS += -g3 # FIXME: this should be based on BR2_ENABLE_DEBUG

DEPS_CFLAGS = `$(PKG_CONFIG) --cflags $(DEPS)`
DEPS_LDFLAGS = `$(PKG_CONFIG) --libs $(DEPS)` -lrockchip_mpp -ldrm -lrga -lrkaiq -lrockit
LDFLAGS += -Wl,--no-undefined

.PHONY: all clean
all: libgstrockchipmpp.so
# FIMXE: remove clangd hacks
RKAIQ_INCLUDE = -I$(LUCKFOX_PICO_SDK_DIR)/media/out/include/rkaiq
CFLAGS += $(RKAIQ_INCLUDE) $(RKAIQ_INCLUDE)/algos $(RKAIQ_INCLUDE)/common $(RKAIQ_INCLUDE)/ipc_server $(RKAIQ_INCLUDE)/iq_parser $(RKAIQ_INCLUDE)/iq_parser_v2 $(RKAIQ_INCLUDE)/smartIr $(RKAIQ_INCLUDE)/uAPI2 $(RKAIQ_INCLUDE)/xcore
#end of clangd hacks

libgstrockchipmpp.so: $(OBJECTS)
	$(CC) -shared $^ -o $@ $(LDFLAGS) $(DEPS_LDFLAGS) $(RK_LDFLAGS)
	
%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS) $(DEPS_CFLAGS) $(RK_CFLAGS) $(DEFS)

clean:
	rm -f *.o libgstrockchipmpp.so
