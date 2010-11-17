all: roboio

#TARBALL_URL = http://www.roboard.com/Files/Code/RoBoIO-src-v1.5b-gcc.zip
#TARBALL_URL = http://www.roboard.com/Files/Code/RoBoIO-src-v1.6-gcc.zip
TARBALL_URL = http://www.roboard.com/Files/Code/RoBoIO-src-v1.61-gcc.zip
TARBALL = build/RoBoIO-src-v1.61-gcc.zip
SOURCE_DIR = build/RoBoIO-src-v1.61-gcc
UNPACK_CMD = unzip -o
INITIAL_DIR = build/RoBoIO
#MD5SUM_FILE = 

include $(shell rospack find mk)/download_unpack_build.mk

#	cd $(SOURCE_DIR) && patch -p1 -i ../../RoBoIO-src-v1.5b-gcc-veltrop-1.0.patch
#	cd $(SOURCE_DIR) && patch -p1 -i ../../RoBoIO-src-v1.6-gcc-veltrop-1.0.patch
$(SOURCE_DIR)/patched:
	cd $(SOURCE_DIR) && patch -p1 -i ../../RoBoIO-src-v1.6-gcc-veltrop-1.0.patch
	touch $(SOURCE_DIR)/patched

roboio: $(SOURCE_DIR)/unpacked
#roboio: $(SOURCE_DIR)/unpacked $(SOURCE_DIR)/patched
	mkdir -p lib include
	cd $(SOURCE_DIR) && make -f Make.LINUX 
	cp -f $(SOURCE_DIR)/libRBIO.a lib
	cp -f $(SOURCE_DIR)/libsrc/*.h include
	touch lib
	touch include

clean:
	cd $(SOURCE_DIR) && make -f Make.LINUX clean
	rm -f lib/* include/*

wipe:
	-rm -rf build lib include $(SOURCE_DIR)
