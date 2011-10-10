all: vr920

TARBALL_URL = http://www.mygnu.de/wp-content/plugins/download-monitor/download.php?id=55
TARBALL = build/vrtrack-1.0.tar.gz
SOURCE_DIR = build/vrtrack-1.0
#UNPACK_CMD = unzip -o
#INITIAL_DIR = build/vrtrack-1.0
#MD5SUM_FILE = 

include $(shell rospack find mk)/download_unpack_build.mk

vr920: $(SOURCE_DIR)/unpacked
	mkdir -p bin
	cd $(SOURCE_DIR) && make
	cp -f $(SOURCE_DIR)/vrtrack $(SOURCE_DIR)/vrcontrol $(SOURCE_DIR)/multicastdemoclient bin
	touch bin

clean:
	cd $(SOURCE_DIR) && make clean
	rm -f bin/* 

wipe:
	-rm -rf build bin 

