all: vr920

TARBALL_URL = https://www.mygnu.de/wp-content/uploads/vrtrack-1.0.tar.gz
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
	sudo chown root bin/vrtrack bin/vrcontrol
	sudo chmod +s bin/vrtrack bin/vrcontrol
	touch bin

clean:
	cd $(SOURCE_DIR) && make clean
	rm -rf bin

wipe:
	-rm -rf build bin 

