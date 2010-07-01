# image data is always 2bpp yuyv compressed with zlib.
# if other formats/compressessions are desired, more properties will be needed.
# images must be same size.
time stamp
uint32 height
uint32 width
uint8[] left_data
uint8[] right_data

