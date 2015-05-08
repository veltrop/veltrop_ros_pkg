

#rosbuild_add_executable(acquire_stereo_images src/experimental/acquire_stereo_images.cpp src/experimental/libcam.cpp)

#rosbuild_add_library(v4l2_cam src/experimental/v4l2_cam.cpp)
#target_link_libraries(v4l2_cam avcodec swscale)

#rosbuild_add_executable(v4l2_stereo_acquire src/experimental/v4l2_stereo_acquire.cpp)
#target_link_libraries(v4l2_stereo_acquire v4l2_cam avcodec swscale)

#rosbuild_add_executable(uvc_stereo_acquire_compact src/experimental/uvc_stereo_acquire_compact.cpp src/experimental/uvc_cam.cpp)
#target_link_libraries(uvc_stereo_acquire_compact uvc_cam)
#rosbuild_add_executable(stereo_acquire_decompact src/experimental/stereo_acquire_decompact.cpp)
