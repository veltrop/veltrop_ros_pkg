// http://bobobobo.wordpress.com/2008/02/23/how-to-use-zlib/

#include <zlib.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
//#include <std_msgs/UInt8MultiArray.h>

namespace veltrobot_sensors
{

//int width = 480;
//int height = 320;
//unsigned long frame_size_original = width * height * 2;

// TODO: check out yuyv -> rgb conversion in uvc_cam to see if it is faster than this method

#define SAT(c) if (c & (~255)) { if (c < 0) c = 0; else c = 255; }
static void
yuyv16_to_bgr24(int width, int height, const unsigned char *src, unsigned char *dst)
{
   unsigned char *s;
   unsigned char *d;
   int l, c;
   int r, g, b, cr, cg, cb, y1, y2;

   l = height;
   s = (unsigned char*)src;
   d = dst;
   while (l--)
   {
      c = width >> 1;
      while (c--)
      {
         y1 = *s++;
         cb = ((*s - 128) * 454) >> 8;
         cg = (*s++ - 128) * 88;
         y2 = *s++;
         cr = ((*s - 128) * 359) >> 8;
         cg = (cg + (*s++ - 128) * 183) >> 8;

         r = y1 + cr;
         b = y1 + cb;
         g = y1 - cg;
         SAT(r);
         SAT(g);
         SAT(b);

         *d++ = b;
         *d++ = g;
         *d++ = r;

         r = y2 + cr;
         b = y2 + cb;
         g = y2 - cg;
         SAT(r);
         SAT(g);
         SAT(b);

         *d++ = b;
         *d++ = g;
         *d++ = r;
      }
   }
}

class FixStereoImages
{
public:
  FixStereoImages(std::string stereo_name)
  {
    left_sub_ = n_.subscribe(stereo_name + "/left/image_yuyv_z", 1, &FixStereoImages::leftCB, this);
    right_sub_ = n_.subscribe(stereo_name + "/right/image_yuyv_z", 1, &FixStereoImages::rightCB, this);
    //left_sub_ = n_.subscribe("/stereo/left/image_yuyv_z", 1, &FixStereoImages::leftCB, this);
    //right_sub_ = n_.subscribe("/stereo/right/image_yuyv_z", 1, &FixStereoImages::rightCB, this);
  }
  
  void spin()
  {
    t_prev_ = ros::Time::now();
    frames_ = 0;
    ros::spin();
  }

private:
  ros::NodeHandle n_; 
  ros::Subscriber left_sub_;
  ros::Subscriber right_sub_;
  ros::Time t_prev_;
  int frames_;
    /*
  void leftCB(const std_msgs::UInt8MultiArrayConstPtr& msg)
  {
    unsigned long frame_size_uncompressed = frame_size_original;
    unsigned char* frame_uncompressed = (unsigned char*)malloc(frame_size_uncompressed);

    int z_result = uncompress(frame_uncompressed, &frame_size_uncompressed,
                              &msg->data[0], msg->data.size());

    switch (z_result)
    {
    case Z_OK:
        printf("***** L SUCCESS! *****\n");
        break;
    case Z_MEM_ERROR:
        printf("L out of memory\n");
        exit(1);    // quit.
        break;
    case Z_BUF_ERROR:
        printf("L output buffer wasn't large enough!\n");
        exit(1);    // quit.
        break;
    }
    
    IplImage* ipl=cvCreateImage(cvSize(width, height), 8, 3);
    yuyv16_to_bgr24(width, height, frame_uncompressed, (unsigned char*)ipl->imageData);
    cvShowImage("win0", ipl);
    cvReleaseImage(&ipl);   
    free(frame_uncompressed);
  }
  
  void rightCB(const std_msgs::UInt8MultiArrayConstPtr& msg)
  {
    unsigned long frame_size_uncompressed = frame_size_original;
    unsigned char* frame_uncompressed = (unsigned char*)malloc(frame_size_uncompressed);

    int z_result = uncompress(frame_uncompressed, &frame_size_uncompressed,
                              &msg->data[0], msg->data.size());

    switch (z_result)
    {
    case Z_OK:
        printf("***** R SUCCESS! *****\n");
        break;
    case Z_MEM_ERROR:
        printf("R out of memory\n");
        exit(1);    // quit.
        break;
    case Z_BUF_ERROR:
        printf("R output buffer wasn't large enough!\n");
        exit(1);    // quit.
        break;
    }
    
    IplImage* ipl=cvCreateImage(cvSize(width, height), 8, 3);
    yuyv16_to_bgr24(width, height, frame_uncompressed, (unsigned char*)ipl->imageData);
    cvShowImage("win1", ipl);
    cvReleaseImage(&ipl);   
    free(frame_uncompressed); 
  }   */ 
    
  void doFps()
  {
    if (++frames_ == 10)
    {
      frames_ = 0;
      ros::Duration d = ros::Time::now() - t_prev_;
      t_prev_ = ros::Time::now();
      float fps = 10.0f / d.toSec();
      printf("last 10 frames at %f fps (%f)\n", fps, fps / 2.0f);
    }  
  }
  
  void leftCB(const sensor_msgs::ImageConstPtr& msg)
  {
    ros::Duration d = ros::Time::now() - msg->header.stamp;
    printf("left delay %f\n", d.toSec());
    doFps();
    IplImage* ipl=cvCreateImage(cvSize(msg->width, msg->height), 8, 3);
    unsigned char* iplbuff=(unsigned char*)ipl->imageData;
    yuyv16_to_bgr24(msg->width, msg->height, &msg->data[0], iplbuff);
    cvShowImage("win0", ipl);
    cvReleaseImage(&ipl);
  }
  
  void rightCB(const sensor_msgs::ImageConstPtr& msg)
  {
    ros::Duration d = ros::Time::now() - msg->header.stamp;
    printf("right delay %f\n", d.toSec());
    doFps();
    //ros::Time t = ros::Time::now();
    IplImage* ipl=cvCreateImage(cvSize(msg->width, msg->height), 8, 3);
    unsigned char* iplbuff=(unsigned char*)ipl->imageData;
    yuyv16_to_bgr24(msg->width, msg->height, &msg->data[0], iplbuff);
    cvShowImage("win1", ipl);
    cvReleaseImage(&ipl); 
    //d = ros::Time::now() - t;
    //printf("right display time %f\n", d.toSec()); 
  }
  
};

} // namespace veltrobot_sensors

int main(int argc, char** argv)
{
  if (argc != 2)
    return -1;
  
  ros::init(argc, argv, "stereo_acquire_decompact");
  cvInitSystem(argc, argv);
  cvStartWindowThread();
  
  cvNamedWindow("win0");
  cvNamedWindow("win1");
  
  veltrobot_sensors::FixStereoImages this_node(argv[1]);
  this_node.spin();
  
  return 0;
}

