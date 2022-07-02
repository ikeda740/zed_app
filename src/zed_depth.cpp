#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>

std::vector<double> g_depth;   // depth image in lower resolution
const int g_ratio = 8;         // downsample ratio 
const int g_depth_size = 3600; // size of the depth image in lower resolution

// return median of the vector v
double median(std::vector<double> &v)
{
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}

// callback fuction for depth image
// compute lower resolution of the image and publish it as
// Float32MultiArray (one dimensional array)
void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // Get a pointer to the depth values casting the data
  // pointer to floating point
  float* depths = (float*)(&msg->data[0]);

  int width = msg->width;   // 640 for zedm
  int height = msg->height; // 360 for zedm

  // convert 640x360 to 80x45 (ratio=8)
  g_depth.resize(g_depth_size);

  int k = 0;
  for (int j=0; j<height; j+=g_ratio){
    for (int i=0; i<width; i+=g_ratio){

      // compute median values in a ratio x ratio area
      std::vector<double> depth_area; 

      for (int jj=0; jj<g_ratio; jj++){
        for (int ii=0; ii<g_ratio; ii++){
          double d = depths[width*(j+jj)+(i+ii)];
          if (isfinite(d)) {
            depth_area.push_back(d);
          }
        }
      }

      if (depth_area.size() > 0){
        g_depth[k++] = median(depth_area);
      }  else {
        g_depth[k++] = 0.0;
      }
    }
  }

  // Output the measure
  //ROS_INFO("Center distance : %g m", g_depth[centerIdx]);
}

// main function
int main(int argc, char** argv)
{
  ros::init(argc, argv, "zed_depth_publisher");

  ros::NodeHandle n;
  ros::Subscriber sub_depth = n.subscribe("/zedm/zed_node/depth/depth_registered", 15, depthCallback);

  ros::Publisher pub_depth = n.advertise<std_msgs::Float32MultiArray>("/depth", 15);

  ros::Rate loop_rate(15);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::Float32MultiArray msg;
    msg.data.resize(g_depth_size);
    std::copy(g_depth.begin(), g_depth.end(), msg.data.begin());
    
    pub_depth.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}










