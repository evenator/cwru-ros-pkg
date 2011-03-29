#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_visualization/cloud_viewer.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <cwru_vision/visionlib.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ExtrinsicsCalibrator {
  public:
    ExtrinsicsCalibrator();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool findCloudCentroid();
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    image_transport::Publisher image_pub_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfl_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_filter_;
    PointCloud cloud_;
    pcl_visualization::CloudViewer viewer_;

    pcl::PassThrough<pcl::PointXYZ> passthrough_;
    pcl::ExtractIndices<pcl::PointXYZ> extract_;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg_;
};

ExtrinsicsCalibrator::ExtrinsicsCalibrator():
  it_(nh_), laser_sub_(nh_, "scan", 1), 
  laser_filter_(laser_sub_, tfl_, "base_link", 1), viewer_("Viewer")
{
  sub_ = it_.subscribe("image", 1, &ExtrinsicsCalibrator::imageCallback, this);
  image_pub_ = it_.advertise("output_image", 1);
  laser_filter_.registerCallback(boost::bind(&ExtrinsicsCalibrator::laserCallback, this, _1));

  passthrough_.setFilterFieldName ("z");
  passthrough_.setFilterLimits (0, 1.5);
  // Optional
  seg_.setOptimizeCoefficients (true);
  // Mandatory
  seg_.setModelType (pcl::SACMODEL_CIRCLE2D);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setMaxIterations (1000);
  seg_.setDistanceThreshold (0.05);
  seg_.setRadiusLimits(0.01, 0.1);

}

void ExtrinsicsCalibrator::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  sensor_msgs::PointCloud2 temp;
  projector_.transformLaserScanToPointCloud("base_link", *scan, temp, tfl_);
  pcl::fromROSMsg(temp, cloud_);
}

bool ExtrinsicsCalibrator::findCloudCentroid() {
  bool retval = false;
  PointCloud::Ptr temp(new PointCloud);
  passthrough_.setInputCloud(cloud_.makeShared());
  passthrough_.filter(*temp);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  seg_.setInputCloud(temp);
  seg_.segment (*inliers, *coefficients);
  ROS_INFO_STREAM("Number of inliers: " << inliers->indices.size());
  extract_.setInputCloud (temp);
  extract_.setIndices (inliers);
  extract_.setNegative (false);
  extract_.filter (*temp);
  ROS_INFO_STREAM("Number of points: " << temp->points.size());

  viewer_.showCloud(*temp);
  return retval;
}

void ExtrinsicsCalibrator::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::CvBridge bridge;
  cv::Mat image;
  cv::Mat output;
  try
  {
    image = cv::Mat(bridge.imgMsgToCv(msg, "bgr8"));

  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'. E was %s", msg->encoding.c_str(), e.what());
  }
  try {
    //normalizeColors(image, output);
    cv::Point centroid;
    ros::Time start = ros::Time::now();
    blobfind(image, output, centroid);
    ROS_DEBUG("Blobfind took: %f seconds", (ros::Time::now() - start).toSec());
    //findLines(image, output);
    IplImage temp = output;
    image_pub_.publish(bridge.cvToImgMsg(&temp, "bgr8"));
  }
  catch (sensor_msgs::CvBridgeException& e) {

    ROS_ERROR("Could not convert to 'bgr8'. Ex was %s", e.what());
  }
  findCloudCentroid();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "extrinsics_calibrator");
  ExtrinsicsCalibrator calibrator;
  ros::spin();
}
