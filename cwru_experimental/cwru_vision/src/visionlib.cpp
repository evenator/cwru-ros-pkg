#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <cwru_vision/visionlib.h>

int rlow = 220;
int rhigh = 255;
int glow = 80;
int ghigh = 120;
int blow = 45;
int bhigh = 60;

using namespace cv;
//using namespace cvb;

void normalizeColors(const cv::Mat& src, cv::Mat& out) {
  Mat temp;
  src.convertTo(temp, CV_32F); // convert to floats first?
  vector<Mat> mats; //make a vector of Mats to hold each channel
  split(temp, mats); //split the src image into B, G and R channels
  Mat total = mats[0] + mats[1] + mats[2]; //sum of B+G+R
  //Mat total = mats[0].mul(mats[0]) + mats[1].mul(mats[1]) + mats[2].mul(mats[2]); //sum of B+G+R
  //sqrt(total, total);
  mats[0] = mats[0] / total; // normalize B channel
  mats[1] = mats[1] / total; // normalize G channel
  mats[2] = mats[2] / total; // normalize R channel
  merge(mats, temp); // merge the individual channels back into a BGR image
  temp.convertTo(out, CV_8U, 255);
}

void findLines(const cv::Mat& src, cv::Mat& out) {
  Mat temp, color_temp; //setup some temps
  cvtColor(src, temp, CV_BGR2GRAY); //convert to grayscale for the edge detector
  //Sobel(temp, temp, CV_8U, 1, 1);
  Canny( temp, temp, 50, 200, 3 ); //run Canny edge detector with some default values
  cvtColor( temp, color_temp, CV_GRAY2BGR ); //Convert Canny edges back to 3-channel

  vector<Vec4i> lines;
  HoughLinesP( temp, lines, 1, CV_PI/180, 80, 30, 10 ); //Find lines in the Canny image
  for( size_t i = 0; i < lines.size(); i++ )
  {
    line( color_temp, Point(lines[i][0], lines[i][1]),
        Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 ); //Draw
  }
  out = color_temp;
}

bool blobfind(const cv::Mat& src, cv::Mat& out, cv::Point& centroid)
{
  Mat temp;

  //cvtColor(src, temp, CV_BGR2HSV);
  temp = src;

  //Make a vector of Mats to hold the invidiual B,G,R channels
  vector<Mat> mats;

  //Split the input into 3 separate channels
  split(temp, mats);

  // Set all values below value to zero, leave rest the same
  // Then inverse binary threshold the remaining pixels
  // Threshold blue channel
  threshold(mats[0], mats[0], bhigh, 255, THRESH_TOZERO_INV);
  threshold(mats[0], mats[0], blow, 255, THRESH_BINARY);
  // Threshold green channel
  threshold(mats[1], mats[1], ghigh, 255, THRESH_TOZERO_INV);
  threshold(mats[1], mats[1], glow, 255, THRESH_BINARY);
  // Threshold red channel
  threshold(mats[2], mats[2], rhigh, 255, THRESH_TOZERO_INV);
  threshold(mats[2], mats[2], rlow, 255, THRESH_BINARY);

  multiply(mats[0], mats[1], out);
  multiply(out, mats[2], out);

  erode(out, out, Mat());

  dilate(out, out, Mat(), Point(-1,-1), 30);

  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  findContours( out, contours, hierarchy, 
      CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

  ROS_DEBUG_STREAM("Num contours: " << contours.size());

  /*int idx = 0;
  for( ; idx >= 0 && idx < (int) hierarchy.size(); idx = hierarchy[idx][0] )
  {
    Scalar color( rand()&255, rand()&255, rand()&255 );
    drawContours( temp, contours, idx, color, CV_FILLED, 8, hierarchy );
  }*/

  bool retval = false;

  if(contours.size() > 0) {
    Rect bounding_box = boundingRect(Mat(contours[0]));
    centroid.x = bounding_box.tl().x + bounding_box.width / 2;
    centroid.y = bounding_box.tl().y + bounding_box.height / 2;
    circle(temp, centroid, 5, Scalar(255,0,0), -1);
    retval = true;
  }
  out = temp;
  return retval;
}

