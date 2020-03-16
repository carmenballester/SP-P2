// Carmen Ballester Bernabeu - 54204928T
// Sheila Sánchez Rodríguez - 49250479G

#include <opencv2/opencv.hpp>
#include <iostream>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>

#include <boost/thread/thread.hpp>


using namespace std;
using namespace cv;
using namespace pcl;
