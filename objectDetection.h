// Carmen Ballester Bernabeu - 54204928T
// Sheila Sánchez Rodríguez - 49250479G

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_types.h>

using namespace std;
using namespace cv;
using namespace pcl;

// Be
