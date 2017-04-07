#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <float.h>
#include <math.h>
#include <string>
#include <pcl/features/integral_image_normal.h>
#include <pcl/keypoints/agast_2d.h>
// #include <pcl/keypoints/brisk_2d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/ros/conversions.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/pfh.h>
#include <pcl/registration/gicp.h> //tan for gicp

#define POINTTYPE pcl::PointXYZRGB
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;


#ifndef FUNCREG_H
#define FUNCREG_H

Eigen::Matrix4f FuncReg();
void showCloudsLeft(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source);

void showCloudsRight(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target, const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source);

Eigen::Matrix4f pairAlign (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false);
/*
void computeintegralimagenoramls(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputcloud,pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>::AVERAGE_DEPTH_CHANGE,float maxdepthchangefactor=0.05f,float normalsmoothingsize=10.0f estimationMethod );
*/

void computeagastkeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputcloud, pcl::PointCloud<pcl::PointXYZRGB> &keypoints);

void computesiftkeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputcloud, pcl::PointCloud<pcl::PointXYZRGB> &keypoints);

void computefpfh33descriptors(pcl::PointCloud<pcl::FPFHSignature33> &descriptors,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &searchsurface,pcl::PointCloud<pcl::Normal>::Ptr &normals,pcl::PointCloud<pcl::PointXYZRGB> &keypoints);

void
findcorrespondences (const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,        pcl::Correspondences &all_correspondences);

void
rejectbadcorrespondences (const pcl::CorrespondencesPtr &all_correspondences,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_src,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_tgt,
                          pcl::Correspondences &remaining_correspondences);

#endif
