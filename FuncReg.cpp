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
#include "FuncReg.h"

#define POINTTYPE pcl::PointXYZRGB
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
	pcl::visualization::PCLVisualizer *p;
	
	int vp_1, vp_2;


////////////////////////////////////////////////////////////////////////////////
/** Rendering source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  //PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> tgt_h (cloud_target);//Tan change
  //PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_h (cloud_source);//Tan change
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO ("Press q to begin the registration.\n");
  p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** Rendering Normalized source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target, const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerGenericField<pcl::PointNormal> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  PointCloudColorHandlerGenericField<pcl::PointNormal> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}



////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
Eigen::Matrix4f pairAlign (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample )
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  //PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  //PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
  //pcl::PointCloud<pcl::PointNormal>
  pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt (new pcl::PointCloud<pcl::PointNormal>);

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> norm_est;
  //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());//Tan change
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
 // MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  //float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  //point_representation.setRescaleValues (alpha);

  //
    
pcl::GeneralizedIterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> reg;
    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);
    //reg.align(*transformed);


   
  // Align
 // pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setRANSACOutlierRejectionThreshold( 0.1 ); 
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (1);  
  // Set the point representation
  //reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
 // reg.setInputSource (points_with_normals_src);
 // reg.setInputTarget (points_with_normals_tgt);


  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (200);
  reg.align (*reg_result);// added by tan
  Ti = reg.getFinalTransformation ();
  std::cout << "has converged:" << reg.hasConverged() << " score: " <<
  reg.getFitnessScore() << std::endl;
  std::cout << reg.getFinalTransformation() << std::endl;
  return(Ti);
	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
 
 }
 
 
 
///////
//////////// ICP Functions End //////
/*
void computeintegralimagenoramls(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputcloud,pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>::AVERAGE_DEPTH_CHANGE,float maxdepthchangefactor,float normalsmoothingsize estimationMethod )
{
   pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
   ne.setNormalEstimationMethod (estimationMethod);
   ne.setMaxDepthChangeFactor(maxdepthchangefactor);
   ne.setNormalSmoothingSize(normalsmoothingsize);
   ne.setInputCloud(inputcloud);
   ne.compute(*normals);
}
*/
//*
void computeagastkeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputcloud,pcl::PointCloud<pcl::PointXYZRGB> &keypoints)
{
   pcl::PointCloud<pcl::PointUV> agast_keypoints;
   pcl::AgastKeypoint2D<pcl::PointXYZRGB> agast;
   agast.setThreshold (30);
   agast.setInputCloud (inputcloud);
   agast.compute (agast_keypoints);
   keypoints.resize(agast_keypoints.size());
   int k=agast_keypoints.size();
   for(int i=0,j=0;i<k;++i)
   {
   j=agast_keypoints.points[i].u+agast_keypoints.points[i].v*inputcloud->width;
   if(isnan(inputcloud->points[j].x))
   --k;
   else
   keypoints.points[i].b=inputcloud->points[j].b,
   keypoints.points[i].g=inputcloud->points[j].g,
   keypoints.points[i].r=inputcloud->points[j].r,
   keypoints.points[i].x=inputcloud->points[j].x,
   keypoints.points[i].y=inputcloud->points[j].y,
   keypoints.points[i].z=inputcloud->points[j].z; 
   }
   std::vector<pcl::PointXYZRGB,Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator    keypointIt=keypoints.begin(); 
   for(int i=k;k<agast_keypoints.size();++k)
   keypoints.erase(keypointIt+i);
}

/*
void computebriskkeypoints(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &inputcloud,pcl::PointCloud<pcl::PointXYZRGBA> &keypoints,int threshold=60,int octaves=4)
{
  pcl::PointCloud<pcl::PointWithScale> brisk_keypoints;
  pcl::BriskKeypoint2D<pcl::PointXYZRGBA> brisk;
  brisk.setThreshold (threshold);
  brisk.setOctaves (octaves);
  brisk.setInputCloud(inputcloud);
  brisk.compute(brisk_keypoints);
  //remove nan
  std::vector<pcl::PointWithScale,Eigen::aligned_allocator<pcl::PointWithScale> >::iterator keypointIt=brisk_keypoints.begin(); 
  for(int i=0;i<brisk_keypoints.size();++i)
  if(isnan(brisk_keypoints.points[i].x))
  brisk_keypoints.erase(keypointIt+i),--i;
  pcl::copyPointCloud(brisk_keypoints,keypoints);
}
*/
void computesiftkeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputcloud,pcl::PointCloud<pcl::PointXYZRGB> &keypoints)
{
  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;
  pcl::PointCloud<pcl::PointWithScale> sift_keypoints;
  sift_detect.setScales (0.03, 6, 15);
  sift_detect.setMinimumContrast (0.9);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr siftkdtree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
  sift_detect.setSearchMethod(siftkdtree);
  sift_detect.setInputCloud (inputcloud);
  sift_detect.compute (sift_keypoints);
  //remove nan
  std::vector<pcl::PointWithScale,Eigen::aligned_allocator<pcl::PointWithScale> >::iterator keypointIt=sift_keypoints.begin(); 
  for(int i=0;i<sift_keypoints.size();++i)
  if(isnan(sift_keypoints.points[i].x))
  sift_keypoints.erase(keypointIt+i),--i;
  pcl::copyPointCloud(sift_keypoints,keypoints);
}
void computefpfh33descriptors(pcl::PointCloud<pcl::FPFHSignature33> &descriptors,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &searchsurface,pcl::PointCloud<pcl::Normal>::Ptr &normals,pcl::PointCloud<pcl::PointXYZRGB> &keypoints)
{
  pcl::FPFHEstimation<pcl::PointXYZRGB,pcl::Normal, pcl::FPFHSignature33> feature_est;
  feature_est.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));//changes
  feature_est.setRadiusSearch (0.05);
  feature_est.setSearchSurface(searchsurface);
  feature_est.setInputNormals(normals);
  feature_est.setInputCloud(keypoints.makeShared());
  feature_est.compute (descriptors);
  //remove nan from fpfh33descriptors and keypoints
  std::vector<pcl::FPFHSignature33,Eigen::aligned_allocator<pcl::FPFHSignature33> >::iterator    descriptorIt=descriptors.points.begin(); 
  std::vector<pcl::PointXYZRGB,Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator    keypointIt=keypoints.begin(); 
  for(int i=0;i<descriptors.size();++i)
  if(isnan(descriptors.points[i].histogram[0]))descriptors.erase(descriptorIt+i),keypoints.erase(keypointIt+i),--i;
 }
void
findcorrespondences (const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,        pcl::Correspondences &all_correspondences)
{
  pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
  est.setInputSource (fpfhs_src);
  est.setInputTarget (fpfhs_tgt);
  est.determineReciprocalCorrespondences (all_correspondences);
}
void
rejectbadcorrespondences (const pcl::CorrespondencesPtr &all_correspondences,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_src,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_tgt,
                          pcl::Correspondences &remaining_correspondences)
{
  pcl::registration::CorrespondenceRejectorDistance rej;
  rej.setInputSource<pcl::PointXYZRGB> (keypoints_src);
  rej.setInputTarget<pcl::PointXYZRGB> (keypoints_tgt);
  rej.setMaximumDistance (1);    // 1m
  rej.setInputCorrespondences (all_correspondences);
  rej.getCorrespondences (remaining_correspondences);
}
//// Functions ICP ////


Eigen::Matrix4f FuncReg()
{
  printf("Registration Function is called");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGB>);
  //std::vector<int> p_file_indices= pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  pcl::io::loadPCDFile ("Source0.pcd", *cloud1); //change1 	
  pcl::io::loadPCDFile ("Target0.pcd", *cloud2); //change2
  //remove NaN
  //std::vector<int> ind,ind2;
  //pcl::removeNaNFromPointCloud(*cloud1,*cloud1,ind);
  //pcl::removeNaNFromPointCloud(*cloud2,*cloud2,ind2);
  printf("size:%ld %ld\n",cloud1->size(),cloud2->size());
  printf("cloud size:%lu(%u*%u) %lu(%u*%u)\n",cloud1->size(),cloud1->width,cloud1->height,cloud2->size(),cloud2->width,cloud2->height);

//rendering 
  int v1=1,v2=2,v3=3,v4=4;
  //pcl::visualization::PCLVisualizer viewer("Source cloud (Left) Target cloud (Right)");
  pcl::visualization::PCLVisualizer viewer("Source cloud (Left) Target cloud (Right)");
 // viewer.addPointCloud(cloud1);
  viewer.createViewPort(0,0,0.5,1.0,v1);
  viewer.createViewPort(0.5, 0, 1.0, 1.0,v2);
  viewer.addPointCloud(cloud1,"Source",v1);
  viewer.addPointCloud(cloud2,"Target",v2);
  viewer.spin();

  //compute normals
  
  //pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
  //pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr normals3(new pcl::PointCloud<pcl::Normal>);

                pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<POINTTYPE>::Ptr normSM(new pcl::search::KdTree<POINTTYPE>);
		pcl::NormalEstimation<POINTTYPE, pcl::Normal> norm_est;
		norm_est.setSearchMethod (normSM);
		norm_est.setRadiusSearch (0.05);
		norm_est.setInputCloud(cloud1);
		norm_est.compute(*normals1);
		norm_est.setInputCloud(cloud2);
		norm_est.compute(*normals2);




 //computeintegralimagenoramls(cloud1,normals1);
 //computeintegralimagenoramls(cloud2,normals2);
  
  static double time1=pcl::getTime(),time2=pcl::getTime();

  // Remove NaN	
  for(int i=0;i<cloud1->size();++i)if(isnan(cloud1->points[i].x))cloud1->points[i].rgb=0;
  for(int i=0;i<cloud2->size();++i)if(isnan(cloud2->points[i].x))cloud2->points[i].rgb=0;

  // AGAS key point
   pcl::PointCloud<pcl::PointXYZRGB> keypoints1;
   pcl::PointCloud<pcl::PointXYZRGB> keypoints2;

   computeagastkeypoints(cloud1,keypoints1);
   computeagastkeypoints(cloud2,keypoints2);
   //computebriskkeypoints(cloud1,keypoints1);computebriskkeypoints(cloud2,keypoints2);
   //computesiftkeypoints(cloud1,keypoints1);computesiftkeypoints(cloud2,keypoints2);
   printf("keypoints source size:%ld keypoints target size:%ld,takes %fs\n",keypoints1.size(),keypoints2.size(),pcl::getTime()-time1);
   time1=pcl::getTime();
   
   // FPFH descriptor 	
   pcl::PointCloud<pcl::FPFHSignature33> descriptors1;
   pcl::PointCloud<pcl::FPFHSignature33> descriptors2;
  
   computefpfh33descriptors(descriptors1,cloud1,normals1,keypoints1);
   computefpfh33descriptors(descriptors2,cloud2,normals2,keypoints2);
   printf("descriptors size:%ld %ld,takes %fs\n",descriptors1.size(),descriptors2.size(),pcl::getTime()-time1);
   time1=pcl::getTime();
   
  //Find Correspondences
  pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences),good_correspondences (new pcl::Correspondences);
  findcorrespondences (descriptors1.makeShared(), descriptors2.makeShared(), *all_correspondences);
  printf("descriptors size:%ld %ld,takes %fs to find coresspondences\n",descriptors1.size(),descriptors2.size(),pcl::getTime()-time1);
  rejectbadcorrespondences (all_correspondences, keypoints1.makeShared(), keypoints2.makeShared(), *good_correspondences);
  

  Eigen::Matrix4f transform;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> trans_est;
  trans_est.estimateRigidTransformation (keypoints1, keypoints2, *good_correspondences, transform);
  //pcl::transformPointCloud(*normals1,*normals3,transform);
  
  //computeintegralimagenoramls(cloud3,normals3);
  
  
  
  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer ( "ICP Registration ");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>), source, target, resultant;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform,Global_transform, ICP_transform;
  
  pcl::transformPointCloud(*cloud1,*result,transform);
  
    source = result;
    target = cloud2;

    // Add visualization data
    showCloudsLeft(source, target);

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
  
   ICP_transform= pairAlign (source, target, temp, pairTransform, true);


   pcl::transformPointCloud (*source, *source, ICP_transform);
   Global_transform=transform* ICP_transform;

  p->removePointCloud ("source");
  p->removePointCloud ("target");

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb3 (target);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb4 (source);
  p->addPointCloud<pcl::PointXYZRGB>(source, rgb4, "source", vp_2);
  p->addPointCloud<pcl::PointXYZRGB>(target, rgb3, "target", vp_2);
  p->spin ();

  p->removePointCloud ("source"); 
  p->removePointCloud ("target");
  cout<<"Pose :\n" <<Global_transform<<"\n"; 

  //add the source to the transformed target
//  *output += *cloud_src;
  
 return (Global_transform);
}
