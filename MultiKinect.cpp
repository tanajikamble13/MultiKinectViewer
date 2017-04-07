
/* Code by Tanaji 
   Help: To save cloud press space key
         To capture 3D co-cordinates of point press shift key and click with mouse to select the point
*/

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>//extra 
#include <pcl/console/parse.h>//extra
#include <pcl/visualization/point_picking_event.h>//for event.getpoint
#include "FuncReg.h"

//typedef pcl::PointXYZRGBA PointT;//tan change
//typedef pcl::PointCloud<PointT> PointCloud;
//typedef pcl::PointNormal PointNormalT;
//typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

using namespace std;
using namespace pcl;
float x,y,z; 

boost::mutex mutex;
boost::condition_variable_any cond;



unsigned int filesSaved = 0;  
bool saveCloud(false), saveTarget(false), StartReg(false), TransYes(false);
bool pickPoint(false);
Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();// extra change
Eigen::Affine3f Pose = Eigen::Affine3f::Identity(), Pose_temp;


PointCloud<PointXYZRGBA>::Ptr outCld1 (new PointCloud<PointXYZRGBA>);//existing outCld change outCld1
PointCloud<PointXYZRGBA>::Ptr outCld2 (new PointCloud<PointXYZRGBA>);//extra change

Eigen::Matrix4f FunctionReg();

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}
     

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     {
       if (!viewer.wasStopped())
	
	pcl::transformPointCloud(*cloud, *outCld1, transform_2); //existing outCld change outCld1
        //existing outCld change outCld1
       // pcl::transformPointCloud(*cloud, *outCld2, transform_3); //change extra
        //viewer.showCloud (outCld2);//change extra
       // pcl::transformPointCloud(*cloud, *outCld1, Pose);
        if(TransYes)
        {
         printf("Registration is done");
        pcl::transformPointCloud(*cloud, *outCld1, Pose);
        TransYes=false;
        }
        viewer.showCloud (outCld1);
       
        viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)&viewer);

        if (saveCloud) 
        {
             
                stringstream stream;
		stream << "Source" << filesSaved << ".pcd";//make change to source to save by different name
		string filename = stream.str();
		if (io::savePCDFile(filename, *outCld1, true) == 0) 
		{
	          //pcl::io::savePCDFile (filename, *outCld); 
                      //  filesSaved++;
			cout << "Saved " << filename << "." << endl;
                        saveCloud = false;
                       printf("\nPress key F1 to capture target cloud \n");
                 }
		else 
                {
                  PCL_ERROR("Problem saving %s.\n", filename.c_str());
           
                }
       }




     }


      void cloud_cb (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     {
       if (!viewer.wasStopped())

     pcl::transformPointCloud(*cloud, *outCld2, transform_3); //change extra
      viewer.showCloud (outCld2);//change extra

     viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)&viewer);

        if (saveTarget) 
        {
             
                stringstream stream1;
		stream1 << "Target" << filesSaved << ".pcd";//make change to source to save by different name
		string filename = stream1.str();
		if (io::savePCDFile(filename, *outCld2, true) == 0) 
		{
	          //pcl::io::savePCDFile (filename, *outCld); 
                   //     filesSaved++;
			cout << "Saved " << filename << "." << endl;
                        saveTarget = false;
                        StartReg= true;
                         
                 }
		else 
                {
                  PCL_ERROR("Problem saving %s.\n", filename.c_str());
           
                }
       }

    
     }





     void run1 ()
     {
       // pcl::Grabber* interface1 = new pcl::OpenNIGrabber("#1");
        pcl::Grabber* interface1 = new pcl::OpenNIGrabber();

       //transform_2.translation() << 0, -0.5, 0.0;	
       transform_2.rotate(Eigen::AngleAxisf (3.14, Eigen::Vector3f::UnitX()));   
     
 	
       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f1 =
       boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1); //existing f change f1

       
       interface1->registerCallback (f1);//existing f change f1
      
       interface1->start ();//existing interface change interface1
      
      
       while (!viewer.wasStopped())
       {
         //boost::this_thread::sleep (boost::posix_time::seconds (1));
           boost::this_thread::sleep (boost::posix_time::milliseconds(10));
       }
       


       interface1->stop ();
     }


     void run2 ()
     {
       pcl::Grabber* interface2 = new pcl::OpenNIGrabber("#2");

       //transform_2.translation() << 0, -0.5, 0.0;	
      transform_3.rotate(Eigen::AngleAxisf (3.14, Eigen::Vector3f::UnitX()));  //change extra 
       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f2 =
       boost::bind (&SimpleOpenNIViewer::cloud_cb, this, _1);//extra

       interface2->registerCallback (f2);//extra
       interface2->start ();//extra

       while (!viewer.wasStopped())
       {
         //boost::this_thread::sleep (boost::posix_time::seconds (1));
           boost::this_thread::sleep (boost::posix_time::milliseconds(2));
          
       }
       interface2->stop ();
     }



static void
keyboardEventOccurred(const visualization::KeyboardEvent& event,
					  void* nothing)
{
       
	if (event.getKeySym() == "space" && event.keyDown())
		saveCloud = true;
        if (event.getKeySym() == "F1" && event.keyDown())
		saveTarget = true;
}

void TransformEstimate()
{
 if(StartReg==true)
       {
        Pose_temp= FuncReg();
        printf("Registration Function is called");
        Pose= Pose_temp;
        TransYes= true;
        StartReg= false;
       }
 
}



     pcl::visualization::CloudViewer viewer;
      //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
 };



 int main ()
 {
  //myclass* ob = new myclass();
  SimpleOpenNIViewer v;
  printf("\nPress key SPACE to capture source cloud\n");
 // v.run1 ();
 // v.run2 ();
 // TranformEstimate();

     boost::thread* Kinect1= new boost::thread(boost::bind (&SimpleOpenNIViewer::run1, &v));
     boost::thread* Kinect2= new boost::thread(boost::bind (&SimpleOpenNIViewer::run2, &v));
     //boost::thread Transformation(TransformEstimate);
     boost::thread* Transformation= new boost::thread(boost::bind (&SimpleOpenNIViewer::TransformEstimate, &v));
     Transformation->join();
     Kinect1->join();
     Kinect2->join();
     //Transformation.join();
     

  
 return 0;
 }


