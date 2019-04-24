// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointType;

int main( int argc, char* argv[] )
{
	int colorHeight = 1080, colorWidth = 1920;
    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;
    cv::Mat rgb = cv::Mat(colorHeight, colorWidth, CV_8UC4);

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
        [&cloud, &mutex]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex );

            /* Point Cloud Processing */

            cloud = ptr->makeShared();
        };

    boost::function<void(cv::Mat& )> function2 =
        [&rgb, &mutex,colorHeight, colorWidth](cv::Mat& ptr ){
            boost::mutex::scoped_lock lock( mutex );
			ptr.copyTo(rgb);
        };

    // Kinect2Grabber
    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );
    boost::signals2::connection connection2 = grabber->registerCallback(function2);

    // Start Grabber
    grabber->start();

    while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();
		cv::imshow("Color", rgb);
		cv::waitKey(1);
        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock() && cloud ){
            // Update Point Cloud
            if( !viewer->updatePointCloud( cloud, "cloud" ) ){
                viewer->addPointCloud( cloud, "cloud" );
            }
        }
    }

    // Stop Grabber
    grabber->stop();
    
    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
        connection2.disconnect();
    }

    return 0;
}
