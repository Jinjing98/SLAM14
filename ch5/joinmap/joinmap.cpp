#include<iostream>
#include<fstream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<Eigen/Geometry>
#include<boost/format.hpp>
#include<pcl-1.8/pcl/point_types.h>
#include<pcl-1.8/pcl/io/pcd_io.h>
#include<pcl-1.8/pcl/visualization/pcl_visualizer.h>

using namespace std;

int main(int argc, char** argv) {
    vector<cv::Mat> colorImgs,depthImgs;
    vector<Eigen::Isometry3d> poses;
 
 
    ifstream fin("./pose.txt");
    for (int i = 0; i<5; i++) {
        boost::format fmt("%s/%d.%s");
        cout<<(fmt%"/home/jinjing/slam_14/slambook2/ch5/rgbd/color"%(i+1)%"png").str()<<endl;
        colorImgs.push_back(cv::imread((fmt%"/home/jinjing/slam_14/slambook2/ch5/rgbd/color"%(i+1)%"png").str()));
        depthImgs.push_back(cv::imread((fmt%"/home/jinjing/slam_14/slambook2/ch5/rgbd/depth"%(i+1)%"pgm").str(),-1));

        double data[7] = {0};
        for(auto& d:data)
            fin>>d;
        Eigen::Quaterniond q(data[6],data[3],data[4],data[5]);
        Eigen::Isometry3d  T(q);
        T.pretranslate(Eigen::Vector3d(data[0],data[1],data[2]));
        poses.push_back(T);


    }

    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;


    typedef pcl::PointXYZRGB pointT;
    typedef pcl::PointCloud<pointT> PointCloud;

    PointCloud::Ptr pointCloud(new PointCloud);

    
    
    
    
  
 
    cout<<"正在将图像转换为点云..."<<endl;
    
    // 新建一个点云
 
    
    //pcl::visualization::CloudViewer viewer("pcd viewer");
    PointCloud::Ptr current( new PointCloud );
    for ( int i=0; i<5; i++ )
    {

        cout<<"转换图像中: "<<i+1<<endl; 
        cv::Mat color = colorImgs[i]; 
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                if ( d==0 ) continue; // 为0表示没有测量到
                if ( d >= 3500 ) continue; // 深度太大时不稳定，去掉
                Eigen::Vector3d point; 
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 
                Eigen::Vector3d pointWorld = T*point;

                pointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];
                current->points.push_back( p );
            }
            
            
            

        
 
    }
 
    
    current->is_dense = False;
    cout<<current->size()<<endl;
    pcl::io::savePCDFileBinary("map.pcd",*pointCloud);
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    







    return 0;
}

