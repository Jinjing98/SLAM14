#include<iostream>
#include<cmath>
 
#include<Eigen/Core>
#include<Eigen/Geometry>
#include "sophus/so3.h"

#include "sophus/se3.h"   # very strange error,   use auto typing fix the error. but it actually looks the same.
 




using namespace std;
int main(int argc, char** argv){
    
    
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(0,0,1)).toRotationMatrix();
    Sophus::SO3 SO3_R(R);   //method3 of passing param
    Eigen::Quaterniond q( R );
    Sophus::SO3 SO3_q(q);
    Sophus::SO3 SO3_v(0,0,M_PI/2);
    
    cout<<SO3_q<<endl;
    
    cout<<SO3_q.log()<<endl;
    Eigen::Matrix3d M = Sophus::SO3::hat(SO3_q.log());
    cout<< Sophus::SO3::vee(M).transpose() <<endl;
    cout<< Sophus::SO3::exp(SO3_q.log())<<endl;
    
    Eigen::Vector3d v(1e-4,0,0);
    Sophus::SO3 SO3_R_PRIME = Sophus::SO3::exp(v)*SO3_R;
    cout<<SO3_R_PRIME<<endl;
    
    
    
    Eigen::Vector3d t(1,0,0);
    Sophus::SE3 SE3_RT(SO3_q,t);
    Sophus::SE3 SE3_QT(q,t);
    
    
    typedef Eigen::Matrix<double,6,1> Vector6d;
    Vector6d se3 = SE3_QT.log();
    cout<<se3<<endl;
    cout<<"fdfsf"<<SE3_QT<<endl;
    
    
    
    cout<<SO3_q.log()<<endl;
    cout<<SO3_q<<endl;
    
    
    
    
    
    
    
    
    
    
    return 0;
}
