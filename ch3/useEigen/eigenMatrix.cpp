#include <iostream>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
#define MATRIX_SIZE 100
using namespace std;


int main(int argc, char** argv){

Eigen::Vector3d p;
Eigen::Matrix3d R;


Eigen::AngleAxisd v(M_PI/4,Eigen::Vector3d(0,0,1));//passing parameter method1
Eigen::Quaterniond q = Eigen::Quaterniond(v.matrix());
Eigen::Vector3d rpy;
Eigen::Vector3d translation;
translation = Eigen::Vector3d::Zero();
Eigen::Isometry3d T = Eigen::Isometry3d::Identity();   //first init it to Identity,then rotate(),pretranslate() can be right!!!
T.rotate(v);
T.pretranslate(translation);

p << 1,0,0;   //passing parameter method2
cout<< v.matrix()*p<<endl;
cout<< v.matrix()*v.matrix()*v.matrix()*v.matrix()*p<<endl;





cout<< v.toRotationMatrix() << endl;
cout<< T.matrix()<< endl;
cout<< T.matrix().inverse()<< endl;
 
cout<< v.toRotationMatrix() << endl;
cout<< v.toRotationMatrix().inverse() << endl;
cout<< (rpy=v.matrix().eulerAngles(2,1,0)) << endl;
cout<< Eigen::Quaterniond(v.matrix()).coeffs()<<endl;   //cout can print out limited type,you can not print out q directly.

 

 

cout<< v*p << endl;
cout<< q.coeffs()<< 0.92*0.92+0.38*0.38 <<endl;   //rotation matrix present as Quaterniond, it must be a unit Quaterniond which has a unit length of 1.
cout<< (v.matrix()*p - q * p).transpose()<< endl;



Eigen::Matrix<double,10,10> m = Eigen::Matrix<double,10,10>::Random();
cout<< m <<endl;
cout<< (m.block(0,0,3,3) = Eigen::Matrix3d::Identity())<<endl;
cout<< m <<endl;
 



}
