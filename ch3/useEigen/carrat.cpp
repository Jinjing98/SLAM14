#include<iostream>
#include<Eigen/Core>
#include<Eigen/Geometry>

using namespace std;
int main(int argc, char** argv){
    
    
    Eigen::Quaterniond q1(0.35,0.2,0.3,0.1);
    q1 = q1.normalized();
    Eigen::Quaterniond q2(-0.5,0.4,-0.1,0.2);
    q2 = q2.normalized();
    Eigen::Vector3d t1(0.3,0.1,0.1);
    Eigen::Vector3d t2(-0.1,0.5,0.3);
    Eigen::Vector4d pc1(0.5,0,0.2,1);
    
    Eigen::Isometry3d Twc1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d Twc2 = Eigen::Isometry3d::Identity();
    
    Twc1.rotate(q1.matrix());
    Twc1.pretranslate(t1);
    Twc2.rotate(q2.matrix());
    Twc2.pretranslate(t2);
    
    Eigen::Vector4d pc2 = Twc2.inverse()*Twc1*pc1;
    cout<< Twc2.inverse().matrix()<<'*'<<Twc1.matrix()<<'*'<<pc1<<'='<<pc2<<endl;
    
    
    
    
    
    
    
    
    
    
    return 0;
}
