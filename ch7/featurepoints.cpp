#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv){
    
    
    
    
    Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2],CV_LOAD_IMAGE_COLOR);
 
    std::vector<KeyPoint> keypoints_1,keypoints_2,Keypoints_1good,Keypoints_2good;
    Mat descriptors_1, descriptors_2;
    Ptr<ORB> orb = ORB::create(500,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31,20);
    
    orb->detect(img_1,keypoints_1);
    orb->detect(img_2,keypoints_2);
    orb->compute(img_1,keypoints_1,descriptors_1);
    orb->compute(img_2,keypoints_2,descriptors_2);
    
    Mat outimg1;
    drawKeypoints(img_1,keypoints_1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    imshow("",outimg1);
    
    
    vector<DMatch> matches;
    BFMatcher matcher (NORM_HAMMING);
    matcher.match(descriptors_1,descriptors_2,matches);
    
    
    
    
    
    double min_dist=10000,max_dist = 0;
    
    
    for (int i = 0; i < descriptors_1.rows;i++){
        double dist = matches[i].distance;
        if(dist< min_dist) min_dist = dist;
        if(dist>max_dist) max_dist = dist;
    }
    
    std::vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows;i++){
        if(matches[i].distance <= max(2*min_dist,30.0)){
            good_matches.push_back(matches[i]);
        }
        
    }
    
    
    
    for (int j = 0;j < good_matches.size();j++){
        cout<<"match:"<<j<<"  "<<good_matches[j].queryIdx<<"  "<<good_matches[j].trainIdx<<keypoints_1[good_matches[j].queryIdx].pt<<keypoints_2[good_matches[j].trainIdx].pt<<endl;
        Keypoints_1good.push_back(keypoints_1[good_matches[j].queryIdx]);
        Keypoints_2good.push_back(keypoints_2[good_matches[j].trainIdx]);
    }  //the points in pic1(querypoint) find match point in pic2,
    
 
     
    cout<<good_matches.size()<<endl;
 
    
    
    
    Mat img_match,img_goodmatch,img_1_prime,img_2_prime;
    drawMatches(img_1,keypoints_1,img_2,keypoints_2,matches,img_match);
    drawMatches(img_1,keypoints_1,img_2,keypoints_2,good_matches,img_goodmatch);
 
    imshow("",img_match);
   // waitKey(0);
    imshow("",img_goodmatch);
   // waitKey(0);
 
    drawKeypoints(img_1,Keypoints_1good,img_1_prime,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    drawKeypoints(img_2,Keypoints_2good,img_2_prime,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    imshow("",img_1_prime);
    waitKey(0);
    imshow("",img_2_prime);
    waitKey(0);
    
    
    
    return 0;
}
