#include "DBoW3/DBoW3.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <string>
#include <vector>


using namespace cv;
using namespace std;

int main(int argc, char** argv){
    vector<Mat> images;
    for(int i = 0; i < 10; i++){
        string path = "../data/" + to_string(i+1) + ".png";
        images.push_back(imread(path));
    }
    Ptr<Feature2D> detector = ORB::create();
    vector<Mat> descriptors;
    for (Mat& image:images){
        vector<KeyPoint> KeyPoints;
        Mat descriptor;
        detector->detectAndCompute(image, Mat(),KeyPoints,descriptor);
        descriptors.push_back(descriptor);
    }
    
    DBoW3::Vocabulary vocab;
    vocab.create(descriptors);
    vocab.save("vocabulary.yml.gz");
    
    
    
    
    return 0;
}
