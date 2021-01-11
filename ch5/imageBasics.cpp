#include<iostream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace std;

int main (int argc, char** argv){
    
    cv::Mat image;
    image = cv::imread(argv[1]);

    if(image.data == nullptr){
        
    cerr<<argv[1]<<"is not existed!"<<endl;
    return 0;
        
        
        
    }
    cout<<image.rows<<",,,,"<<image.cols<<"sssss"<<image.channels()<<endl;
 
    cv::waitKey(0);
    if (image.type()!= CV_8UC1  && image.type()!=CV_8UC3){
    return 0;
        
    }
    
    
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (size_t x = 0; x<image.cols;x++    ){
        for (size_t y = 0; y<image.rows;y++){
            
            unsigned char* row_ptr = image.ptr<unsigned char>(y);
            unsigned char* data_ptr = &row_ptr[x*image.channels()];
            for (int i = 0; i < image.channels();i++){
                
                
                
            }
            
    
            
            
            
            
            
        }
        
        
        
        
        
        
    }
    
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<time_used.count()<<endl;
    
    
    cv::Mat image_2 = image;
    cv::Mat image_3 = image.clone();
    image_2(cv::Rect(0,0,111,111)).setTo(255);
    cv::imshow("",image);
    cv::waitKey(0);
    cv::imshow("",image_3);
    cv::waitKey(0);
    
    cv::destroyAllWindows();
    return 0;
    
    
    
    
    
    
    return 0;
}
