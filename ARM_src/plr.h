#ifndef _PLR_H_
#define _PLR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <cvaux.h> 
#include <stdio.h>
#include <ctime>
#include <opencv2/ml/ml.hpp>

using namespace cv; 
using namespace std;

bool verifySizes_closeImg(const RotatedRect & candidate);
void RgbConvToGray(const Mat& inputImage,Mat & outpuImage);
void normal_area(Mat &intputImg, vector<RotatedRect> &rects_optimal, vector <Mat>& output_area );
bool char_verifySizes(const RotatedRect & candidate);
void char_sort(vector <RotatedRect > & in_char );
bool clearMaoDing(Mat &img);
bool is_plate(const Mat & inputImg);
bool char_segment(const Mat & inputImg,vector <Mat>& dst_mat);
Mat ProjectedHistogram(Mat img, int t);
void features(const Mat & in , Mat & out ,int sizeData);
void ann_train(CvANN_MLP &ann ,int numCharacters, int nlayers, string str);
void svm_train(CvSVM & svmClassifier);
vector<int> plr(Mat img_input);

#endif
