/*
This code is intended for academic use only.
You are free to use and modify the code, at your own risk.

If you use this code, or find it useful, please refer to the paper:

Michele Fornaciari, Andrea Prati, Rita Cucchiara,
A fast and effective ellipse detector for embedded vision applications
Pattern Recognition, Volume 47, Issue 11, November 2014, Pages 3693-3708, ISSN 0031-3203,
http://dx.doi.org/10.1016/j.patcog.2014.05.012.
(http://www.sciencedirect.com/science/article/pii/S0031320314001976)


The comments in the code refer to the abovementioned paper.
If you need further details about the code or the algorithm, please contact me at:

michele.fornaciari@unimore.it

last update: 23/12/2014
*/

#pragma once
#include <limits.h> /* PATH_MAX */
#include <stdlib.h>
#include <stdio.h>
// #include <cv.h>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp> 
#include "find_cup_ros/EllipseDetectorYaed.h"
#include <fstream> 
 
void LoadGT(vector<Ellipse>& gt, const string& sGtFileName, bool bIsAngleInRadians);


bool TestOverlap(const Mat1b& gt, const Mat1b& test, float th);

int Count(const vector<bool> v);
 
// Should be checked !!!!!
std::tuple<float, float, float> Evaluate(const vector<Ellipse>& ellGT, const vector<Ellipse>& ellTest, const float th_score, const Mat3b& img);
 
cv::Point2i OnImage(cv::Mat);
    
void multiElipseFind(cv::Mat image, vector<Ellipse> &ellsYaed);

void maxScoreElipseFind(cv::Mat image, cv::Point2i &ellpse_centor, float &diameter);