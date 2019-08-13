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

last update: 12/8/2019
*/
 
#include "find_cup_ros/fast_find_ellipse_detector.h"
#include <iostream>


using namespace std;
using namespace cv;
 
  
void LoadGT(vector<Ellipse>& gt, const string& sGtFileName, bool bIsAngleInRadians)
{ 
	// ifstream in(sGtFileName);
	// if (!in.good())
	// {
	// 	std::cout << "Error opening: " << sGtFileName << std::endl;
	// 	return;
	// }

	// unsigned n;
	// in >> n;

	// gt.clear();
	// gt.reserve(n);

	// while (in.good() && n--)
	// {
	// 	Ellipse e;
	// 	in >> e._xc >> e._yc >> e._a >> e._b >> e._rad;

	// 	if (!bIsAngleInRadians)
	// 	{
	// 		// convert to radians
	// 		e._rad = float(e._rad * CV_PI / 180.0);
	// 	}

	// 	if (e._a < e._b)
	// 	{
	// 		float temp = e._a;
	// 		e._a = e._b;
	// 		e._b = temp;

	// 		e._rad = e._rad + float(0.5*CV_PI);
	// 	}

	// 	e._rad = fmod(float(e._rad + 2.f*CV_PI), float(CV_PI));
	// 	e._score = 1.f;
	// 	gt.push_back(e);
	// }
	// in.close();
}

bool TestOverlap(const Mat1b& gt, const Mat1b& test, float th)
{
	float fAND = float(countNonZero(gt & test));
	float fOR = float(countNonZero(gt | test));
	float fsim = fAND / fOR;

	return (fsim >= th);
}

int Count(const vector<bool> v)
{
	int counter = 0;
	for (unsigned i = 0; i < v.size(); ++i)
	{
		if (v[i]) { ++counter; }
	}
	return counter;
}


// Should be checked !!!!!
std::tuple<float, float, float> Evaluate(const vector<Ellipse>& ellGT, const vector<Ellipse>& ellTest, const float th_score, const Mat3b& img)
{
	float threshold_overlap = 0.8f;
	//float threshold = 0.95f;

	unsigned sz_gt = ellGT.size();
	unsigned size_test = ellTest.size();

	unsigned sz_test = unsigned(min(1000, int(size_test)));

	vector<Mat1b> gts(sz_gt);
	vector<Mat1b> tests(sz_test);

	for (unsigned i = 0; i < sz_gt; ++i)
	{
		const Ellipse& e = ellGT[i];

		Mat1b tmp(img.rows, img.cols, uchar(0));
		ellipse(tmp, Point(e._xc, e._yc), Size(e._a, e._b), e._rad * 180.0 / CV_PI, 0.0, 360.0, Scalar(255), -1);
		gts[i] = tmp;
	}

	for (unsigned i = 0; i < sz_test; ++i)
	{
		const Ellipse& e = ellTest[i];

		Mat1b tmp(img.rows, img.cols, uchar(0));
		ellipse(tmp, Point(e._xc, e._yc), Size(e._a, e._b), e._rad * 180.0 / CV_PI, 0.0, 360.0, Scalar(255), -1);
		tests[i] = tmp;
	}

	Mat1b overlap(sz_gt, sz_test, uchar(0));
	for (int r = 0; r < overlap.rows; ++r)
	{
		for (int c = 0; c < overlap.cols; ++c)
		{
			overlap(r, c) = TestOverlap(gts[r], tests[c], threshold_overlap) ? uchar(255) : uchar(0);
		}
	}

	int counter = 0;

	vector<bool> vec_gt(sz_gt, false);

	for (int i = 0; i < sz_test; ++i)
	{
		const Ellipse& e = ellTest[i];
		for (int j = 0; j < sz_gt; ++j)
		{
			if (vec_gt[j]) { continue; }

			bool bTest = overlap(j, i) != 0;

			if (bTest)
			{
				vec_gt[j] = true;
				break;
			}
		}
	}

	int tp = Count(vec_gt);
	int fn = int(sz_gt) - tp;
	int fp = size_test - tp; // !!!!

	float pr(0.f);
	float re(0.f);
	float fmeasure(0.f);

	if (tp == 0)
	{
		if (fp == 0)
		{
			pr = 1.f;
			re = 0.f;
			fmeasure = (2.f * pr * re) / (pr + re);
		}
		else
		{
			pr = 0.f;
			re = 0.f;
			fmeasure = 0.f;
		}
	}
	else
	{
		pr = float(tp) / float(tp + fp);
		re = float(tp) / float(tp + fn);
		fmeasure = (2.f * pr * re) / (pr + re);
	}

	return make_tuple(pr, re, fmeasure);
}

cv::Point2i OnImage(cv::Mat image)
{ 
	cv::Point2i ellpse_centor;
	
	Size sz = image.size();
	if(!image.empty())
	{
		// Convert to grayscale
		Mat1b gray;
		cvtColor(image, gray, CV_BGR2GRAY);


		// Parameters Settings (Sect. 4.2)
		int		iThLength = 16;
		float	fThObb = 3.0f;
		float	fThPos = 1.0f;
		float	fTaoCenters = 0.05f;
		int 	iNs = 16;
		float	fMaxCenterDistance = sqrt(float(sz.width*sz.width + sz.height*sz.height)) * fTaoCenters;

		float	fThScoreScore = 0.4f;

		// Other constant parameters settings. 

		// Gaussian filter parameters, in pre-processing
		Size	szPreProcessingGaussKernelSize = Size(5, 5);
		double	dPreProcessingGaussSigma = 1.0;

		float	fDistanceToEllipseContour = 0.1f;	// (Sect. 3.3.1 - Validation)
		float	fMinReliability = 0.4f;	// Const parameters to discard bad ellipses


		// Initialize Detector with selected parameters
		CEllipseDetectorYaed* yaed = new CEllipseDetectorYaed();
		yaed->SetParameters(szPreProcessingGaussKernelSize,
			dPreProcessingGaussSigma,
			fThPos,
			fMaxCenterDistance,
			iThLength,
			fThObb,
			fDistanceToEllipseContour,
			fThScoreScore,
			fMinReliability,
			iNs
			);


		// Detect
		vector<Ellipse> ellsYaed;
		Mat1b gray2 = gray.clone();
		yaed->Detect(gray2, ellsYaed);

		vector<double> times = yaed->GetTimes();
		// std::cout << "--------------------------------" << std::endl;
		// std::cout << "Execution Time: " << std::endl;
		// std::cout << "Edge Detection: \t" << times[0] << std::endl;
		// std::cout << "Pre processing: \t" << times[1] << std::endl;
		// std::cout << "Grouping:       \t" << times[2] << std::endl;
		// std::cout << "Estimation:     \t" << times[3] << std::endl;
		// std::cout << "Validation:     \t" << times[4] << std::endl;
		// std::cout << "Clustering:     \t" << times[5] << std::endl;
		// std::cout << "--------------------------------" << std::endl;
		// std::cout << "Total:	         \t" << yaed->GetExecTime() << std::endl;
		// std::cout << "--------------------------------" << std::endl;


		vector<Ellipse> gt;
		// LoadGT(gt, filename_minus_ext + ".txt", true); // Prasad is in radians

		Mat3b resultImage = image.clone();

		// Draw GT ellipses
		for (unsigned i = 0; i < gt.size(); ++i)
		{
			Ellipse& e = gt[i];
			Scalar color(0, 0, 255);
			ellipse(resultImage, Point(cvRound(e._xc), cvRound(e._yc)), Size(cvRound(e._a), cvRound(e._b)), e._rad*180.0 / CV_PI, 0.0, 360.0, color, 3);
		}

		yaed->DrawDetectedEllipses(resultImage, ellsYaed);

		Mat3b res = image.clone();

		Evaluate(gt, ellsYaed, fThScoreScore, res);

		// Show the image in a scalable window.
		namedWindow("Annotated Image", WINDOW_NORMAL);
		imshow("Annotated Image", resultImage);
		waitKey(2);
		
		int iTopN = 0; 
		int sz_ell = int(ellsYaed.size()) - 1; // 搜素到的椭圆数量
		int max_score_index = sz_ell; 
		double max_score = 0.0;
		int n = (iTopN == 0) ? sz_ell : min(iTopN, sz_ell); // 遍历的椭圆数量
		for (int i = 0; i < sz_ell; ++i) 
		{
			Ellipse& e = ellsYaed[n - i - 1];
			if(e._score >= max_score)
			{
				max_score_index = n - i - 1;
				max_score = e._score;
			} 	
		}
		if(sz_ell > 0)
			ellpse_centor = cv::Point2i(cvRound(ellsYaed[max_score_index]._xc), cvRound(ellsYaed[max_score_index]._yc));
	}
	
	return ellpse_centor;
}


void maxScoreElipseFind(cv::Mat image, cv::Point2i &ellpse_centor, float &diameter)
{ 
	
	Size sz = image.size();
	if(!image.empty())
	{
		// Convert to grayscale
		Mat1b gray;
		cvtColor(image, gray, CV_BGR2GRAY);


		// Parameters Settings (Sect. 4.2)
		int		iThLength = 16;
		float	fThObb = 3.0f;
		float	fThPos = 1.0f;
		float	fTaoCenters = 0.05f;
		int 	iNs = 16;
		float	fMaxCenterDistance = sqrt(float(sz.width*sz.width + sz.height*sz.height)) * fTaoCenters;

		float	fThScoreScore = 0.4f;

		// Other constant parameters settings. 

		// Gaussian filter parameters, in pre-processing
		Size	szPreProcessingGaussKernelSize = Size(5, 5);
		double	dPreProcessingGaussSigma = 1.0;

		float	fDistanceToEllipseContour = 0.1f;	// (Sect. 3.3.1 - Validation)
		float	fMinReliability = 0.4f;	// Const parameters to discard bad ellipses


		// Initialize Detector with selected parameters
		CEllipseDetectorYaed* yaed = new CEllipseDetectorYaed();
		yaed->SetParameters(szPreProcessingGaussKernelSize,
			dPreProcessingGaussSigma,
			fThPos,
			fMaxCenterDistance,
			iThLength,
			fThObb,
			fDistanceToEllipseContour,
			fThScoreScore,
			fMinReliability,
			iNs
			);


		// Detect
		vector<Ellipse> ellsYaed;
		Mat1b gray2 = gray.clone();
		yaed->Detect(gray2, ellsYaed);

		vector<double> times = yaed->GetTimes();
		// std::cout << "--------------------------------" << std::endl;
		// std::cout << "Execution Time: " << std::endl;
		// std::cout << "Edge Detection: \t" << times[0] << std::endl;
		// std::cout << "Pre processing: \t" << times[1] << std::endl;
		// std::cout << "Grouping:       \t" << times[2] << std::endl;
		// std::cout << "Estimation:     \t" << times[3] << std::endl;
		// std::cout << "Validation:     \t" << times[4] << std::endl;
		// std::cout << "Clustering:     \t" << times[5] << std::endl;
		// std::cout << "--------------------------------" << std::endl;
		// std::cout << "Total:	         \t" << yaed->GetExecTime() << std::endl;
		// std::cout << "--------------------------------" << std::endl;


		vector<Ellipse> gt;
		// LoadGT(gt, filename_minus_ext + ".txt", true); // Prasad is in radians

		Mat3b resultImage = image.clone();

		// Draw GT ellipses
		for (unsigned i = 0; i < gt.size(); ++i)
		{
			Ellipse& e = gt[i];
			Scalar color(0, 0, 255);
			ellipse(resultImage, Point(cvRound(e._xc), cvRound(e._yc)), Size(cvRound(e._a), cvRound(e._b)), e._rad*180.0 / CV_PI, 0.0, 360.0, color, 3);
		}

		yaed->DrawDetectedEllipses(resultImage, ellsYaed);

		Mat3b res = image.clone();

		Evaluate(gt, ellsYaed, fThScoreScore, res);

		// Show the image in a scalable window.
		namedWindow("Annotated Image", WINDOW_NORMAL);
		imshow("Annotated Image", resultImage);
		waitKey(2);
		
		int iTopN = 0; 
		int sz_ell = int(ellsYaed.size()) - 1; // 搜素到的椭圆数量
		int max_score_index = sz_ell; 
		double max_score = 0.0;
		int n = (iTopN == 0) ? sz_ell : min(iTopN, sz_ell); // 遍历的椭圆数量
		for (int i = 0; i < sz_ell; ++i) 
		{
			Ellipse& e = ellsYaed[n - i - 1];
			if(e._score >= max_score)
			{
				max_score_index = n - i - 1;
				max_score = e._score;
			} 	
		}
		if(sz_ell > 0)
		{
			ellpse_centor = cv::Point2i(cvRound(ellsYaed[max_score_index]._xc), cvRound(ellsYaed[max_score_index]._yc));
			diameter = (ellsYaed[max_score_index]._a > ellsYaed[max_score_index]._b)? ellsYaed[max_score_index]._a : ellsYaed[max_score_index]._b; 
		} 
	}
}

void multiElipseFind(cv::Mat image, vector<Ellipse> &ellsYaed)
{ 
	cv::Point2i ellpse_centor;
	
	Size sz = image.size(); 	
	if(!image.empty())
	{
		// Convert to grayscale
		Mat1b gray;
		cvtColor(image, gray, CV_BGR2GRAY);
 
		// Parameters Settings (Sect. 4.2)
		int		iThLength = 16;
		float	fThObb = 3.0f;
		float	fThPos = 1.0f;
		float	fTaoCenters = 0.05f;
		int 	iNs = 16;
		float	fMaxCenterDistance = sqrt(float(sz.width*sz.width + sz.height*sz.height)) * fTaoCenters;

		float	fThScoreScore = 0.4f;

		// Other constant parameters settings. 

		// Gaussian filter parameters, in pre-processing
		Size	szPreProcessingGaussKernelSize = Size(5, 5);
		double	dPreProcessingGaussSigma = 1.0;

		float	fDistanceToEllipseContour = 0.1f;	// (Sect. 3.3.1 - Validation)
		float	fMinReliability = 0.4f;	// Const parameters to discard bad ellipses
 
		// Initialize Detector with selected parameters
		CEllipseDetectorYaed* yaed = new CEllipseDetectorYaed();
		yaed->SetParameters(szPreProcessingGaussKernelSize,
			dPreProcessingGaussSigma,
			fThPos,
			fMaxCenterDistance,
			iThLength,
			fThObb,
			fDistanceToEllipseContour,
			fThScoreScore,
			fMinReliability,
			iNs
			);
 
		// Detect 
		Mat1b gray2 = gray.clone();
		yaed->Detect(gray2, ellsYaed);

		vector<double> times = yaed->GetTimes();
		// std::cout << "--------------------------------" << std::endl;
		// std::cout << "Execution Time: " << std::endl;
		// std::cout << "Edge Detection: \t" << times[0] << std::endl;
		// std::cout << "Pre processing: \t" << times[1] << std::endl;
		// std::cout << "Grouping:       \t" << times[2] << std::endl;
		// std::cout << "Estimation:     \t" << times[3] << std::endl;
		// std::cout << "Validation:     \t" << times[4] << std::endl;
		// std::cout << "Clustering:     \t" << times[5] << std::endl;
		// std::cout << "--------------------------------" << std::endl;
		// std::cout << "Total:	         \t" << yaed->GetExecTime() << std::endl;
		// std::cout << "--------------------------------" << std::endl;


		vector<Ellipse> gt;
		// LoadGT(gt, filename_minus_ext + ".txt", true); // Prasad is in radians

		Mat3b resultImage = image.clone();

		// Draw GT ellipses
		for (unsigned i = 0; i < gt.size(); ++i)
		{
			Ellipse& e = gt[i];
			Scalar color(0, 0, 255);
			ellipse(resultImage, Point(cvRound(e._xc), cvRound(e._yc)), Size(cvRound(e._a), cvRound(e._b)), e._rad*180.0 / CV_PI, 0.0, 360.0, color, 3);
		}

		yaed->DrawDetectedEllipses(resultImage, ellsYaed);

		Mat3b res = image.clone();

		Evaluate(gt, ellsYaed, fThScoreScore, res);

		// Show the image in a scalable window.
		namedWindow("Annotated Image", WINDOW_NORMAL);
		imshow("Annotated Image", resultImage);
		waitKey(2); 
	}
}