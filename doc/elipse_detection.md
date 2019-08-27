# elipse_detection 说明

椭圆检测用户层程序请参考 src/fast_find_ellipse_detector.cpp 文件

### 返回检测到的所有可能的椭圆

void multiElipseFind(cv::Mat image, vector<Ellipse> &ellsYaed);

### 返回效果最好的椭圆
void maxScoreElipseFind(cv::Mat image, cv::Point2i &ellpse_centor, float &diameter);