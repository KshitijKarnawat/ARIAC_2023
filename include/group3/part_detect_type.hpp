#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>
#include <iostream>

int detect_type(cv::Mat img, std::vector<cv::Point> cnt);

std::vector<int> detect_color(cv::Mat img, cv::Mat new_image, std::vector<cv::Point> c, int x_m, int y_m);

std::vector<std::vector<int>> rightbin(cv::Mat img);

std::vector<std::vector<int>> leftbin(cv::Mat img);