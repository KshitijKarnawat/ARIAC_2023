#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

#include <iostream>
#include <vector>
#include <string>

std::vector<int> tray_detect(cv::Mat frame);