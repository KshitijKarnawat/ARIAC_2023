/**
 * @copyright Copyright (c) 2023
 * @file part_type_detect.hpp
 * @author Sanchit Kedia (sanchit@terpmail.umd.edu)
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Tanmay Haldankar (tanmayh@terpmail.umd.edu)
 * @author Sahruday Patti (sahruday@umd.edu)
 * @author Kshitij Karnawat (kshitij@umd.edu)
 * @brief Implementation of Part Detection using OpenCV for ARIAC 2023 (Group 3)
 * @version 0.1
 * @date 2023-04-30
 * 
 * 
 */
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <iostream>

/**
 * @brief Function to return the type of the parts in the image
 * 
 * @param img 
 * @param cnt 
 * @return int 
 */
int detect_type(cv::Mat img, std::vector<cv::Point> cnt);

/**
 * @brief Function to return the color of the parts in the image
 * 
 * @param img 
 * @param new_image 
 * @param c 
 * @param x_m 
 * @param y_m 
 * @return std::vector<int> 
 */
std::vector<int> detect_color(cv::Mat img, cv::Mat new_image, std::vector<cv::Point> c, int x_m, int y_m);

/**
 * @brief Function to return the right bin parts
 * 
 * @param img 
 * @return std::vector<std::vector<int>> 
 */
std::vector<std::vector<int>> rightbin(cv::Mat img);

/**
 * @brief Function to return the left bin parts
 * 
 * @param img 
 * @return std::vector<std::vector<int>> 
 */
std::vector<std::vector<int>> leftbin(cv::Mat img);