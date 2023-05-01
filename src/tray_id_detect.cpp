/**
 * @copyright Copyright (c) 2023
 * @file tray_id_detect.cpp
 * @author Sanchit Kedia (sanchit@terpmail.umd.edu)
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Tanmay Haldankar (tanmayh@terpmail.umd.edu)
 * @author Sahruday Patti (sahruday@umd.edu)
 * @author Kshitij Karnawat (kshitij@umd.edu)
 * @brief Implementation of Tray Detection using OpenCV for ARIAC 2023 (Group 3)
 * @version 0.2
 * @date 2023-03-04
 * 
 * 
 */
#include "tray_id_detect.hpp"

std::vector<int> tray_detect(cv::Mat frame){

    std::vector<int> tray_aruco_id{-1, -1, -1};

    cv::Mat im = frame;
    cv::Mat img = im.clone();
    
    cv::Mat blank_img(img.size().height, img.size().width, CV_8UC3, cv::Scalar(255, 255, 255));
    blank_img(cv::Range(212,243), cv::Range(167,198)) = 0;
    blank_img(cv::Range(212,243), cv::Range(305,336)) = 0;
    blank_img(cv::Range(212,243), cv::Range(443,475)) = 0;
    
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); 
    cv::cvtColor(blank_img, blank_img, cv::COLOR_BGR2GRAY);
    cv::add(img, blank_img, img);
    
    std::vector<int> markerIDs;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    
    // For OpenCV 4.7.0
    // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    // cv::aruco::detectMarkers(img, dictionary, corners, markerIDs);
    
    // For OpenCV 4.2.0
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    detector.detectMarkers(img, corners, markerIDs, rejected);

    cv::Mat outputImage = im.clone();
    // cv::aruco::drawDetectedMarkers(outputImage, corners, markerIDs);

    if (markerIDs.size() > 0) {
        int count = 0;
        for(const auto& corner : corners) {
            cv::Point2f center(0.f, 0.f);

            for(const auto& c : corner) {
                center += c;
            }
            center /= 4.f;
            if (center.x < 250) {
                tray_aruco_id.at(0) = markerIDs[count];
            } else if (center.x < 400) {
                tray_aruco_id.at(1) = markerIDs[count];
            } else {
                tray_aruco_id.at(2) = markerIDs[count];
            }
            count += 1; 
            // cv::circle(outputImage, center, 4, cv::Scalar(0,0,255), cv::FILLED);
        }
    }
    
    return tray_aruco_id;
}