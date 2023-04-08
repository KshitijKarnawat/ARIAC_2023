#include "tray_id_detect.hpp"
#include <type_traits>

std::vector<int> tray_detect(cv::Mat frame){

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
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    detector.detectMarkers(img, corners, markerIDs, rejected);

    cv::Mat outputImage = im.clone();
    // cv::aruco::drawDetectedMarkers(outputImage, corners, markerIDs);

    // if (markerIDs.size() > 0) {
    //     cv::aruco::drawDetectedMarkers(outputImage, corners, markerIDs);

    //     for(const auto& corner : corners) {
    //         cv::Point2f center(0.f, 0.f);

    //         for(const auto& c : corner) {
    //             center += c;
    //         }
    //         center /= 4.f;
    //         cv::circle(outputImage, center, 4, cv::Scalar(0,0,255), cv::FILLED);
    //     }
    // }
    
    return // some vector
}