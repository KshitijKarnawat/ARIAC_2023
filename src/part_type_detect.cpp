#include "part_type_detect.hpp"

int detect_type(cv::Mat img, std::vector<cv::Point> cnt) {
    int type;
    cv::Rect rect = cv::boundingRect(cnt);
    int x = rect.x;
    int y = rect.y;
    int w = rect.width;
    int h = rect.height;

    cv::rectangle(img, rect, cv::Scalar(0, 255, 0), 2);
    // cv::imshow("uncut image", img); 
    // cv::waitKey(0);

    cv::Mat im = img.clone();
    im = im(cv::Range(y+1,y+h+1), cv::Range(x+1,x+w+1));
    cv::Mat im_hsv;
    cv::cvtColor(im.clone(), im_hsv, cv::COLOR_BGR2HSV);

    cv::Scalar lower_gray(0, 0, 32);
    cv::Scalar upper_gray(0, 0, 117);
    cv::Mat mask;
    cv::inRange(im_hsv, lower_gray, upper_gray, mask);

    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element, cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    int count = 0;

    double area_gray;
    double perimeter;

    for (auto c : contours) {
        area_gray = cv::contourArea(c);
        perimeter = cv::arcLength(cnt, true);
        if (area_gray > 10) {
            count += 1;
        } 
    }

    if (count == 2) {
        type = ariac_msgs::msg::Part::REGULATOR; //13 - Regulator
    } else if (count == 1) {
        if (perimeter < 155 && 57 < area_gray && area_gray < 214) {
            type = ariac_msgs::msg::Part::BATTERY; //10 - Battery
        } else if (area_gray < 20) {
            type = ariac_msgs::msg::Part::PUMP;   //11 - Pump
        } else if (perimeter > 151 && 23 < area_gray  && area_gray < 203) {
            type = ariac_msgs::msg::Part::SENSOR;  //12 - Sensor
        }
    } else if (count == 0) {
        type = ariac_msgs::msg::Part::PUMP;  //11 - Pump
    } 

    return type;
}


std::vector<int> detect_color(cv::Mat img, cv::Mat new_image, std::vector<cv::Point> c, int x_m, int y_m){
    int part_clr;
    int part_type;
    std::vector<int> clr_type;
    cv::Mat img_hsv;
    cv::cvtColor(img.clone(), img_hsv, cv::COLOR_BGR2HSV);
    
    cv::Vec3b hsv = img_hsv.at<cv::Vec3b>(y_m, x_m);
    
    if (hsv[0]>10 && hsv[0]<25) {
        part_clr = ariac_msgs::msg::Part::ORANGE;  // 3 - Orange
        part_type = detect_type(new_image.clone(), c);
    } else if(hsv[0]>=130 && hsv[0]<170) {
        part_clr = ariac_msgs::msg::Part::PURPLE;  // 4 - Purple
        part_type = detect_type(new_image.clone(), c);
    } else if (hsv[0]>=90 && hsv[0]<130) {
        part_clr = ariac_msgs::msg::Part::BLUE;  // 2 - Blue
        part_type = detect_type(new_image.clone(), c);
    } else if (hsv[0]>=0 && hsv[0]<=10) { 
        part_clr = ariac_msgs::msg::Part::RED;  // 0 - Red
        part_type = detect_type(new_image.clone(), c);
    } else if (hsv[0]>36 && hsv[0]<89) {
        part_clr = ariac_msgs::msg::Part::GREEN;  // 1 - Green  
        part_type = detect_type(new_image.clone(), c);
    }
    clr_type.push_back(part_clr);
    clr_type.push_back(part_type);

    return clr_type;
}
    

std::vector<std::vector<int>> rightbin(cv::Mat img){
    std::vector<std::vector<int>> right_bin_info;
    std::vector<int> part_info;
    cv::Mat img_bin_tr = img(cv::Range(28,223), cv::Range(356,548));
    cv::Mat img_bin_tl = img(cv::Range(28,223), cv::Range(118,308));
    cv::Mat img_bin_br = img(cv::Range(267,458), cv::Range(356,548));
    cv::Mat img_bin_bl = img(cv::Range(267,458), cv::Range(118,308));

    std::vector<cv::Mat> image{img_bin_br, img_bin_bl, img_bin_tl, img_bin_tr};
    int i = 0;

    for (auto img : image) {

        cv::Mat img_hsv;
        cv::cvtColor(img.clone(), img_hsv, cv::COLOR_BGR2HSV);

        cv::Scalar lower_red(0, 25, 25);
        cv::Scalar upper_red(10, 255, 255);
        cv::Scalar lower_red1(170, 25, 25);
        cv::Scalar upper_red1(180, 255, 255);

        cv::Scalar lower_green(36, 25, 25);
        cv::Scalar upper_green(70, 255, 255);

        cv::Scalar lower_blue(110, 25, 25);
        cv::Scalar upper_blue(130, 255, 255);

        cv::Scalar lower_orange(6, 25, 25);
        cv::Scalar upper_orange(26, 255, 255);

        cv::Scalar lower_purple(128, 25, 25);
        cv::Scalar upper_purple(148, 255, 255);

        cv::Scalar lower_gray(0, 0, 32);
        cv::Scalar upper_gray(0, 255, 117);
        
        cv::Mat mask1, mask11;
        cv::inRange(img_hsv, lower_red, upper_red, mask1);
        cv::inRange(img_hsv, lower_red1, upper_red1, mask11);
        
        cv::Mat mask2;
        cv::inRange(img_hsv, lower_green, upper_green, mask2);
        
        cv::Mat mask3;
        cv::inRange(img_hsv, lower_blue, upper_blue, mask3);

        cv::Mat mask4;
        cv::inRange(img_hsv, lower_orange, upper_orange, mask4);

        cv::Mat mask5;
        cv::inRange(img_hsv, lower_purple, upper_purple, mask5);
        
        cv::Mat mask6;
        cv::inRange(img_hsv, lower_gray, upper_gray, mask6);

        cv::Mat mask = mask1 + mask11 + mask2 + mask3 + mask4 + mask5 + mask6;
        
        cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element, cv::Point(-1,-1), 2);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element, cv::Point(-1,-1), 1);
        
        cv::Mat new_image;
        bitwise_and(img, img, new_image, mask);
        
        cv::Mat blur;
        cv::GaussianBlur(mask, blur, cv::Size(5, 5), 0);
        
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        findContours(blur, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        int x_m;
        int y_m;
        int quadrant;

        for (auto c : contours) {
            auto area_cnt = cv::contourArea(c);
            cv::Moments M = cv::moments(c);
            if (M.m00 != 0.0) { 
                cv::Point p(M.m10/M.m00, M.m01/M.m00);
                x_m = int(p.x);
                y_m = int(p.y);
            }

            if (area_cnt > 500 && area_cnt < 3000) {
                if ((137<=x_m && x_m<=195) &&  (130<=y_m && y_m<=192)){
                    quadrant =  9*i + 9;
                }
                else if ((0<=x_m && x_m<=70) &&  (0<=y_m && y_m<=65)){
                    quadrant =  9*i + 1;
                }
                else if ((71<=x_m && x_m<=135) &&  (0<=y_m && y_m<=65)){
                    quadrant =  9*i + 2;
                }
                else if ((137<=x_m && x_m<=195) &&  (0<=y_m && y_m<=65)){
                    quadrant =  9*i + 3;
                }
                else if ((0<=x_m && x_m<=70) &&  (69<=y_m && y_m<=126)){
                    quadrant =  9*i + 4;
                }
                else if ((71<=x_m && x_m<=135) &&  (69<=y_m && y_m<=126)){
                    quadrant =  9*i + 5;
                }
                else if ((137<=x_m && x_m<=195) &&  (69<=y_m && y_m<=126)){
                    quadrant =  9*i + 6;
                }
                else if ((0<=x_m && x_m<=70) &&  (130<=y_m && y_m<=192)){
                    quadrant =  9*i + 7;
                }
                else if ((71<=x_m && x_m<=135) &&  (130<=y_m && y_m<=192)){
                    quadrant =  9*i + 8;
                }
                
                part_info = detect_color(img, new_image, c, x_m, y_m);
                part_info.push_back(quadrant);
                right_bin_info.push_back(part_info);
            }
        }
        i++;
    }
    return right_bin_info;
}

std::vector<std::vector<int>> leftbin(cv::Mat img){
    std::vector<std::vector<int>> left_bin_info;
    std::vector<int> part_info;
    cv::Mat img_bin_tr = img(cv::Range(28,223), cv::Range(356,548));
    cv::Mat img_bin_tl = img(cv::Range(28,223), cv::Range(118,308));
    cv::Mat img_bin_br = img(cv::Range(267,458), cv::Range(356,548));
    cv::Mat img_bin_bl = img(cv::Range(267,458), cv::Range(118,308));

    std::vector<cv::Mat> image{img_bin_br, img_bin_bl, img_bin_tl, img_bin_tr};
    int i = 0;

    for (auto img : image) {

        cv::Mat img_hsv;
        cv::cvtColor(img.clone(), img_hsv, cv::COLOR_BGR2HSV);

        cv::Scalar lower_red(0, 25, 25);
        cv::Scalar upper_red(10, 255, 255);
        cv::Scalar lower_red1(170, 25, 25);
        cv::Scalar upper_red1(180, 255, 255);

        cv::Scalar lower_green(36, 25, 25);
        cv::Scalar upper_green(70, 255, 255);

        cv::Scalar lower_blue(110, 25, 25);
        cv::Scalar upper_blue(130, 255, 255);

        cv::Scalar lower_orange(6, 25, 25);
        cv::Scalar upper_orange(26, 255, 255);

        cv::Scalar lower_purple(128, 25, 25);
        cv::Scalar upper_purple(148, 255, 255);

        cv::Scalar lower_gray(0, 0, 32);
        cv::Scalar upper_gray(0, 255, 117);
        
        cv::Mat mask1, mask11;
        cv::inRange(img_hsv, lower_red, upper_red, mask1);
        cv::inRange(img_hsv, lower_red1, upper_red1, mask11);
        
        cv::Mat mask2;
        cv::inRange(img_hsv, lower_green, upper_green, mask2);
        
        cv::Mat mask3;
        cv::inRange(img_hsv, lower_blue, upper_blue, mask3);

        cv::Mat mask4;
        cv::inRange(img_hsv, lower_orange, upper_orange, mask4);

        cv::Mat mask5;
        cv::inRange(img_hsv, lower_purple, upper_purple, mask5);
        
        cv::Mat mask6;
        cv::inRange(img_hsv, lower_gray, upper_gray, mask6);

        cv::Mat mask = mask1 + mask11 + mask2 + mask3 + mask4 + mask5 + mask6;
        
        cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element, cv::Point(-1,-1), 2);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element, cv::Point(-1,-1), 1);
        
        cv::Mat new_image;
        bitwise_and(img, img, new_image, mask);
        
        cv::Mat blur;
        cv::GaussianBlur(mask, blur, cv::Size(5, 5), 0);
        
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        findContours(blur, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        int x_m;
        int y_m;
        int quadrant;

        for (auto c : contours) {
            auto area_cnt = cv::contourArea(c);
            cv::Moments M = cv::moments(c);
            if (M.m00 != 0.0) { 
                cv::Point p(M.m10/M.m00, M.m01/M.m00);
                x_m = int(p.x);
                y_m = int(p.y);
            }

            if (area_cnt > 500 && area_cnt < 3000) {
                if ((137<=x_m && x_m<=195) &&  (130<=y_m && y_m<=192)){
                    quadrant =  9*i + 45;
                }
                else if ((0<=x_m && x_m<=70) &&  (0<=y_m && y_m<=65)){
                    quadrant =  9*i + 37;
                }
                else if ((71<=x_m && x_m<=135) &&  (0<=y_m && y_m<=65)){
                    quadrant =  9*i + 38;
                }
                else if ((137<=x_m && x_m<=195) &&  (0<=y_m && y_m<=65)){
                    quadrant =  9*i + 39;
                }
                else if ((0<=x_m && x_m<=70) &&  (69<=y_m && y_m<=126)){
                    quadrant =  9*i + 40;
                }
                else if ((71<=x_m && x_m<=135) &&  (69<=y_m && y_m<=126)){
                    quadrant =  9*i + 41;
                }
                else if ((137<=x_m && x_m<=195) &&  (69<=y_m && y_m<=126)){
                    quadrant =  9*i + 42;
                }
                else if ((0<=x_m && x_m<=70) &&  (130<=y_m && y_m<=192)){
                    quadrant =  9*i + 43;
                }
                else if ((71<=x_m && x_m<=135) &&  (130<=y_m && y_m<=192)){
                    quadrant =  9*i + 44;
                }
                
                part_info = detect_color(img, new_image, c, x_m, y_m);
                part_info.push_back(quadrant);
                
                left_bin_info.push_back(part_info);

            }
        }
        i++;
    }
    return left_bin_info;
}

// int main(){
//     cv::Mat image = cv::imread("ariac_img.png", cv::IMREAD_COLOR);
//     std::vector<std::vector<int>> info; 
//     // rightbin(image.clone());
//     info = rightbin(image.clone());

// }