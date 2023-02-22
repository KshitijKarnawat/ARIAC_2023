#pragma once
#include<iostream>
#include<rclcpp/rclcpp.hpp>
#include<string>
#include<vector>
#include<memory>

namespace group3{

    class Order{
        std::string id;
        int tray_id;
        std::string type;
    };

    class Kitting: private Order{
        int agv_number;
        int tray_id;
        int destination;
        struct part{
            std::string type;
            std::string color;
            int quadrant;
        };
        std::vector<part> parts;
        
    };

    class Assembly: private Order{
        int agv_number;
        int tray_id;
        int destination;
        struct part{
            std::string type;
            std::string color;
            int quadrant;
        };
        std::vector<part> parts;
        
    };

    class Combined: private Order{
        int agv_number;
        int tray_id;
        int destination;
        struct part{
            std::string type;
            std::string color;
            int quadrant;
        };
        std::vector<part> parts;

    };
}