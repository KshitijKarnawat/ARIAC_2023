/**
 * @copyright Copyright (c) 2023
 * @file map_poses.cpp
 * @author Sanchit Kedia (sanchit@terpmail.umd.edu)
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Tanmay Haldankar (tanmayh@terpmail.umd.edu)
 * @author Sahruday Patti (sahruday@umd.edu)
 * @author Kshitij Karnawat (kshitij@umd.edu)
 * @brief Implementation of Bin and Tray Pose Mapping for ARIAC 2023 (Group 3)
 * @version 0.2
 * @date 2023-03-04
 * 
 * 
 */
#include "map_poses.hpp"

std::map<int, geometry_msgs::msg::Pose> define_poses(){

    std::map<int, geometry_msgs::msg::Pose> bin_quadrant_poses;

    geometry_msgs::msg::Pose q11;
    geometry_msgs::msg::Pose q12;
    geometry_msgs::msg::Pose q13;
    geometry_msgs::msg::Pose q14;
    geometry_msgs::msg::Pose q15;
    geometry_msgs::msg::Pose q16;
    geometry_msgs::msg::Pose q17;
    geometry_msgs::msg::Pose q18;
    geometry_msgs::msg::Pose q19;

    geometry_msgs::msg::Pose q21;
    geometry_msgs::msg::Pose q22;
    geometry_msgs::msg::Pose q23;
    geometry_msgs::msg::Pose q24;
    geometry_msgs::msg::Pose q25;
    geometry_msgs::msg::Pose q26;
    geometry_msgs::msg::Pose q27;
    geometry_msgs::msg::Pose q28;
    geometry_msgs::msg::Pose q29;

    geometry_msgs::msg::Pose q31;
    geometry_msgs::msg::Pose q32;
    geometry_msgs::msg::Pose q33;
    geometry_msgs::msg::Pose q34;
    geometry_msgs::msg::Pose q35;
    geometry_msgs::msg::Pose q36;
    geometry_msgs::msg::Pose q37;
    geometry_msgs::msg::Pose q38;
    geometry_msgs::msg::Pose q39;

    geometry_msgs::msg::Pose q41;
    geometry_msgs::msg::Pose q42;
    geometry_msgs::msg::Pose q43;
    geometry_msgs::msg::Pose q44;
    geometry_msgs::msg::Pose q45;
    geometry_msgs::msg::Pose q46;
    geometry_msgs::msg::Pose q47;
    geometry_msgs::msg::Pose q48;
    geometry_msgs::msg::Pose q49;

    geometry_msgs::msg::Pose q51;
    geometry_msgs::msg::Pose q52;
    geometry_msgs::msg::Pose q53;
    geometry_msgs::msg::Pose q54;
    geometry_msgs::msg::Pose q55;
    geometry_msgs::msg::Pose q56;
    geometry_msgs::msg::Pose q57;
    geometry_msgs::msg::Pose q58;
    geometry_msgs::msg::Pose q59;

    geometry_msgs::msg::Pose q61;
    geometry_msgs::msg::Pose q62;
    geometry_msgs::msg::Pose q63;
    geometry_msgs::msg::Pose q64;
    geometry_msgs::msg::Pose q65;
    geometry_msgs::msg::Pose q66;
    geometry_msgs::msg::Pose q67;
    geometry_msgs::msg::Pose q68;
    geometry_msgs::msg::Pose q69;

    geometry_msgs::msg::Pose q71;
    geometry_msgs::msg::Pose q72;
    geometry_msgs::msg::Pose q73;
    geometry_msgs::msg::Pose q74;
    geometry_msgs::msg::Pose q75;
    geometry_msgs::msg::Pose q76;
    geometry_msgs::msg::Pose q77;
    geometry_msgs::msg::Pose q78;
    geometry_msgs::msg::Pose q79;

    geometry_msgs::msg::Pose q81;
    geometry_msgs::msg::Pose q82;
    geometry_msgs::msg::Pose q83;
    geometry_msgs::msg::Pose q84;
    geometry_msgs::msg::Pose q85;
    geometry_msgs::msg::Pose q86;
    geometry_msgs::msg::Pose q87;
    geometry_msgs::msg::Pose q88;
    geometry_msgs::msg::Pose q89;

    q11.position.x = -2.08;
    q11.position.y = 3.195;
    q11.position.z = 0.723481;
    q11.orientation.x = 0.0;
    q11.orientation.y = 0.0;
    q11.orientation.z = 0.0;
    q11.orientation.w = 1.0;

    q12.position.x = -2.080003;
    q12.position.y = 3.374998;
    q12.position.z = 0.723481;
    q12.orientation.x = 0.0;
    q12.orientation.y = 0.0;
    q12.orientation.z = 0.0;
    q12.orientation.w = 1.0;

    q13.position.x = -2.079999;
    q13.position.y = 3.555001;
    q13.position.z = 0.723481;
    q13.orientation.x = 0.0;
    q13.orientation.y = 0.0;
    q13.orientation.z = 0.0;
    q13.orientation.w = 1.0;

    q14.position.x = -1.900003;
    q14.position.y = 3.194998;
    q14.position.z = 0.723481;
    q14.orientation.x = 0.0;
    q14.orientation.y = 0.0;
    q14.orientation.z = 0.0;
    q14.orientation.w = 1.0;

    q15.position.x = -1.9;
    q15.position.y = 3.375;
    q15.position.z = 0.723481;
    q15.orientation.x = 0.0;
    q15.orientation.y = 0.0;
    q15.orientation.z = 0.0;
    q15.orientation.w = 1.0;

    q16.position.x = -1.900004;
    q16.position.y = 3.554997;
    q16.position.z = 0.723481;
    q16.orientation.x = 0.0;
    q16.orientation.y = 0.0;
    q16.orientation.z = 0.0;
    q16.orientation.w = 1.0;

    q17.position.x = -1.72;
    q17.position.y = 3.195;
    q17.position.z = 0.723481;
    q17.orientation.x = 0.0;
    q17.orientation.y = 0.0;
    q17.orientation.z = 0.0;
    q17.orientation.w = 1.0;

    q18.position.x = -1.720004;
    q18.position.y = 3.374997;
    q18.position.z = 0.723481;
    q18.orientation.x = 0.0;
    q18.orientation.y = 0.0;
    q18.orientation.z = 0.0;
    q18.orientation.w = 1.0;

    q19.position.x = -1.720004;
    q19.position.y = 3.554997;
    q19.position.z = 0.723481;
    q19.orientation.x = 0.0;
    q19.orientation.y = 0.0;
    q19.orientation.z = 0.0;
    q19.orientation.w = 1.0;

    q21.position.x = -2.080005;
    q21.position.y = 2.444997;
    q21.position.z = 0.723481;
    q21.orientation.x = 0.0;
    q21.orientation.y = 0.0;
    q21.orientation.z = 0.0;
    q21.orientation.w = 1.0;

    q22.position.x = -2.08;
    q22.position.y = 2.62499;
    q22.position.z = 0.723481;
    q22.orientation.x = 0.0;
    q22.orientation.y = 0.0;
    q22.orientation.z = 0.0;
    q22.orientation.w = 1.0;

    q23.position.x = -2.079999;
    q23.position.y = 2.805001;
    q23.position.z = 0.723481;
    q23.orientation.x = 0.0;
    q23.orientation.y = 0.0;
    q23.orientation.z = 0.0;
    q23.orientation.w = 1.0;

    q24.position.x = -1.900005;
    q24.position.y = 2.444996;
    q24.position.z = 0.723481;
    q24.orientation.x = 0.0;
    q24.orientation.y = 0.0;
    q24.orientation.z = 0.0;
    q24.orientation.w = 1.0;

    q25.position.x = -1.9;
    q25.position.y = 2.625;
    q25.position.z = 0.723481;
    q25.orientation.x = 0.0;
    q25.orientation.y = 0.0;
    q25.orientation.z = 0.0;
    q25.orientation.w = 1.0;

    q26.position.x = -1.900006;
    q26.position.y = 2.804996;
    q26.position.z = 0.723481;
    q26.orientation.x = 0.0;
    q26.orientation.y = 0.0;
    q26.orientation.z = 0.0;
    q26.orientation.w = 1.0;

    q27.position.x = -1.72;
    q27.position.y = 2.444999;
    q27.position.z = 0.723481;
    q27.orientation.x = 0.0;
    q27.orientation.y = 0.0;
    q27.orientation.z = 0.0;
    q27.orientation.w = 1.0;

    q28.position.x = -1.720006;
    q28.position.y = 2.624996;
    q28.position.z = 0.723481;
    q28.orientation.x = 0.0;
    q28.orientation.y = 0.0;
    q28.orientation.z = 0.0;
    q28.orientation.w = 1.0;

    q29.position.x = -1.720006;
    q29.position.y = 2.804996;
    q29.position.z = 0.723481;
    q29.orientation.x = 0.0;
    q29.orientation.y = 0.0;
    q29.orientation.z = 0.0;
    q29.orientation.w = 1.0;

    q31.position.x = -2.83;
    q31.position.y = 2.445001;
    q31.position.z = 0.723481;
    q31.orientation.x = 0.0;
    q31.orientation.y = 0.0;
    q31.orientation.z = 0.0;
    q31.orientation.w = 1.0;

    q32.position.x = -2.83;
    q32.position.y = 2.625001;
    q32.position.z = 0.723481;
    q32.orientation.x = 0.0;
    q32.orientation.y = 0.0;
    q32.orientation.z = 0.0;
    q32.orientation.w = 1.0;

    q33.position.x = -2.829999;
    q33.position.y = 2.805001;
    q33.position.z = 0.723481;
    q33.orientation.x = 0.0;
    q33.orientation.y = 0.0;
    q33.orientation.z = 0.0;
    q33.orientation.w = 1.0;

    q34.position.x = -2.65;
    q34.position.y = 2.445;
    q34.position.z = 0.723481;
    q34.orientation.x = 0.0;
    q34.orientation.y = 0.0;
    q34.orientation.z = 0.0;
    q34.orientation.w = 1.0;

    q35.position.x = -2.65;
    q35.position.y = 2.625;
    q35.position.z = 0.723481;
    q35.orientation.x = 0.0;
    q35.orientation.y = 0.0;
    q35.orientation.z = 0.0;
    q35.orientation.w = 1.0;

    q36.position.x = -2.65;
    q36.position.y = 2.805;
    q36.position.z = 0.723481;;
    q36.orientation.x = 0.0;
    q36.orientation.y = 0.0;
    q36.orientation.z = 0.0;
    q36.orientation.w = 1.0;

    q37.position.x = -2.47;
    q37.position.y = 2.445;
    q37.position.z = 0.723481;;
    q37.orientation.x = 0.0;
    q37.orientation.y = 0.0;
    q37.orientation.z = 0.0;
    q37.orientation.w = 1.0;

    q38.position.x = -2.47;
    q38.position.y = 2.625;
    q38.position.z = 0.723481;;
    q38.orientation.x = 0.0;
    q38.orientation.y = 0.0;
    q38.orientation.z = 0.0;
    q38.orientation.w = 1.0;

    q39.position.x = -2.47;
    q39.position.y = 2.805;
    q39.position.z = 0.723481;;
    q39.orientation.x = 0.0;
    q39.orientation.y = 0.0;
    q39.orientation.z = 0.0;
    q39.orientation.w = 1.0;

    q41.position.x = -2.83;
    q41.position.y = 3.195001;
    q41.position.z = 0.723481;
    q41.orientation.x = 0.0;
    q41.orientation.y = 0.0;
    q41.orientation.z = 0.0;
    q41.orientation.w = 1.0;

    q42.position.x = -2.829998;
    q42.position.y = 3.375001;
    q42.position.z = 0.723481;
    q42.orientation.x = 0.0;
    q42.orientation.y = 0.0;
    q42.orientation.z = 0.0;
    q42.orientation.w = 1.0;

    q43.position.x = -2.83;
    q43.position.y = 3.555001;
    q43.position.z = 0.723481;
    q43.orientation.x = 0.0;
    q43.orientation.y = 0.0;
    q43.orientation.z = 0.0;
    q43.orientation.w = 1.0;

    q44.position.x = -2.649998;
    q44.position.y = 3.195001;
    q44.position.z = 0.723481;
    q44.orientation.x = 0.0;
    q44.orientation.y = 0.0;
    q44.orientation.z = 0.0;

    q45.position.x = -2.650001;
    q45.position.y = 3.375001;
    q45.position.z = 0.723481;
    q45.orientation.x = 0.0;
    q45.orientation.y = 0.0;
    q45.orientation.z = 0.0;
    q45.orientation.w = 1.0;

    q46.position.x = -2.650001;
    q46.position.y = 3.555001;
    q46.position.z = 0.723481;
    q46.orientation.x = 0.0;
    q46.orientation.y = 0.0;
    q46.orientation.z = 0.0;
    q46.orientation.w = 1.0;

    q47.position.x = -2.470001;
    q47.position.y = 3.195001;
    q47.position.z = 0.723481;
    q47.orientation.x = 0.0;
    q47.orientation.y = 0.0;
    q47.orientation.z = 0.0;
    q47.orientation.w = 1.0;

    q48.position.x = -2.470001;
    q48.position.y = 3.375001;
    q48.position.z = 0.723481;
    q48.orientation.x = 0.0;
    q48.orientation.y = 0.0;
    q48.orientation.z = 0.0;
    q48.orientation.w = 1.0;

    q49.position.x = -2.470001;
    q49.position.y = 3.555001;
    q49.position.z = 0.723481;
    q49.orientation.x = 0.0;
    q49.orientation.y = 0.0;
    q49.orientation.z = 0.0;
    q49.orientation.w = 1.0;

    q51.position.x = -2.08;
    q51.position.y = -3.554999;
    q51.position.z = 0.723481;
    q51.orientation.x = 0.0;
    q51.orientation.y = 0.0;
    q51.orientation.z = 0.0;
    q51.orientation.w = 1.0;

    q52.position.x = -2.079997;
    q52.position.y = -3.374998;
    q52.position.z = 0.723481;
    q52.orientation.x = 0.0;
    q52.orientation.y = 0.0;
    q52.orientation.z = 0.0;
    q52.orientation.w = 1.0;

    q53.position.x = -2.079997;
    q53.position.y = -3.195001;
    q53.position.z = 0.723481;
    q53.orientation.x = 0.0;
    q53.orientation.y = 0.0;
    q53.orientation.z = 0.0;
    q53.orientation.w = 1.0;

    q54.position.x = -1.899997;
    q54.position.y = -3.554999;
    q54.position.z = 0.723481;
    q54.orientation.x = 0.0;
    q54.orientation.y = 0.0;
    q54.orientation.z = 0.0;
    q54.orientation.w = 1.0;

    q55.position.x = -1.900001;
    q55.position.y = -3.375;
    q55.position.z = 0.723481;
    q55.orientation.x = 0.0;
    q55.orientation.y = 0.0;
    q55.orientation.z = 0.0;
    q55.orientation.w = 1.0;

    q56.position.x = -1.9;
    q56.position.y = -3.194999;
    q56.position.z = 0.723481;
    q56.orientation.x = 0.0;
    q56.orientation.y = 0.0;
    q56.orientation.z = 0.0;
    q56.orientation.w = 1.0;

    q57.position.x = -1.720001;
    q57.position.y = -3.555;
    q57.position.z = 0.723481;
    q57.orientation.x = 0.0;
    q57.orientation.y = 0.0;
    q57.orientation.z = 0.0;
    q57.orientation.w = 1.0;

    q58.position.x = -1.720001;
    q58.position.y = -3.194999;
    q58.position.z = 0.723481;
    q58.orientation.x = 0.0;
    q58.orientation.y = 0.0;
    q58.orientation.z = 0.0;
    q58.orientation.w = 1.0;

    q59.position.x = -1.720001;
    q59.position.y = -3.194999;
    q59.position.z = 0.723481;
    q59.orientation.x = 0.0;
    q59.orientation.y = 0.0;
    q59.orientation.z = 0.0;
    q59.orientation.w = 1.0;

    q61.position.x = -2.08;
    q61.position.y = -2.804999;
    q61.position.z = 0.723481;
    q61.orientation.x = 0.0;
    q61.orientation.y = 0.0;
    q61.orientation.z = 0.0;
    q61.orientation.w = 1.0;

    q62.position.x = -2.079997;
    q62.position.y = -2.624997;
    q62.position.z = 0.723481;
    q62.orientation.x = 0.0;
    q62.orientation.y = 0.0;
    q62.orientation.z = 0.0;
    q62.orientation.w = 1.0;

    q63.position.x = -2.079997;
    q63.position.y = -2.44500;
    q63.position.z = 0.723481;
    q63.orientation.x = 0.0;
    q63.orientation.y = 0.0;
    q63.orientation.z = 0.0;
    q63.orientation.w = 1.0;

    q64.position.x = -1.899997;
    q64.position.y = -2.804999;
    q64.position.z = 0.723481;
    q64.orientation.x = 0.0;
    q64.orientation.y = 0.0;
    q64.orientation.z = 0.0;
    q64.orientation.w = 1.0;

    q65.position.x = -1.900001;
    q65.position.y = -2.625;
    q65.position.z = 0.723481;
    q65.orientation.x = 0.0;
    q65.orientation.y = 0.0;
    q65.orientation.z = 0.0;
    q65.orientation.w = 1.0;

    q66.position.x = -1.899999;
    q66.position.y = -2.444999;
    q66.position.z = 0.723481;
    q66.orientation.x = 0.0;
    q66.orientation.y = 0.0;
    q66.orientation.z = 0.0;
    q66.orientation.w = 1.0;


    q67.position.x = -1.720001;
    q67.position.y = -2.805000;
    q67.position.z = 0.723481;
    q67.orientation.x = 0.0;
    q67.orientation.y = 0.0;
    q67.orientation.z = 0.0;
    q67.orientation.w = 1.0;

    q68.position.x = -1.719999;
    q68.position.y = -2.624999;
    q68.position.z = 0.723481;
    q68.orientation.x = 0.0;
    q68.orientation.y = 0.0;
    q68.orientation.z = 0.0;
    q68.orientation.w = 1.0;

    q69.position.x = -1.719999;
    q69.position.y = -2.444999;
    q69.position.z = 0.723481;
    q69.orientation.x = 0.0;
    q69.orientation.y = 0.0;
    q69.orientation.z = 0.0;
    q69.orientation.w = 1.0;

    q71.position.x = -2.829999;
    q71.position.y = -2.804999;
    q71.position.z = 0.723481;
    q71.orientation.x = 0.0;
    q71.orientation.y = 0.0;
    q71.orientation.z = 0.0;
    q71.orientation.w = 1.0;

    q72.position.x = -2.829995;
    q72.position.y = -2.624999;
    q72.position.z = 0.723481;
    q72.orientation.x = 0.0;
    q72.orientation.y = 0.0;
    q72.orientation.z = 0.0;
    q72.orientation.w = 1.0;

    q73.position.x = -2.829999;
    q73.position.y = -2.445001;
    q73.position.z = 0.723481;
    q73.orientation.x = 0.0;
    q73.orientation.y = 0.0;
    q73.orientation.z = 0.0;
    q73.orientation.w = 1.0;

    q74.position.x = -2.649999;
    q74.position.y = -2.804997;
    q74.position.z = 0.723481;
    q74.orientation.x = 0.0;
    q74.orientation.y = 0.0;
    q74.orientation.z = 0.0;
    q74.orientation.w = 1.0;

    q75.position.x = -2.65;
    q75.position.y = -2.625;
    q75.position.z = 0.723481;
    q75.orientation.x = 0.0;
    q75.orientation.y = 0.0;
    q75.orientation.z = 0.0;
    q75.orientation.w = 1.0;

    q76.position.x = -2.49999;
    q76.position.y = -2.444999;
    q76.position.z = 0.723481;
    q76.orientation.x = 0.0;
    q76.orientation.y = 0.0;
    q76.orientation.z = 0.0;
    q76.orientation.w = 1.0;

    q77.position.x = -2.47;
    q77.position.y = -2.804999;
    q77.position.z = 0.723481;
    q77.orientation.x = 0.0;
    q77.orientation.y = 0.0;
    q77.orientation.z = 0.0;
    q77.orientation.w = 1.0;

    q78.position.x = -2.49999;
    q78.position.y = -2.624999;
    q78.position.z = 0.723481;
    q78.orientation.x = 0.0;
    q78.orientation.y = 0.0;
    q78.orientation.z = 0.0;
    q78.orientation.w = 1.0;

    q79.position.x = -2.469999;
    q79.position.y = -2.444999;
    q79.position.z = 0.723481;
    q79.orientation.x = 0.0;
    q79.orientation.y = 0.0;
    q79.orientation.z = 0.0;
    q79.orientation.w = 1.0;

    q81.position.x = -2.829999;
    q81.position.y = -3.554999;
    q81.position.z = 0.723481;
    q81.orientation.x = 0.0;
    q81.orientation.y = 0.0;
    q81.orientation.z = 0.0;
    q81.orientation.w = 1.0;

    q82.position.x = -2.82994;
    q82.position.y = -3.374996;
    q82.position.z = 0.723481;
    q82.orientation.x = 0.0;
    q82.orientation.y = 0.0;
    q82.orientation.z = 0.0;
    q82.orientation.w = 1.0;

    q83.position.x = -2.829999;
    q83.position.y = -3.195001;
    q83.position.z = 0.723481;
    q83.orientation.x = 0.0;
    q83.orientation.y = 0.0;
    q83.orientation.z = 0.0;
    q83.orientation.w = 1.0;

    q84.position.x = -2.649999;
    q84.position.y = -3.554997;
    q84.position.z = 0.723481;
    q84.orientation.x = 0.0;
    q84.orientation.y = 0.0;
    q84.orientation.z = 0.0;
    q84.orientation.w = 1.0;

    q85.position.x = -2.65;
    q85.position.y = -3.375;
    q85.position.z = 0.723481;
    q85.orientation.x = 0.0;
    q85.orientation.y = 0.0;
    q85.orientation.z = 0.0;
    q85.orientation.w = 1.0;

    q86.position.x = -2.649999;
    q86.position.y = -3.194999;
    q86.position.z = 0.723481;
    q86.orientation.x = 0.0;
    q86.orientation.y = 0.0;
    q86.orientation.z = 0.0;
    q86.orientation.w = 1.0;

    q87.position.x = -2.47;
    q87.position.y = -3.554999;
    q87.position.z = 0.723481;
    q87.orientation.x = 0.0;
    q87.orientation.y = 0.0;
    q87.orientation.z = 0.0;
    q87.orientation.w = 1.0;

    q88.position.x = -2.469999;
    q88.position.y = -3.374999;
    q88.position.z = 0.723481;
    q88.orientation.x = 0.0;
    q88.orientation.y = 0.0;
    q88.orientation.z = 0.0;
    q88.orientation.w = 1.0;

    q89.position.x = -2.469999;
    q89.position.y = -3.194999;
    q89.position.z = 0.723481;
    q89.orientation.x = 0.0;
    q89.orientation.y = 0.0;
    q89.orientation.z = 0.0;
    q89.orientation.w = 1.0;

    bin_quadrant_poses[1] = q11;
    bin_quadrant_poses[2] = q12;
    bin_quadrant_poses[3] = q13;
    bin_quadrant_poses[4] = q14;
    bin_quadrant_poses[5] = q15;
    bin_quadrant_poses[6] = q16;
    bin_quadrant_poses[7] = q17;
    bin_quadrant_poses[8] = q18;
    bin_quadrant_poses[9] = q19;
    bin_quadrant_poses[10] = q21;
    bin_quadrant_poses[11] = q22;
    bin_quadrant_poses[12] = q23;
    bin_quadrant_poses[13] = q24;
    bin_quadrant_poses[14] = q25;
    bin_quadrant_poses[15] = q26;
    bin_quadrant_poses[16] = q27;
    bin_quadrant_poses[17] = q28;
    bin_quadrant_poses[18] = q29;
    bin_quadrant_poses[19] = q31;
    bin_quadrant_poses[20] = q32;
    bin_quadrant_poses[21] = q33;
    bin_quadrant_poses[22] = q34;
    bin_quadrant_poses[23] = q35;
    bin_quadrant_poses[24] = q36;
    bin_quadrant_poses[25] = q37;
    bin_quadrant_poses[26] = q38;
    bin_quadrant_poses[27] = q39;
    bin_quadrant_poses[28] = q41;
    bin_quadrant_poses[29] = q42;
    bin_quadrant_poses[30] = q43;
    bin_quadrant_poses[31] = q44;
    bin_quadrant_poses[32] = q45;
    bin_quadrant_poses[33] = q46;
    bin_quadrant_poses[34] = q47;
    bin_quadrant_poses[35] = q48;
    bin_quadrant_poses[36] = q49;
    bin_quadrant_poses[37] = q51;
    bin_quadrant_poses[38] = q52;
    bin_quadrant_poses[39] = q53;
    bin_quadrant_poses[40] = q54;
    bin_quadrant_poses[41] = q55;
    bin_quadrant_poses[42] = q56;
    bin_quadrant_poses[43] = q57;
    bin_quadrant_poses[44] = q58;
    bin_quadrant_poses[45] = q59;
    bin_quadrant_poses[46] = q61;
    bin_quadrant_poses[47] = q62;
    bin_quadrant_poses[48] = q63;
    bin_quadrant_poses[49] = q64;
    bin_quadrant_poses[50] = q65;
    bin_quadrant_poses[51] = q66;
    bin_quadrant_poses[52] = q67;
    bin_quadrant_poses[53] = q68;
    bin_quadrant_poses[54] = q69;
    bin_quadrant_poses[55] = q71;
    bin_quadrant_poses[56] = q72;
    bin_quadrant_poses[57] = q73;
    bin_quadrant_poses[58] = q74;
    bin_quadrant_poses[59] = q75;
    bin_quadrant_poses[60] = q76;
    bin_quadrant_poses[61] = q77;
    bin_quadrant_poses[62] = q78;
    bin_quadrant_poses[63] = q79;
    bin_quadrant_poses[64] = q81;
    bin_quadrant_poses[65] = q82;
    bin_quadrant_poses[66] = q83;
    bin_quadrant_poses[67] = q84;
    bin_quadrant_poses[68] = q85;
    bin_quadrant_poses[69] = q86;
    bin_quadrant_poses[70] = q87;
    bin_quadrant_poses[71] = q88;
    bin_quadrant_poses[72] = q89;

    return bin_quadrant_poses;
}

std::map<int, geometry_msgs::msg::Pose> define_tray_poses(){

    std::map<int, geometry_msgs::msg::Pose> tray_poses;

    geometry_msgs::msg::Pose v1;
    geometry_msgs::msg::Pose v2;
    geometry_msgs::msg::Pose v3;
    geometry_msgs::msg::Pose v4;
    geometry_msgs::msg::Pose v5;
    geometry_msgs::msg::Pose v6;

    v1.position.x = -0.870000;
    v1.position.y = -5.840000;
    v1.position.z = 0.734990;
    v1.orientation.x = 0.0;
    v1.orientation.y = 0.0;
    v1.orientation.z = 0.0;
    v1.orientation.w = 1.0;

    v2.position.x = -1.300000;
    v2.position.y = -5.840000;
    v2.position.z = 0.734990;
    v2.orientation.x = 0.0;
    v2.orientation.y = 0.0;
    v2.orientation.z = 0.0;
    v2.orientation.w = 1.0;

    v3.position.x = -1.730000;
    v3.position.y = -5.840000;
    v3.position.z = 0.734990;
    v3.orientation.x = 0.0;
    v3.orientation.y = 0.0;
    v3.orientation.z = 0.0;
    v3.orientation.w = 1.0;

    v4.position.x = -1.730000;
    v4.position.y = 5.840000;
    v4.position.z = 0.734990;
    v4.orientation.x = 0.0;
    v4.orientation.y = 0.0;
    v4.orientation.z = 0.0;
    v4.orientation.w = 1.0;

    v5.position.x = -1.300000;
    v5.position.y = 5.840000;
    v5.position.z = 0.734990;
    v5.orientation.x = 0.0;
    v5.orientation.y = 0.0;
    v5.orientation.z = 0.0;

    v6.position.x = -0.870000;
    v6.position.y = 5.840000;
    v6.position.z = 0.734990;
    v6.orientation.x = 0.0;
    v6.orientation.y = 0.0;
    v6.orientation.z = 0.0;
    v6.orientation.w = 1.0;

    tray_poses[0] = v1; 
    tray_poses[1] = v2; 
    tray_poses[2] = v3; 
    tray_poses[3] = v4; 
    tray_poses[4] = v5; 
    tray_poses[5] = v6; 

    return tray_poses;
}
