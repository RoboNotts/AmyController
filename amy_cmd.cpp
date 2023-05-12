#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <time.h>
#include <std_msgs/Int32.h>
#include <tinyxml.h>
#include <std_srvs/Empty.h>
// #include <exception>
#include <thread>
#include "amy_comm/amy_cmd.h"
#include "amy_comm/sensordata.h"

using namespace amy_comm;

bool Amy_Cmd::emergency_stop_status = true;//emergency stop button is on
int Amy_Cmd::dock_result = 0;
int Amy_Cmd::RobotState = 0;
std::string Amy_Cmd::bat_str_quantity_charging = std::string();
bool Amy_Cmd::overload_flag = false;

char *Amy_Cmd::amy_cmd(char *cmds )
{
    ROS_INFO("%s",cmds);
    std::string c = cmds;

    if (strcmp(cmds, "error") == 0)
    {
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("#SL") <= c.length())
    {
        const char *vel_str = c.c_str();
        pub_velocity(vel_str, 2);
        strcpy(cmds, "done\n");
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("#DANC") <= c.length())
    {
        std::lock_guard<std::mutex> DANC_guard(mutex_dance_follow);
        if(dance_flag == false)
        {
            dance_flag = true;
            thread_dance = std::thread(system, "rosrun amy_comm dance");
        }
        strcpy(cmds, "done\n");
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if (c.find("#FOLL") <= c.length())
    {
        std::lock_guard<std::mutex> FOLL_guard(mutex_dance_follow);
        if (follow_flag == false)
        {
            follow_flag = true;
            thread_follow = std::thread(system, "roslaunch amy_follower amy_follower.launch");
        }
        strcpy(cmds, "done\n");
        ROS_INFO("return: %s",cmds);
        return cmds;

    }
    else if(c.find("#VL") <= c.length())
    {
        const char *vel_str = c.c_str();
        pub_velocity(vel_str, 1);
        // usleep(100);
        strcpy(cmds, "done\n");
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("#CANCEL") <= c.length())
    {   
        // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base");
        // ROS_INFO("#CANCEL: Waiting for action server to start.");
        bool waitForServer_finished_before_timeout = ac.waitForServer(ros::Duration(5, 0));
        ROS_INFO("#CANCEL: Action server started.");
        if(nav_init_flag == true && naving_flag == true)
        {
            // ac.cancelAllGoals();
            ac.cancelGoal();
            ROS_INFO("#CANCEL: cancel goal....");
        }else
        {
            // ac.cancelAllGoals();
            ac.cancelGoal();
            ROS_INFO("#CANCEL: robot is not on the way of goal now,you should not cancel goal");
        }

        //wait for the action to return
        bool waitForResult_finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
        //TO DO: estimate timeout value
        if (waitForResult_finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("#CANCEL: Action finished: %s",state.toString().c_str());
        }
        else
            ROS_WARN("#CANCEL: Action did not finish before the time out.");

        naving_flag = false;
        strcpy(cmds, "#NAV06\n");
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("#MAPO") <= c.length())
    {
        std::lock_guard<std::recursive_mutex> MAPO_guard(mutex_map);
        //TO DO: not neccessary to read from server every time
        bool lidar = false;
        std::string robot_type;
        ros::param::get("~lidar",lidar);
        ros::param::get("~robot_type",robot_type);
        ROS_INFO_STREAM("lidar: " << std::boolalpha << lidar << " | robot_type: " << robot_type);

        if(nav_init_flag == true) navigation_stop();

        if(map_init_flag == false)
        {
            if(lidar == false && (robot_type == "a1" || robot_type == "a2"))
            {
                thread_mapo_fused = std::thread(system, "roslaunch amy_navigation vo_fuse.launch");
                ROS_INFO("#MAPO: A1_PD or A2_PD");
            }else if(lidar == true && (robot_type == "a1" || robot_type == "a2"))
            {
                thread_mapo_fused = std::thread(system, "roslaunch amy_navigation fused_odom_a.launch");
                ROS_INFO("#MAPO: A1_PL or A2_PL");
            }else
            {
                thread_mapo_fused = std::thread(system, "roslaunch amy_navigation fused_odom_m.launch");
                ROS_INFO("#MAPO: ELSE");
            }
            
            sleep(2);

            if(lidar == false && (robot_type == "a1" || robot_type == "a2"))
            {
                thread_mapo_mapping = std::thread(system, "roslaunch amy_navigation 1_mapping_a_d.launch");
                ROS_INFO("#MAPO: A1_PD or A2_PD");
            }else if(lidar == true && (robot_type == "a1" || robot_type == "a2"))
            {
                thread_mapo_mapping = std::thread(system, "roslaunch amy_navigation 2_mapping_a_l.launch");
                ROS_INFO("#MAPO: A1_PL or A2_PL");
            }else if(lidar == true && (robot_type == "m1" || robot_type == "m1arkl"))
            {
                thread_mapo_mapping = std::thread(system, "roslaunch amy_navigation 2_mapping_a_l.launch");
                ROS_INFO("#MAPO: M1 or M1ARKL");
            }else if(lidar == true && robot_type == "m1arks")
            {
                thread_mapo_mapping = std::thread(system, "roslaunch amy_navigation 2_mapping_a_l.launch");
                ROS_INFO("#MAPO: M1ARKS");
            }else
            {
                ROS_ERROR("#MAPO: robot_type ERROR!");
            }

            //check whether the mapping has started.
            bool waitFormapping_finished_before_timeout = false;
            int mapo_cnt = 0;
            while(!waitFormapping_finished_before_timeout && mapo_cnt <= 30)
            {
                ++mapo_cnt;
                waitFormapping_finished_before_timeout = listener.waitForTransform("/map", "/base_link", ros::Time::now(), ros::Duration(1.0));
                ROS_INFO("#MAPO: mapo_cnt: %d", mapo_cnt);
            }
            ROS_INFO("#MAPO: mapo_cnt: %d", mapo_cnt);
            map_init_flag = true;

            if(!waitFormapping_finished_before_timeout)
            {
                ROS_ERROR("#MAPO: wait for mapping module time out!");
                map_stop();
                strcpy(cmds, "#MAPF\n");
                ROS_INFO("return: %s",cmds);
                return cmds;
            }

            system("rm /home/amy/amyos/mymap.yaml");
            system("rm /home/amy/amyos/mymap.pgm");
            system("rm /home/amy/amyos/location.txt");

            write_pose();

            char b[10],c[10],d[10];
            std::string a = "#MAPS,";
            sprintf(b, "%f", location_origin_x);
            sprintf(c, "%f", location_origin_y);
            sprintf(d, "%f", location_rotation_yaw);
            a += b;
            a +=',';
            a += c;
            a +=',';
            a +=d;
            a += '\n';
            const char *cmd_p = a.c_str();
            strcpy(cmds, cmd_p);
        }
        else
        {
            ROS_WARN("#MAPO: you have opened the mapping.");
            strcpy(cmds, "#MAPS\n");
        }
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("#QMAP") <= c.length())
    {
        strcpy(cmds, "NOM\n");
        // sleep(1);   //wait for #STOP
        if(map_init_flag)
        {
            map_stop();
        }else
        {
            ROS_WARN("#QMAP: you should open the mapping first.");
        }
        // map_init_flag = false;
        // sleep(1);   //wait for closing gmapping
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("#SMAP") <= c.length())
    {
        if(map_init_flag)
        {
            system("rosrun map_server map_saver -f /home/amy/amyos/mymap");
            // map_init_flag = false;
            map_stop();
        }else
        {
            ROS_WARN("#SMAP: you should open the mapping first.");
        }
        strcpy(cmds, "NOM\n");

        if((!access("/home/amy/amyos/mymap.pgm", 0)) && (!access("/home/amy/amyos/mymap.yaml", 0)))
        {
            strcpy(cmds, "FINDM\n");
            // break;
        }
        
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("#GBAT") <= c.length())
    {
        strcpy(cmds, bat_str_quantity_charging.c_str());
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("#GEBS") <= c.length())
    {
        char b[1];
        std::string str_tmp;
        if(emergency_stop_status)
        {
            str_tmp = "#ESON\n";
        }else
        {
            str_tmp = "#ESOFF\n";
        }
        
        const char *cmd_p = str_tmp.c_str();
        strcpy(cmds, cmd_p);
        // ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("#NAVS") <= c.length())
    {
        std::lock_guard<std::recursive_mutex> NAVS_guard(mutex_navigation);
        //TO DO: not neccessary to read from server every time
        bool lidar = false;
        std::string robot_type;
        ros::param::get("~lidar",lidar);
        ros::param::get("~robot_type",robot_type);
        ROS_INFO_STREAM("lidar: " << std::boolalpha << lidar << " | robot_type: " << robot_type);

        if(map_init_flag == true) map_stop();

        if(nav_init_flag == false)
        {
            amy_comm::CmdLed led_cmd_data;
            led_cmd_data.Cmd = 5;
            led_cmd_data.Value = 2;
            led_cmd_pub.publish(led_cmd_data);
            
            if(lidar == false && (robot_type == "a1" || robot_type == "a2"))
            {
                thread_navs = std::thread(system, "roslaunch amy_navigation 1_navigation_a_d.launch");
                ROS_INFO("#NAVS: navigation:A1_PD or A2_PD");
            }else if(lidar == true && (robot_type == "a1" || robot_type == "a2"))
            {
                thread_navs = std::thread(system, "roslaunch amy_navigation 2_navigation_a_l.launch");
                ROS_INFO("#NAVS: navigation:A1_PL or A2_PL");
            }else if(lidar == true && (robot_type == "m1" || robot_type == "m1arkl"))
            {
                thread_navs = std::thread(system, "roslaunch amy_navigation 3_navigation_m.launch");
                ROS_INFO("#NAVS: navigation:M1 or M1ARKL");
            }else if(lidar == true && robot_type == "m1arks")
            {
                thread_navs = std::thread(system, "roslaunch amy_navigation 4_navigation_ms.launch");
                ROS_INFO("#NAVS: navigation: M1ARKS");
            }else
            {
                ROS_ERROR("#NAVS: robot_type ERROR!");
            }

            // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base");
            // ROS_INFO("#NAVS: Waiting for action server to start.");

            //wait for localization module start
            //check whether the localization module has started.
            bool waitForLocalization_finished_before_timeout = false;
            int localization_cnt = 0;
            while(!waitForLocalization_finished_before_timeout && localization_cnt <= 60)
            {
                ++localization_cnt;
                waitForLocalization_finished_before_timeout = listener.waitForTransform("/map", "/base_link", ros::Time::now(), ros::Duration(1.0));
                ROS_INFO("#NAVS: localization_cnt: %d", localization_cnt);
            }
            ROS_INFO("#NAVS: localization_cnt: %d", localization_cnt);

            nav_init_flag = true;
            naving_flag = false;

            if(!waitForLocalization_finished_before_timeout)
            {
                ROS_ERROR("#NAVS: wait for localization module time out!");
                navigation_stop();
                strcpy(cmds, "#NAV07\n");
                ROS_INFO("return: %s",cmds);
                return cmds;
            }

            //check whether the move_base has started
            bool waitForServer_finished_before_timeout = ac.waitForServer(ros::Duration(20, 0));
            if (!waitForServer_finished_before_timeout)
            {
                ROS_ERROR("#NAVS: waitForServer_finished_before_timeout");
            }

            std::ifstream file("/home/amy/amyos/location.txt");
            std::string text_line, text_word;
            std::vector<std::string> vs;
            std::vector<double> vd;
            std::getline(file, text_line);
            std::cout << text_line << std::endl; 
            std::istringstream line(text_line);
            double d_tmp;
            // while(line >> d_tmp) vd.push_back(d_tmp);
            while(std::getline(line, text_word, ',')) vs.push_back(text_word);
            for(auto str : vs)
            {
                vd.push_back(stod(str));
            }
            for(auto d : vd)
                std::cout << d << " ";
            std::cout << std::endl;


            geometry_msgs::PoseWithCovarianceStamped initial_pose;
            // initial_pose.header.seq = 0;
            initial_pose.header.stamp = ros::Time::now();
            initial_pose.header.frame_id = "map";
            initial_pose.pose.pose.position.x = vd.at(0);
            initial_pose.pose.pose.position.y = vd.at(1);
            initial_pose.pose.pose.position.z = vd.at(2);
            initial_pose.pose.pose.orientation.x = vd.at(3);   //attention: x2 + y2 + z2 + w2 = 1
            initial_pose.pose.pose.orientation.y = vd.at(4);
            initial_pose.pose.pose.orientation.z = vd.at(5);
            initial_pose.pose.pose.orientation.w = vd.at(6);
            initial_pose.pose.covariance[0] = 0.25;
            initial_pose.pose.covariance[7] = 0.25;
            initial_pose.pose.covariance[35] = 0.06853891945200942;

            if(!isQuaternionValid(initial_pose.pose.pose.orientation))
            {
                ROS_ERROR("#NAVS: initial_pose.pose.pose.orientation invalid!!!");
                navigation_stop();
                strcpy(cmds, "#NAV07\n");
                ROS_INFO("return: %s",cmds);
                return cmds;
            }
            ROS_INFO("Action server started, init pose.");
            pose_init_pub.publish(initial_pose);

            sleep(1);
            std_srvs::Empty srv;
            if (client.call(srv))
            {
                ROS_INFO("successed to call service clear_costmap");
            }
            else
            {
                ROS_ERROR("Failed to call service clear_costmap");
            }

            strcpy(cmds, "#NAV02\n");
        }else
        {
            ROS_WARN("#NAVS: you have opened the navigation.");
            strcpy(cmds, "#NAV02\n");
        }
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("#PT!") <= c.length())
    {
        if(nav_init_flag == true)
        {
            if(naving_flag == true)
            {
                ROS_INFO("#PT: cancel last goal first.");
                ac.cancelGoal();
                sleep(1);   //must sleep some time, other thread will get result
            }

            if(docking_flag == true)
            {
                std_msgs::Int32 dock_cmd_data;
                dock_cmd_data.data = 0;  //cancel dock
                CmdAutoDock_pub.publish(dock_cmd_data);

                sleep(4);

                int l_t = 0;
                while(l_t < 600)//60s
                {
                    //ROS_INFO("while--%d\n",dock_result);
                    usleep(100000);
                    
                    if(dock_result == 4 || dock_result == 5)
                    {

                        ROS_INFO("#PT!: cancel dock success!!!");
                        break;
                    }else
                        ++l_t;
                }
                if(l_t >= 600)
                {
                    ROS_ERROR("#PT!: cancel dock time out!!!");
                }
                docking_flag = false;
            }

            const char *points = c.c_str();	
            double cmd_locationx_x,cmd_locationx_y,cmd_locationx_yaw;

            // std::cout<<points<<std::endl;
            sscanf(points, "%*[^0-9-]%lf,%lf,%lf",&cmd_locationx_x, &cmd_locationx_y,&cmd_locationx_yaw);
            // std::cout<<cmd_locationx_x<<","<<cmd_locationx_y<<","<<cmd_locationx_yaw<<std::endl;

            double cosRoll,sinRoll,cosPitch,sinPitch,cosyaw,sinyaw,qx,qy,qz,qw;
            cosRoll = cosf(0 * 0.5f); //X
            sinRoll = sinf(0 * 0.5f);//X

            cosPitch = cosf(0 * 0.5f);//Y
            sinPitch = sinf(0 * 0.5f);//Y

            cosyaw = cosf(cmd_locationx_yaw * 0.5f);//Z
            sinyaw = sinf(cmd_locationx_yaw * 0.5f);//Z

            qx = sinRoll * cosPitch * cosyaw - cosRoll * sinPitch * sinyaw;
            qy = cosRoll * sinPitch * cosyaw + sinRoll * cosPitch * sinyaw;
            qz = cosRoll * cosPitch * sinyaw - sinRoll * sinPitch * cosyaw;
            qw = cosRoll * cosPitch * cosyaw + sinRoll * sinPitch * sinyaw;

            amy_comm::CmdLed led_cmd_data;
            led_cmd_data.Cmd = 5;
            led_cmd_data.Value = 2;
            led_cmd_pub.publish(led_cmd_data);
            // quaternion=tf.transformations.quaternion_from_euler(0, 0, self.nav_cmd_data.yaw);

            auto stamp = ros::Time::now();
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.seq = 0;
            goal.target_pose.header.stamp = stamp;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.pose.position.x = cmd_locationx_x;
            goal.target_pose.pose.position.y = cmd_locationx_y;
            goal.target_pose.pose.position.z = 0;
            goal.target_pose.pose.orientation.x = qx;
            goal.target_pose.pose.orientation.y = qy;
            goal.target_pose.pose.orientation.z = qz;
            goal.target_pose.pose.orientation.w = qw;

            // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base");
            // ROS_INFO("#PT: Waiting for action server to start.");
            bool waitForServer_finished_before_timeout = ac.waitForServer(ros::Duration(5, 0));
            ROS_INFO("#PT: Action server started, send goal.");
            ac.sendGoal(goal);
            naving_flag = true;

            bool waitForResult_finished_before_timeout = ac.waitForResult(ros::Duration(3600.0)); //3600s
            //TO DO: estimate timeout
            if (waitForResult_finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("#PT: Action finished: %s", state.toString().c_str());   

                if(state == actionlib::SimpleClientGoalState::SUCCEEDED) strcpy(cmds, "#NAV01\n");  //overload operator==
                else if(state == actionlib::SimpleClientGoalState::LOST) strcpy(cmds, "#NAV03\n");
                else if(state == actionlib::SimpleClientGoalState::ABORTED) strcpy(cmds, "#NAV04\n");
                else if(state == actionlib::SimpleClientGoalState::PREEMPTED) strcpy(cmds, "#NAV06\n");
                else if(state == actionlib::SimpleClientGoalState::REJECTED) strcpy(cmds, "#NAV08\n");
                else if(state == actionlib::SimpleClientGoalState::RECALLED) strcpy(cmds, "#NAV09\n");
                else ROS_ERROR_NAMED("amy_socket_node", "BUG: Unhandled SimpleGoalState: %s", state.toString().c_str());
            }
            else
            {
                ROS_WARN("#PT: Action did not finish before the time out.");
                strcpy(cmds, "#NAV05\n");
            }
        }
        else
        {
            ROS_WARN("#PT: you should open the navigation first");
            strcpy(cmds, "#NAV07\n");
        }
        naving_flag = false;
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if (c.find("#STFO") <= c.length())
    {
        std::lock_guard<std::mutex> STFO_guard(mutex_dance_follow);

        if(dance_flag)
        {
            system("rosnode kill /dance");
            if(thread_dance.joinable())
            {
                ROS_INFO("#STFO: thread_dance.joinable(): true");
                thread_dance.join();
                ROS_INFO("#STFO: thread_dance.join()");
            }else
            {
                
                ROS_INFO("#STFO: thread_dance.joinable(): false");
            }
            dance_flag = false;
        }
        if (follow_flag)
        {
            system("rosnode kill /amy_follower");
            if(thread_follow.joinable())
            {
                ROS_INFO("#STFO: thread_follow.joinable(): true");
                thread_follow.join();
                ROS_INFO("#STFO: thread_follow.join()");
            }else
            {
                
                ROS_INFO("#STFO: thread_follow.joinable(): false");
            }
            follow_flag = false;
        }
        
        test_vel_pub.publish(geometry_msgs::Twist());
        usleep(100000); //100ms
        test_vel_pub.publish(geometry_msgs::Twist());
        usleep(100000);
        test_vel_pub.publish(geometry_msgs::Twist());
        usleep(100000);
        test_vel_pub.publish(geometry_msgs::Twist());
        strcpy(cmds, "done\n");
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if (c.find("#TURN") <= c.length())
    {
        const char *vel_str = c.c_str();
        if (vel_str[5] == 'L' )
            turn(1);
        else if(vel_str[5] == 'R')
            turn(2);
        else if(vel_str[5] == 'B')
            turn(3);
        else
            turn(3);
        strcpy(cmds, "done\n");
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if (c.find("#STOP") <= c.length())
    {
        if(nav_init_flag == true)
        {
            try
            {
                listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            }catch (tf::TransformException &ex)
            {
                navigation_stop();
                ROS_ERROR("%s", ex.what());
                char pose_err[512];
                strcpy(pose_err, "#STOPNP\n");
                return pose_err;
            }
            
            location_origin_x=transform.getOrigin().x();
            location_origin_y=transform.getOrigin().y();
            location_origin_z=transform.getOrigin().z();
            location_rotation_x=transform.getRotation().x();
            location_rotation_y=transform.getRotation().y();
            location_rotation_z=transform.getRotation().z();
            location_rotation_w=transform.getRotation().w();
            
            tf::Quaternion q(location_rotation_x, location_rotation_y, location_rotation_z, location_rotation_w);
            tf::Matrix3x3 m(q);
            m.getRPY(location_rotation_roll, location_rotation_pitch, location_rotation_yaw);

            navigation_stop();

            char b[10],c[10],d[10];
            std::string a = "#NAV07,";
            sprintf(b, "%f", location_origin_x);
            sprintf(c, "%f", location_origin_y);
            sprintf(d, "%f", location_rotation_yaw);
            a += b;
            a +=',';
            a += c;
            a +=',';
            a +=d;
            a += '\n';
            const char *cmd_p = a.c_str();
            strcpy(cmds, cmd_p);
        }else
        {
            ROS_WARN("you have closed the navigation");
            strcpy(cmds, "#NAV07\n");
        }
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("#INPO!") <= c.length())
    {
        if(nav_init_flag == true)
        {
            const char *points = c.c_str();	
            double cmd_locationx_x,cmd_locationx_y,cmd_locationx_yaw;

            std::cout<<points<<std::endl;
            sscanf(points, "%*[^0-9-]%lf,%lf,%lf",&cmd_locationx_x, &cmd_locationx_y,&cmd_locationx_yaw);
            std::cout << "#INPO:" << cmd_locationx_x << "," << cmd_locationx_y << "," << cmd_locationx_yaw << std::endl;

            double cosRoll,sinRoll,cosPitch,sinPitch,cosyaw,sinyaw,qx,qy,qz,qw;
            cosRoll = cosf(0 * 0.5f); //X
            sinRoll = sinf(0 * 0.5f);//X

            cosPitch = cosf(0 * 0.5f);//Y
            sinPitch = sinf(0 * 0.5f);//Y

            cosyaw = cosf(cmd_locationx_yaw * 0.5f);//Z
            sinyaw = sinf(cmd_locationx_yaw * 0.5f);//Z

            qx = sinRoll * cosPitch * cosyaw - cosRoll * sinPitch * sinyaw;
            qy = cosRoll * sinPitch * cosyaw + sinRoll * cosPitch * sinyaw;
            qz = cosRoll * cosPitch * sinyaw - sinRoll * sinPitch * cosyaw;
            qw = cosRoll * cosPitch * cosyaw + sinRoll * sinPitch * sinyaw;

            std::cout << "qx = " << qx << std::endl;
            std::cout << "qy = " << qy << std::endl;
            std::cout << "qz = " << qz << std::endl;
            std::cout << "qw = " << qw << std::endl;

            geometry_msgs::PoseWithCovarianceStamped initial_pose;
            initial_pose.header.seq = 0;
            initial_pose.header.stamp = ros::Time::now();
            initial_pose.header.frame_id = "map";
            initial_pose.pose.pose.position.x = cmd_locationx_x;
            initial_pose.pose.pose.position.y = cmd_locationx_y;
            initial_pose.pose.pose.position.z = 0;
            initial_pose.pose.pose.orientation.x = qx;   //attention: x2 + y2 + z2 + w2 = 1
            initial_pose.pose.pose.orientation.y = qy;
            initial_pose.pose.pose.orientation.z = qz;
            initial_pose.pose.pose.orientation.w = qw;
            initial_pose.pose.covariance[0] = 0.25;
            initial_pose.pose.covariance[7] = 0.25;
            initial_pose.pose.covariance[35] = 0.06853891945200942;

            if(!isQuaternionValid(initial_pose.pose.pose.orientation))
            {
                ROS_ERROR("#INPO: initial_pose.pose.pose.orientation invalid!!!");
                strcpy(cmds, "INPOF\n");
                ROS_INFO("return: %s",cmds);
                return cmds;
            }
            pose_init_pub.publish(initial_pose);

            sleep(1);
            std_srvs::Empty srv;
            if (client.call(srv))
            {
                ROS_INFO("successed to call service clear_costmap");
                strcpy(cmds, "INPOS\n");
            }
            else
            {
                ROS_ERROR("Failed to call service clear_costmap");
                strcpy(cmds, "INPOF\n");
            }

            
        }else
        {
            ROS_WARN("#INPO!: the robot is not in navigation mode,you can not init pose.");
            strcpy(cmds, "INPOF\n");
        }
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if (c.find("SHUTDOWN") <= c.length())
    {
        std_msgs::Int32 shut_s;
        shut_s.data = 1;
        shut_pub.publish(shut_s);//topic:android_off
        strcpy(cmds, "done\n");
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if (c.find("#DOCK") <= c.length())//start docking
    {
        // std::lock_guard<std::mutex> DOCK_guard(mutex_dock);

        if(naving_flag == true)
        {
            ROS_INFO("#DOCK: cancel last goal first.");
            ac.cancelGoal();
        }

        if(docking_flag == false)
        {
            docking_flag = true;
            dock_result = 0;
            std_msgs::Int32 dock_cmd_data;
            dock_cmd_data.data = 1;  //start dock
            CmdAutoDock_pub.publish(dock_cmd_data);
            int l_t = 0;
            while(l_t < 4000)//4000s
            {
                //ROS_INFO("while--%d\n",dock_result);
                sleep(1);
                ++l_t;
                if(dock_result==1)
                {
                    strcpy(cmds, "#DRST01\n");//auto docking success
                    sleep(1);
                    dock_result=0;
                    ROS_INFO("return: %s",cmds);
                    return cmds;
                }else if(dock_result==2)
                {
                    docking_flag = false;
                    strcpy(cmds, "#DRST02\n");//auto docking fail
                    sleep(1);
                    dock_result=0;
                    ROS_INFO("return: %s",cmds);
                    return cmds;
                }else if(dock_result==3)//can not find IR
                {
                    docking_flag = false;
                    strcpy(cmds, "#DRST03\n");
                    sleep(1);
                    dock_result=0;
                    ROS_INFO("return: %s",cmds);
                    return cmds;
                }else if(dock_result==4)//cancel the dock
                {
                    docking_flag = false;
                    strcpy(cmds, "#DRST04\n");
                    sleep(1);
                    dock_result=0;
                    ROS_INFO("return: %s",cmds);
                    return cmds;
                }else if(dock_result==5)//e.g.move the dock or the dock power off
                {
                    docking_flag = false;
                    strcpy(cmds, "#DRST05\n");
                    sleep(1);
                    dock_result=0;
                    ROS_INFO("return: %s",cmds);
                    return cmds;
                }else if(dock_result==6)//dock time out
                {
                    docking_flag = false;
                    strcpy(cmds, "#DRST08\n");
                    sleep(1);
                    dock_result=0;
                    ROS_INFO("return: %s",cmds);
                    return cmds;
                }else if(dock_result==7)//impact error
                {
                    docking_flag = false;
                    strcpy(cmds, "#DRST06\n");
                    sleep(1);
                    dock_result=0;
                    ROS_INFO("return: %s",cmds);
                    return cmds;
                }
            }
            docking_flag = false;
            strcpy(cmds, "#DRST08\n");//dock time out
            ROS_INFO("return: %s",cmds);
            return cmds;
        }else
        {
            strcpy(cmds, "#DCKING\n");//dock time out
            ROS_INFO("return: %s",cmds);
            return cmds;
        }
        
    }
    else if (c.find("#CDCK") <= c.length())//cancel docking
    {
        std_msgs::Int32 dock_cmd_data;
        dock_cmd_data.data = 0;  //cancel dock
        CmdAutoDock_pub.publish(dock_cmd_data);

        sleep(4);

        int l_t = 0;
        while(l_t < 600)//60s
        {
            //ROS_INFO("while--%d\n",dock_result);
            usleep(100000);
            
            if(dock_result == 4 || dock_result == 5)
            {
                docking_flag = false;
                strcpy(cmds, "#CDS\n");//cancle success
                sleep(1);
                dock_result=0;
                ROS_INFO("return: %s",cmds);
                return cmds;
            }else
                ++l_t;

        }
        docking_flag = false;
        strcpy(cmds, "#CDF\n");
        dock_result=0;
        ROS_WARN("return: %s",cmds);
        return cmds;
    }else if(c.find("#GDFLAG") <= c.length())
    {
        if(docking_flag == true)
        {
            strcpy(cmds, "#DCKING\n");
        }else
        {
            strcpy(cmds, "#NDCK\n");
        }
        ROS_WARN("return: %s",cmds);
        return cmds;
    }
    else if(c.find("#GNFLAG") <= c.length())
    {
        std::string str_tmp = "#NAV0";
        if(nav_init_flag == false && naving_flag == false)
        {
            str_tmp += std::to_string(7) + '\n';
        }else if(nav_init_flag == true && naving_flag == false)
        {
            str_tmp += std::to_string(2) + '\n';
        }else if(nav_init_flag == true && naving_flag == true)
        {
            str_tmp += std::to_string(0) + '\n';
        }else
        {
            ROS_ERROR("#GNFLAG: get error state!!!");
        }
        
        strcpy(cmds, str_tmp.c_str());
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if (c.find("#MACC") <= c.length())
    {
        if ( (!access("/home/amy/amyos/mymap.pgm", 0) ) && (!access("/home/amy/amyos/mymap.yaml", 0)) )
        {
            if(!access("/home/amy/amyos/location.txt", 0))
            {
                strcpy(cmds, "FINDM\n");
                ROS_INFO("return: %s",cmds);
                return cmds;
            }
            else
            {
                strcpy(cmds, "NOLOC\n");
                ROS_INFO("return: %s",cmds);
                return cmds;
            }
        }
        else
        {
            strcpy(cmds, "NOM\n");
            ROS_INFO("return: %s",cmds);
            return cmds;
        }
    }
    else if(c.find("VERBO")<= c.length())
    {
        std::ifstream infile;
        std::string str;
        infile.open("/home/amy/amyos/Driver_FW_version.txt");
        std::getline(infile,str);
        const char* content=str.c_str();
        strcpy(cmds, content);
        strcat(cmds, "\n");
        std::cout<<cmds<<std::endl;
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("VERAC")<= c.length())
    {
        //query_all_version();
        std::ifstream infile;
        std::string str;
        infile.open("/home/amy/amyos/DATA_FW_version.txt");
        std::getline(infile,str);
        const char* content=str.c_str();
        strcpy(cmds, content);
        //std::cout<<"hahaha"<<cmds<<std::endl;
        strcat(cmds, "\n");
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("VERSY")<= c.length())
    {
        //query_all_version();
        //std::cout<<"ha0"<<std::endl;
        TiXmlDocument doc;
        doc.LoadFile("/home/amy/amyos/amy_ver.xml");
        //std::cout<<"ha0"<<std::endl;
        TiXmlElement *pkg=doc.RootElement();
        //std::cout<<"ha1"<<std::endl;
        TiXmlElement *version=pkg->FirstChildElement("version"); 
        //std::cout<<"ha2"<<std::endl;
        const char* content= version->GetText();
        //std::cout<<"ha3"<<std::endl;
        strcpy(cmds, content);
        strcat(cmds, "\n");
        //std::cout<<"hahahaha"<<cmds<<std::endl;
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else if(c.find("CVERA")<= c.length())
    {
        //query_all_version();
        std::ifstream infileo,infilet;
        std::string stro,strt;
        infileo.open("/home/amy/amyos/Driver_FW_version.txt");
        std::getline(infileo,stro);
        const char* contento=stro.c_str();
        strcpy(cmds, contento);
        if(stro=="")
        {
            strcpy(cmds, "0.0.0");
        }
        strcat(cmds, "@");

        infilet.open("/home/amy/amyos/DATA_FW_version.txt");
            std::getline(infilet,strt);	
        const char* contentt=strt.c_str();
        strcat(cmds, contentt);
        if(strt=="")
        {
            strcat(cmds, "0.0.0");
        }
        strcat(cmds, "@");
        
        TiXmlDocument doc;
        bool loadOkay=doc.LoadFile("/home/amy/amyos/amy_ver.xml");
        if(loadOkay)
        {
            TiXmlElement *pkg=doc.RootElement();
            TiXmlElement *version=pkg->FirstChildElement("version"); 
                const char* content= version->GetText();
            strcat(cmds, content);
        
            strcat(cmds, "\n");
            std::cout<<"all"<<cmds<<std::endl;
            ROS_INFO("return: %s",cmds);
            return cmds;
        }
        else
        {
            strcat(cmds, "0.0.0\n");
            ROS_INFO("return: %s",cmds);
            return cmds;
        }
	
    }else if(c.find("#UPDDR")<= c.length())
    {
        std_msgs::Int32 cmd_upddr;
        cmd_upddr.data = 6;
        int l_t=0;
        if(upddr_flag == false)
        {
            CmdState_pub.publish(cmd_upddr);//topic:CmdState
            upddr_flag = true;
            //ROS_INFO("if--%d\n",RobotState);
            while(l_t<60)//60s
            {
                sleep(1);
                ++l_t;
                if(RobotState==6)
                {
                    strcpy(cmds, "#UPDRST01\n");//update success
                    upddr_flag = false;
                    RobotState=0;
                    ROS_INFO("return: %s",cmds);
                    return cmds;
                }else if(RobotState==7)
                {
                    strcpy(cmds, "#UPDRST02\n");//update fail
                    upddr_flag = false;
                    RobotState=0;
                    ROS_INFO("return: %s",cmds);
                    return cmds;
                }
            }
            strcpy(cmds, "#UPDRST08\n");//update time out
            upddr_flag = false;
            ROS_INFO("return: %s",cmds);
            return cmds;
        }
        else
        {
            strcpy(cmds, "#UPDRST09\n");//updating
            ROS_INFO("return: %s",cmds);
            return cmds;
        }
    }else if (c.find("#CPUO") <= c.length())
    {
        std::lock_guard<std::mutex> CPUO_guard(mutex_cpu);
        if(cpu_open_flag == false)
        {
            thread_cpu = std::thread(system, "stress -c 3");
            cpu_open_flag = true;
            strcpy(cmds, "done\n");
            ROS_INFO("return: %s",cmds);
            return cmds;
        }else
        {
            strcpy(cmds, "CPUO error\n");
            ROS_INFO("return: %s",cmds);
            return cmds;
        }
    }else if (c.find("#CPUS") <= c.length())
    {
        std::lock_guard<std::mutex> CPUS_guard(mutex_cpu);
        if(cpu_open_flag == true)
        {
            system("kill $(ps -aux|grep stress|grep -v grep|awk '{print $2}')");
            cpu_open_flag = false;
            strcpy(cmds, "done\n");
            ROS_INFO("return: %s",cmds);
            return cmds;
        }else
        {
            strcpy(cmds, "CPUS error\n");
            ROS_INFO("return: %s",cmds);
            return cmds;
        }
    }else if (c.find("#SSCC!") <= c.length())
    {
        // std::istringstream istrs(c);
        const char *safety_control_str = c.c_str();
        // std::cout << "111:" << safety_control_str << std::endl;
        unsigned int safety_control_param;
        sscanf(safety_control_str, "%*[^0-9-]%d", &safety_control_param);
        
        // if(safety_control_param & 0x01)

        if(safety_control_param == 0)
        {
            system("rosrun dynamic_reconfigure dynparam set /safety_control \"{\'touch_flag\':false, \'sonar_flag\':false, \'lidar_flag\':false, \'depth_camera_flag\':false, \'error_flag\':false}\"");
        }else if(safety_control_param == 1)
        {
            system("rosrun dynamic_reconfigure dynparam set /safety_control \"{\'touch_flag\':true, \'sonar_flag\':false, \'lidar_flag\':false, \'depth_camera_flag\':false, \'error_flag\':false}\"");
        }else if(safety_control_param == 2)
        {
            system("rosrun dynamic_reconfigure dynparam set /safety_control \"{\'touch_flag\':false, \'sonar_flag\':true, \'lidar_flag\':false, \'depth_camera_flag\':false, \'error_flag\':false}\"");
        }else if(safety_control_param == 3)
        {
            system("rosrun dynamic_reconfigure dynparam set /safety_control \"{\'touch_flag\':true, \'sonar_flag\':true, \'lidar_flag\':false, \'depth_camera_flag\':false, \'error_flag\':false}\"");
        }else if(safety_control_param == 31)
        {
            system("rosrun dynamic_reconfigure dynparam set /safety_control \"{\'touch_flag\':true, \'sonar_flag\':true, \'lidar_flag\':true, \'depth_camera_flag\':true, \'error_flag\':true}\"");
        }

        std::string ssc_str = "#SSCR!";
        ssc_str += std::to_string(safety_control_param) + '\n';
        // istrs >> i;
        // std::cout << safety_control_param << std::endl;
        strcpy(cmds, ssc_str.c_str());
        ROS_INFO("return: %s",cmds);
        return cmds;
    }else if(c.find("#SSPD!") <= c.length())
    {
        std::string robot_type;
        ros::param::get("~robot_type",robot_type);
        ROS_INFO_STREAM("robot_type: " << robot_type);

        // std::istringstream line(c);
        // double d_tmp;
        // line >> d_tmp;
        // std::cout << d_tmp << std::endl;
        double d_tmp = stod(c.substr(c.find_first_of(std::string("0123456789"))));
        std::cout << d_tmp << std::endl;

        if(robot_type == "a1")
        {
            if(d_tmp >= 0.45) d_tmp = 0.45;
            else if(d_tmp <= 0.2) d_tmp = 0.2;
        }else if(robot_type == "a2")
        {
            if(d_tmp >= 1.2) d_tmp = 1.2;
            else if(d_tmp <= 0.2) d_tmp = 0.2;
        }else if(robot_type == "m1" || robot_type == "m1arkl")
        {
            if(d_tmp >= 1.2) d_tmp = 1.2;
            else if(d_tmp <= 0.2) d_tmp = 0.2;
        }else
        {
            ROS_ERROR("#SSPD!: robot_type ERROR!");
        }
        std::string str_tmp;
        str_tmp += "rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS \"{\'max_vel_x\':" + std::to_string(d_tmp) + "}\"";
        // str_tmp += "rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS \"{\'max_vel_x\':" + std::to_string(d_tmp) + ", \'max_vel_theta\':0.7, \'acc_lim_x\':0.8, \'acc_lim_theta\':0.6}\"";
        // system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS \"{\'max_vel_x\':0.7, \'max_vel_theta\':0.7, \'acc_lim_x\':0.8, \'acc_lim_theta\':0.6}\"");
        system(str_tmp.c_str());
        strcpy(cmds, "#SSPDS\n");
        ROS_INFO("return: %s",cmds);
        return cmds;
    }else if(c.find("#GSPD") <= c.length())
    {
        FILE *fp;

        char buf[200];
        memset(buf, 0, 200);
        
        if((fp = popen("rosrun dynamic_reconfigure dynparam get /move_base/TebLocalPlannerROS", "r")) == NULL)
        {
            ROS_ERROR("#GSPD: Fail to popen\n");
        }

        std::string str_tmp;
        while(fgets(buf, 200, fp) != NULL)
        {
            str_tmp += std::string(buf);
            // printf("%s", buf);
        }
        // std::cout << str_tmp << std::endl;
        pclose(fp);
        // std::cout << str_tmp.substr(str_tmp.find(std::string("'max_vel_x'")) + 12) << std::endl;
        double d_tmp = stod(str_tmp.substr(str_tmp.find(std::string("'max_vel_x'")) + 12));
        // double d_tmp = 0;
        std::cout << d_tmp << std::endl;
        str_tmp.clear();
        str_tmp += "#GSPD!" + std::to_string(d_tmp) + "\n";
        strcpy(cmds, str_tmp.c_str());
        ROS_INFO("return: %s",cmds);
        return cmds;
    }else if(c.find("#GOLF") <= c.length())
    {
        if(overload_flag)
        {
            strcpy(cmds, "#OLTR\n");
        }else
        {
            strcpy(cmds, "#OLFA\n");
        }
        
        ROS_INFO("return: %s",cmds);
        return cmds;
    }else if(c.find("#REOL") <= c.length())
    {
        ros::ServiceClient client = socket_nh.serviceClient<std_srvs::Empty>("reset_overload_flag");
        std_srvs::Empty srv;

        if (client.call(srv))
        {
            strcpy(cmds, "#REOLS\n");
            ROS_INFO("reset overload flag success!");
        }
        else
        {
            strcpy(cmds, "#REOLF\n");
            ROS_ERROR("Failed to call service reset_overload_flag!");
        }
        ROS_INFO("return: %s",cmds);
        return cmds;
    }else if(c.find("#RANP") <= c.length())     //A2 cmd
    {
        ros::ServiceClient reset_android_power_client = socket_nh.serviceClient<std_srvs::Empty>("reset_android_power");
        std_srvs::Empty srv;

        if (reset_android_power_client.call(srv))
        {
            strcpy(cmds, "#RANPS\n");
            ROS_INFO("success to call reset android power!");
        }
        else
        {
            strcpy(cmds, "#RANPF\n");
            ROS_ERROR("Failed to call service reset android power!");
        }
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    // else if(c.find("GMBID")<= c.length())
    // {
    //     //query_all_version();
    //     std::ifstream fmbid;
    //     std::string strmbid;
    //     mbid.open("/home/amy/amyos/DATA_FW_version.txt");
    //     std::getline(mbid,strmbid);
    //     const char* content=strmbid.c_str();
    //     strcpy(cmds, content);
    //     //std::cout<<"hahaha"<<cmds<<std::endl;
    //     strcat(cmds, "\n");
    //     ROS_INFO("return: %s",cmds);
    //     return cmds;
    // }
    else if(c.find("#BJZ:START")<= c.length())
    {
        std_msgs::Int32 bjz_speed_cmd_data, bjz_cmd_data;
        
        if(c.find("0") <= c.length())
        {
            bjz_speed_cmd_data.data = 0;
            bjz_speed_cmd_pub.publish(bjz_speed_cmd_data);
        }else if(c.find("1") <= c.length())
        {
            bjz_speed_cmd_data.data = 1;
            bjz_speed_cmd_pub.publish(bjz_speed_cmd_data);
        }else if(c.find("2") <= c.length())
        {
            bjz_speed_cmd_data.data = 2;
            bjz_speed_cmd_pub.publish(bjz_speed_cmd_data);
        }else if(c.find("3") <= c.length())
        {
            bjz_speed_cmd_data.data = 3;
            bjz_speed_cmd_pub.publish(bjz_speed_cmd_data);
        }else if(c.find("4") <= c.length())
        {
            bjz_speed_cmd_data.data = 4;
            bjz_speed_cmd_pub.publish(bjz_speed_cmd_data);
        }else
        {
            bjz_speed_cmd_data.data = 2;
            bjz_speed_cmd_pub.publish(bjz_speed_cmd_data);
        }

        bjz_cmd_data.data = 2;
        bjz_cmd_pub.publish(bjz_cmd_data);

        strcpy(cmds, "#done\n");
        ROS_INFO("return: %s",cmds);
        return cmds;
    }else if(c.find("#BJZ:STOP")<= c.length())
    {
        std_msgs::Int32 bjz_cmd_data;
        bjz_cmd_data.data = 0;
        bjz_cmd_pub.publish(bjz_cmd_data);
        
        strcpy(cmds, "#done\n");
        ROS_INFO("return: %s",cmds);
        return cmds;
    }else if (c.find("#BJZ:DOCK") <= c.length())//start docking
    {
        dock_result = 0;
        // std_msgs::Int32 dock_cmd_data;
        // dock_cmd_data.data = 1;  //start dock
        // CmdAutoDock_pub.publish(dock_cmd_data);
        std_msgs::Int32 bjz_cmd_data;
        bjz_cmd_data.data = 1;  //start dock
        bjz_cmd_pub.publish(bjz_cmd_data);
        int l_t = 0;
        while(l_t < 4000)//400s
        {
            //ROS_INFO("while--%d\n",dock_result);
            sleep(1);
            ++l_t;
            if(dock_result==1)
            {
                strcpy(cmds, "#DRST01\n");//auto docking success
                dock_result=0;
                ROS_INFO("return: %s",cmds);
                return cmds;
            }else if(dock_result==2)
            {
                strcpy(cmds, "#DRST02\n");//auto docking fail
                dock_result=0;
                ROS_INFO("return: %s",cmds);
                return cmds;
            }else if(dock_result==3)//can not find IR
            {
                strcpy(cmds, "#DRST03\n");
                dock_result=0;
                ROS_INFO("return: %s",cmds);
                return cmds;
            }else if(dock_result==4)//cancel the dock
            {
                strcpy(cmds, "#DRST04\n");
                dock_result=0;
                ROS_INFO("return: %s",cmds);
                return cmds;
            }else if(dock_result==5)//e.g.move the dock or the dock power off
            {
                strcpy(cmds, "#DRST05\n");
                dock_result=0;
                ROS_INFO("return: %s",cmds);
                return cmds;
            }else if(dock_result==6)//dock time out
            {
                strcpy(cmds, "#DRST08\n");
                dock_result=0;
                ROS_INFO("return: %s",cmds);
                return cmds;
            }
        }
        strcpy(cmds, "#DRST08\n");//dock time out
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
    else
    {
        strcpy(cmds, "error\n");
        ROS_INFO("return: %s",cmds);
        return cmds;
    }
}

void Amy_Cmd::pub_velocity(const char *velocity, int type)
{
    int vel = 0;
    int angular_spd = 0;
    double duration = 0;
    sscanf(velocity, "%*[^0-9-]%d%*[^0-9-]%d", &vel, &angular_spd);
    socket_twist.linear.x = vel * 0.2;
    socket_twist.angular.z = angular_spd * 0.4;
    tel_vel_pub.publish(socket_twist);
    std::cout << "vel" << vel << "angular" << angular_spd << std::endl;
    if (type == 2)
    {
        while(duration <= 20)
        {
            tel_vel_pub.publish(socket_twist);
            duration++;
            usleep(100000);
        }
        socket_twist.linear.x = 0;
        socket_twist.angular.z = 0;
        tel_vel_pub.publish(socket_twist);
    }
}

char *Amy_Cmd::write_pose(void)
{

    while(ros::ok())
    {
        try
        {
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            char pose_err[512];
            strcpy(pose_err, "#SETF\n");
            return pose_err;
        }
        sleep(0.5);
        std::ofstream loca;
        loca.open("/home/amy/amyos/location.txt", std::ios::app);
        sleep(0.5);
        if (loca.is_open())
        {
            location_origin_x=transform.getOrigin().x();
            location_origin_y=transform.getOrigin().y();
            location_origin_z=transform.getOrigin().z();
            location_rotation_x=transform.getRotation().x();
            location_rotation_y=transform.getRotation().y();
            location_rotation_z=transform.getRotation().z();
            location_rotation_w=transform.getRotation().w();
            
            tf::Quaternion q(location_rotation_x, location_rotation_y, location_rotation_z, location_rotation_w);
            tf::Matrix3x3 m(q);
            //double roll, pitch;
            m.getRPY(location_rotation_roll, location_rotation_pitch, location_rotation_yaw);
            ROS_INFO("%f|%f|%f||%f|%f|%f|%f||%f|%f|%f",location_origin_x,location_origin_y,location_origin_z,location_rotation_x,location_rotation_y,location_rotation_z,location_rotation_w,location_rotation_roll,location_rotation_pitch,location_rotation_yaw);
            loca << location_origin_x << ',' << location_origin_y << ',' << location_origin_z << ',';
            loca << location_rotation_x << ',' << location_rotation_y << ',' << location_rotation_z << ',' << location_rotation_w << "\n";
            loca.close();
            break;
        }

    }
    char pose_suc[10];
    strcpy(pose_suc, "#SETS\n");
    return pose_suc;
}

//1,L;2,R;3,B.
void Amy_Cmd::turn(int type)
{
    int timer_100ms = 0, loop_time = 0;

    if(type == 1)
    {
        loop_time = 30;
        socket_twist.linear.x = 0;
        socket_twist.angular.z = 0.523;
    }else if(type == 2)
    {
        loop_time = 30;
        socket_twist.linear.x = 0;
        socket_twist.angular.z = -0.523;
    }else if(type == 3)
    {
        loop_time = 60;
        socket_twist.linear.x = 0;
        socket_twist.angular.z = 0.523;
    }

    while(timer_100ms < loop_time)
    {
        tel_vel_pub.publish(socket_twist);
        ++timer_100ms;
        usleep(100000);
    }

    socket_twist.linear.x = 0;
    socket_twist.angular.z = 0;
    tel_vel_pub.publish(socket_twist);
}

void Amy_Cmd::map_stop(void)
{
    if(map_init_flag == false) return;
    std::lock_guard<std::recursive_mutex> map_stop_guard(mutex_map);
    system("rosnode kill slam_gmapping");
    system("rosnode kill ekf_localization_local");
    system("rosnode kill imu_remap");
    system("rosnode kill rtabmap");
    system("rosnode kill rgbd_odometry");
    if(thread_mapo_fused.joinable())
    {
        ROS_INFO("map_stop: thread_mapo_fused.joinable(): true");
        thread_mapo_fused.join();
        ROS_INFO("map_stop: thread_mapo_fused.join()");
    }else
    {
        
        ROS_INFO("map_stop: thread_mapo_fused.joinable(): false");
    }
    if(thread_mapo_mapping.joinable())
    {
        ROS_INFO("map_stop: thread_mapo_mapping.joinable(): true");
        thread_mapo_mapping.join();
        ROS_INFO("map_stop: thread_mapo_mapping.join()");
    }else
    {
        
        ROS_INFO("map_stop: thread_mapo_mapping.joinable(): false");
    }
    map_init_flag = false;
}

void Amy_Cmd::navigation_stop()
{
    if(nav_init_flag == false) return;
    std::lock_guard<std::recursive_mutex> navigation_stop_guard(mutex_navigation);

    amy_comm::CmdLed led_cmd_data;
    led_cmd_data.Cmd = 5;
    led_cmd_data.Value = 1;
    led_cmd_pub.publish(led_cmd_data);

    if(naving_flag == true)
    {
        ROS_INFO("navigation_stop: cancel the goal first");
        // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base");
        // ac.cancelAllGoals();
        ac.cancelGoal();
        naving_flag = false;
    }

    // system("rosnode kill ekf_localization_global");
    system("rosnode kill amcl_manager");
    system("rosnode kill amcl");
    system("rosnode kill ekf_localization_local");
    system("rosnode kill move_base");
    system("rosnode kill map_server");
    system("rosnode kill imu_remap");
    system("rosnode kill depthscan_filter_node");
    system("rosnode kill snap_map_icp");
    system("rosnode kill rtabmap");

    if(thread_navs.joinable())
    {
        ROS_INFO("navigation_stop: thread_navs.joinable(): true");
        thread_navs.join();
        ROS_INFO("navigation_stop: thread_navs.join()");
    }else
    {
        ROS_INFO("navigation_stop: thread_navs.joinable(): false");
    }
    nav_init_flag = false;
}

bool Amy_Cmd::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
        ROS_ERROR("Quaternion has nans or infs... discarding as a goal");
        return false;
    }

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
        ROS_ERROR("Quaternion has length close to zero... discarding as goal");
        return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
        ROS_ERROR("Quaternion is invalid... for the z-axis of the quaternion must be close to vertical.");
        return false;
    }

    return true;
}

void emergency_stop_status_Callback(const std_msgs::Bool::ConstPtr &dmsg)
{
    Amy_Cmd::emergency_stop_status = dmsg->data;
}

void bat_data_callback(const amy_comm::bat::ConstPtr &msg)
{
    Amy_Cmd::bat_str_quantity_charging.clear();
    Amy_Cmd::bat_str_quantity_charging = Amy_Cmd::bat_str_quantity_charging + "#BATP" + std::to_string(msg->C) + ':' + std::to_string(msg->ChgState) + '\n';
}

void RobotState_Callback(const std_msgs::Int32::ConstPtr &dmsg)
{
    Amy_Cmd::RobotState = dmsg->data;
}
void StateAutoDock_Callback(const std_msgs::Int32::ConstPtr &dmsg)
{
    Amy_Cmd::dock_result = dmsg->data;
}

void overload_flag_Callback(const std_msgs::Bool::ConstPtr &dmsg)
{
    Amy_Cmd::overload_flag = dmsg->data;
}
