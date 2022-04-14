#pragma once

#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <pcl/io/pcd_io.h>

#include "evo_autonom_driving/common/Resources.hpp"

namespace evo_autonom_driving {

    class SharedBehaviour{
        public: 

            explicit SharedBehaviour(ros::NodeHandle& handle): nodeHandle(handle){
                filePath_ = getPackagePath();
            };

            template<typename T> void getParameter(std::string name, T& outValue){
                if(!nodeHandle.getParam(name, outValue)){
                    ROS_ERROR("Could not read | %s | from configuration file. Shutting Down", name.c_str());
                    ros::requestShutdown();
                }
            }

            SharedBehaviour & useDataFolder(){
                filePath_ = getPackagePath() + dataPath_;
                return *this;
            }

            SharedBehaviour & useConfigFolder(){
                filePath_ = getPackagePath() + configPath_;
                return *this;
            }

            std::string getPackagePath(){
                return ros::package::getPath(Resources::packageName);
            }

            void storeCloudToFile(const std::string& file_name, const PCLCloud& cloud) {
                std::string path = filePath_ + file_name;
                pcl::io::savePCDFileASCII (path, cloud);
            }

            void readCloudFromFile(const std::string& file_name, PCLCloud& cloud) {
                std::string path = filePath_ + file_name;
                if (pcl::io::loadPCDFile<PCLPoint> (path, cloud) == -1) //* load the file
                {
                    ROS_ERROR ("Couldn't read file %s\n", file_name.c_str());
                    ros::requestShutdown();
                }
            }

        private:

            std::string const dataPath_ = "/data/";
            std::string const configPath_ = "/config/";

            std::string filePath_;
            ros::NodeHandle& nodeHandle;
    };
}