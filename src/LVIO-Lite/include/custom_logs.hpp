#ifndef CUSTOM_LOG_H_
#define CUSTOM_LOG_H_


#include <ctime>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <filesystem>          // C++17
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <ros/package.h>


using namespace std;
using namespace google;


void init_log(){
  FLAGS_logbufsecs = 0;
	FLAGS_minloglevel = google::INFO;
	FLAGS_alsologtostderr = true;
}


void save_log(const string& dir){
  auto now = std::chrono::system_clock::now();
	std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  char time_str[100];
  strftime(time_str, sizeof(time_str), "%Y%m%d-%H%M%S", localtime(&now_c));
	std::string package_path, log_folder;
	if (dir == "local"){
		package_path = ros::package::getPath("lvio_lite");
		log_folder = package_path + "/logs/" + time_str + "/";
		if (!std::filesystem::exists(log_folder)) {
			std::filesystem::create_directory(log_folder);
		}
	}else{
		log_folder = dir + "/" + time_str + "/";
		if (!std::filesystem::exists(log_folder)) {
			std::filesystem::create_directory(log_folder);
		}
	}
  FLAGS_log_dir = log_folder;
}


#endif
using namespace google;