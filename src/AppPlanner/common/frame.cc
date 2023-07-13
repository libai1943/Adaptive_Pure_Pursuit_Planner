//
// Created by yenkn on 3/7/22.
//

#include "frame.h"

// Frame Frame::Read(const std::string &env_json) {
//   Frame frame;
//   json j = json::parse(env_json);
//   j.get_to(frame);

//   ROS_INFO("read points: %zu, obstacles: %zu", frame.env_->points.points().size(), frame.env_->obstacles.size());
//   return frame;
// }

// void Frame::Save(const std::string &env_file) const {
//   std::ofstream os(env_file, std::ios::binary);
//   if(os.is_open()) {
//     json j = *this;
//     os << j;
//     ROS_INFO("saved points: %zu, obstacles: %zu", env_->points.points().size(), env_->obstacles.size());
//   }
// }

// void Frame::CollectResult(const std::string &env_file, const std::string &tag, double run_time) const {
//   char name_buf[128] = {0}, time_buf[16] = {0};
//   std::time_t t = std::time(nullptr);
//   std::tm *tm = std::localtime(&t);
//   std::strftime(time_buf, 16, "%y%m%d%H%M%S", tm);

//   auto tags = tag;
//   std::snprintf(name_buf, 128, "env-%s-%s-%.2f.json", time_buf, tags.c_str(), run_time);

//   size_t pos = env_file.find_last_of('/');
//   std::string path = (std::string::npos == pos) ? "." : env_file.substr(0, pos);

//   Save(path + "/" + std::string(name_buf));
// }
