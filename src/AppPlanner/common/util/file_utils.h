//
// Created by yenkn on 2021/2/7.
//
#pragma once
#include <vector>
#include <string>
#include <fstream>


namespace common {
namespace util {

bool ReadCSV(const std::string &path, std::vector<std::vector<double>> &table, bool skip_header = false);

bool WriteCSV(const std::string &path, std::vector<std::vector<double>> &table, const std::vector<std::string> &header);


}
}
