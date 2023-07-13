//
// Created by yenkn on 2021/2/7.
//

#include "common/util/file_utils.h"
#include "common/math/math_utils.h"

#include <cmath>
#include <sstream>
#include <fstream>

namespace common {
namespace util {

bool ReadCSV(const std::string &path, std::vector<std::vector<double>> &table, bool skip_header) {
  std::ifstream fs(path, std::ios::in);
  if(!fs.is_open()) {
    return false;
  }

  std::string line;

  if(skip_header) {
    std::getline(fs, line);
  }

  table.clear();

  while (std::getline(fs, line)) {
    std::stringstream ss(line);
    std::vector<double> row;

    double val = 0.0;
    int col_idx = 0;

    while(ss >> val) {
      row.push_back(val);

      if(ss.peek() == '\t' || ss.peek() == ' ' || ss.peek() == ',') ss.ignore();
      col_idx++;
    }

    table.push_back(row);
  }

  return true;
}

bool WriteCSV(const std::string &path, std::vector<std::vector<double>> &table, const std::vector<std::string> &header) {
  std::ofstream os(path);
  if(os.is_open()) {
    for(auto &row: table) {
      for(auto &col: row) {
        os << col << ',';
      }
      os << std::endl;
    }
    return true;
  }
  return false;
}

}
}
