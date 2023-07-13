//
// Created by 欧阳亚坤 on 2021/9/15.
//
#pragma once
#include <chrono>

namespace common {
namespace util {

using namespace std::chrono;

inline double GetCurrentTimestamp() {
  return ((double) duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() / 1000);
}

}
}