#pragma once

#include"liespline/liespline.hpp"
#include"liespline/se3.hpp"

namespace liespline {

  struct se3
  {
    static auto log(const auto& a){ return logse3(a); }
    static auto exp(const auto& a){ return expse3(a); }
    static auto place(const auto& a, const auto& b){ return (a.inverse() * b); }
    static auto prod(const auto& a){ return a; }
    static auto prod(const auto& a, const auto& b, const auto&... t){ return (a * prod(b, t...)); }
  };

}
