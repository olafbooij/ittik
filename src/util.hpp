#pragma once
#include <tuple>

// http://www.reedbeta.com/blog/python-like-enumerate-in-cpp17/  Nathan Reed 
template<typename T, typename TIter = decltype(std::begin(std::declval<T>())), typename = decltype(std::end(std::declval<T>()))>
constexpr auto enumerate(T&& iterable)
{
  struct iterator
  {
    size_t i;
    TIter iter;
    bool operator!=(const iterator & other) const {return iter != other.iter;}
    void operator++() {++i; ++iter;}
    auto operator*() const {return std::tie(i, *iter);}
  };
  struct iterable_wrapper
  {
    T iterable;
    auto begin() {return iterator{0, std::begin(iterable)};}
    auto end() {return iterator{0, std::end(iterable)};}
  };
  return iterable_wrapper{ std::forward<T>(iterable) };
}

//template<typename Func>
//void repeat(std::size_t n, Func func)
//{
//  while(n--)
//    func();
//}
//
//template<typename Func>
//auto generate_vector(std::size_t n, Func&& func)
//{
//  std::vector<decltype(func())> vec;
//  repeat(n, [&]{vec.emplace_back(func());});
//  return vec;
//}

