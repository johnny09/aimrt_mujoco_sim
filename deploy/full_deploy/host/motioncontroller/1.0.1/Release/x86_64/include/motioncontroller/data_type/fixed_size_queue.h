#pragma once
#include <deque>
#include <iostream>
namespace mc {
namespace data_type {

template <typename T>
class FixedSizeQueue {
 public:
  FixedSizeQueue(std::size_t size);
  // insert element
  void push(const T& val);
  // pop up element
  void pop();
  // get the number of elements
  std::size_t size() const;
  // is the queue empty
  bool empty() const;
  // get the first element
  T& front();
  // get the last element
  T& back();
  // get the sum of elements
  T sum();
  // get the mean of elements
  T mean();
  // get the max element
  T max();
  // get the min element
  T min();
  // get the variance of all elements
  T variance();
  // get the standard deviation of all elements
  T std();
  // print all elements
  void print();

 private:
  std::deque<T> m_deque_;
  std::size_t m_max_size_{0};
};
}  // namespace data_type
}  // namespace mc