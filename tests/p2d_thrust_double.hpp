#pragma once
// HOST TEST DOUBLE ONLY. This never executes CUDA or emulates its scheduling.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <functional>
#include <tuple>
#include <vector>
#include <memory>
#include <cmath>
#include <cassert>
#include <iostream>
#include <stdexcept>
#define __host__
#define __device__
using cudaStream_t = int;
inline int cudaDeviceSynchronize() { return 0; }
namespace fake_cuda {
static unsigned allocations = 0, frees = 0;
}
namespace thrust {
template<class T> class device_allocator : public Eigen::aligned_allocator<T> {
public:
  using value_type = T;
  template<class U> struct rebind { using other = device_allocator<U>; };
};
template<class T> using device_ptr = T*;
template<class A, class B> class pair : public std::pair<A, B> {
public:
  pair() = default;
  pair(const A& a, const B& b) : std::pair<A, B>(a,b) {}
};
template<class A, class B> pair<A,B> make_pair(A a, B b) { return pair<A,B>(a,b); }
using std::tuple;
using std::make_tuple;
using std::get;
template<class T> using plus = std::plus<T>;
template<class T> inline T* raw_pointer_cast(T* p) { return p; }
template<class T, class Alloc = std::allocator<T>> using host_vector = std::vector<T, Alloc>;
template<class T, class Alloc = device_allocator<T>> class device_vector {
  std::vector<T, Eigen::aligned_allocator<T>> values_;
public:
  using iterator = typename decltype(values_)::iterator;
  using const_iterator = typename decltype(values_)::const_iterator;
  device_vector() = default;
  explicit device_vector(std::size_t n) { resize(n); }
  device_vector(const device_vector& other) { reserve(other.size()); values_.assign(other.begin(), other.end()); }
  template<class A> device_vector(const std::vector<T,A>& other) {
    reserve(other.size()); values_.assign(other.begin(), other.end());
  }
  ~device_vector() { if (capacity()) ++fake_cuda::frees; }
  template<class A> device_vector& operator=(const std::vector<T,A>& other) {
    reserve(other.size()); values_.assign(other.begin(), other.end()); return *this;
  }
  device_vector& operator=(const device_vector& other) {
    reserve(other.size()); values_.assign(other.begin(), other.end()); return *this;
  }
  void reserve(std::size_t n) {
    if(n>capacity()) { if(capacity()) ++fake_cuda::frees; ++fake_cuda::allocations; values_.reserve(n); }
  }
  void resize(std::size_t n) { reserve(n); values_.resize(n); }
  void clear() { values_.clear(); }
  std::size_t size() const { return values_.size(); }
  std::size_t capacity() const { return values_.capacity(); }
  T* data() { return values_.data(); }
  const T* data() const { return values_.data(); }
  T& operator[](std::size_t n) { return values_[n]; }
  const T& operator[](std::size_t n) const { return values_[n]; }
  iterator begin() { return values_.begin(); }
  iterator end() { return values_.end(); }
  const_iterator begin() const { return values_.begin(); }
  const_iterator end() const { return values_.end(); }
  iterator erase(iterator first, iterator last) { return values_.erase(first,last); }
};
template<class It, class Fn, class Sum, class Op>
Sum transform_reduce(It first, It last, Fn fn, Sum sum, Op op) {
  for (; first != last; ++first) sum = op(sum, fn(*first));
  return sum;
}
template<class It, class Out> Out copy(It first, It last, Out out) {
  return std::copy(first,last,out);
}
}
