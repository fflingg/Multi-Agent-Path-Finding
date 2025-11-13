
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#ifndef ZHONGQIANGREN_BASIC_VECTYPE_H_
#define ZHONGQIANGREN_BASIC_VECTYPE_H_

#include <string>
#include <vector>
#include <iostream>
// #include <math.h>
#include <cmath>

namespace raplab{

template<typename Scalar>
std::vector<Scalar> operator+(const std::vector<Scalar>& a, const std::vector<Scalar>& b)
{
  std::vector<Scalar> out;
  out.resize(a.size());
  for (int i = 0; i < a.size(); i++){
    out[i] = a[i]+b[i];
  }
  return out;
};

template<typename Scalar>
std::vector<Scalar>& operator+=(std::vector<Scalar>& a, const std::vector<Scalar>& b)
{
  for (int i = 0; i < a.size(); i++){
    a[i] += b[i];
  }
  return a;
};

template<typename Scalar>
std::vector<Scalar> operator-(const std::vector<Scalar>& a, const std::vector<Scalar>& b)
{
  std::vector<Scalar> out;
  out.resize(a.size());
  for (int i = 0; i < a.size(); i++){
    out[i] = a[i]-b[i];
  }
  return out;
};

template<typename Scalar>
double NormL2(const std::vector<Scalar>& a)
{
  double out = 0;
  for (int i = 0; i < a.size(); i++){
    out += a[i]*a[i];
  }
  return std::sqrt(out);
};

template<typename Scalar>
double Normalize(const std::vector<Scalar>& a)
{
  double norm = NormL2(a);
  return a * (1.0 / norm);
};

template<typename Scalar>
Scalar InnerProduct(const std::vector<Scalar>& a, const std::vector<Scalar>& b)
{
  Scalar out = 0;
  for (int i = 0; i < a.size(); i++){
    out += a[i]*b[i];
  }
  return out;
};


template<typename Scalar>
std::vector<Scalar> operator*(const Scalar& k, const std::vector<Scalar>& a)
{
  std::vector<Scalar> out;
  out.resize(a.size());
  for (int i = 0; i < a.size(); i++){
    out[i] = k * a[i];
  }
  return out;
};

template<typename Scalar>
std::vector<Scalar> operator*(const std::vector<Scalar>& a, const Scalar& k)
{
  std::vector<Scalar> out;
  out.resize(a.size());
  for (int i = 0; i < a.size(); i++){
    out[i] = k * a[i];
  }
  return out;
};

template<typename Scalar>
std::vector<Scalar> InitVecType(size_t dim=0, Scalar val=0.0)
{
  std::vector<Scalar> out;
  out.resize(dim);
  for (int i = 0; i < dim; i++){
    out[i] = val;
  }
  return out;
};


/**
 * @brief Lexicographic comparison.
 * Return -1 if v1 < v2
 * Return 0 if v1 = v2
 * Return 1 if v1 > v2
 */
template<typename Scalar>
int LexCompare( const std::vector<Scalar>& v1, const std::vector<Scalar>& v2)
{
  for (size_t i = 0; i < v1.size(); i++) {
    if (v1[i] < v2[i]) {
      return -1;
    } else if (v1[i] > v2[i]) {
      return 1;
    }
  }
  return 0;
};

/**
 * @brief Epsilon Dominance.
 * When eps=0.0, this is weak dominance.
 */
template<typename Scalar>
int EpsDom( const std::vector<Scalar>& v1, const std::vector<Scalar>& v2, double eps=0.0, bool less=true)
{
  auto i2 = v2.begin();
  for (auto i1 = v1.begin(); i1 != v1.end(); i1++){
    if (less) {
      if (*i1 > (1.0+eps)*(*i2)) { return false; }
    }else{
      if (*i1 < (1.0+eps)*(*i2)) { return false; }
    }
    i2++;
  }
  return true;
};

template<typename Scalar>
bool operator==(const std::vector<Scalar>& v1, const std::vector<Scalar>& v2)
{
  for (size_t i = 0; i < v1.size(); i++) {
    if (v1[i] != v2[i]) {
      return false;
    }
  }
  return true;
};

template<typename Scalar>
std::vector<Scalar> Min(const std::vector<Scalar>& v1, const std::vector<Scalar>& v2)
{
  std::vector<Scalar> out = v2;
  for (size_t i = 0; i < v1.size(); i++) {
    if (v1[i] < v2[i]) {
      out[i] = v1[i];
    }
  }
  return out;
};

template<typename Scalar>
Scalar Min(const std::vector<Scalar>& v1)
{
  Scalar out = v1[0];
  for (size_t i = 1; i < v1.size(); i++) {
    if (v1[i] < out) {
      out = v1[i];
    }
  }
  return out;
};

template<typename Scalar>
std::vector<Scalar> Max(const std::vector<Scalar>& v1, const std::vector<Scalar>& v2)
{
  std::vector<Scalar> out = v2;
  for (size_t i = 0; i < v1.size(); i++) {
    if (v1[i] > v2[i]) {
      out[i] = v1[i];
    }
  }
  return out;
};

template<typename Scalar>
Scalar Max(const std::vector<Scalar>& v1)
{
  Scalar out = v1[0];
  for (size_t i = 1; i < v1.size(); i++) {
    if (v1[i] > out) {
      out = v1[i];
    }
  }
  return out;
};

} // end namespace zr


/**
 * @brief take the combination by select an element from every 
 *  sub-vector in the input vec.
 */
template <typename DataType>
void TakeCombination(
  const std::vector< std::vector<DataType> >& ivec, std::vector< std::vector<DataType> >* out_ptr) 
{
  std::vector< std::vector<DataType> >& out = *out_ptr;
  auto l = ivec.size();

  std::vector<int> indices;
  indices.resize(l,0);

  std::vector<DataType> a;
  a.resize(l);
  while (true) {
    // generate new element and insert into output result.
    for (int ri = 0; ri < l; ri++) {
      a[ri] = ivec[ri][indices[ri]];
    }
    out.push_back(a);
    // compute next indices
    indices[0]++;
    for (int ri = 0; ri < l-1; ri++) {
      if (indices[ri] == ivec[ri].size()) {
        indices[ri] = 0;
        indices[ri+1]++;
      }else{ break; }
    }
    if (indices[l-1] == ivec[l-1].size()) {
      break;
    }
  }
  return ;
};


// to support unordered_map
namespace std {

  template<typename Scalar>
  std::string ToString(std::vector<Scalar> c){
    std::string s = "[";
    // for (auto a : c ) {
    for (int i = 0; i < c.size(); i++){
      s += std::to_string(c[i]);
      if (i < c.size()-1 ){s += ",";}
    }
    s += "]";
    return s;
  };

  template<typename Scalar>
  std::ostream& operator<<(std::ostream& os, const std::vector<Scalar>& c) {
    os << ToString(c);
    return os;
  };

  template<typename Scalar>
  std::ostream& operator<<(std::ostream& os, const std::vector<std::vector<Scalar> >& c) {
    os << "[";
    for (int i = 0; i < c.size(); i++){
      os << c[i];
      if (i < c.size()-1) {os << std::endl;}
      else {os << "]";}
    }
    return os;
  };

  template<typename Scalar>
  struct hash<std::vector<Scalar>> {
      const size_t operator()(const std::vector<Scalar>& c) const
      {
        size_t res = 0;
        for (auto i : c) {
          res ^= std::hash<Scalar>()(i);
        }
        return res;
      }
  };
}


#endif  // ZHONGQIANGREN_BASIC_VECTYPE_H_
