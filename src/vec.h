//  Copyright (c) www.chongyangma.com
//
//  author: Chongyang Ma - 2013-03-22
//  email:  chongyangm@gmail.com
//  info: template class declaration of a vector
// --------------------------------------------------------------

#ifndef VEC_H
#define VEC_H

#include "util.h"
#include <cassert>
#include <cmath>
#include <iostream>

template <unsigned int N, class T> struct Vec {
  T v[N];

  Vec<N, T>(void) {}

  Vec<N, T>(T value_for_all) {
    for (unsigned int i = 0; i < N; ++i)
      v[i] = value_for_all;
  }

  template <class S> Vec<N, T>(const S *source) {
    for (unsigned int i = 0; i < N; ++i)
      v[i] = static_cast<T>(source[i]);
  }

  Vec<N, T>(T v0, T v1) {
    assert(N == 2);
    v[0] = v0;
    v[1] = v1;
  }

  Vec<N, T>(T v0, T v1, T v2) {
    assert(N == 3);
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
  }

  Vec<N, T>(T v0, T v1, T v2, T v3) {
    assert(N == 4);
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
    v[3] = v3;
  }

  Vec<N, T>(T v0, T v1, T v2, T v3, T v4) {
    assert(N == 5);
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
    v[3] = v3;
    v[4] = v4;
  }

  Vec<N, T>(T v0, T v1, T v2, T v3, T v4, T v5) {
    assert(N == 6);
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
    v[3] = v3;
    v[4] = v4;
    v[5] = v5;
  }

  T &operator[](size_t index) {
    assert(0 <= index && static_cast<unsigned>(index) < N);
    return v[index];
  }

  const T &operator[](size_t index) const {
    assert(0 <= index && static_cast<unsigned>(index) < N);
    return v[index];
  }

  Vec<N, T> operator+=(const Vec<N, T> &w) {
    for (unsigned int i = 0; i < N; ++i)
      v[i] += w[i];
    return *this;
  }

  Vec<N, T> operator+(const Vec<N, T> &w) const {
    Vec<N, T> sum(*this);
    sum += w;
    return sum;
  }

  Vec<N, T> operator-=(const Vec<N, T> &w) {
    for (unsigned int i = 0; i < N; ++i)
      v[i] -= w[i];
    return *this;
  }

  Vec<N, T> operator-(void) const // unary minus
  {
    Vec<N, T> negative;
    for (unsigned int i = 0; i < N; ++i)
      negative.v[i] = -v[i];
    return negative;
  }

  Vec<N, T> operator-(const Vec<N, T> &w) const // (binary) subtraction
  {
    Vec<N, T> diff(*this);
    diff -= w;
    return diff;
  }

  Vec<N, T> operator*=(T a) {
    for (unsigned int i = 0; i < N; ++i)
      v[i] *= a;
    return *this;
  }

  Vec<N, T> operator*(T a) const {
    Vec<N, T> w(*this);
    w *= a;
    return w;
  }

  Vec<N, T> operator*(const Vec<N, T> &w) const {
    Vec<N, T> componentwise_product;
    for (unsigned int i = 0; i < N; ++i)
      componentwise_product[i] = v[i] * w.v[i];
    return componentwise_product;
  }

  Vec<N, T> operator/=(T a) {
    for (unsigned int i = 0; i < N; ++i)
      v[i] /= a;
    return *this;
  }

  Vec<N, T> operator/(T a) const {
    Vec<N, T> w(*this);
    w /= a;
    return w;
  }
};

typedef Vec<2, float> v2f;
typedef Vec<2, int> v2i;

typedef Vec<3, float> v3f;
typedef Vec<3, int> v3i;

template <class T> inline T sqr(const T &x) { return x * x; }

template <unsigned int N, class T> T mag2(const Vec<N, T> &a) {
  T l = sqr(a.v[0]);
  for (unsigned int i = 1; i < N; ++i)
    l += sqr(a.v[i]);
  return l;
}

template <unsigned int N, class T> T mag(const Vec<N, T> &a) {
  return static_cast<T>(sqrt(static_cast<double>(mag2(a))));
}

template <unsigned int N, class T> inline Vec<N, T> normalize(Vec<N, T> &a) {
  T m = mag(a);
  return m == 0 ? a : a / m;
}

template <unsigned int N, class T>
std::ostream &operator<<(std::ostream &out, const Vec<N, T> &v) {
  out << v.v[0];
  for (unsigned int i = 1; i < N; ++i)
    out << ' ' << v.v[i];
  return out;
}

template <unsigned int N, class T>
std::istream &operator>>(std::istream &in, Vec<N, T> &v) {
  in >> v.v[0];
  for (unsigned int i = 1; i < N; ++i)
    in >> v.v[i];
  return in;
}

template <unsigned int N, class T>
inline bool operator==(const Vec<N, T> &a, const Vec<N, T> &b) {
  bool t = (a.v[0] == b.v[0]);
  unsigned int i = 1;
  while (i < N && t) {
    t = t && (a.v[i] == b.v[i]);
    ++i;
  }
  return t;
}

template <unsigned int N, class T>
inline bool operator!=(const Vec<N, T> &a, const Vec<N, T> &b) {
  bool t = (a.v[0] != b.v[0]);
  unsigned int i = 1;
  while (i < N && !t) {
    t = t || (a.v[i] != b.v[i]);
    ++i;
  }
  return t;
}

template <unsigned int N, class T>
inline Vec<N, T> operator*(T a, const Vec<N, T> &v) {
  Vec<N, T> w(v);
  w *= a;
  return w;
}

template <unsigned int N, class T> inline T min(const Vec<N, T> &a) {
  T m = a.v[0];
  for (unsigned int i = 1; i < N; ++i)
    if (a.v[i] < m)
      m = a.v[i];
  return m;
}

template <unsigned int N, class T> inline T max(const Vec<N, T> &a) {
  T m = a.v[0];
  for (unsigned int i = 1; i < N; ++i)
    if (a.v[i] > m)
      m = a.v[i];
  return m;
}

template <unsigned int N, class T>
inline Vec<N, T> min_union(const Vec<N, T> &a, const Vec<N, T> &b) {
  Vec<N, T> m;
  for (unsigned int i = 0; i < N; ++i)
    (a.v[i] < b.v[i]) ? m.v[i] = a.v[i] : m.v[i] = b.v[i];
  return m;
}

template <unsigned int N, class T>
inline Vec<N, T> max_union(const Vec<N, T> &a, const Vec<N, T> &b) {
  Vec<N, T> m;
  for (unsigned int i = 0; i < N; ++i)
    (a.v[i] > b.v[i]) ? m.v[i] = a.v[i] : m.v[i] = b.v[i];
  return m;
}

template <unsigned int N, class T>
inline T dot(const Vec<N, T> &a, const Vec<N, T> &b) {
  T d = a.v[0] * b.v[0];
  for (unsigned int i = 1; i < N; ++i)
    d += a.v[i] * b.v[i];
  return d;
}

template <class T> inline T cross(const Vec<2, T> &a, const Vec<2, T> &b) {
  return a.v[0] * b.v[1] - a.v[1] * b.v[0];
}

template <class T>
inline Vec<3, T> cross(const Vec<3, T> &a, const Vec<3, T> &b) {
  return Vec<3, T>(a.v[1] * b.v[2] - a.v[2] * b.v[1],
                   a.v[2] * b.v[0] - a.v[0] * b.v[2],
                   a.v[0] * b.v[1] - a.v[1] * b.v[0]);
}

inline v2f v2ffloor(v2f vec) {
  return v2f(static_cast<int>(floor(vec[0])), static_cast<int>(floor(vec[1])));
}

inline v2f v2ffloor(v2i vec) {
  return v2f(static_cast<int>(floor(vec[0])), static_cast<int>(floor(vec[1])));
}

inline v2i v2ifloor(v2f vec) {
  return v2i(static_cast<int>(floor(vec[0])), static_cast<int>(floor(vec[1])));
}

inline v2f floor_midpoint(v2f a) { return v2ffloor(a * 0.5f); }

inline v2f floor_midpoint(v2f a, v2f b) { return floor_midpoint(a + b); }

inline v2i floor_midpoint(v2i a, v2i b) {
  return v2ifloor((v2f(a[0], a[1]) + v2f(b[0], b[1])) * 0.5f);
}

// REMOVE ME
inline v2f v2i_to_v2f(v2i v) {
  return v2f(static_cast<float>(v[0]), static_cast<float>(v[1]));
}

inline v2i v2f_to_v2i(v2f v) {
  return v2i(static_cast<int>(v[0]), static_cast<int>(v[1]));
}

#endif // VEC_H
