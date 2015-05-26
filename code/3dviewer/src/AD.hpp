
// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Modified by Q. Merigot: removed expression templates for
// compatibility with CGAL

#ifndef AUTODIFF_SCALAR_H
#define AUTODIFF_SCALAR_H
#include <Eigen/Sparse>

class AD
{
  typedef double Scalar;
  typedef Eigen::SparseVector<double> Vector;
  Scalar m_value;
  Vector m_derivatives;

 public:
  /** Default constructor without any initialization. */
  AD() {}

  /** Constructs an active scalar from its \a value,
      and initializes the \a nbDer derivatives such that it corresponds to the \a derNumber -th variable */
 AD(const Scalar& value, int nbDer, int derNumber)
   : m_value(value), m_derivatives(Vector(nbDer))
    {
      m_derivatives.coeffRef(derNumber) = Scalar(1);
    }
  
  /** Conversion from a scalar constant to an active scalar.
   * The derivatives are set to zero. */
  /*explicit*/
 AD(const Scalar& value)
    : m_value(value)
  {
    if(m_derivatives.size()>0)
      m_derivatives.setZero();
  }

  AD(const Scalar& value, int nbDer)
    : m_value(value), m_derivatives(Vector(nbDer))
  {
  }

  /** Constructs an active scalar from its \a value and derivatives \a der */
 AD(const Scalar& value, const Vector& der)
   : m_value(value), m_derivatives(der)
  {}
  
  friend inline
    std::ostream & operator << (std::ostream & s, const AD& a)
  {
    return s << a.value();
  }
  
 AD(const AD& other)
   : m_value(other.value()), m_derivatives(other.derivatives())
    {}

  inline AD& operator=(const AD& other)
    {
      m_value = other.value();
      m_derivatives = other.derivatives();
      return *this;
    }
    
  inline const Scalar& value() const { return m_value; }
  inline Scalar& value() { return m_value; }
  
  inline const Vector& derivatives() const { return m_derivatives; }
  inline Vector& derivatives() { return m_derivatives; }
  
  inline bool operator< (const Scalar& other) const 
  { return m_value <  other; }
  inline bool operator< (const AD& other) const  
  { return m_value <  other.value(); }
  inline bool operator<=(const Scalar& other) const  
  { return m_value <= other; }
  inline bool operator<= (const AD& other) const  
  { return m_value <= other.value(); }
  inline bool operator> (const Scalar& other) const  
  { return m_value >  other; }
  inline bool operator> (const AD& other) const  
  { return m_value >  other.value(); }
  inline bool operator>=(const Scalar& other) const  
  { return m_value >= other; }
  inline bool operator>= (const AD& other) const  
  { return m_value >= other.value(); }
  inline bool operator==(const Scalar& other) const  
  { return m_value == other; }
  inline bool operator==(const AD& other) const  
  { return m_value == other; }
  inline bool operator!=(const Scalar& other) const  
  { return m_value != other; }
  inline bool operator!=(const AD& other) const  
  { return m_value != other; }
  
  friend inline bool operator< (const Scalar& a, const AD& b) 
  { return a <  b.value(); }
  friend inline bool operator<=(const Scalar& a, const AD& b) 
  { return a <= b.value(); }
  friend inline bool operator> (const Scalar& a, const AD& b) 
  { return a >  b.value(); }
  friend inline bool operator>=(const Scalar& a, const AD& b) 
  { return a >= b.value(); }
  friend inline bool operator==(const Scalar& a, const AD& b) 
  { return a == b.value(); }
  friend inline bool operator!=(const Scalar& a, const AD& b) 
  { return a != b.value(); }

  inline AD operator+(const Scalar& other) const
  {
    return AD(m_value + other, m_derivatives);
  }

  friend inline AD operator+(const Scalar& a, const AD& b)
  {
    return AD(a + b.value(), b.derivatives());
  }

  inline AD& operator+=(const Scalar& other)
  {
    value() += other;
    return *this;
  }

  AD operator+(const AD& other) const
  {
    return AD(m_value + other.value(),
	      m_derivatives + other.derivatives());
  }

  AD& operator+=(const AD& other)
  {
    (*this) = (*this) + other;
    return *this;
  }

  AD operator-(const Scalar& b) const
  {
    return AD(m_value - b, m_derivatives);
  }

  friend inline AD operator-(const Scalar& a, const AD& b)
  {
    return AD(a - b.value(), -b.derivatives());
  }

  AD& operator-=(const Scalar& other)
  {
    value() -= other;
    return *this;
  }

  AD operator-(const AD& other) const
  {
    return AD(m_value - other.value(),
	      m_derivatives - other.derivatives());
  }
  
  AD& operator-=(const AD& other)
  {
    *this = *this - other;
    return *this;
  }

  AD operator-() const
  {
    return AD(-m_value, -m_derivatives);
  }
  
  AD operator*(const Scalar& other) const
  {
    return AD(m_value * other, (m_derivatives * other));
  }

  friend inline AD operator*(const Scalar& other, const AD& a)
  {
    return AD(a.value() * other,
	      a.derivatives() * other);
  }

  AD operator/(const Scalar& other) const
  {
    return AD(m_value / other, (m_derivatives * (Scalar(1)/other)));
  }

  friend inline AD operator/(const Scalar& other, const AD& a)
  {
    return AD(other / a.value(),
	      a.derivatives() *
	      (Scalar(-other) / (a.value()*a.value())));
  }

  AD operator/(const AD& other) const
  {
    return AD(m_value / other.value(),
	      ((m_derivatives * other.value()) - 
	       (m_value * other.derivatives())) * 
	      (Scalar(1)/(other.value()*other.value())));
  }

  AD operator*(const AD& other) const
  {
    return AD(m_value * other.value(),
	      (m_derivatives * other.value()) +
	      (m_value * other.derivatives()));
  }

  inline AD& operator*=(const Scalar& other)
  {
    *this = *this * other;
    return *this;
  }

  inline AD& operator*=(const AD& other)
  {
    *this = *this * other;
    return *this;
  }

  inline AD& operator/=(const Scalar& other)
  {
    *this = *this / other;
    return *this;
  }

  inline AD& operator/=(const AD& other)
  {
    *this = *this / other;
    return *this;
  }
};

static AD sqrt(const AD &x)
{
    using std::sqrt;
    double sqrtx = sqrt(x.value());
    return AD(sqrtx,x.derivatives() * (double(0.5) / sqrtx));
}

static AD atan2(const AD &y, const AD& x)
{
    using std::atan2;
    double atan2yx = atan2(y.value(), x.value());

    double tmp2 = y.value() * y.value();
    double tmp3 = x.value() * x.value();
    double tmp4 = tmp3/(tmp2+tmp3);

    /* if (tmp4 != 0) { */
    return AD(atan2yx,
              (y.derivatives() * x.value() - y.value() * x.derivatives()) / (tmp2+tmp3));
    /* } */

    /* return AD(atan2yx); */
}

#endif // EIGEN_AUTODIFF_SCALAR_H
