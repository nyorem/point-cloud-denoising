#ifndef CGAL_AD_H

#include "AD.hpp"
#include <CGAL/Gmpq.h>
#include <CGAL/NT_converter.h>

namespace CGAL
{
  template <> class Algebraic_structure_traits<AD>
    : public Algebraic_structure_traits_base<AD, Field_tag >
  {
  public:
    typedef Tag_false            Is_exact;
    typedef Tag_true             Is_numerical_sensitive;
  };

  template <>
  class Real_embeddable_traits< AD >
    : public INTERN_RET::Real_embeddable_traits_base< AD , CGAL::Tag_true>
  {
  public:
    class To_double : public std::unary_function<AD, double >
    {
    public:
      double operator()( const AD& x ) const {
    	return x.value();
      }
    };
    class To_interval
      : public std::unary_function< AD, std::pair<double,double> > {
    public:
      std::pair<double,double> operator()( const AD& x ) const {
	double dx(x.value());
	return std::make_pair(dx,dx);
      }
    };

  };

  template <>
  struct NT_converter <AD, Gmpq>
    : public std::unary_function<AD, Gmpq>
  {
    Gmpq
    operator()(const AD &a) const
    {
      return Gmpq(a.value());
    }
  };

}

#endif
