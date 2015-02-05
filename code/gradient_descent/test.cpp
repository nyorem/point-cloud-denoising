#include <iostream>
#include <cmath>

class fpad // forward propagating automatic differentiation
{
  double value_;
  double deriv_;
public:
  fpad(double v, double d=0) : value_(v), deriv_(d) {}

  double value() const {return value_;}
  double derivative() const {return deriv_;}

  const fpad& equalsTransform(double newVal, double outer_deriv)
  { deriv_ = deriv_ * outer_deriv; // Chain rule
    value_ = newVal;
    return *this;
  }

  const fpad& operator+=(fpad const& x)
  { value_ += x.value_;
    deriv_ += x.deriv_;
    return *this;
  }

  const fpad& operator-=(fpad const& x)
  { value_ -= x.value_;
    deriv_ -= x.deriv_;
    return *this;
  }

  const fpad& operator*=(fpad const& x)
  {
    deriv_ = deriv_ * x.value_ + value_ * x.deriv_;
    value_ *= x.value_;
    return *this;
  }

  friend const fpad operator+(fpad const& a, fpad const& b)
  { fpad r(a); r+=b; return r; }

  friend const fpad operator-(fpad const& a, fpad const& b)
  { fpad r(a); r-=b; return r; }

  friend const fpad operator*(fpad const& a, fpad const& b)
  { fpad r(a); r*=b; return r; }
};

fpad sin(fpad t)
{
  using std::sin;
  using std::cos;
  double v = t.value();
  t.equalsTransform(sin(v),cos(v)); // Chain
  return t;
}

int main()
{
  fpad x (0.123, 1.0); // Identitaet für x=0.123
                       // 1.0 ist die derivative der Identitaet
  fpad y = x*x - 3*sin(x) + 2;
  std::cout << "f(x) = x*x - 3*sin(x) + 2\n"
               "fuer x=0.123 ist f (x) = " << y.value() << "\n"
               "             ist f'(x) = " << y.derivative() << "\n";
}
