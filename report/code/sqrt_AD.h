AD sqrt(const AD &x) {
  using std::sqrt;
  double sqrtx = sqrt(x.value());
  return AD(sqrtx, x.derivatives() * (double(0.5) / sqrtx));
}

