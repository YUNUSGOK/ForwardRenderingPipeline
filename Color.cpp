#include "Color.h"
#include <iostream>
#include <iomanip>

using namespace std;

Color::Color() {}

Color::Color(double r, double g, double b)
{
    this->r = r;
    this->g = g;
    this->b = b;
}

Color::Color(const Color &other)
{
    this->r = other.r;
    this->g = other.g;
    this->b = other.b;
}

ostream& operator<<(ostream& os, const Color& c)
{
    os << fixed << setprecision(0) << "rgb(" << c.r << ", " << c.g << ", " << c.b << ")";
    return os;
}
Color Color::operator+(Color other){
  Color c(r+other.r,g+other.g,b+other.b);
  return c;
}

Color Color::operator-(Color other){
  Color c(r-other.r,g-other.g,b-other.b);
  return c;
}

Color Color::operator*(double delta)
{
  Color c(r*delta,g*delta,b*delta);
  return c;
}
Color Color::operator/(double delta)
{
  Color c(r/delta,g/delta,b/delta);
  return c;
}
Color Color::clippedColor(){
  Color c((int)(r),(int)(g),(int)(b));
  return c;
}
