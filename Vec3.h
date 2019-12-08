#ifndef __VEC3_H__
#define __VEC3_H__

#include "Vec4.h"
#include <iostream>
using namespace std;

class Vec3
{
public:
    double x, y, z;
    int colorId;

    Vec3();
    Vec3(double x, double y, double z, int colorId);
    Vec3(const Vec3 &other);

    Vec3 operator-();
    Vec3 operator+(Vec3 &v);
    double getElementAt(int index);

    friend std::ostream& operator<<(std::ostream& os, const Vec3& v);
};

#endif
