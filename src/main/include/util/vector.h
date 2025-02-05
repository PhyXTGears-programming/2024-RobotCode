#pragma once 

#include "util/math.h"

#include <cmath>

class Vector {
public:
    Vector(double x, double y) : x(x), y(y) {}
    Vector() { //heehoo default constructor 
        x = 0;
        y = 0;

    }

    double dot(Vector & rhs) {return (x * rhs.x) + (y *rhs.y); }

    double len() { return std::sqrt(x * x + y * y); }

    Vector unit() {
        double m = len();

        if (m < 0.0001) {
            return Vector(1.0, 0.0);
        } else {
            return Vector(x / m, y / m);
        }
    }

    double x;
    double y;
};

