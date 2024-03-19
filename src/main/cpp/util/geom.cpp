#include "util/geom.h"

Vector operator-(const Point &lhs, const Point &rhs) {
    return Vector(lhs.x - rhs.x, lhs.y - rhs.y);
}

Point operator+(const Point &lhs, const Vector &rhs) {
    return Point{
        lhs.x + rhs.x,
        lhs.y + rhs.y
    };
}

Point operator-(const Point &lhs, const Vector &rhs) {
    return Point{
        lhs.x - rhs.x,
        lhs.y - rhs.y
    };
}

Vector operator*(const Vector &vector, double scalar) {
    return Vector(vector.x * scalar, vector.y * scalar);
}

Vector operator*(double scalar, const Vector &vector) {
    return Vector(vector.x * scalar, vector.y * scalar);
}