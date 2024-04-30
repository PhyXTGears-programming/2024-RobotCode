#pragma once

#include "util/point.h"
#include "util/vector.h"

Vector operator-(const Point &lhs, const Point &rhs);

Point operator+(const Point &lhs, const Vector &rhs);
Point operator-(const Point &lhs, const Vector &rhs);

Vector operator*(const Vector &vector, double scalar);
Vector operator*(double scalar, const Vector &vector);