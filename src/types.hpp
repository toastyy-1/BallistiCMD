#pragma once

#include <cmath>

struct Vec3 {
    double x, y, z;

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s)      const { return {x * s,   y * s,   z * s};   }
    Vec3 operator/(double s)      const { return {x / s,   y / s,   z / s};   }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
    Vec3& operator/=(double s) { x /= s; y /= s; z /= s; return *this; }
    Vec3 operator-() const { return {-x, -y, -z}; }

    double dot(const Vec3& o)  const { return x*o.x + y*o.y + z*o.z; }
    Vec3   vector_individual_multiply(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3   cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    double norm() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const {
        double mag = norm();
        return {x / mag, y / mag, z / mag};
    }
};

struct Mat4 {
    double m[4][4] = {};

    Mat4 operator*(const Mat4& B) const {
        Mat4 C;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 4; k++)
                    C.m[i][j] += m[i][k] * B.m[k][j];
        return C;
    }
};

struct Quat {
    double w, x, y, z;

    double norm() const { return std::sqrt(w*w + x*x + y*y + z*z); }
    Quat normalize() const {
        double mag = norm();
        return {w / mag, x / mag, y / mag, z / mag};
    }

    Quat operator*(const Quat& o) const {
        return {
            w*o.w - x*o.x - y*o.y - z*o.z,
            w*o.x + x*o.w + y*o.z - z*o.y,
            w*o.y - x*o.z + y*o.w + z*o.x,
            w*o.z + x*o.y - y*o.x + z*o.w,
        };
    }

    Quat conjugate() const { return {w, -x, -y, -z}; }
    Quat inverse() const {
        double n2 = w*w + x*x + y*y + z*z;
        return {w / n2, -x / n2, -y / n2, -z / n2};
    }
};

inline Quat operator*(const Mat4& M, const Quat& q) {
    return {
        M.m[0][0]*q.w + M.m[0][1]*q.x + M.m[0][2]*q.y + M.m[0][3]*q.z,
        M.m[1][0]*q.w + M.m[1][1]*q.x + M.m[1][2]*q.y + M.m[1][3]*q.z,
        M.m[2][0]*q.w + M.m[2][1]*q.x + M.m[2][2]*q.y + M.m[2][3]*q.z,
        M.m[3][0]*q.w + M.m[3][1]*q.x + M.m[3][2]*q.y + M.m[3][3]*q.z,
    };
}