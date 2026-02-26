#pragma once
#include <array>
#include <cmath>
struct Vec3
{
    Vec3(std::array<float, 3> arr) : _vec(arr) {};
    Vec3(float x, float y, float z) : _vec{x, y, z} {};
    Vec3 operator+(const Vec3 &other) const
    {
        return {x() + other.x(), y() + other.y(), z() + other.z()};
    }
    Vec3 operator-(const Vec3 &other) const
    {
        return {x() - other.x(), y() - other.y(), z() - other.z()};
    }
    Vec3 operator*(float scalar) const
    {
        return {x() * scalar, y() * scalar, z() * scalar};
    }
    float abs() const
    {
        return std::sqrt(x() * x() + y() * y() + z() * z());
    }
    Vec3 operator/(float scalar) const
    {
        return {x() / scalar, y() / scalar, z() / scalar};
    }
    inline float x() const { return _vec[0]; }
    inline float y() const { return _vec[1]; }
    inline float z() const { return _vec[2]; }

    std::array<float, 3> _vec;
};

struct Quaternion
{
    Quaternion operator+(const Quaternion &other) const
    {
        return {w() + other.w(), x() + other.x(), y() + other.y(), z() + other.z()};
    }
    Quaternion operator*(float scalar) const
    {
        return {w() * scalar, x() * scalar, y() * scalar, z() * scalar};
    }
    Quaternion operator*(const Quaternion &other) const
    {
        return {
            w() * other.w() - x() * other.x() - y() * other.y() - z() * other.z(),
            w() * other.x() + x() * other.w() + y() * other.z() - z() * other.y(),
            w() * other.y() - x() * other.z() + y() * other.w() + z() * other.x(),
            w() * other.z() + x() * other.y() - y() * other.x() + z() * other.w()};
    }
    inline float w() const { return quat[0]; }
    inline float x() const { return quat[1]; }
    inline float y() const { return quat[2]; }
    inline float z() const { return quat[3]; }
    std::array<float, 4> quat;
};