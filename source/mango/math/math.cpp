/*
    MANGO Multimedia Development Platform
    Copyright (C) 2012-2024 Twilight Finland 3D Oy Ltd. All rights reserved.
*/

#include <limits>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <mango/core/bits.hpp>
#include <mango/math/math.hpp>

namespace mango::math
{

    static constexpr float epsilon = std::numeric_limits<float>::epsilon();

    // ------------------------------------------------------------------------
    // Matrix3x3
    // ------------------------------------------------------------------------

    const Matrix3x3& Matrix3x3::operator = (float s)
    {
        m[0] = float32x3(s, 0, 0);
        m[1] = float32x3(0, s, 0);
        m[2] = float32x3(0, 0, s);
        return *this;
    }

    const Matrix3x3& Matrix3x3::operator = (const float* ptr)
    {
        m[0][0] = ptr[0];
        m[0][1] = ptr[1];
        m[0][2] = ptr[2];
        m[1][0] = ptr[3];
        m[1][1] = ptr[4];
        m[1][2] = ptr[5];
        m[2][0] = ptr[6];
        m[2][1] = ptr[7];
        m[2][2] = ptr[8];
        return *this;
    }

    const Matrix3x3& Matrix3x3::operator = (const Quaternion& q)
    {
        const float x2 = q.x * 2.0f;
        const float y2 = q.y * 2.0f;
        const float z2 = q.z * 2.0f;

        const float wx = q.w * x2;
        const float wy = q.w * y2;
        const float wz = q.w * z2;

        const float xx = q.x * x2;
        const float xy = q.x * y2;
        const float xz = q.x * z2;

        const float yy = q.y * y2;
        const float yz = q.y * z2;
        const float zz = q.z * z2;

        m[0] = float32x3(1.0f - yy - zz, xy + wz, xz - wy);
        m[1] = float32x3(xy - wz, 1.0f - xx - zz, yz + wx);
        m[2] = float32x3(xz + wy, yz - wx, 1.0f - xx - yy);

        return *this;
    }

    const Matrix3x3& Matrix3x3::operator = (const AngleAxis& a)
    {
        float length2 = square(a.axis);
        if (length2 < epsilon)
        {
            *this = 1.0f; // set identity
            return *this;
        }

        float s = std::sin(a.angle);
        float c = std::cos(a.angle);
        float k = 1.0f - c;

        float length = 1.0f / std::sqrt(length2);

        float x = a.axis.x * length;
        float y = a.axis.y * length;
        float z = a.axis.z * length;

        float xk = x * k;
        float yk = y * k;
        float zk = z * k;

        float xy = x * yk;
        float yz = y * zk;
        float zx = z * xk;
        float xs = x * s;
        float ys = y * s;
        float zs = z * s;

        m[0] = float32x3(x * xk + c, xy + zs, zx - ys);
        m[1] = float32x3(xy - zs, y * yk + c, yz + xs);
        m[2] = float32x3(zx + ys, yz - xs, z * zk + c);

        return *this;
    }

    const Matrix3x3& Matrix3x3::operator = (const EulerAngles& euler)
    {
        // use vectorized sin / cos
        const float32x4 v = float32x4(euler.x, euler.y, euler.z, 0.0f);
        const float32x4 s = sin(v);
        const float32x4 c = cos(v);

        const float sx = s.x;
        const float sy = s.y;
        const float sz = s.z;
        const float cx = c.x;
        const float cy = c.y;
        const float cz = c.z;
        const float sysx = sy * sx;
        const float sycx = sy * cx;

        m[0] = float32x3(cz * cy, sz * cy, -sy);
        m[1] = float32x3(cz * sysx - sz * cx, sz * sysx + cz * cx, cy * sx);
        m[2] = float32x3(cz * sycx + sz * sx, sz * sycx - cz * sx, cy * cx);

        return *this;
    }

    float Matrix3x3::determinant2x2() const
    {
        return m[0][0] * m[1][1] - m[1][0] * m[0][1];
    }

    float Matrix3x3::determinant3x3() const
    {
        return m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
               m[0][1] * (m[1][0] * m[2][2] - m[2][0] * m[1][2]) +
               m[0][2] * (m[1][0] * m[2][1] - m[2][0] * m[1][1]);
    }

    Matrix3x3 Matrix3x3::identity()
    {
        return Matrix3x3(1.0f);
    }

    Matrix3x3 Matrix3x3::scale(float s)
    {
        Matrix3x3 m;
        m[0] = float32x3(s, 0, 0);
        m[1] = float32x3(0, s, 0);
        m[2] = float32x3(0, 0, s);
        return m;
    }

    Matrix3x3 Matrix3x3::scale(float x, float y, float z)
    {
        Matrix3x3 m;
        m[0] = float32x3(x, 0, 0);
        m[1] = float32x3(0, y, 0);
        m[2] = float32x3(0, 0, z);
        return m;
    }

    Matrix3x3 Matrix3x3::rotate(float angle, const float32x3& axis)
    {
        const Matrix3x3 m = AngleAxis(angle, axis);
        return m;
    }

    Matrix3x3 Matrix3x3::rotateX(float angle)
    {
        const float s = std::sin(angle);
        const float c = std::cos(angle);

        Matrix3x3 m;
        m[0] = float32x3(1, 0, 0);
        m[1] = float32x3(0, c, s);
        m[2] = float32x3(0,-s, c);
        return m;
    }

    Matrix3x3 Matrix3x3::rotateY(float angle)
    {
        const float s = std::sin(angle);
        const float c = std::cos(angle);

        Matrix3x3 m;
        m[0] = float32x3(c, 0,-s);
        m[1] = float32x3(0, 1, 0);
        m[2] = float32x3(s, 0, c);
        return m;
     }

    Matrix3x3 Matrix3x3::rotateZ(float angle)
    {
        const float s = std::sin(angle);
        const float c = std::cos(angle);

        Matrix3x3 m;
        m[0] = float32x3( c, s, 0);
        m[1] = float32x3(-s, c, 0);
        m[2] = float32x3( 0, 0, 1);
        return m;
    }

    Matrix3x3 Matrix3x3::rotateXYZ(float x, float y, float z)
    {
        return Matrix3x3(EulerAngles(x, y, z));
    }

    Matrix3x3 scale(const Matrix3x3& input, float s)
    {
        const float32x3 v = float32x3(s, s, s);

        Matrix3x3 m;
        m[0] = input[0] * v;
        m[1] = input[1] * v;
        m[2] = input[2] * v;
        return m;
    }

    Matrix3x3 scale(const Matrix3x3& input, float x, float y, float z)
    {
        const float32x3 v = float32x3(x, y, z);

        Matrix3x3 m;
        m[0] = input[0] * v;
        m[1] = input[1] * v;
        m[2] = input[2] * v;
        return m;
    }

    Matrix3x3 rotate(const Matrix3x3& input, float angle, const float32x3& axis)
    {
        const Matrix3x3 temp = AngleAxis(angle, axis);
        return input * temp;
    }

    Matrix3x3 rotateX(const Matrix3x3& input, float angle)
    {
        const float s = std::sin(angle);
        const float c = std::cos(angle);

        Matrix3x3 m;

        for (int i = 0; i < 4; ++i)
        {
            const float x = input[i][0];
            const float y = input[i][1];
            const float z = input[i][2];
            m[i][0] = x;
            m[i][1] = y * c - z * s;
            m[i][2] = z * c + y * s;
        }

        return m;
    }

    Matrix3x3 rotateY(const Matrix3x3& input, float angle)
    {
        const float s = std::sin(angle);
        const float c = std::cos(angle);

        Matrix3x3 m;

        for (int i = 0; i < 4; ++i)
        {
            const float x = input[i][0];
            const float y = input[i][1];
            const float z = input[i][2];
            m[i][0] = x * c + z * s;
            m[i][1] = y;
            m[i][2] = z * c - x * s;
        }

        return m;
    }

    Matrix3x3 rotateZ(const Matrix3x3& input, float angle)
    {
        const float s = std::sin(angle);
        const float c = std::cos(angle);

        Matrix3x3 m;

        for (int i = 0; i < 4; ++i)
        {
            const float x = input[i][0];
            const float y = input[i][1];
            const float z = input[i][2];
            m[i][0] = x * c - y * s;
            m[i][1] = y * c + x * s;
            m[i][2] = z;
        }

        return m;
    }

    Matrix3x3 rotateXYZ(const Matrix3x3& input, float x, float y, float z)
    {
        const Matrix3x3 temp = Matrix3x3::rotateXYZ(x, y, z);
        return input * temp;
    }

    Matrix3x3 normalize(const Matrix3x3& input)
    {
        float32x3 x = input[0];
        float32x3 y = input[1];
        float32x3 z = input[2];
        x = normalize(x);
        y = normalize(y - x * dot(x, y));
        z = cross(x, y);
        return Matrix3x3(x, y, z);
    }

    // ------------------------------------------------------------------------
    // Matrix4x4
    // ------------------------------------------------------------------------

    const Matrix4x4& Matrix4x4::operator = (float s)
    {
        m[0] = float32x4(s, 0, 0, 0);
        m[1] = float32x4(0, s, 0, 0);
        m[2] = float32x4(0, 0, s, 0);
        m[3] = float32x4(0, 0, 0, 1);
        return *this;
    }

    const Matrix4x4& Matrix4x4::operator = (const float* ptr)
    {
        m[0] = simd::f32x4_uload(ptr + 0);
        m[1] = simd::f32x4_uload(ptr + 4);
        m[2] = simd::f32x4_uload(ptr + 8);
        m[3] = simd::f32x4_uload(ptr + 12);
        return *this;
    }

    const Matrix4x4& Matrix4x4::operator = (const Quaternion& q)
    {
        const float x2 = q.x * 2.0f;
        const float y2 = q.y * 2.0f;
        const float z2 = q.z * 2.0f;

        const float wx = q.w * x2;
        const float wy = q.w * y2;
        const float wz = q.w * z2;

        const float xx = q.x * x2;
        const float xy = q.x * y2;
        const float xz = q.x * z2;

        const float yy = q.y * y2;
        const float yz = q.y * z2;
        const float zz = q.z * z2;

        m[0] = float32x4(1.0f - yy - zz, xy + wz, xz - wy, 0.0f);
        m[1] = float32x4(xy - wz, 1.0f - xx - zz, yz + wx, 0.0f);
        m[2] = float32x4(xz + wy, yz - wx, 1.0f - xx - yy, 0.0f);
        m[3] = float32x4(0.0f, 0.0f, 0.0f, 1.0f);

        return *this;
    }

    const Matrix4x4& Matrix4x4::operator = (const AngleAxis& a)
    {
        float length2 = square(a.axis);
        if (length2 < epsilon)
        {
            *this = 1.0f; // set identity
            return *this;
        }

        float s = std::sin(a.angle);
        float c = std::cos(a.angle);
        float k = 1.0f - c;

        float length = 1.0f / std::sqrt(length2);

        float x = a.axis.x * length;
        float y = a.axis.y * length;
        float z = a.axis.z * length;

        float xk = x * k;
        float yk = y * k;
        float zk = z * k;

        float xy = x * yk;
        float yz = y * zk;
        float zx = z * xk;
        float xs = x * s;
        float ys = y * s;
        float zs = z * s;

        m[0] = float32x4(x * xk + c, xy + zs, zx - ys, 0.0f);
        m[1] = float32x4(xy - zs, y * yk + c, yz + xs, 0.0f);
        m[2] = float32x4(zx + ys, yz - xs, z * zk + c, 0.0f);
        m[3] = float32x4(0.0f, 0.0f, 0.0f, 1.0f);

        return *this;
    }

    const Matrix4x4& Matrix4x4::operator = (const EulerAngles& euler)
    {
        const float32x4 v = float32x4(euler.x, euler.y, euler.z, 0.0f);
        const float32x4 s = sin(v);
        const float32x4 c = cos(v);

        const float sx = s.x;
        const float sy = s.y;
        const float sz = s.z;
        const float cx = c.x;
        const float cy = c.y;
        const float cz = c.z;
        const float sysx = sy * sx;
        const float sycx = sy * cx;

        m[0] = float32x4(cz * cy, sz * cy, -sy, 0);
        m[1] = float32x4(cz * sysx - sz * cx, sz * sysx + cz * cx, cy * sx, 0);
        m[2] = float32x4(cz * sycx + sz * sx, sz * sycx - cz * sx, cy * cx, 0);
        m[3] = float32x4(0, 0, 0, 1);

        return *this;
    }

    bool Matrix4x4::isAffine() const
    {
        float32x4 c = column<3>();
        return all_of(c == float32x4(0, 0, 0, 1));
    }

    float Matrix4x4::determinant2x2() const
    {
        return m[0][0] * m[1][1] - m[1][0] * m[0][1];
    }

    float Matrix4x4::determinant3x3() const
    {
        return m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
               m[0][1] * (m[1][0] * m[2][2] - m[2][0] * m[1][2]) +
               m[0][2] * (m[1][0] * m[2][1] - m[2][0] * m[1][1]);
    }

    float Matrix4x4::determinant4x4() const
    {
        float s0 = m[1][1] * (m[2][2] * m[3][3] - m[3][2] * m[2][3]) -
                   m[1][2] * (m[2][1] * m[3][3] - m[3][1] * m[2][3]) +
                   m[1][3] * (m[2][1] * m[3][2] - m[3][1] * m[2][2]);
        float s1 = m[1][0] * (m[2][2] * m[3][3] - m[3][2] * m[2][3]) -
                   m[1][2] * (m[2][0] * m[3][3] - m[3][0] * m[2][3]) +
                   m[1][3] * (m[2][0] * m[3][2] - m[3][0] * m[2][2]);
        float s2 = m[1][0] * (m[2][1] * m[3][3] - m[3][1] * m[2][3]) -
                   m[1][1] * (m[2][0] * m[3][3] - m[3][0] * m[2][3]) +
                   m[1][3] * (m[2][0] * m[3][1] - m[3][0] * m[2][1]);
        float s3 = m[1][0] * (m[2][1] * m[3][2] - m[3][1] * m[2][2]) -
                   m[1][1] * (m[2][0] * m[3][2] - m[3][0] * m[2][2]) +
                   m[1][1] * (m[2][0] * m[3][1] - m[3][0] * m[2][1]);
        return m[0][0] * s0 - m[0][1] * s1 + m[0][2] * s2 - m[0][3] * s3;
    }

    Matrix4x4 Matrix4x4::identity()
    {
        return Matrix4x4(1.0f);
    }

    Matrix4x4 Matrix4x4::translate(float x, float y, float z)
    {
        Matrix4x4 m;
        m[0] = float32x4(1, 0, 0, 0);
        m[1] = float32x4(0, 1, 0, 0);
        m[2] = float32x4(0, 0, 1, 0);
        m[3] = float32x4(x, y, z, 1);
        return m;
    }

    Matrix4x4 Matrix4x4::scale(float s)
    {
        Matrix4x4 m;
        m[0] = float32x4(s, 0, 0, 0);
        m[1] = float32x4(0, s, 0, 0);
        m[2] = float32x4(0, 0, s, 0);
        m[3] = float32x4(0, 0, 0, 1);
        return m;
    }

    Matrix4x4 Matrix4x4::scale(float x, float y, float z)
    {
        Matrix4x4 m;
        m[0] = float32x4(x, 0, 0, 0);
        m[1] = float32x4(0, y, 0, 0);
        m[2] = float32x4(0, 0, z, 0);
        m[3] = float32x4(0, 0, 0, 1);
        return m;
    }

    Matrix4x4 Matrix4x4::rotate(float angle, const float32x3& axis)
    {
        const Matrix4x4 m = AngleAxis(angle, axis);
        return m;
    }

    Matrix4x4 Matrix4x4::rotateX(float angle)
    {
        const float s = std::sin(angle);
        const float c = std::cos(angle);

        Matrix4x4 m;
        m[0] = float32x4(1, 0, 0, 0);
        m[1] = float32x4(0, c, s, 0);
        m[2] = float32x4(0,-s, c, 0);
        m[3] = float32x4(0, 0, 0, 1);
        return m;
    }

    Matrix4x4 Matrix4x4::rotateY(float angle)
    {
        const float s = std::sin(angle);
        const float c = std::cos(angle);

        Matrix4x4 m;
        m[0] = float32x4(c, 0,-s, 0);
        m[1] = float32x4(0, 1, 0, 0);
        m[2] = float32x4(s, 0, c, 0);
        m[3] = float32x4(0, 0, 0, 1);
        return m;
     }

    Matrix4x4 Matrix4x4::rotateZ(float angle)
    {
        const float s = std::sin(angle);
        const float c = std::cos(angle);

        Matrix4x4 m;
        m[0] = float32x4( c, s, 0, 0);
        m[1] = float32x4(-s, c, 0, 0);
        m[2] = float32x4( 0, 0, 1, 0);
        m[3] = float32x4( 0, 0, 0, 1);
        return m;
    }

    Matrix4x4 Matrix4x4::rotateXYZ(float x, float y, float z)
    {
        return Matrix4x4(EulerAngles(x, y, z));
    }

    Matrix4x4 Matrix4x4::lookat(const float32x3& target, const float32x3& viewer, const float32x3& up)
    {
        const float32x3 zaxis = normalize(target - viewer);
        const float32x3 xaxis = normalize(cross(up, zaxis));
        const float32x3 yaxis = cross(zaxis, xaxis);

        Matrix4x4 m;
        m[0] = float32x4(xaxis.x, yaxis.x, zaxis.x, 0);
        m[1] = float32x4(xaxis.y, yaxis.y, zaxis.y, 0);
        m[2] = float32x4(xaxis.z, yaxis.z, zaxis.z, 0);
        m[3] = float32x4(-dot(xaxis, viewer), -dot(yaxis, viewer), -dot(zaxis, viewer), 1.0f);
        return m;
    }

    Matrix4x4 Matrix4x4::orthoGL(float left, float right, float bottom, float top, float znear, float zfar)
    {
        float x = 2.0f / (right - left);
        float y = 2.0f / (top - bottom);
        float z = -2.0f / (zfar - znear);
        float a = -(left + right) / (right - left);
        float b = -(bottom + top) / (top - bottom);
        float c = -(znear + zfar) / (zfar - znear);

        Matrix4x4 m;
        m[0] = float32x4(x, 0, 0, 0);
        m[1] = float32x4(0, y, 0, 0);
        m[2] = float32x4(0, 0, z, 0);
        m[3] = float32x4(a, b, c, 1);
        return m;
    }

    Matrix4x4 Matrix4x4::frustumGL(float left, float right, float bottom, float top, float znear, float zfar)
    {
        float a = (right + left) / (right - left);
        float b = (top + bottom) / (top - bottom);
        float c = -(zfar + znear) / (zfar - znear);
        float d = -(2.0f * znear * zfar) / (zfar - znear);
        float x = (2.0f * znear) / (right - left);
        float y = (2.0f * znear) / (top - bottom);
        float z = -1.0f;

        Matrix4x4 m;
        m[0] = float32x4(x, 0, 0, 0);
        m[1] = float32x4(0, y, 0, 0);
        m[2] = float32x4(a, b, c, z);
        m[3] = float32x4(0, 0, d, 0);
        return m;
    }

    Matrix4x4 Matrix4x4::perspectiveGL(float xfov, float yfov, float znear, float zfar)
    {
        float x = znear * std::tan(xfov * 0.5f);
        float y = znear * std::tan(yfov * 0.5f);
        return frustumGL(-x, x, -y, y, znear, zfar);
    }

    Matrix4x4 Matrix4x4::orthoVK(float left, float right, float bottom, float top, float znear, float zfar)
    {
        float x = 2.0f / (right - left);
        float y = 2.0f / (bottom - top);
        float z = 1.0f / (znear - zfar);
        float a = (right + left) / (left - right);
        float b = (top + bottom) / (top - bottom);
        float c = (zfar + znear) / (zfar - znear) * -0.5f;

        Matrix4x4 m;
        m[0] = float32x4(x, 0, 0, 0);
        m[1] = float32x4(0, y, 0, 0);
        m[2] = float32x4(0, 0, z, z);
        m[3] = float32x4(a, b, c, c + 1.0f);
        return m;
    }

    Matrix4x4 Matrix4x4::frustumVK(float left, float right, float bottom, float top, float znear, float zfar)
    {
        float a = (right + left) / (right - left);
        float b = (top + bottom) / (bottom - top);
        float c = (zfar + znear) / (znear - zfar) * 0.5f;
        float d = (2.0f * znear * zfar) / (znear - zfar) * 0.5f;
        float x = (2.0f * znear) / (right - left);
        float y = (2.0f * znear) / (bottom - top);

        Matrix4x4 m;
        m[0] = float32x4(x, 0, 0, 0);
        m[1] = float32x4(0, y, 0, 0);
        m[2] = float32x4(a, b, c, c - 1.0f);
        m[3] = float32x4(0, 0, d, d);
        return m;
    }

    Matrix4x4 Matrix4x4::perspectiveVK(float xfov, float yfov, float znear, float zfar)
    {
        float x = znear * std::tan(xfov * 0.5f);
        float y = znear * std::tan(yfov * 0.5f);
        return frustumVK(-x, x, -y, y, znear, zfar);
    }

    Matrix4x4 Matrix4x4::orthoD3D(float left, float right, float bottom, float top, float znear, float zfar)
    {
        const float x = 1.0f / (right - left);
        const float y = 1.0f / (top - bottom);
        const float z = 1.0f / (zfar - znear);

        const float w = 2.0f * x;
        const float h = 2.0f * y;
        const float a = -x * (left + right);
        const float b = -y * (bottom + top);
        const float c = -z * znear;

        Matrix4x4 m;
        m[0] = float32x4(w, 0, 0, 0);
        m[1] = float32x4(0, h, 0, 0);
        m[2] = float32x4(0, 0, z, 0);
        m[3] = float32x4(a, b, c, 1);
        return m;
    }

    Matrix4x4 Matrix4x4::frustumD3D(float left, float right, float bottom, float top, float znear, float zfar)
    {
        const float x = 1.0f / (right - left);
        const float y = 1.0f / (top - bottom);
        const float z = 1.0f / (zfar - znear);

        const float w = x * znear * 2.0f;
        const float h = y * znear * 2.0f;
        const float a = -x * (left + right);
        const float b = -y * (bottom + top);
        const float c = z * zfar;
        const float d = z * zfar * -znear;

        Matrix4x4 m;
        m[0] = float32x4(w, 0, 0, 0);
        m[1] = float32x4(0, h, 0, 0);
        m[2] = float32x4(a, b, c, 1);
        m[3] = float32x4(0, 0, d, 0);
        return m;
    }

    Matrix4x4 Matrix4x4::perspectiveD3D(float xfov, float yfov, float znear, float zfar)
    {
        const float w = 1.0f / std::tan(xfov * 0.5f);
        const float h = 1.0f / std::tan(yfov * 0.5f);
        const float a = zfar / (zfar - znear);
        const float b = -a * znear;

        Matrix4x4 m;
        m[0] = float32x4(w, 0, 0, 0);
        m[1] = float32x4(0, h, 0, 0);
        m[2] = float32x4(0, 0, a, 1);
        m[3] = float32x4(0, 0, b, 0);
        return m;
    }

    Matrix4x4 translate(const Matrix4x4& input, float x, float y, float z)
    {
        const float32x4 v = float32x4(x, y, z, 0.0f);

        Matrix4x4 m;
        m[0] = madd(input[0], input[0].wwww, v);
        m[1] = madd(input[1], input[1].wwww, v);
        m[2] = madd(input[2], input[2].wwww, v);
        m[3] = madd(input[3], input[3].wwww, v);
        return m;
    }

    Matrix4x4 scale(const Matrix4x4& input, float s)
    {
        const float32x4 v = float32x4(s, s, s, 1.0f);

        Matrix4x4 m;
        m[0] = input[0] * v;
        m[1] = input[1] * v;
        m[2] = input[2] * v;
        m[3] = input[3] * v;
        return m;
    }

    Matrix4x4 scale(const Matrix4x4& input, float x, float y, float z)
    {
        const float32x4 v = float32x4(x, y, z, 1.0f);

        Matrix4x4 m;
        m[0] = input[0] * v;
        m[1] = input[1] * v;
        m[2] = input[2] * v;
        m[3] = input[3] * v;
        return m;
    }

    Matrix4x4 rotate(const Matrix4x4& input, float angle, const float32x3& axis)
    {
        const Matrix4x4 temp = AngleAxis(angle, axis);
        return input * temp;
    }

    Matrix4x4 rotateX(const Matrix4x4& input, float angle)
    {
        const float s = std::sin(angle);
        const float c = std::cos(angle);

        Matrix4x4 m;

        for (int i = 0; i < 4; ++i)
        {
            const float x = input[i][0];
            const float y = input[i][1];
            const float z = input[i][2];
            const float w = input[i][3];
            m[i][0] = x;
            m[i][1] = y * c - z * s;
            m[i][2] = z * c + y * s;
            m[i][3] = w;
        }

        return m;
    }

    Matrix4x4 rotateY(const Matrix4x4& input, float angle)
    {
        const float s = std::sin(angle);
        const float c = std::cos(angle);

        Matrix4x4 m;

        for (int i = 0; i < 4; ++i)
        {
            const float x = input[i][0];
            const float y = input[i][1];
            const float z = input[i][2];
            const float w = input[i][3];
            m[i][0] = x * c + z * s;
            m[i][1] = y;
            m[i][2] = z * c - x * s;
            m[i][3] = w;
        }

        return m;
    }

    Matrix4x4 rotateZ(const Matrix4x4& input, float angle)
    {
        const float s = std::sin(angle);
        const float c = std::cos(angle);

        Matrix4x4 m;

        for (int i = 0; i < 4; ++i)
        {
            const float x = input[i][0];
            const float y = input[i][1];
            const float z = input[i][2];
            const float w = input[i][3];
            m[i][0] = x * c - y * s;
            m[i][1] = y * c + x * s;
            m[i][2] = z;
            m[i][3] = w;
        }

        return m;
    }

    Matrix4x4 rotateXYZ(const Matrix4x4& input, float x, float y, float z)
    {
        const Matrix4x4 temp = Matrix4x4::rotateXYZ(x, y, z);
        return input * temp;
    }

    Matrix4x4 normalize(const Matrix4x4& input)
    {
        float32x4 x = input[0];
        float32x4 y = input[1];
        float32x4 z = input[2];
        x = normalize(x);
        y = normalize(y - x * dot(x, y));
        z = cross(x, y);
        return Matrix4x4(x, y, z, input[3]);
    }

    Matrix4x4 mirror(const Matrix4x4& input, const float32x4& plane)
    {
        const float* m = input;

        // components
        float32x3 xaxis(m[0], m[1], m[2]);
        float32x3 yaxis(m[4], m[5], m[6]);
        float32x3 zaxis(m[8], m[9], m[10]);
        float32x3 trans(m[12], m[13], m[14]);

        float32x3 normal = plane.xyz;
        float32x3 normal2 = normal * -2.0f;
        float dist = plane.w;

        // mirror translation
        float32x3 pos = trans + normal2 * (dot(trans, normal) - dist);

        // mirror x rotation
        xaxis += trans;
        xaxis += normal2 * (dot(xaxis, normal) - dist);
        xaxis -= pos;

        // mirror y rotation
        yaxis += trans;
        yaxis += normal2 * (dot(yaxis, normal) - dist);
        yaxis -= pos;

        // mirror z rotation
        zaxis += trans;
        zaxis += normal2 * (dot(zaxis, normal) - dist);
        zaxis -= pos;

        Matrix4x4 result;
        result[0] = float32x4(xaxis.x, xaxis.y, xaxis.z, 0.0f);
        result[1] = float32x4(yaxis.x, yaxis.y, yaxis.z, 0.0f);
        result[2] = float32x4(zaxis.x, zaxis.y, zaxis.z, 0.0f);
        result[3] = float32x4(pos.x, pos.y, pos.z, 1.0f);
        return result;
    }

    Matrix4x4 affineInverse(const Matrix4x4& input)
    {
        const float* m = input;

        float s = input.determinant3x3();
        if (s)
        {
            s = 1.0f / s;
        }

        float m00 = (m[ 5] * m[10] - m[ 6] * m[ 9]) * s;
        float m01 = (m[ 9] * m[ 2] - m[10] * m[ 1]) * s;
        float m02 = (m[ 1] * m[ 6] - m[ 2] * m[ 5]) * s;
        float m10 = (m[ 6] * m[ 8] - m[ 4] * m[10]) * s;
        float m11 = (m[10] * m[ 0] - m[ 8] * m[ 2]) * s;
        float m12 = (m[ 2] * m[ 4] - m[ 0] * m[ 6]) * s;
        float m20 = (m[ 4] * m[ 9] - m[ 5] * m[ 8]) * s;
        float m21 = (m[ 8] * m[ 1] - m[ 9] * m[ 0]) * s;
        float m22 = (m[ 0] * m[ 5] - m[ 1] * m[ 4]) * s;
        float m30 = -(m00 * m[12] + m10 * m[13] + m20 * m[14]);
        float m31 = -(m01 * m[12] + m11 * m[13] + m21 * m[14]);
        float m32 = -(m02 * m[12] + m12 * m[13] + m22 * m[14]);

        Matrix4x4 result;
        result[0] = float32x4(m00, m01, m02, m[ 3]);
        result[1] = float32x4(m10, m11, m12, m[ 7]);
        result[2] = float32x4(m20, m21, m22, m[11]);
        result[3] = float32x4(m30, m31, m32, m[15]);
        return result;
    }

    Matrix4x4 adjoint(const Matrix4x4& input)
    {
        const float* m = input;

        float m00 = m[5] * m[10] - m[6] * m[9];
        float m01 = m[9] * m[2] - m[10] * m[1];
        float m02 = m[1] * m[6] - m[2] * m[5];
        float m10 = m[6] * m[8] - m[4] * m[10];
        float m11 = m[10] * m[0] - m[8] * m[2];
        float m12 = m[2] * m[4] - m[0] * m[6];
        float m20 = m[4] * m[9] - m[5] * m[8];
        float m21 = m[8] * m[1] - m[9] * m[0];
        float m22 = m[0] * m[5] - m[1] * m[4];
        float m30 = -(m[0] * m[12] + m[4] * m[13] + m[8] * m[14]);
        float m31 = -(m[1] * m[12] + m[5] * m[13] + m[9] * m[14]);
        float m32 = -(m[2] * m[12] + m[6] * m[13] + m[10] * m[14]);

        Matrix4x4 result;
        result[0] = float32x4(m00, m01, m02, m[3]);
        result[1] = float32x4(m10, m11, m12, m[7]);
        result[2] = float32x4(m20, m21, m22, m[11]);
        result[3] = float32x4(m30, m31, m32, m[15]);
        return result;
    }

    Matrix4x4 obliqueGL(const Matrix4x4& proj, const float32x4& nearclip)
    {
        float32x4 s = sign(nearclip);
        float xsign = s.x;
        float ysign = s.y;

        float32x4 q((xsign - proj(2,0)) / proj(0,0),
                    (ysign - proj(2,1)) / proj(1,1),
                    -1.0f, (1.0f + proj(2,2)) / proj(3,2));

        float32x4 c = nearclip * (2.0f / dot(nearclip, q));
        c += float32x4(0.0f, 0.0f, 1.0f, 0.0f);

        Matrix4x4 p = proj;
        p[0][2] = c.x;
        p[1][2] = c.y;
        p[2][2] = c.z;
        p[3][2] = c.w;
        return p;
    }

    Matrix4x4 obliqueVK(const Matrix4x4& proj, const float32x4& nearclip)
    {
        // conversion from GL to VK matrix format
        const Matrix4x4 to_vk
        {
            float32x4(1.0f, 0.0f, 0.0f, 0.0f),
            float32x4(0.0f,-1.0f, 0.0f, 0.0f),
            float32x4(0.0f, 0.0f, 0.5f, 0.5f),
            float32x4(0.0f, 0.0f, 0.0f, 1.0f)
        };

        // inverse of to_vk matrix
        const Matrix4x4 from_vk
        {
            float32x4(1.0f, 0.0f, 0.0f, 0.0f),
            float32x4(0.0f,-1.0f, 0.0f, 0.0f),
            float32x4(0.0f, 0.0f, 2.0f,-1.0f),
            float32x4(0.0f, 0.0f, 0.0f, 1.0f)
        };

        // NOTE: using the existing OpenGL function requires a round-trip to it's matrix format. :(
        Matrix4x4 p = obliqueGL(proj * from_vk, nearclip);
        return p * to_vk;
    }

    Matrix4x4 obliqueD3D(const Matrix4x4& proj, const float32x4& nearclip)
    {
        float32x4 clip = nearclip;
        float32x4 s = sign(clip);
        float xsign = s.x;
        float ysign = s.y;

        float32x4 q((xsign - proj(2,0)) / proj(0,0),
                    (ysign - proj(2,1)) / proj(1,1),
                    1.0f,
                    (1.0f - proj(2,2)) / proj(3,2));
        float32x4 c = clip / dot(clip, q);

        Matrix4x4 p = proj;
        p[0][2] = c.x;
        p[1][2] = c.y;
        p[2][2] = c.z;
        p[3][2] = c.w;
        return p;
    }

    // ------------------------------------------------------------------------
    // EulerAngles
    // ------------------------------------------------------------------------

    EulerAngles::EulerAngles(const Quaternion& q)
    {
        // x-axis
        float sr_cp = 2.0f * (q.w * q.x + q.y * q.z);
        float cr_cp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
        x = std::atan2(sr_cp, cr_cp);

        // y-axis
        float sp = 2.0f * (q.w * q.y - q.z * q.x);
        if (std::abs(sp) >= 1.0f)
        {
            // use 90 degrees if out of range
            y = std::copysign(float(pi) / 2.0f, sp);
        }
        else
        {
            y = std::asin(sp);
        }

        // z-axis
        float sy_cp = 2.0f * (q.w * q.z + q.x * q.y);
        float cy_cp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        z = std::atan2(sy_cp, cy_cp);
    }

    // ------------------------------------------------------------------------
    // AngleAxis
    // ------------------------------------------------------------------------

    AngleAxis::AngleAxis(const Matrix3x3& m)
    {
        axis = float32x3(m[1][2] - m[2][1], m[2][0] - m[0][2], m[0][1] - m[1][0]);
        const float s = square(axis) * 0.5f;
        const float c = (m[0][0] + m[1][1] + m[2][2] - 1.0f) * 0.5f;
        angle = std::atan2(s, c);
    }

    AngleAxis::AngleAxis(const Matrix4x4& m)
    {
        axis = float32x3(m[1][2] - m[2][1], m[2][0] - m[0][2], m[0][1] - m[1][0]);
        const float s = square(axis) * 0.5f;
        const float c = (m[0][0] + m[1][1] + m[2][2] - 1.0f) * 0.5f;
        angle = std::atan2(s, c);
    }

    AngleAxis::AngleAxis(const Quaternion& q)
    {
        angle = std::acos(q.w) * 2.0f;
        axis = float32x3(q.x, q.y, q.z) / std::sqrt(1.0f - q.w * q.w);
    }

    // ------------------------------------------------------------------------
    // Quaternion
    // ------------------------------------------------------------------------

    template <int Width, int Height>
    static inline
    Quaternion matrixToQuaternion(const Matrix<float, Width, Height>& m)
    {
        const float m00 = m(0, 0);
        const float m01 = m(0, 1);
        const float m02 = m(0, 2);
        const float m10 = m(1, 0);
        const float m11 = m(1, 1);
        const float m12 = m(1, 2);
        const float m20 = m(2, 0);
        const float m21 = m(2, 1);
        const float m22 = m(2, 2);

        float s;
        Quaternion q;

        if (m22 < 0)
        {
            if (m00 > m11)
            {
                s = 1.0f + m00 - m11 - m22;
                q = Quaternion(s, m01 + m10, m20 + m02, m12 - m21);
            }
            else
            {
                s = 1.0f - m00 + m11 - m22;
                q = Quaternion(m01 + m10, s, m12 + m21, m20 - m02);
            }
        }
        else
        {
            if (m00 < -m11)
            {
                s = 1.0f - m00 - m11 + m22;
                q = Quaternion(m20 + m02, m12 + m21, s, m01 - m10);
            }
            else
            {
                s = 1.0f + m00 + m11 + m22;
                q = Quaternion(m12 - m21, m20 - m02, m01 - m10, s);
            }
        }

        return q * float(0.5f / std::sqrt(s));
    }

    const Quaternion& Quaternion::operator = (const Matrix<float, 3, 3>& m)
    {
        *this = matrixToQuaternion(m);
        return *this;
    }

    const Quaternion& Quaternion::operator = (const Matrix<float, 4, 4>& m)
    {
        *this = matrixToQuaternion(m);
        return *this;
    }

    const Quaternion& Quaternion::operator = (const AngleAxis& a)
    {
        const float theta = a.angle * 0.5f;
        const float s = std::sin(theta) / length(a.axis);
        const float c = std::cos(theta);

        x = a.axis.x * s;
        y = a.axis.y * s;
        z = a.axis.z * s;
        w = c;
        return *this;
    }

    const Quaternion& Quaternion::operator = (const EulerAngles& euler)
    {
        const float sx = std::sin(euler.x * 0.5f);
        const float cx = std::cos(euler.x * 0.5f);
        const float sy = std::sin(euler.y * 0.5f);
        const float cy = std::cos(euler.y * 0.5f);
        const float sz = std::sin(euler.z * 0.5f);
        const float cz = std::cos(euler.z * 0.5f);

        x = cz * sx * cy - sz * cx * sy;
        y = cz * cx * sy + sz * sx * cy;
        z = sz * cx * cy - cz * sx * sy;
        w = cz * cx * cy + sz * sx * sy;
        return *this;
    }

    Quaternion Quaternion::identity()
    {
        return Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    }

    Quaternion Quaternion::rotateX(float angle)
    {
        float s = std::sin(angle * 0.5f);
        float c = std::cos(angle * 0.5f);
        return Quaternion(s, 0, 0, c);
    }

    Quaternion Quaternion::rotateY(float angle)
    {
        float s = std::sin(angle * 0.5f);
        float c = std::cos(angle * 0.5f);
        return Quaternion(0, s, 0, c);
    }

    Quaternion Quaternion::rotateZ(float angle)
    {
        float s = std::sin(angle * 0.5f);
        float c = std::cos(angle * 0.5f);
        return Quaternion(0, 0, s, c);
    }

    Quaternion Quaternion::rotateXYZ(float xangle, float yangle, float zangle)
    {
        return Quaternion(EulerAngles(xangle, yangle, zangle));
    }

    Quaternion Quaternion::rotate(const EulerAngles& euler)
    {
        return Quaternion(euler);
    }

    Quaternion Quaternion::rotate(const float32x3& from, const float32x3& to)
    {
        const float32x3 h = normalize(from + to);
        const float32x3 xyz = cross(from, h);
        const float w = dot(from, h);
        return Quaternion(xyz, w);
    }

    Quaternion log(const Quaternion& q)
    {
        float s = q.w ? std::atan2(std::sqrt(square(q)), q.w) : float(pi) * 2.0f;
        return Quaternion(q.x * s, q.y * s, q.z * s, 0.0f);
    }

    Quaternion exp(const Quaternion& q)
    {
        float s = std::sqrt(square(q));
        const float c = std::cos(s);
        s = (s > epsilon * 100.0f) ? std::sin(s) / s : 1.0f;
        return Quaternion(q.x * s, q.y * s, q.z * s, c);
    }

    Quaternion pow(const Quaternion& q, float p)
    {
        float s = square(q);
        const float c = std::cos(s * p);
        s = s ? std::sin(s * p) / s : 1.0f;
        return Quaternion(q.x * s, q.y * s, q.z * s, c);
    }

    Quaternion normalize(const Quaternion& q)
    {
        float s = norm(q);
        if (s)
        {
            s = 1.0f / std::sqrt(s);
        }
        return q * s;
    }

    Quaternion lndif(const Quaternion& a, const Quaternion& b)
    {
        Quaternion p = inverse(a) * b;
        const float length = std::sqrt(square(p));
        const float scale = norm(a);
        float s = scale ? std::atan2(length, scale) : float(pi) * 2.0f;
        if (length) s /= length;

        return Quaternion(p.x * s, p.y * s, p.z * s, 0.0f);
    }

    Quaternion lerp(const Quaternion& a, const Quaternion& b, float time)
    {
        float x = lerp(a.x, b.x, time);
        float y = lerp(a.y, b.y, time);
        float z = lerp(a.z, b.z, time);
		float w = a.w + (b.w - a.w) * time;
		return Quaternion(x, y, z, w);
    }

    Quaternion slerp(const Quaternion& a, const Quaternion& b, float time)
    {
        const float cosom = dot(a, b);

        if ((1.0f + cosom) > epsilon)
        {
            float sp;
            float sq;

            if ((1.0f - cosom) > epsilon)
            {
                const float omega = std::acos(cosom);
                const float sinom = 1.0f / std::sin(omega);

                sp = std::sin((1.0f - time) * omega) * sinom;
                sq = std::sin(time * omega) * sinom;
            }
            else
            {
                sp = 1.0f - time;
                sq = time;
            }

            return a * sp + b * sq;
        }
        else
        {
            const float halfpi = float(pi * 0.5);
            const float sp = std::sin((1.0f - time) * halfpi);
            const float sq = std::sin(time * halfpi);

            return Quaternion(a.x * sp - a.y * sq,
                              a.y * sp + a.x * sq,
                              a.z * sp - a.w * sq,
                              a.z);
        }
    }

    Quaternion slerp(const Quaternion& a, const Quaternion& b, int spin, float time)
    {
        float bflip = 1.0f;
        float tcos = dot(a, b);

        if (tcos < 0)
        {
            tcos = -tcos;
            bflip = -1;
        }

        float beta;
        float alpha;

        if ((1.0f - tcos) < epsilon * 100.0f)
        {
            // linear interpolate
            beta = 1.0f - time;
            alpha = time * bflip;
        }
        else
        {
            const float theta = std::acos(tcos);
            const float phi   = theta + spin * float(pi);
            const float tsin  = std::sin(theta);
            beta  = std::sin(theta - time * phi) / tsin;
            alpha = std::sin(time * phi) / tsin * bflip;
        }

        return a * beta + b * alpha;
    }

    Quaternion squad(const Quaternion& p, const Quaternion& a, const Quaternion& b, const Quaternion& q, float time)
    {
        Quaternion qa = slerp(p, q, 0, time);
        Quaternion qb = slerp(a, b, 0, time);
        return slerp(qa, qb, 0, 2.0f * time * (1.0f - time));
    }

    // ------------------------------------------------------------------------
    // sRGB
    // ------------------------------------------------------------------------

#if 0

    //
    // precise approximations
    //

    namespace detail
    {

        static inline
        float pow24(float v)
        {
            s32 i = reinterpret_bits<s32>(v);
            i = (i >> 2) + (i >> 4);
            i += (i >> 4);
            i += (i >> 8);
            i += 0x2a514d80;
            float s = reinterpret_bits<float>(i);
            s = 0.3332454f * (2.0f * s + v / (s * s));
            return s * sqrt(sqrt(s));
        }

        static inline
        float32x4 pow24(float32x4 v)
        {
            int32x4 i = reinterpret<int32x4>(v);
            i = (i >> 2) + (i >> 4);
            i += (i >> 4);
            i += (i >> 8);
            i += 0x2a514d80;
            float32x4 s = reinterpret<float32x4>(i);
            s = 0.3332454f * (2.0f * s + v / (s * s));
            return s * sqrt(sqrt(s));
        }

        static inline
        float32x8 pow24(float32x8 v)
        {
            int32x8 i = reinterpret<int32x8>(v);
            i = (i >> 2) + (i >> 4);
            i += (i >> 4);
            i += (i >> 8);
            i += 0x2a514d80;
            float32x8 s = reinterpret<float32x8>(i);
            s = 0.3332454f * (2.0f * s + v / (s * s));
            return s * sqrt(sqrt(s));
        }

        static inline
        float root5(float v)
        {
            s32 i = reinterpret_bits<s32>(v);
            s32 d = (i >> 2) - (i >> 4) + (i >> 6) - (i >> 8) + (i >> 10);
            i = 0x32c9af22 + d;
            float f = reinterpret_bits<float>(i);
            float s = f * f;
            f -= (f - v / (s * s)) * 0.2f;
            return f;
        }

        static inline
        float32x4 root5(float32x4 v)
        {
            int32x4 i = reinterpret<int32x4>(v);
            int32x4 d = (i >> 2) - (i >> 4) + (i >> 6) - (i >> 8) + (i >> 10);
            i = 0x32c9af22 + d;
            float32x4 f = reinterpret<float32x4>(i);
            float32x4 s = f * f;
            f -= (f - v / (s * s)) * 0.2f;
            return f;
        }

        static inline
        float32x8 root5(float32x8 v)
        {
            int32x8 i = reinterpret<int32x8>(v);
            int32x8 d = (i >> 2) - (i >> 4) + (i >> 6) - (i >> 8) + (i >> 10);
            i = 0x32c9af22 + d;
            float32x8 f = reinterpret<float32x8>(i);
            float32x8 s = f * f;
            f -= (f - v / (s * s)) * 0.2f;
            return f;
        }

    } // namespace detail

    float linear_to_srgb(float linear)
    {
        linear = clamp(linear, 0.0f, 1.0f);
        float srgb = (linear < 0.0031308f) ?
            12.92f * linear :
            1.055f * detail::pow24(linear) - 0.055f;
        return srgb;
    }

    float srgb_to_linear(float srgb)
    {
        float linear;
        if (srgb <= 0.04045f)
        {
            linear = srgb * (1.0f / 12.92f);
        }
        else
        {
            float s = (srgb * (1.f / 1.055f) + 0.055f / 1.055f);
            linear = (s * s) * detail::root5(s * s);
        }
        return linear;
    }

    float32x4 linear_to_srgb(float32x4 linear)
    {
        linear = clamp(linear, 0.0f, 1.0f);
        float32x4 a = linear * 12.92f;
        float32x4 b = 1.055f * detail::pow24(linear) - 0.055f;
        float32x4 srgb = select(linear < 0.0031308f, a, b);
        return srgb;
    }

    float32x4 srgb_to_linear(float32x4 srgb)
    {
        float32x4 a = srgb * (1.0f / 12.92f);
        float32x4 s = (srgb * (1.f / 1.055f) + 0.055f / 1.055f);
        float32x4 b = (s * s) * detail::root5(s * s);
        float32x4 linear = select(srgb <= 0.04045f, a, b);
        return linear;
    }

    float32x8 linear_to_srgb(float32x8 linear)
    {
        linear = clamp(linear, 0.0f, 1.0f);
        float32x8 a = linear * 12.92f;
        float32x8 b = 1.055f * detail::pow24(linear) - 0.055f;
        float32x8 srgb = select(linear < 0.0031308f, a, b);
        return srgb;
    }

    float32x8 srgb_to_linear(float32x8 srgb)
    {
        float32x8 a = srgb * (1.0f / 12.92f);
        float32x8 s = (srgb * (1.f / 1.055f) + 0.055f / 1.055f);
        float32x8 b = (s * s) * detail::root5(s * s);
        float32x8 linear = select(srgb <= 0.04045f, a, b);
        return linear;
    }

#endif

} // namespace mango::math
