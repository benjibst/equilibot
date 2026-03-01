#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <numeric>
#include <type_traits>
#include <utility>
template <size_t sz>
struct Vec
{
    static constexpr size_t size = sz;
    explicit Vec(const std::array<float, sz> &arr) : _vec(arr) {}
    explicit Vec(const float *data) { std::memcpy(_vec.data(), data, sizeof(_vec)); }

    Vec() = default;
    float &operator[](size_t idx)
    {
        return _vec[idx];
    }
    float operator[](size_t idx) const
    {
        return _vec[idx];
    }
    Vec operator+(const Vec &other) const
    {
        Vec res;
        for (size_t i = 0; i < sz; i++)
        {
            res[i] = _vec[i] + other[i];
        }
        return res;
    }
    Vec operator-(const Vec &other) const
    {
        Vec res;
        for (size_t i = 0; i < sz; i++)
        {
            res[i] = _vec[i] + other[i];
        }
        return res;
    }
    Vec operator*(float scalar) const
    {
        Vec res;
        for (size_t i = 0; i < sz; i++)
        {
            res[i] = _vec[i] * scalar;
        }
        return res;
    }
    Vec operator/(float scalar) const
    {
        return (*this) * (1.0f / scalar);
    }
    float abs() const
    {
        float sqsum = 0;
        for (size_t i = 0; i < sz; i++)
        {
            sqsum += _vec[i] * _vec[i];
        }
        return std::sqrt(sqsum);
    }
    operator std::array<float, sz> &() { return _vec; };
    std::array<float, sz> _vec;
};

template <size_t rows, size_t columns>
class Matrix
{
public:
    static constexpr size_t kRows = rows;
    static constexpr size_t kColumns = columns;
    static constexpr size_t kElements = rows * columns;
    static constexpr Matrix<rows, columns> identity()
    {
        static_assert(rows == columns);
        Matrix<rows, columns> ret;
        int j = 0;
        int shift = columns + 1;
        for (int i = 0; i < rows; i++)
        {
            ret[j] = 1;
            j += shift;
        }
        return ret;
    }

    Matrix() = default;
    explicit Matrix(const std::array<float, kElements> &arr) : _mat(arr) {}
    explicit Matrix(const float *data) { std::memcpy(_mat.data(), data, sizeof(_mat)); }

    float &operator()(size_t r, size_t c)
    {
        return _mat[r * columns + c];
    }

    float operator()(size_t r, size_t c) const
    {
        return _mat[r * columns + c];
    }
    float &operator[](size_t idx)
    {
        return _mat[idx];
    }

    float *data()
    {
        return _mat.data();
    }

    const float *data() const
    {
        return _mat.data();
    }

    template <size_t rhs_columns>
    Matrix<rows, rhs_columns> operator*(const Matrix<columns, rhs_columns> &rhs) const
    {
        Matrix<rows, rhs_columns> result;
        if constexpr (use_dsp_for_matrix_mul<rhs_columns>())
        {
            const esp_err_t err = dspm_mult_f32(_mat.data(),
                                                rhs._mat.data(),
                                                result._mat.data(),
                                                static_cast<int>(rows),
                                                static_cast<int>(columns),
                                                static_cast<int>(rhs_columns));
        }
        else
        {
            multiply_matrix_ansi(rhs, result);
        }
        return result;
    }

    Vec<rows> operator*(const Vec<columns> &rhs) const
    {
        Vec<rows> result;
        if constexpr (use_dsp_for_vector_mul())
        {
            const esp_err_t err = dspm_mult_f32(_mat.data(),
                                                rhs._vec.data(),
                                                result._vec.data(),
                                                static_cast<int>(rows),
                                                static_cast<int>(columns),
                                                1);
        }
        else
        {
            multiply_vector_ansi(rhs, result);
        }
        return result;
    }

private:
    template <size_t rhs_columns>
    static constexpr bool use_dsp_for_matrix_mul()
    {
        return (rows * columns * rhs_columns) >= 256;
    }

    static constexpr bool use_dsp_for_vector_mul()
    {
        return (rows * columns) >= 128;
    }

    template <size_t rhs_columns>
    void multiply_matrix_ansi(const Matrix<columns, rhs_columns> &rhs, Matrix<rows, rhs_columns> &out) const
    {
        for (size_t r = 0; r < rows; ++r)
        {
            for (size_t c = 0; c < rhs_columns; ++c)
            {
                float acc = 0.0f;
                for (size_t k = 0; k < columns; ++k)
                {
                    acc += (*this)(r, k) * rhs(k, c);
                }
                out(r, c) = acc;
            }
        }
    }

    void multiply_vector_ansi(const Vec<columns> &rhs, Vec<rows> &out) const
    {
        for (size_t r = 0; r < rows; ++r)
        {
            float acc = 0.0f;
            for (size_t k = 0; k < columns; ++k)
            {
                acc += (*this)(r, k) * rhs[k];
            }
            out[r] = acc;
        }
    }

    template <size_t, size_t>
    friend class Matrix;

    std::array<float, kElements> _mat = {};
};

struct Quaternion : Vec<4>
{
    using Vec<4>::Vec;
    using Vec<4>::operator[];
    Quaternion(float w, float x, float y, float z)
    {
        _vec = {w, x, y, z};
    }
    Quaternion operator*(const Quaternion &other) const
    {
        return {
            _vec[0] * other[0] - _vec[1] * other[1] - _vec[2] * other[2] - _vec[3] * other[3],
            _vec[0] * other[1] + _vec[1] * other[0] + _vec[2] * other[3] - _vec[3] * other[2],
            _vec[0] * other[2] - _vec[1] * other[3] + _vec[2] * other[0] + _vec[3] * other[1],
            _vec[0] * other[3] + _vec[1] * other[2] - _vec[2] * other[1] + _vec[3] * other[0]};
    };
};
