#pragma once
#include <cstddef>
#include <array>
#include <cassert>
#include <cmath>
#include <span>

template <size_t n>
class Filter
{
public:
    virtual std::array<float, n> process(std::span<const float, n> data, float dt_seconds) = 0;
    virtual ~Filter() = default;
};

template <size_t n_elements>
class LowPassIIR : public Filter<n_elements>
{
public:
    explicit LowPassIIR(float cutoff_hz) : _state{0.0f}
    {
        set_cutoff(cutoff_hz);
    }

    std::array<float, n_elements> process(std::span<const float, n_elements> sample, float dt_seconds) override
    {
        if (!_initialized)
        {
            for (size_t i = 0; i < n_elements; ++i)
            {
                _state[i] = sample[i];
            }
            _initialized = true;
            return _state;
        }

        assert(dt_seconds > 0.0f);
        const float tau = 1.0f / (2.0f * 3.14159265f * _cutoff_hz);
        const float alpha = dt_seconds / (tau + dt_seconds);
        for (size_t i = 0; i < n_elements; i++)
        {
            _state[i] += alpha * (sample[i] - _state[i]);
        }
        return _state;
    }

    void set_cutoff(float cutoff_hz)
    {
        assert(cutoff_hz > 0.0f);
        _cutoff_hz = cutoff_hz;
    }

private:
    std::array<float, n_elements> _state;
    float _cutoff_hz = 1.0f;
    bool _initialized = false;
};
