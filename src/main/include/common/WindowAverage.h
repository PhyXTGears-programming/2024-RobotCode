#pragma once

#include <cstdlib>
#include <memory>

template<std::size_t N>
class WindowAverage {
public:
    WindowAverage() {}

    void Update(double value) {
        if (m_capacity > m_count) {
            // Fill up window with first value provided, so we don't have to worry
            // about a partial window for N-1 updates.
            for (std::size_t a = m_count; a < m_capacity; a += 1) {
                m_values[a] = value;
                m_sum += value;
            }
            m_count = m_capacity;
            m_index = 0;
        } else {
            m_sum += value - m_values[m_index];
            m_values[m_index] = value;
            m_index = (m_index + 1) % m_capacity;
        }
    }

    double Get() const {
        // WARNING: This will throw a divide-by-zero exception and crash the program
        // if m_count is zero.
        // Make sure to call Update() at least once before Get().
        return m_sum / m_count;
    }

private:
    // Array of values serving as a window into the stream of values measured.
    std::array<double, N> m_values = { 0.0 };
    // Keep track of next value in array to update.
    size_t m_index = 0;
    // Sum of values. An optimization to avoid adding all values in window array
    // repeatedly.  
    double m_sum = 0;
    // Number of values saved in window array.
    size_t m_count = 0;
    // Max number of values the window array can hold.
    size_t m_capacity = N;
};