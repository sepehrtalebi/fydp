#pragma once

template<typename T>
class PID {
public:
    /**
     * @brief Basic PID controller, which assumes a constant update rate.
     */
    PID(const T &k_p, const T &k_i, const T &k_d, const T &dt) : k_p(k_p), k_i(k_i), k_d(k_d), dt(dt) {}

    T update(const T &pos) {
        integral += last_pos * dt;
        T result = k_p * pos + k_i * integral + k_d * (pos - last_pos) / dt;
        last_pos = pos;
        return result;
    }

private:
    const T k_p;
    const T k_i;
    const T k_d;
    const T dt;
    T last_pos{};
    T integral{};
};
