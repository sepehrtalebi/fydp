#pragma once

template<typename T>
class PID {
    T last_error, error_area;
    const T K_P, K_I, K_D;
public:
    PID(T K_P, T K_I, T K_D): K_P(K_P), K_I(K_I), K_D(K_D), last_error(0), error_area(0) {}
    PID() =default;

    T update(T error, T dt) {
        double output_signal = 0;
        if (K_P) output_signal += K_P * error;
        if (K_I) {
            error_area += error * dt;
            output_signal += K_I * error_area;
        }
        if (K_D) {
            output_signal += K_D * (error - last_error) / dt;
            last_error = error;
        }
        return output_signal;
    }

    static T proportional_signal(T K_P, T error) {
        return K_P * error;
    }
};