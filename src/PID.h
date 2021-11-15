#pragma once

class PID {
    double last_error, error_area;
    double K_p, K_I, K_d;
public:
    PID(double K_p, double K_I, double K_d): K_p(K_p), K_I(K_I), K_d(K_d), last_error(0), error_area(0) {}
    PID() =default;
    double output_signal(double error, double dt);
    static double proportional_signal(double K_p, double error);
};