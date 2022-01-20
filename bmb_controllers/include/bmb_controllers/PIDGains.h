#pragma once

struct PIDGains {
    double K_P, K_I, K_D, K_ff;
    PIDGains(double K_P, double K_I, double K_D): K_P(K_P), K_I(K_I), K_D(K_D), K_ff(0) {}
    PIDGains(double K_P, double K_I, double K_D, double K_ff): K_P(K_P), K_I(K_I), K_D(K_D), K_ff(K_ff) {}
};