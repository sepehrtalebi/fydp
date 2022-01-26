#pragma once

#include <bmb_math/TransferFunction.h>

template<typename T>
class PID {
    T K_P, K_I, K_D;
    const T low_pass = 100;
    TransferFunction<T, 2, 2> I;
    TransferFunction<T, 2, 2> D;
public:
    PID(T K_P, T K_I, T K_D, T dt): K_P(K_P), K_I(K_I), K_D(K_D) {
        I = K_I * TransferFunction<T, 2, 2>{Polynomial<T, 2> {dt/2, dt/2}, Polynomial<T, 2>{-1, 1}, true}; //discrete version of 1/s
        D = K_D * TransferFunction<T, 2, 2>{Polynomial<T, 2> {-2 * low_pass, 2 * low_pass},
                                      Polynomial<T, 2>{low_pass * dt - 2, low_pass * dt + 2}, true}; //discrete version of Ns/(s + N)
    }

    T update(T error) {
        double output_signal = 0;
        if (K_P) output_signal += K_P * error;
        if (K_I) {
            output_signal += I.next_output(error);
        }
        if (K_D) {
            output_signal += D.next_output(error);
        }
        return output_signal;
    }

    static T proportional_signal(T K_P, T error) {
        return K_P * error;
    }
};
