#pragma once

#include <cstddef>

namespace compiled {
    template<size_t id>
    struct Variable {
        using type = Variable<id>;
        static constexpr size_t id_value = id;

        static double apply(const double* variables) {
            return variables[id];
        }
    };

    template<typename T>
    struct is_variable : std::false_type {};

    template<size_t id>
    struct is_variable<Variable<id>> : std::true_type {};

    template<typename T>
    inline constexpr bool is_variable_v = is_variable<T>::value;
}
