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

    template<char... cs>
    struct parse_size_t;

    template<char... cs>
    inline constexpr size_t parse_size_t_v = parse_size_t<cs...>::value;

    template<char c>
    struct parse_size_t<c> {
        static_assert('0' <= c && c <= '9');

        static constexpr size_t power = 1;
        static constexpr size_t value = c - '0';
    };

    template<char c, char... cs>
    struct parse_size_t<c, cs...> {
        static_assert('0' <= c && c <= '9');

        static constexpr size_t power = 10 * parse_size_t<cs...>::power;
        static constexpr size_t value = power * (c - '0') + parse_size_t_v<cs...>;
    };

    template<char... cs>
    constexpr Variable<parse_size_t_v<cs...>> operator "" _v() {
        return {};
    }
}
