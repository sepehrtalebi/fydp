#pragma once

#include <ratio>
#include <type_traits>

namespace compiled {
    template<typename R>
    struct Constant {
        using type = Constant<R>;

        static constexpr double apply(const double* /** variables **/) {
            return static_cast<double>(R::num) / static_cast<double>(R::den);
        }
    };

    using Zero = Constant<std::ratio<0>>;
    using One = Constant<std::ratio<1>>;
    using MinusOne = Constant<std::ratio<-1>>;

    Zero zero{};
    One one{};
    MinusOne minus_one{};

    template<typename T>
    struct is_constant : std::false_type {};

    template<typename T>
    struct is_constant<Constant<T>> : std::true_type {};

    template<typename T>
    inline constexpr bool is_constant_v = is_constant<T>::value;

    template<typename T>
    using is_zero_or_one = std::disjunction<std::is_same<T, Zero>, std::is_same<T, One>>;

    template<typename T>
    inline constexpr bool is_zero_or_one_v = is_zero_or_one<T>::value;
}
