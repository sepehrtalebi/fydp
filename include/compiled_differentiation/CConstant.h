#pragma once

#include <array>
#include <ratio>
#include <type_traits>

namespace compiled {
    template<typename R>
    struct Constant {
        using type = Constant<R>;
        using ratio = R;
        static constexpr double value = static_cast<double>(R::num) / static_cast<double>(R::den);

        template <size_t n>
        static constexpr double apply(const std::array<double, n>& /** variables **/) {
            return value;
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

    template<char... cs>
    struct parse_ratio;

    template<char... cs>
    using parse_ratio_t = typename parse_ratio<cs...>::type;

    template<>
    struct parse_ratio<'.'> {
        using power = std::ratio<1, 10>;
        using type = std::ratio<0>;
    };

    template<char c>
    struct parse_ratio<c> {
      static_assert('0' <= c && c <= '9');

      using power = std::ratio<1>;
      using type = std::ratio<c - '0'>;
    };

    template<char... cs>
    struct parse_ratio<'.', cs...> {
        using power = std::ratio<1, 10>;
        using type = std::ratio_divide<parse_ratio_t<cs...>,
          std::ratio_multiply<std::ratio<10>, typename parse_ratio<cs...>::power>>;
    };

    template<char c, char... cs>
    struct parse_ratio<c, cs...> {
        static_assert('0' <= c && c <= '9');

        using power = std::ratio_multiply<std::ratio<10>, typename parse_ratio<cs...>::power>;
        using type = std::ratio_add<std::ratio_multiply<std::ratio<c - '0'>, power>, parse_ratio_t<cs...>>;
    };

    template<char... cs>
    constexpr Constant<parse_ratio_t<cs...>> operator "" _c() {
      return {};
    }
}
