#pragma once
#include<type_traits>
#include<ostream>
namespace g_type_traits {
    // use to check type T whether supports std::ostream << T
    template <typename T, typename = void>
    struct has_ostream_operator : std::false_type {};

    /*
    * std::declval<T>() is a prvalue of type T, used to simulate the creation of an object of type T without the need to construct it.
    * It is an hole in the type system.
    * decltype keyword can get the type of an expression.
    * so decltype(std::declval<std::ostringstream&>() << std::declval<T>(), void()) is splited the folowing statement:
    * 1. std::declval<std::ostringstream&>() << std::declval<T>() equals expression ` a << b`,
    *    where var a has type std::ostringstream, var b has type T
    * 2. `std::declval<std::ostringstream&>() << std::declval<T>(), void()` ends with type void.
    *     Without void(), when T has multiple overrided << operator, the compiler won't know which to choose, then throw an error.
    * 3. When T does not support std::ostream << T, substitution will fail, has_ostream_operator<T> is subclass of false_type.
    *    When T support std::ostream << T, substitution will success, has_ostream_operator<T> is subclass of true_tpye. This is called
    *    SFINAE.
    *
    */
    template <typename T>
    struct has_ostream_operator<T, decltype(std::declval<std::ostringstream&>() << std::declval<T>(), void())> : std::true_type {};

    // 下标运算符
    template<typename T, typename K = void>
    struct has_subscript_operator : std::false_type {};
    template<typename T>
    struct has_subscript_operator<T, decltype(std::declval<T>()[0], void())> : std::true_type {};
    template<typename T>
    inline constexpr bool has_subscript_operator_v = has_subscript_operator<T>::value;
    // 获取元素类型
    template<typename T>
    using element_type_t = decltype(std::declval<T>()[0]);


    // 用于检查加法运算
    template<typename T, typename = void>
    struct has_addition : std::false_type {};

    template<typename T>
    struct has_addition<T, decltype(std::declval<T>() + std::declval<T>(), void())> : std::true_type {};

    // 用于检查减法运算
    template<typename T, typename = void>
    struct has_subtraction : std::false_type {};

    template<typename T>
    struct has_subtraction<T, decltype(std::declval<T>() - std::declval<T>(), void())> : std::true_type {};

    // 用于检查乘法运算
    template<typename T, typename = void>
    struct has_multiplication : std::false_type {};

    template<typename T>
    struct has_multiplication<T, decltype(std::declval<T>()* std::declval<T>(), void())> : std::true_type {};

    // 用于检查除法运算
    template<typename T, typename = void>
    struct has_division : std::false_type {};

    template<typename T>
    struct has_division<T, decltype(std::declval<T>() / std::declval<T>(), void())> : std::true_type {};

    // 整合所有运算的类型萃取
    template<typename T>
    struct has_arithmetic_operations {
        static constexpr bool addition = has_addition<T>::value;
        static constexpr bool subtraction = has_subtraction<T>::value;
        static constexpr bool multiplication = has_multiplication<T>::value;
        static constexpr bool division = has_division<T>::value;

        static constexpr bool all = addition && subtraction && multiplication && division;
    };

    template<typename T>
    inline constexpr bool has_arithmetic_operations_v = has_ostream_operator<T>::value;

    // concat string at compile time
    // direct copied from https://stackoverflow.com/questions/38955940/how-to-concatenate-static-strings-at-compile-time
    template <std::string_view const& a, std::size_t t, std::string_view const& b>
    struct join
    {
        // Join all strings into a single std::array of chars
        static constexpr auto impl() noexcept
        {
            constexpr std::size_t a_size = a.size();
            constexpr std::size_t b_size = b.size();

            constexpr std::size_t len = (a_size + b_size + 1);
            std::array<char, len + 1> arr{};
            std::size_t i = 0;
            for (; i < a_size; i++)
            {
                arr[i] = a[i];
            }
            arr[i++] = t + '0';
            for (std::size_t j = 0; j < b_size; j++) // Start from 0 for b
            {
                arr[i++] = b[j]; // Fill b's characters
            }
            arr[len] = 0;
            return arr;
        }
        // Give the joined string static storage
        static constexpr auto arr = impl();
        // View as a std::string_view
        static constexpr std::string_view value{ arr.data(), arr.size() - 1 };
    };
    // Helper to get the value out
    template <std::string_view const& a, std::size_t t, std::string_view const& b>
    static constexpr std::string_view str_join_v = join<a, t, b>::value;
}