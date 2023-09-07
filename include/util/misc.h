#pragma once
#include <type_traits>
#include <cmath>

namespace std {
template <typename Enum>
constexpr std::underlying_type_t<Enum> to_underlying(Enum e) noexcept {
  return static_cast<std::underlying_type_t<Enum>>(e);
};
}  // namespace std

template <typename T>
constexpr T map(T x, T in_min, T in_max, T out_min, T out_max) {
  return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min);
}

[[noreturn]] inline void unreachable() {
  // Uses compiler specific extensions if possible.
  // Even if no extension is used, undefined behavior is still raised by
  // an empty function body and the noreturn attribute.
#ifdef __GNUC__  // GCC, Clang, ICC
  __builtin_unreachable();
#else //_MSC_VER  // MSVC
  __assume(false);
#endif
}

template <typename Base, typename Arg>
constexpr bool is_smaller_size = sizeof(Base) < sizeof(Arg);

template <typename Base, typename Arg>
constexpr bool is_equal_size = sizeof(Base) == sizeof(Arg);

template <typename Base, typename Arg>
constexpr bool is_smaller_or_equal_size = is_smaller_size<Base, Arg> || is_equal_size<Base, Arg>;

template <typename Base, typename Arg>
concept SmallerThanOrEqual = is_smaller_or_equal_size<Base, Arg>;

template <typename T, typename... Ts>
inline constexpr bool all_same = std::conjunction_v<std::is_same<T, Ts>...>;

template <typename... Args>
concept AllSame = all_same<Args...>;

template <typename Base, typename... Args>
concept AllSameAs = all_same<Base, Args...>;


namespace util {
template <typename T>
constexpr T deviation(T x, T y) {
  const T max = std::max(x,y);
  const T min = std::min(x,y);
  return max - min;
}
}