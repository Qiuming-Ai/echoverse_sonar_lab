#ifndef STANDALONE_MVP_BOOST_RANDOM_HPP
#define STANDALONE_MVP_BOOST_RANDOM_HPP

#include <random>

namespace boost {
namespace random {
using mt19937 = std::mt19937;
template <typename T>
using normal_distribution = std::normal_distribution<T>;
} // namespace random
} // namespace boost

#endif
