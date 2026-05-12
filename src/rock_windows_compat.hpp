#ifndef STANDALONE_MVP_ROCK_WINDOWS_COMPAT_HPP
#define STANDALONE_MVP_ROCK_WINDOWS_COMPAT_HPP

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstdint>
typedef unsigned int uint;
#endif

#endif
