#ifndef TERMINAL_COLORS_HPP
#define TERMINAL_COLORS_HPP

#include <string>

namespace Fore {
    constexpr const char* BLACK   = "\033[30m";
    constexpr const char* RED     = "\033[31m";
    constexpr const char* GREEN   = "\033[32m";
    constexpr const char* YELLOW  = "\033[33m";
    constexpr const char* BLUE    = "\033[34m";
    constexpr const char* MAGENTA = "\033[35m";
    constexpr const char* CYAN    = "\033[36m";
    constexpr const char* WHITE   = "\033[37m";
    constexpr const char* RESET   = "\033[39m";
}

namespace Back {
    constexpr const char* BLACK   = "\033[40m";
    constexpr const char* RED     = "\033[41m";
    constexpr const char* GREEN   = "\033[42m";
    constexpr const char* YELLOW  = "\033[43m";
    constexpr const char* BLUE    = "\033[44m";
    constexpr const char* MAGENTA = "\033[45m";
    constexpr const char* CYAN    = "\033[46m";
    constexpr const char* WHITE   = "\033[47m";
    constexpr const char* RESET   = "\033[49m";
}

namespace Style {
    constexpr const char* RESET_ALL  = "\033[0m";
    constexpr const char* BOLD       = "\033[1m";
    constexpr const char* DIM        = "\033[2m";
    constexpr const char* ITALIC     = "\033[3m";
    constexpr const char* UNDERLINE  = "\033[4m";
    constexpr const char* BLINK      = "\033[5m";
    constexpr const char* REVERSE    = "\033[7m";
    constexpr const char* HIDDEN     = "\033[8m";
}

#endif // TERMINAL_COLORS_HPP
