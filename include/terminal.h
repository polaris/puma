#ifndef TERMINAL_H
#define TERMINAL_H

#include <cstddef>
#include <cstdio>

#if defined(_WIN32)
  #include <io.h>
  #include <windows.h>
#else
  #include <sys/ioctl.h>
  #include <unistd.h>
#endif

namespace term {

[[nodiscard]] inline bool isTerminal(std::FILE* stream) noexcept {
#if defined(_WIN32)
    return _isatty(_fileno(stream)) != 0;
#else
    return isatty(fileno(stream)) != 0;
#endif
}

// Width of the terminal stderr is attached to, or 0 if it cannot be told.
[[nodiscard]] inline std::size_t stderrColumns() noexcept {
#if defined(_WIN32)
    CONSOLE_SCREEN_BUFFER_INFO info;
    if (GetConsoleScreenBufferInfo(GetStdHandle(STD_ERROR_HANDLE), &info)) {
        return static_cast<std::size_t>(info.srWindow.Right - info.srWindow.Left + 1);
    }
    return 0;
#else
    winsize size{};
    if (ioctl(STDERR_FILENO, TIOCGWINSZ, &size) == 0) {
        return size.ws_col;
    }
    return 0;
#endif
}

}  // namespace term

#endif  // TERMINAL_H
