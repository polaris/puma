#ifndef EXIT_CODES_H
#define EXIT_CODES_H

// What the sender and the receiver return from main(), besides 0 and the
// command-line parser's own codes.
namespace exit_code {

constexpr int kNetwork = 1;    // no usable interface, or a socket or the clock sync failed
constexpr int kAudio = 2;      // no usable audio device, or it failed to open or start

}  // namespace exit_code

#endif  // EXIT_CODES_H
