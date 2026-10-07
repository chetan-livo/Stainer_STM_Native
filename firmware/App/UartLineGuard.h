#pragma once
#include <stddef.h>

// Bound a line without ever treating an oversized/corrupt suffix as a command.
// The caller consumes and clears a completed line; each UART owns its own flag.
template<class Text>
bool receiveUartLineByte(char value, Text& line, bool& discarding,
                         size_t limit = 512) {
    if (value == '\n' || value == '\r') {
        if (discarding) {
            line = "";
            discarding = false;
            return false;
        }
        return line.length() != 0;
    }
    if (discarding) return false;
    const unsigned char byte = static_cast<unsigned char>(value);
    if ((byte < 32 && value != '\t') || byte >= 127 || line.length() >= limit) {
        line = "";
        discarding = true;
        return false;
    }
    line += value;
    return false;
}
