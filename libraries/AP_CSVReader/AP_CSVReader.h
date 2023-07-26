#pragma once

// Note: term is always null-terminated so a final line with no cr/lf
// on it can still be fetched by the caller

#include <stdint.h>

class AP_CSVReader
{

public:

    AP_CSVReader(uint8_t *_term, uint8_t _term_len, uint8_t _separator=',') :
        separator{_separator},
        term{_term},
        term_len{_term_len}
        {}

    enum class RetCode : uint8_t {
        OK,
        ERROR,
        TERM_DONE,
        VECTOR_DONE,
    };

    RetCode feed(uint8_t c);
//    RetCode feed(const uint8_t *buffer, uint8_t len);

private:

    enum class State : uint8_t {
        START_OF_START_OF_TERM = 46,
        START_OF_TERM = 47,
        END_OF_VECTOR_CR = 48,
        IN_UNQUOTED_TERM = 49,
        IN_QUOTED_TERM = 50,
        END_OF_QUOTED_TERM = 51,
    } state = State::START_OF_START_OF_TERM;

    // term separator
    const uint8_t separator;

    // pointer to memory where term will be assembled
    uint8_t *term;

    // amount of memory term points to
    const uint8_t term_len;

    // offset into term for next character
    uint8_t term_ofs;

    void set_state(State newstate) {
        state = newstate;
    }

    AP_CSVReader::RetCode handle_unquoted_term(uint8_t c);
    AP_CSVReader::RetCode handle_quoted_term(uint8_t c);
};
