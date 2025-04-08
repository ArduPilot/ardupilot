#include "AP_CSVReader.h"

#include <AP_Common/AP_Common.h>

#include <stdio.h>

AP_CSVReader::RetCode AP_CSVReader::handle_unquoted_term(uint8_t c)
{
    if (c == separator) {
        set_state(State::START_OF_START_OF_TERM);
        return RetCode::TERM_DONE;
    }
    switch (c) {
    case '\r':
        set_state(State::END_OF_VECTOR_CR);
        return RetCode::VECTOR_DONE;
    case '\n':
        set_state(State::START_OF_START_OF_TERM);
        return RetCode::VECTOR_DONE;
    default:
        if (term_ofs >= term_len-1) { // -1 for null termination
            return RetCode::ERROR;
        }
        term[term_ofs++] = c;
        term[term_ofs] = '\0';
        return RetCode::OK;
    }
}

AP_CSVReader::RetCode AP_CSVReader::handle_quoted_term(uint8_t c)
{
    if (c == '"') {
        set_state(State::END_OF_QUOTED_TERM);
        return RetCode::OK;
    }
    if (state == State::END_OF_QUOTED_TERM) {
        if (c == separator) {
            set_state(State::START_OF_START_OF_TERM);
            return RetCode::TERM_DONE;
        }

        switch (c) {
        case '\r':
            set_state(State::END_OF_VECTOR_CR);
            return RetCode::VECTOR_DONE;
        case '\n':
            set_state(State::START_OF_START_OF_TERM);
            return RetCode::VECTOR_DONE;
        }
        return RetCode::ERROR;
    }

    // still within the quoted term, append to current value
    if (term_ofs >= term_len-1) { // -1 for null termination
        return RetCode::ERROR;
    }
    term[term_ofs++] = c;
    term[term_ofs] = '\0';
    return RetCode::OK;
}

AP_CSVReader::RetCode AP_CSVReader::feed(uint8_t c)
{
    if (term_len == 0) {
        return RetCode::ERROR;
    }

again:
    switch (state) {
    case State::START_OF_START_OF_TERM:
        term_ofs = 0;
        term[term_ofs] = '\0';
        state = State::START_OF_TERM;
        FALLTHROUGH;
    case State::START_OF_TERM:
        // if (c == '"') {
        //     set_state(State::START_OF_QUOTED_TERM);
        //     return RetCode::OK;
        // }
        if (c == '"') {
            set_state(State::IN_QUOTED_TERM);
            return RetCode::OK;
        } else {
            set_state(State::IN_UNQUOTED_TERM);
            return handle_unquoted_term(c);
        }
    case State::END_OF_VECTOR_CR:
        if (c == '\n') {
            set_state(State::START_OF_START_OF_TERM);
            return RetCode::OK;
        }
        set_state(State::START_OF_START_OF_TERM);
        goto again;
    case State::IN_UNQUOTED_TERM:
        return handle_unquoted_term(c);
    case State::IN_QUOTED_TERM:
    case State::END_OF_QUOTED_TERM:
        return handle_quoted_term(c);
    }

    return RetCode::ERROR;
}
