#include <AP_gtest.h>
#include <AP_Common/AP_Common.h>

#include <AP_Math/AP_Math.h>

#include <AP_CSVReader/AP_CSVReader.h>

TEST(AP_CSVReader, basic)
{
    static const char *basic_csv =
        "A 1\n"
        "B 2\n"
        "C 3\n"
        "Fred 31\n"
        ;
    static const char *basic_csv_crlf =
        "A 1\r\n"
        "B 2\r\n"
        "C 3\r\n"
        "Fred 31\r\n"
        ;
    static const char *basic_csv_results[][2] = {
        {"A", "1"},
        {"B", "2"},
        {"C", "3"},
        {"Fred", "31"}
    };

    uint8_t term[16];
    AP_CSVReader csvreader{term, ARRAY_SIZE(term), ' '};

    const char *csvs[] {
        basic_csv,
        basic_csv_crlf
    };

    for (const char *csv : csvs) {
        uint8_t termcount = 0;
        uint8_t linecount = 0;
        for (uint8_t i=0; i<strlen(csv); i++) {
            switch (csvreader.feed(csv[i])) {
            case AP_CSVReader::RetCode::ERROR:
                abort();
            case AP_CSVReader::RetCode::OK:
                continue;
            case AP_CSVReader::RetCode::TERM_DONE:
                EXPECT_STREQ(basic_csv_results[linecount][termcount], (char*)term);
                termcount++;
                continue;
            case AP_CSVReader::RetCode::VECTOR_DONE:
                EXPECT_STREQ(basic_csv_results[linecount][termcount], (char*)term);
                termcount++;
                EXPECT_EQ(termcount, 2);
                termcount = 0;
                linecount++;
                continue;
            }
        }

        EXPECT_EQ(linecount, 4);
        EXPECT_EQ(termcount, 0);
    }
}

TEST(AP_CSVReader, commabasic)
{
    static const char *basic_csv =
        "A,1\n"
        "B,2\n"
        "C,3\n"
        "Fred,31\n"
        ;
    static const char *basic_csv_results[][2] = {
        {"A", "1"},
        {"B", "2"},
        {"C", "3"},
        {"Fred", "31"}
    };

    uint8_t term[16];
    AP_CSVReader csvreader{term, ARRAY_SIZE(term), ','};

    uint8_t termcount = 0;
    uint8_t linecount = 0;
    for (uint8_t i=0; i<strlen(basic_csv); i++) {
        switch (csvreader.feed(basic_csv[i])) {
        case AP_CSVReader::RetCode::ERROR:
            abort();
        case AP_CSVReader::RetCode::OK:
            continue;
        case AP_CSVReader::RetCode::TERM_DONE:
            EXPECT_STREQ(basic_csv_results[linecount][termcount], (char*)term);
            termcount++;
            continue;
        case AP_CSVReader::RetCode::VECTOR_DONE:
            EXPECT_STREQ(basic_csv_results[linecount][termcount], (char*)term);
            termcount++;
            EXPECT_EQ(termcount, 2);
            termcount = 0;
            linecount++;
            continue;
        }
    }

    EXPECT_EQ(linecount, 4);
    EXPECT_EQ(termcount, 0);
}

TEST(AP_CSVReader, missinglastcr)
{
    static const char *basic_csv =
        "A,1"
        ;
    uint8_t term[16];
    AP_CSVReader csvreader{term, ARRAY_SIZE(term), ','};

    uint8_t termcount = 0;
    uint8_t linecount = 0;
    for (uint8_t i=0; i<strlen(basic_csv); i++) {
        switch (csvreader.feed(basic_csv[i])) {
        case AP_CSVReader::RetCode::ERROR:
            abort();
        case AP_CSVReader::RetCode::OK:
            continue;
        case AP_CSVReader::RetCode::TERM_DONE:
            if (linecount == 0 && termcount == 0) {
                EXPECT_STREQ((char*)term, "A");
            }
            termcount++;
            continue;
        case AP_CSVReader::RetCode::VECTOR_DONE:
            abort();
        }
    }

    EXPECT_STREQ((char*)term, "1");
}

// A Saleae async-serial capture row looks like:
//     Time [s],Value,Parity Error,Framing Error
//     0.000010,0xZZ,,
// AP_HAL_SITL's read_from_async_csv() feeds such rows through AP_CSVReader
// and then decodes the "Value" term's two hex digits (term[2],term[3]) with
// hex_twochars_to_uint8().  This test pushes a row whose Value has an invalid
// hex character through the parser and confirms where the error is (and is
// not) caught: AP_CSVReader splits the row cleanly -- it does no hex
// validation -- and it is the subsequent hex decode that rejects the byte.
TEST(AP_CSVReader, saleae_value_invalid_hex)
{
    static const char *row =
        "0.000010,0xZZ,,\r\n"
        ;

    uint8_t term[64];
    AP_CSVReader csvreader{term, ARRAY_SIZE(term), ','};

    uint8_t value_term[64] = {};
    uint8_t termcount = 0;
    for (uint8_t i=0; i<strlen(row); i++) {
        switch (csvreader.feed(row[i])) {
        case AP_CSVReader::RetCode::ERROR:
            // the parser must NOT flag an error: "0xZZ" is a perfectly valid
            // CSV term, it is just not valid hex
            FAIL() << "unexpected ERROR from parser at offset " << (int)i;
            break;
        case AP_CSVReader::RetCode::OK:
            continue;
        case AP_CSVReader::RetCode::TERM_DONE:
        case AP_CSVReader::RetCode::VECTOR_DONE:
            if (termcount == 1) {  // the "Value" column
                memcpy(value_term, term, sizeof(value_term));
            }
            termcount++;
            continue;
        }
    }

    // four terms: Time, Value, Parity Error, Framing Error
    EXPECT_EQ(termcount, 4);
    // the parser handed us the Value term verbatim, bad hex and all
    EXPECT_STREQ((char*)value_term, "0xZZ");

    // ... and the hex decode (as performed by read_from_async_csv) rejects it
    uint8_t decoded;
    EXPECT_FALSE(hex_twochars_to_uint8((const char*)&value_term[2], decoded));
}

// Companion to the above: a well-formed Value term decodes to the right byte,
// confirming the decode the SITL reader relies on works end-to-end.
TEST(AP_CSVReader, saleae_value_valid_hex)
{
    static const char *row =
        "0.000010,0x55,,\r\n"
        ;

    uint8_t term[64];
    AP_CSVReader csvreader{term, ARRAY_SIZE(term), ','};

    uint8_t value_term[64] = {};
    uint8_t termcount = 0;
    for (uint8_t i=0; i<strlen(row); i++) {
        switch (csvreader.feed(row[i])) {
        case AP_CSVReader::RetCode::ERROR:
            FAIL() << "unexpected ERROR from parser at offset " << (int)i;
            break;
        case AP_CSVReader::RetCode::OK:
            continue;
        case AP_CSVReader::RetCode::TERM_DONE:
        case AP_CSVReader::RetCode::VECTOR_DONE:
            if (termcount == 1) {
                memcpy(value_term, term, sizeof(value_term));
            }
            termcount++;
            continue;
        }
    }

    EXPECT_EQ(termcount, 4);
    EXPECT_STREQ((char*)value_term, "0x55");

    uint8_t decoded;
    EXPECT_TRUE(hex_twochars_to_uint8((const char*)&value_term[2], decoded));
    EXPECT_EQ(decoded, 0x55u);
}

// A term that does not fit in the caller-supplied buffer (term_len-1 chars,
// reserving one byte for the null terminator) must return ERROR rather than
// overrun.
TEST(AP_CSVReader, term_overflow_returns_error)
{
    // term_len 4 => room for 3 characters plus a null terminator
    uint8_t term[4];
    AP_CSVReader csvreader{term, ARRAY_SIZE(term), ','};

    // first three characters are accepted
    EXPECT_EQ(csvreader.feed('A'), AP_CSVReader::RetCode::OK);
    EXPECT_EQ(csvreader.feed('B'), AP_CSVReader::RetCode::OK);
    EXPECT_EQ(csvreader.feed('C'), AP_CSVReader::RetCode::OK);
    // the fourth would overrun the buffer
    EXPECT_EQ(csvreader.feed('D'), AP_CSVReader::RetCode::ERROR);
}

// A quoted term that is followed by non-separator, non-newline data is
// malformed and must return ERROR.
TEST(AP_CSVReader, malformed_quoted_term_returns_error)
{
    uint8_t term[16];
    AP_CSVReader csvreader{term, ARRAY_SIZE(term), ','};

    EXPECT_EQ(csvreader.feed('"'), AP_CSVReader::RetCode::OK);  // open quote
    EXPECT_EQ(csvreader.feed('A'), AP_CSVReader::RetCode::OK);  // contents
    EXPECT_EQ(csvreader.feed('"'), AP_CSVReader::RetCode::OK);  // close quote
    // stray character after the closing quote is not allowed
    EXPECT_EQ(csvreader.feed('X'), AP_CSVReader::RetCode::ERROR);
}

AP_GTEST_MAIN()


int hal = 0; // bizarrely, this fixes an undefined-symbol error but doesn't raise a type exception.  Yay.
