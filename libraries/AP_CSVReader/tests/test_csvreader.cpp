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

AP_GTEST_MAIN()


int hal = 0; // bizarrely, this fixes an undefined-symbol error but doesn't raise a type exception.  Yay.
