#include "Copter.h"
#include <stdio.h>


static const char* CSV_FILE_PATH   = "path_to_file"; 
static const char* EXPECTED_COL_1  = "col1";
static const char* EXPECTED_COL_2  = "col2";
static const char* EXPECTED_COL_3  = "col3";

void Copter::check_csv_integrity()
{
    FILE *fp = fopen(CSV_FILE_PATH, "r");

    if (fp == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "CSV FAIL: File not found: %s", CSV_FILE_PATH);
        return;
    }

    // 2. Read the first line (Header)
    char line_buffer[128];
    if (fgets(line_buffer, sizeof(line_buffer), fp) == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "CSV FAIL: File is empty");
        fclose(fp);
        return;
    }

    fclose(fp);

    // 3. Parse and Check Columns
    const char* delimiters = ",\r\n"; 
    char* token;
    char* saveptr; 

    // --- Check Column 1 ---
    token = strtok_r(line_buffer, delimiters, &saveptr);
    if (token == nullptr || strcmp(token, EXPECTED_COL_1) != 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "CSV FAIL: Col1 mismatch. Got '%s', expected '%s'", 
                        (token ? token : "NULL"), EXPECTED_COL_1);
        return;
    }

    // --- Check Column 2 ---
    token = strtok_r(nullptr, delimiters, &saveptr);
    if (token == nullptr || strcmp(token, EXPECTED_COL_2) != 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "CSV FAIL: Col2 mismatch. Got '%s', expected '%s'", 
                        (token ? token : "NULL"), EXPECTED_COL_2);
        return;
    }

    // --- Check Column 3 ---
    token = strtok_r(nullptr, delimiters, &saveptr);
    if (token == nullptr || strcmp(token, EXPECTED_COL_3) != 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "CSV FAIL: Col3 mismatch. Got '%s', expected '%s'", 
                        (token ? token : "NULL"), EXPECTED_COL_3);
        return;
    }

}