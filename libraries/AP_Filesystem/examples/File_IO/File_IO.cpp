//Simple File I/O example

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <stdio.h>

//set according to the data size in file
#define BUFFER_SIZE 50

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup();
void loop();

static AP_BoardConfig board_config;

int file_count = 0;
const char *dir_path = nullptr;
static void write_file(const char *);
static void read_file(const char *);

void setup()
{
    board_config.init();

    //creating a test directory to store all files
    dir_path = "./test_dir";
    hal.console->printf("Creating directory - %s\n", dir_path);
    int ret = AP::FS().mkdir(dir_path);
    if (ret == -1) {
        if (errno == EEXIST) {
            hal.console->printf("Directory - %s already exist\n", dir_path);
        } else {
            hal.console->printf("Failed to create directory %s - %s\n", dir_path, strerror(errno));
        }
    }
}

void loop()
{
    //creating 10 files in test directory
    if (file_count == 10) {
        hal.console->printf("Finished creating files\n\n");
        hal.scheduler->delay(1000);
        return;
    }

    file_count += 1;
    char file_path[100];
    snprintf(file_path, sizeof(file_path), "%s/test_%d.txt", dir_path, file_count);
    write_file(file_path);
    hal.scheduler->delay(1000);
    read_file(file_path);
    hal.scheduler->delay(1000);
}

static void write_file(const char *file_path)
{
    char write_buf[BUFFER_SIZE];

    //creating or opening a file in write mode
    const int open_fd = AP::FS().open(file_path, O_WRONLY | O_CREAT | O_TRUNC);
    if (open_fd == -1) {
        hal.console->printf("Open %s failed - %s\n", file_path, strerror(errno));
        return;
    }

    //writing to file
    const char *s = "Hello_World";
    hal.console->printf("Writing to file %s => %s_%d\n", file_path, s, file_count);
    snprintf(write_buf, sizeof(write_buf), "%s_%d\n", s, file_count);
    const ssize_t to_write = strlen(write_buf);
    const ssize_t write_size = AP::FS().write(open_fd, write_buf, to_write);
    if (write_size == -1 or write_size != to_write) {
        hal.console->printf("Write failed - %s\n", strerror(errno));
        return;
    }

    //closing file after writing
    AP::FS().close(open_fd);
}

static void read_file(const char *file_path)
{
    char read_buf[BUFFER_SIZE];

    //opening a file in read mode
    const int open_fd = AP::FS().open(file_path, O_RDONLY);
    if (open_fd == -1) {
        hal.console->printf("Open %s failed\n", file_path);
        return;
    }

    //reading from file
    const ssize_t read_size = AP::FS().read(open_fd, read_buf, sizeof(read_buf));
    if (read_size == -1) {
        hal.console->printf("Read failed - %s\n", strerror(errno));
        return;
    }

    hal.console->printf("Reading from file %s => %s\n", file_path, read_buf);

    //closing file after reading
    AP::FS().close(open_fd);
}

AP_HAL_MAIN();
