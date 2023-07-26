
#include "SIM_Aircraft.h"
#include <AP_Filesystem/AP_Filesystem.h>

#if USE_PICOJSON
#include "picojson.h"

/*
  load JSON file, returning a picojson object or nullptr on failure
 */
void *load_json(const char *filename)
{
    struct stat st;
    if (AP::FS().stat(filename, &st) != 0) {
        ::printf("No such json file %s\n", filename);
        return nullptr;
    }
    int fd = AP::FS().open(filename, O_RDONLY);
    if (fd == -1) {
        ::printf("failed to open json %s\n", filename);
        return nullptr;
    }
    char buf[st.st_size+1];
    memset(buf, '\0', sizeof(buf));
    if (AP::FS().read(fd, buf, st.st_size) != st.st_size) {
        ::printf("failed to read json %s\n", filename);
        AP::FS().close(fd);
        return nullptr;
    }
    AP::FS().close(fd);

    char *start = strchr(buf, '{');
    if (!start) {
        ::printf("Invalid json %s", filename);
        return nullptr;
    }

    /*
      remove comments, as not allowed by the parser
     */
    for (char *p = strchr(start,'#'); p; p=strchr(p+1, '#')) {
        // clear to end of line
        do {
            *p++ = ' ';
        } while (*p != '\n' && *p != '\r' && *p);
    }

    picojson::value *obj = new picojson::value;
    if (obj == nullptr) {
        ::printf("Invalid allocate json for %s", filename);
        return nullptr;
    }
    std::string err = picojson::parse(*obj, start);
    if (!err.empty()) {
        ::printf("parse failed for json %s", filename);
        delete obj;
        return nullptr;
    }

    return obj;
}
#endif // USE_PICOJSON
