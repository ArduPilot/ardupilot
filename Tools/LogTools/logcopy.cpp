#include "DFParser.h"
#include <iostream>
#include "json.hpp"
#include <fcntl.h>
#include <sys/mman.h>
#include <iomanip>
#include <sstream>
#include <boost/algorithm/string/join.hpp> // Include for boost::split
#include <fstream>

using namespace std;
using namespace nlohmann;

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "too few arguments" << endl;
        return 1;
    }

    std::ios::sync_with_stdio(false);

    int fp = open(argv[1], O_RDONLY, 0);
    size_t logdata_len = lseek(fp, 0, SEEK_END);
    lseek(fp, 0, SEEK_SET);
    uint8_t* logdata = (uint8_t*)mmap(0, logdata_len, PROT_READ, MAP_SHARED, fp, 0);
    madvise(logdata, logdata_len, POSIX_MADV_SEQUENTIAL);

    DFParser parser(logdata, logdata_len);

    DFParser::message_t msg;

    auto outfile = fopen(argv[2], "wb");

    while(parser.next_message(msg)) {
        // insert filter logic here

        fwrite(parser.get_message_pointer_including_header(msg), parser.get_message_size_including_header(msg), 1, outfile);
    }

    fclose(outfile);
    return 0;
}
