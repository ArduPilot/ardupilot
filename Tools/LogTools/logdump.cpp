#include "DFParser.h"
#include <iostream>
#include "json.hpp"
#include <fcntl.h>
#include <sys/mman.h>
#include <iomanip>
#include <sstream>
#include <boost/algorithm/string/join.hpp> // Include for boost::split


using namespace std;
using namespace nlohmann;

int main(int argc, char** argv) {
    if (argc < 2) {
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

    uint64_t count=0;
    uint8_t type;
    DFParser::message_t msg;
    while(parser.next_message(msg)) {
        auto& fields = parser.get_fields(msg);

        cout << parser.get_message_name(msg) << " {";

        bool first_iter = true;
        for (auto field : fields) {
            if (!first_iter) {
                cout << ", ";
            } else {
                first_iter = false;
            }

            cout << field.name << " : " << parser.get_value_string(msg,field);
        }

        cout << "}" << endl;
    }

    return 0;
}
