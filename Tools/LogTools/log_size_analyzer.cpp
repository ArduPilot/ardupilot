#include "DFParser.h"
#include <iostream>
#include "json.hpp"
#include <fcntl.h>
#include <sys/mman.h>
#include <iomanip>
#include <sstream>
#include <algorithm>

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

    json stats;

    int totalcount = 0;
    int totalbytes = 0;
    stats["msgs"] = json::object();
    stats["msgs_list"] = json::array();

    auto& stats_msgs = stats["msgs"];

    while(parser.next_message(msg)) {
        auto name = parser.get_message_name(msg);

        if (!stats_msgs.contains(name)) {
            stats_msgs[name]["count"] = 0;
            stats_msgs[name]["bytes"] = 0;
        }

        auto& stats_msgs_name = stats_msgs[name];

        stats_msgs_name["count"] = (double)stats_msgs_name["count"] + 1;
        stats_msgs_name["bytes"] = (double)stats_msgs_name["bytes"] + parser.get_message_size_including_header(msg);

        totalcount++;
        totalbytes += parser.get_message_size_including_header(msg);
    }

    for (auto& [key,value] : stats_msgs.items()) {
        json j = value;
        j["name"] = key;
        stats["msgs_list"].push_back(j);
//         cout << key << "," << (int)value["bytes"] << endl;
    }

    std::sort(stats["msgs_list"].begin(), stats["msgs_list"].end(),
        [](const json &a, const json &b){
            return a["bytes"] > b["bytes"];
        });

    for (auto& msgstat : stats["msgs_list"]) {
        cout << msgstat["name"] << " : " << (int)msgstat["bytes"] << endl;
    }

    cout << "totalbytes " << totalbytes << endl;
    cout << "totalcount " << totalcount << endl;

    return 0;
}
