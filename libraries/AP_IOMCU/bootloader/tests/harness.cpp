/* GPLv3 or later. Native transport/flash model for the production protocol. */
#include "../protocol.h"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <deque>
#include <vector>

using namespace IOMCU_BL;
static Protocol protocol;
static uint32_t flash_words[APP_SIZE / 4];
static std::deque<uint8_t> input;
static std::vector<uint8_t> output;
static unsigned boots;
static int fail_write;
static bool fail_erase;

namespace IOMCU_BL
{
int receive(uint32_t)
{
    if (input.empty()) {
        return -1;
    }
    const uint8_t byte = input.front();
    input.pop_front();
    return byte;
}

void send(uint8_t byte)
{
    output.push_back(byte);
}

uint32_t read_word(uint32_t offset)
{
    assert((offset & 3) == 0 && offset < APP_SIZE);
    return flash_words[offset / 4];
}

bool erase_application()
{
    if (fail_erase) {
        return false;
    }
    std::fill(std::begin(flash_words), std::end(flash_words), ERASED);
    return true;
}

bool write_word(uint32_t offset, uint32_t word)
{
    assert((offset & 3) == 0 && offset < APP_SIZE);
    if (int(offset) == fail_write) {
        return false;
    }
    uint32_t &dst = flash_words[offset / 4];
    if ((dst & word) != word) {
        return false;
    }
    dst = word;
    return true;
}
}

extern "C" {
    bool valid_vectors(uint32_t sp, uint32_t pc)
    {
        return valid_app_vectors(sp, pc);
    }

    void reset(bool erase)
    {
        protocol = Protocol{};
        input.clear();
        output.clear();
        boots = 0;
        fail_write = -1;
        fail_erase = false;
        if (erase) {
            erase_application();
        }
    }

    unsigned exchange(const uint8_t *data, unsigned count, uint8_t *reply)
    {
        input.insert(input.end(), data, data + count);
        output.clear();
        while (!input.empty()) {
            const uint8_t opcode = input.front();
            input.pop_front();
            if (protocol.command(opcode)) {
                boots++;
            }
        }
        std::copy(output.begin(), output.end(), reply);
        return output.size();
    }

    uint32_t flash_word(unsigned offset)
    {
        return read_word(offset);
    }
    unsigned boot_count()
    {
        return boots;
    }
    bool active()
    {
        return protocol.active();
    }
    void inject_write_failure(int offset)
    {
        fail_write = offset;
    }
    void inject_erase_failure(bool fail)
    {
        fail_erase = fail;
    }
}
