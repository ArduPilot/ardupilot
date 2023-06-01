#pragma once

#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split
#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>
#include <map>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "BlockingCollection.h"
#pragma GCC diagnostic pop
#include <thread>
#include "dtoa_milo.h"

using namespace std;
using namespace code_machina;

#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

class DFParser {
public:
    typedef struct {
        uint8_t type;
        uint8_t* body;
    } message_t;

    typedef struct {
        typedef struct {
            char typechar;
            string name;
            int ofs;
            int len;
        } Field;

        uint8_t type;
        string name;
        int len;
        vector<Field> fields;
    } Format;

    vector<Format::Field>& get_fields(message_t msg) {
        return _formats[msg.type].fields;
    }

    DFParser(const DFParser&) = delete;
    DFParser& operator=(const DFParser&) = delete;
    DFParser(uint8_t* logdata, size_t logdata_len) : _logdata(logdata), _logdata_len(logdata_len) {
        _formats[0x80] = {
            0x80,
            "FMT",
            89,
            {
                {'B',"Type",0,1},
                {'B',"Length",1,1},
                {'n',"Name",2,4},
                {'N',"Format",6,16},
                {'Z',"Columns",22,64}
            }
        };

        // start parsing thread
        thread parse_thread(&DFParser::parse_thread_func, this);
        parse_thread.detach();
    }

    string get_message_name(const message_t& msg) {
        return _formats[msg.type].name;
    }

    uint8_t* get_message_pointer_including_header(const message_t& msg) {
        return msg.body-3;
    }

    size_t get_message_size_including_header(const message_t& msg) {
        return _formats[msg.type].len;
    }

    Format::Field* get_field_definition(const message_t& msg, const string& fieldname) {
        auto& fields = _formats[msg.type].fields;
        for (auto& f : fields) {
            if (f.name == fieldname) {
                return &f;
            }
        }
        return NULL;
    }

    template <typename T>
    bool get_scalar_field(const message_t& msg, const Format::Field& field, T& ret) {
        switch(field.typechar) {
            case 'B':
            case 'M':
                ret = (T)*reinterpret_cast<const uint8_t*>(&msg.body[field.ofs]);
                return true;
            case 'b':
                ret = (T)*reinterpret_cast<const int8_t*>(&msg.body[field.ofs]);
                return true;
            case 'h':
                ret = (T)*reinterpret_cast<const int16_t*>(&msg.body[field.ofs]);
                return true;
            case 'H':
                ret = (T)*reinterpret_cast<const uint16_t*>(&msg.body[field.ofs]);
                return true;
            case 'i':
                ret = (T)*reinterpret_cast<const int32_t*>(&msg.body[field.ofs]);
                return true;
            case 'I':
                ret = (T)*reinterpret_cast<const uint32_t*>(&msg.body[field.ofs]);
                return true;
            case 'q':
                ret = (T)*reinterpret_cast<const int64_t*>(&msg.body[field.ofs]);
                return true;
            case 'Q':
                ret = (T)*reinterpret_cast<const uint64_t*>(&msg.body[field.ofs]);
                return true;
            case 'f':
                ret = (T)*reinterpret_cast<const float*>(&msg.body[field.ofs]);
                return true;
            case 'L':
                ret = (T)*reinterpret_cast<const uint32_t*>(&msg.body[field.ofs]);
                ret /= 1e7;
                return true;
            case 'd':
                ret = (T)*reinterpret_cast<const double*>(&msg.body[field.ofs]);
                return true;
            case 'c':
                ret = (T)*reinterpret_cast<const int16_t*>(&msg.body[field.ofs]);
                ret /= 100;
                return true;
            case 'C':
                ret = (T)*reinterpret_cast<const uint16_t*>(&msg.body[field.ofs]);
                ret /= 100;
                return true;
            case 'e':
                ret = (T)*reinterpret_cast<const int32_t*>(&msg.body[field.ofs]);
                ret /= 100;
                return true;
            case 'E':
                ret = (T)*reinterpret_cast<const uint32_t*>(&msg.body[field.ofs]);
                ret /= 100;
                return true;
        }
        return false;
    }
    template<typename T> bool get_scalar_field(const message_t& msg, const Format::Field& field, uint8_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const Format::Field& field, int8_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const Format::Field& field, uint16_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const Format::Field& field, int16_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const Format::Field& field, uint32_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const Format::Field& field, int32_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const Format::Field& field, int64_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const Format::Field& field, uint64_t& ret);

    template <typename T>
    bool get_scalar_field(const message_t& msg, const string& fieldname, T& ret) {
        Format::Field* field_ptr = get_field_definition(msg, fieldname);
        if (!field_ptr) return false;
        return get_scalar_field(msg, *field_ptr, ret);

    }
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, uint8_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, int8_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, uint16_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, int16_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, uint32_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, int32_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, int64_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, uint64_t& ret);

    bool get_string_field(const message_t& msg, const Format::Field& field, string& ret) {
        switch(field.typechar) {
            case 'n':
            case 'N':
            case 'Z': {
                char str[65];
                memcpy(str,&msg.body[field.ofs],field.len);
                str[field.len] = 0;
                ret = str;
                return true;
            }
        }
        return false;
    }

    bool get_string_field(const message_t& msg, const string& fieldname, string& ret) {
        Format::Field* field_ptr = get_field_definition(msg, fieldname);
        if (!field_ptr) return false;
        return get_string_field(msg, *field_ptr, ret);
    }

    bool field_is_signed_int(const Format::Field& field) {
        switch(field.typechar) {
            case 'b':
            case 'h':
            case 'i':
            case 'q':
                return true;
        }
        return false;
    }

    bool field_is_unsigned_int(const Format::Field& field) {
        switch(field.typechar) {
            case 'B':
            case 'M':
            case 'H':
            case 'I':
            case 'Q':
                return true;
        }
        return false;
    }

    bool field_is_float(const Format::Field& field) {
        switch(field.typechar) {
            case 'f':
            case 'L':
            case 'd':
            case 'c':
            case 'C':
            case 'e':
            case 'E':
                return true;
        }
        return false;
    }

    bool field_is_string(const Format::Field& field) {
        switch(field.typechar) {
            case 'n':
            case 'N':
            case 'Z':
                return true;
        }
        return false;
    }

    string get_value_string(const message_t& msg, const Format::Field& field) {
        static char buffer[256];

        /*if (field_is_signed_int(field)) {
            ss.seekp(ios::beg);
            int64_t val;
            if(get_scalar_field(msg, field, val)) {

                return ss.str().c_str();
            }
        } else if (field_is_unsigned_int(field)) {
            ss.seekp(ios::beg);
            uint64_t val;
            if(get_scalar_field(msg, field, val)) {
                ss << val << '\0';
                return ss.str().c_str();
            }
        } else if (field_is_float(field)) {
            ss.seekp(ios::beg);
            double val;
            if(get_scalar_field(msg, field, val)) {
                ss << val << '\0';
                return ss.str().c_str();
            }
        } else */if(field_is_string(field)) {
            string ret;
            if (get_string_field(msg, field, ret)) {
                return ret;
            }
        } else {
            double val;

            if(get_scalar_field(msg, field, val)) {
                // potential optimization: dtoa_milo is very fast, but faster libraries exist
                // could also treat integers differently instead of casting
                
                if (isnan(val)) {
                    return "nan";
                } else if (isinf(val)) {
                    return "inf";
                }
                
                dtoa_milo(val,buffer);
                return buffer;
            }
        }

        return "";
    }



    void process_fmt(const message_t& msg) {
        Format new_format;

        get_scalar_field(msg, "Type", new_format.type);
        get_scalar_field(msg, "Length", new_format.len);

        get_string_field(msg, "Name", new_format.name);
        string fmtstr;
        get_string_field(msg, "Format", fmtstr);
        vector<string> fieldnames;


        string colstr;
        get_string_field(msg, "Columns", colstr);
        boost::split(fieldnames, colstr, boost::is_any_of(","));

//         cout << fmtstr << endl;
//         cout << colstr << endl;
//         cout << fieldnames.size() << endl;
//         cout << fieldnames.size() << endl;
        assert(fieldnames.size() == fmtstr.length());

        int ofs = 0;
        for (size_t i=0; i<fieldnames.size(); i++) {
            int size =  _fieldsizemap[fmtstr[i]];
            new_format.fields.push_back({fmtstr[i], fieldnames[i], ofs, size});
            ofs += size;
        }
//         cout << new_format.name << " " << fmtstr << " " << ofs << " " << new_format.len-3 << endl;
        assert(ofs == new_format.len-3);
        _formats[int(new_format.type)] = new_format;
    }

    bool next_message(message_t& msg) {
        auto status = _msgqueue.take(msg);
        return status == BlockingCollectionStatus::Ok;
    }

private:

    void parse_thread_func() {
//         _msgqueue.attach_producer();

        message_t msg;
        while (parse_next(msg)) {

            if (msg.type == 0x80) {
                process_fmt(msg);
            }

            _msgqueue.add(msg);
        }

        _msgqueue.complete_adding();
//         _msgqueue.detach_producer();
    }

    bool parse_next(message_t& msg) {
        if (_logdata_len-_ofs < 3) {
            return false;
        }
        uint8_t* header = &_logdata[_ofs];

        int skip_count = 0;
        while (header[0] != HEAD_BYTE1 || header[1] != HEAD_BYTE2 || (_formats.find(header[2]) == _formats.end())) {
            if (_logdata_len-_ofs < 3) {
                return false;
            }
            skip_count++;
            _ofs++;
            header = &_logdata[_ofs];
        }

        if (skip_count > 0) {
            cerr << "skipped " << skip_count << " bytes" << endl;
        }

        _ofs += 3;

        msg.type = header[2];

        size_t body_len = _formats[msg.type].len-3;

        if(_logdata_len-_ofs < body_len) {
            cerr << "skipped " << _logdata_len-_ofs+3 << " bytes at end of log, msg size " << body_len << endl;

            return false;
        }

        msg.body = &_logdata[_ofs];

        _ofs += body_len;

        return true;
    }

    uint8_t* _logdata;
    size_t _logdata_len;
    size_t _ofs {};
    map<uint8_t,Format> _formats;
    BlockingCollection<message_t> _msgqueue;

    map<char,uint8_t> _fieldsizemap = {
        {'b', 1},
        {'B', 1},
        {'M', 1},
        {'h', 2},
        {'H', 2},
        {'i', 4},
        {'I', 4},
        {'q', 8},
        {'Q', 8},
        {'n', 4},
        {'N', 16},
        {'Z', 64},
        {'c', 2},
        {'C', 2},
        {'e', 4},
        {'E', 4},
        {'f', 4},
        {'d', 8},
        {'L', 4},
        {'a', 64}
    };
};
