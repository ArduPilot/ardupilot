#include "AP_GPS_BinaryCRCMessageParser.h"

extern const AP_HAL::HAL& hal;

#define AP_GPS_COMMON_BINARY_CRC_MSG_PARSER_DEBUG 0

#if AP_GPS_COMMON_BINARY_CRC_MSG_PARSER_DEBUG
# define Debug(fmt, args ...)                  \
do {                                            \
    hal.console->printf("%s:%d: " fmt "\n",     \
                        __FUNCTION__, __LINE__, \
                        ## args);               \
    hal.scheduler->delay(1);                    \
} while(0)
#else
 # define Debug(fmt, args ...)
#endif

AP_GPS_BinaryCRCMessageParser::AP_GPS_BinaryCRCMessageParser(const std::vector<uint8_t> &preambles,  uint8_t min_header_length, uint8_t max_header_length, uint16_t max_body_length)
    :_preambles(preambles),
    _min_header_length(min_header_length),
    _max_header_length(max_header_length),
    _max_body_length(max_body_length)   
{
}

AP_GPS_BinaryCRCMessageParser::~AP_GPS_BinaryCRCMessageParser()
{
}

bool AP_GPS_BinaryCRCMessageParser::parse(uint8_t data)
{
    uint8_t *msg_header_buff = get_message_header_buff();
    uint8_t *msg_body_buff = get_message_body_buff();
    reset:
        switch (_decode_step) {
            default:
            case 0: //preambles
                if (data == _preambles[_preamble_step]) {
                    msg_header_buff[_preamble_step] = data;
                    _preamble_step++;
                    _read_size = _preamble_step;
                    Debug("Got preamble %x",data);
                } else if(0 == _preamble_step) {  // failed check the first preamble
                    break;
                } else {
                    reset_parse();
                    goto reset;
                }       
                if(_preamble_step >= _preambles.size()) {   // finish preambles 
                    _decode_step++;
                    Debug("Preambles exit");
                }
                break;
            case 1: //get real header length
                msg_header_buff[_read_size] = data;
                _read_size++;
                if(_read_size >= _min_header_length) {
                    uint8_t header_len = get_message_header_length_from_header_buff();
                    if(header_len < _min_header_length || header_len > _max_header_length) {
                        Debug("Wrong header length : %u in header buff",header_len);
                        Debug("buffinfo:%x,%x,%x,%u,size:%lu",msg_header_buff[0],msg_header_buff[1],msg_header_buff[2],msg_header_buff[3],_read_size);
                        reset_parse();
                        goto reset;
                    }
                    _real_header_length = header_len;
                    _decode_step++;
                    Debug("Got real message header length: %u",header_len);
                }
                break;
            case 2: // header data
                if (_read_size < _real_header_length) {
                    msg_header_buff[_read_size] = data;
                    _read_size++;
                    
                } else {
                    uint16_t body_len = get_message_body_length_from_header_buff();
                    if(body_len > _max_body_length) {
                        Debug("Wrong body length : %u in header buff",body_len);
                        reset_parse();
                        goto reset;
                    }
                    _real_body_length = body_len;
                    _decode_step++;
                    Debug("Header data exit,Got real body length:%u",body_len);
                    goto reset; // current data have not been readed, goto next case to read the data
                }   
                break;
            case 3: //body data
                if(_read_size < (_real_header_length + _real_body_length)) {
                    msg_body_buff[_read_size - _real_header_length] = data;
                    _read_size++;
                    
                } else {
                    _decode_step++;
                    Debug("Body data exit");
                    goto reset; // current data have not been readed, goto next case to read the data 
                }   
                break;
            case 4: //CRC1
                _msg_crc = (uint32_t) (data << 0);
                _decode_step++;
                break;
            case 5: //CRC2
                _msg_crc += (uint32_t) (data << 8);
                _decode_step++;
                break;
            case 6: //CRC3
                _msg_crc += (uint32_t) (data << 16);
                _decode_step++;
                break;
            case 7: //CRC4
                _msg_crc += (uint32_t) (data << 24);
                reset_parse();

                uint32_t real_crc = crc_crc32((uint32_t)0, msg_header_buff, (uint32_t)_real_header_length);
                real_crc = crc_crc32(real_crc, msg_body_buff, (uint32_t)_real_body_length);

                if(real_crc != _msg_crc){
                    Debug("CRC check failed,msg_crc:%lx,real_crc:%lx",_msg_crc,real_crc);
                    goto reset;
                }
                Debug("Got new message");
                return process_message();   /**< One whole message received */
        }

    return false;
}

void AP_GPS_BinaryCRCMessageParser::reset_parse()
{
    _decode_step = 0;
    _read_size = 0;
    _preamble_step = 0;
}