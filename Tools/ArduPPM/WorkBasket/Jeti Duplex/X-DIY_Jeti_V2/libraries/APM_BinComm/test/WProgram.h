
#ifndef WPROGRAM_H
#define WPROGRAM_H

class Stream {
public:
        void    write(uint8_t val);
        int     available(void);
        int     read(void);
};


extern unsigned int millis(void);

#endif
