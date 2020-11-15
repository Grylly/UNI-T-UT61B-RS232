
/* 
 * File:   FS9922_DMM4.hpp
 * Author: Grylly
 */

#ifndef FS9922_DMM4_HPP
#define FS9922_DMM4_HPP

/*
 * Page: 31
 * https://cdn.hackaday.io/files/21300911299200/FS9922-DMM4-DS-13_EN.pdf
 */

#include <stdint.h>

using namespace std;

class FS9922_DMM4 {
public:
    FS9922_DMM4();
    FS9922_DMM4(const FS9922_DMM4& orig);
    virtual ~FS9922_DMM4();

    bool init();
    void parser();   

private:
    int serial;
    bool isInit = false;

    const static uint8_t frameLen = 14;
    uint8_t bytes[frameLen];

    uint64_t startTimestamp = 0;
    uint64_t getTimestampMs();

    void readFrame();
};

#endif /* FS9922_DMM4_HPP */

