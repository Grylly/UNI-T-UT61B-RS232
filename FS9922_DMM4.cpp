/* 
 * File:   FS9922_DMM4.cpp
 * Author: Grylly 
 */

#include "FS9922_DMM4.hpp"
#include <fcntl.h>       
#include <termios.h>    
#include <unistd.h>      
#include <errno.h>       
#include <sys/ioctl.h>   
#include <chrono>
#include <string.h>
#include <string>
#include <iostream>

FS9922_DMM4::FS9922_DMM4 () {
 }

FS9922_DMM4::FS9922_DMM4 (const FS9922_DMM4& orig) { }

FS9922_DMM4::~FS9922_DMM4 () { }

bool
FS9922_DMM4::init ()
{
  this->serial = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (this->serial, &tty) != 0)
    {
      std::cout << "Error " << errno << " from tcgetattr: " << strerror (errno) << std::endl;

    }
  else
    {
      cfsetospeed (&tty, (speed_t) B2400);
      cfsetispeed (&tty, (speed_t) B2400);

      tty.c_cflag &= ~PARENB;
      tty.c_cflag &= ~CSTOPB;
      tty.c_cflag &= ~CSIZE;
      tty.c_cflag |= CS8;

      tty.c_cflag |= CRTSCTS;
      tty.c_cc[VMIN] = 1;
      tty.c_cc[VTIME] = 5;
      tty.c_cflag |= CREAD;

      cfmakeraw (&tty);

      tcflush (this->serial, TCIFLUSH);
      if (tcsetattr (this->serial, TCSANOW, &tty) != 0)
        {
          std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        }
      else
        {
          this->isInit = true;
          cout << "RS232 INIT" << endl;

          int RTS_flag;
          RTS_flag = TIOCM_RTS;
          ioctl (this->serial, TIOCMBIC, &RTS_flag); //Set RTS pin
          this->startTimestamp = this->getTimestampMs ();
        }
    }

  return this->isInit;
}

void
FS9922_DMM4::readFrame ()
{
  memset (this->bytes, 0, sizeof (this->bytes));
  uint8_t readSize = 0;

  while (readSize < this->frameLen)
    {
      readSize += read (this->serial, &this->bytes[readSize], this->frameLen - readSize);
    }
}

void
FS9922_DMM4::parser ()
{
  string output;
  this->readFrame ();

  output.append ((char *) this->bytes, 6);
  uint8_t pointPos = this->bytes[6] - 48;

  if (pointPos)
    {
      pointPos++;
      if (pointPos > 4) pointPos = 4;
      output.insert (pointPos, ".");
    }
  
  uint8_t SB1 = this->bytes[7];
  uint8_t SB2 = this->bytes[8];
  uint8_t SB3 = this->bytes[9];
  uint8_t SB4 = this->bytes[10];

  if (SB2 & (1 << 1)) output += "n";
  if (SB3 & (1 << 7)) output += "u";
  if (SB3 & (1 << 6)) output += "m";
  if (SB3 & (1 << 5)) output += "k";
  if (SB3 & (1 << 4)) output += "M";
  if (SB3 & (1 << 1)) output += "%";

  if (SB4 & (1 << 7)) output += "V";
  if (SB4 & (1 << 6)) output += "A";
  if (SB4 & (1 << 5)) output += "Ω";
  if (SB4 & (1 << 4)) output += "hFE";
  if (SB4 & (1 << 3)) output += "Hz";
  if (SB4 & (1 << 2)) output += "F";
  if (SB4 & (1 << 1)) output += "°C";
  if (SB4 & (1 << 0)) output += "°F";

  if (output[0] == 0x2B) output.erase (0, 1);

  float time = ((float) (this->getTimestampMs () - this->startTimestamp)) / 1000.0;
  cout << "[" << time << "s] " << output << endl;
}

uint64_t
FS9922_DMM4::getTimestampMs ()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now ().time_since_epoch ()).count ();
}
