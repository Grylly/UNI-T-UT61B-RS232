
/* 
 * File:   main.cpp
 * Author: Grylly 
 */

#include <cstdlib>
#include "FS9922_DMM4.hpp"
using namespace std;

int
main (int argc, char** argv)
{

  FS9922_DMM4 * uni_t_UT61B = new FS9922_DMM4 ();

  if (uni_t_UT61B->init () == false) return 1;

  while (1)
    {
      uni_t_UT61B->parser ();
    }

  return 0;
}

