#ifndef __GLOBAL_H__
#define __GLOBAL_H__


#ifdef __WIN32__
  #define COM1 "Port=1"
  #define COM2 "Port=2"
  #define COM3 "Port=3"
  #define COM4 "Port=4"
#else
  #define COM1 "ComDeviceName=/dev/ttyS0"
  #define COM2 "ComDeviceName=/dev/ttyS1"
  #define COM3 "ComDeviceName=/dev/ttyS2"
  #define COM4 "ComDeviceName=/dev/ttyS3"
#endif

#define USB     0
#define SERIELL 1



#ifdef __WIN32__
  // **** Windows
  #if COM_MODE == USB
    #ifdef HW_SNR
      #define DEVICE_OPEN "Snr="HW_SNR
    #endif
  #else
    #define DEVICE_OPEN COM_PORT";BaudRate="BAUD_RATE
  #endif
  #define TREIBER_NAME NULL   
  #include "windows.h"
  #define CALLBACK_TYPE CALLBACK

  #define UtilInit()
  #define KeyHit kbhit
#else
  // **** Linux
  #if COM_MODE == USB
    #ifdef HW_SNR
      #define DEVICE_OPEN "Snr="HW_SNR
    #endif
  #else
    #define DEVICE_OPEN COM_PORT";BaudRate="BAUD_RATE
  #endif
  #ifdef __APPLE__
    #define TREIBER_NAME "./../../../can_api/libmhstcan.dylib"
  #else
    //#define TREIBER_NAME "./../../../can_api/libmhstcan.so"
    #define TREIBER_NAME "/home/vmuser/catkin_ws/src/can/lib/libmhstcan.so"  
  #endif  
  #include <unistd.h>
  #include "linux_util.h"

  #define Sleep(x) usleep((x) * 1000)
  #define CALLBACK_TYPE

#endif

#ifndef TREIBER_INIT
#define TREIBER_INIT NULL
#endif

#ifndef DEVICE_OPEN
#define DEVICE_OPEN NULL
#endif


#endif
