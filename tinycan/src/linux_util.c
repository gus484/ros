/**************************************************************************/
/*                        Linux Utilitiy Funktionen                       */
/* ---------------------------------------------------------------------- */
/*  Beschreibung    : - Nachbildung der "kbhit" Funktion                  */
/*                                                                        */
/*  Version         : 1.00                                                */
/*  Datei Name      : linux_util.c                                        */
/* ---------------------------------------------------------------------- */
/*  Datum           : 07.03.07                                            */
/*  Autor           : Demlehner Klaus, MHS-Elektronik, 94149 Kößlarn      */
/*                    info@mhs-elektronik.de  www.mhs-elektronik.de       */
/* ---------------------------------------------------------------------- */
/*  Compiler        : GNU C Compiler                                      */
/**************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
//#include <sys/select.h>
#include <sys/ioctl.h>  // neu
#include <termios.h>
//#include <stropts.h>  // nicht in osx

#ifndef STDIN_FILENO
  #define STDIN_FILENO 0
#endif


static struct termios OldTermattr;


void UtilCleanup(void);


int UtilInit(void)
{
struct termios termattr;

if (tcgetattr(STDIN_FILENO, &OldTermattr) < 0)
  return(-1);

memcpy(&termattr, &OldTermattr, sizeof(struct termios));

termattr.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
termattr.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
termattr.c_cflag &= ~(CSIZE | PARENB);
termattr.c_cflag |= CS8;
termattr.c_oflag &= ~(OPOST);

termattr.c_cc[VMIN] = 1;  /* or 0 for some Unices;  see note 1 */
termattr.c_cc[VTIME] = 0;

if (tcsetattr (STDIN_FILENO, TCSANOW, &termattr) < 0)
  return(-1);
return(atexit(UtilCleanup));
}


void UtilCleanup(void)
{
tcsetattr(STDIN_FILENO, TCSAFLUSH, &OldTermattr);
}



int KeyHit(void)
{
int bytes_waiting;

ioctl(STDIN_FILENO, FIONREAD, &bytes_waiting);
return(bytes_waiting);
}


