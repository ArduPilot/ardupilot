//#include "soapH.h"
/***********************************************************************
 *
 *	Global variables
 *
 ***********************************************************************/
//static Boolean HideSecretRecords;
int interopA(const char *url);
int interopB(const char *url);
int interopC(const char *url);
void displayText(char *text);

int main(int argc, char *args[])
{
  interopA("http://websrv.cs.fsu.edu/~engelen/interop2.cgi");
  interopB("http://websrv.cs.fsu.edu/~engelen/interop2B.cgi");
  interopC("http://websrv.cs.fsu.edu/~engelen/interop2C.cgi");
//    interoptA("http://unknown:9081/~engelen/interop2.cgi");
}

void displayText(char *text)
{printf(text);
  printf("\n");}



  
