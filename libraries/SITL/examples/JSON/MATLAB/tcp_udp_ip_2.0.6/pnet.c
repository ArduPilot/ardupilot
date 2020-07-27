/**********************************************************

  MEX file for the tcpip toolbox.
                                                          */
#define VERSION "Version  2.0.5  2003-09-16"

/*
%   This file(s) is part of the tcp_udp_ip toolbox (C) Peter Rydes�ter et al.
%   et al.  1998-2003 for running in MATLAB(R) as scripts and/or plug-ins.
%
%   This program is free software; you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation; either version 2 of the License, or
%   (at your option) any later version.
%
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%
%   You should have received a copy of the GNU General Public License
%   along with this program; if not, write to the Free Software
%   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
%
%   In addition, as a SPECIAL EXCEPTION, Peter Rydes�ter, SWEDEN,
%   gives permission to link the code of this program with any library,
%   and distribute linked combinations of it. You must obey the GNU
%   General Public License in all respects for all of the code in the
%   tcp_udp_ip toolbox and any . If you modify any source file included,
%   you may extend this exception to your version of the file, but you are
%   not obligated to do so.  If you do not wish to do so, delete this exception
%   statement from your version. This exception makes it possible to use
%   pnet.c (.dll) as a plug-in as it is intended and let it be (dynamical)
%   linked to MATLAB(R) or any compiled stand alone application.


  Notes for Unix implementation
  Compile this with:

  mex -O pnet.c

  Notes for Windows implementation

  When using LCC, compile this with:
  mex -O pnet.c {MATLAB_INSTALL_DIR}\sys\lcc\lib\wsock32.lib -DWIN32

  When using Visual C++, compile this with:
  mex -O pnet.c ws2_32.lib -DWIN32


  == Main Authour ==           == Windows support ==      == Earlie/Basic UDP support ==
  Peter Rydes�ter              Mario Bergeron             Mike Medeiros at 23-Jan-2001.
                               LYRtech
  �stersund, Sweden            Qu�bec, Canada
  +46 70 560 68 16
  Peter.Rydesater@mh.se        Mario.Bergeron@lyrtech.com


  SE MORE:  http://www.rydesater.com

  **********************************************************/

/******* GENERAL DEFINES *********/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

/******* WINDOWS ONLY DEFINES *********/
#ifdef WIN32
#define IFWINDOWS(dothis) dothis
#define IFUNIX(dothis)
//#include <windows.h>
#include <winsock2.h>
#define close(s) closesocket(s)
#define nonblockingsocket(s) {unsigned long ctl = 1;ioctlsocket( s, FIONBIO, &ctl );}
#define s_errno WSAGetLastError()
#define EWOULDBLOCK WSAEWOULDBLOCK
#define usleep(a) Sleep((a)/1000)
#define MSG_NOSIGNAL 0

/******* NON WINDOWS DEFINES *********/
#else
#define IFWINDOWS(dothis)
#define IFUNIX(dothis) dothis

#include <errno.h>
#define s_errno errno // ?? Is this OK ??
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#define nonblockingsocket(s)  fcntl(s,F_SETFL,O_NONBLOCK)
#endif

#ifndef INADDR_NONE
#define INADDR_NONE (-1)
#endif

// Do this hack cause SIGPIPE that kills matlab on any platform???
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

/* Include header file for matlab mex file functionality */
#include "mex.h"

/********** DEFINES related to pnet own functionality *****************/
/*   Set debuging on/off   */
#define debug_view_con_status(X)   // __debug_view_con_status(X)

#define MAX_CON         100       /* Maximum number of simultanius tcpip connections.*/
#define NAMEBUFF_LEN    100
#define MINBUFFSIZE     1000      /* readbuffer will shrink to this size if datalength is smaller. */
#define CBNAMELEN       30

#define CON_FREE         -1       /* Find a new free struct in struct array */

#define BACKLOG           50       /* How many pending connections queue will hold */

#define double_inf            HUGE_VAL
#define DEFAULT_WRITETIMEOUT  double_inf
#define DEFAULT_READTIMEOUT   double_inf
#define DEFAULT_INPUT_SIZE    50000
#define DEFAULT_USLEEP        10000

/* Different status of a con_info struct handles a file descriptor    */
#define STATUS_FREE       -1
#define STATUS_NOCONNECT   0    // Disconnected pipe that is note closed 
#define STATUS_TCP_SOCKET  1
#define STATUS_IO_OK       5    // Used for IS_... test
#define STATUS_UDP_CLIENT  6
#define STATUS_UDP_SERVER  8
#define STATUS_CONNECT     10   // Used for IS_... test
#define STATUS_TCP_CLIENT  11
#define STATUS_TCP_SERVER  12
#define STATUS_UDP_CLIENT_CONNECT 18
#define STATUS_UDP_SERVER_CONNECT 19

#define IS_STATUS_IO_OK(x) ((x)>=STATUS_IO_OK)
#define IS_STATUS_CONNECTED(x) ((x)>=STATUS_CONNECT)
#define IS_STATUS_UDP_NO_CON(x) ((x)==STATUS_UDP_CLIENT || (x)==STATUS_UDP_SERVER)
#define IS_STATUS_TCP_CONNECTED(x) ((x)==STATUS_TCP_CLIENT || (x)==STATUS_TCP_SERVER)

typedef struct
{
    char *ptr;       /* Pointer to buffert. */
    int len;         /* Length of allocated buffert. */
    int pos;         /* Length used of buffer for data storage.*/
} io_buff;

/* Structure that hold all information about a tcpip connection. */
typedef struct
{
    int fid;
    double writetimeout;
    double readtimeout;
    struct sockaddr_in remote_addr;
    io_buff write;
    io_buff read;
    int status;       /* STATUS_... FREE, NOCONNECT, SERVER, CLIENT ... */
    char callback[CBNAMELEN+1];
} con_info;


/////////////////////////////////////////////////////////////////////////////////////////////////
/* Some global variables */
int        gret_args=0;         /* Global variable that holds number of matlab return argumens returned */
int            gnlhs;           /* number of expected outputs */
mxArray       **gplhs;          /* array of pointers to output arguments */
int            gnrhs;           /* number of inputs */
const mxArray  **gprhs;         /* array of pointers to input arguments */

/* Global list with static length of connections structures holding info about current connection */
con_info con[MAX_CON];
int con_index=0;                   /* Current index possition for list of handlers */
unsigned long mex_call_counter=0;  /* Counter that counts how many calls that have been done to pnet */

/***********************************************************************/
void Print_Start_Message() {
    mexPrintf("\n===============================================================================\n"
              "Loaded pnet MEX-file for the tcp/udp/ip-toolbox Compiled @ "
              __DATE__ " " __TIME__  "\n"
              VERSION "\n"
              "Copyright (C) Peter Rydes�ter, Sweden, et al. , 1998 - 2003\n"
              "GNU General Public License, se license.txt for full license notis.\n"
              "You are allowed to (dynamicaly) link this file with non-free code. \n\n"
              "   http://www.rydesater.com \n\n"
              "===============================================================================\n\n"
             );
}

/***********************************************************************/
int myoptstrcmp(const char *s1,const char *s2)
{
    int val;
    while( (val= toupper(*s1) - toupper(*s2))==0 ) {
        if(*s1==0 || *s2==0) return 0;
        s1++;
        s2++;
        while(*s1=='_') s1++;
        while(*s2=='_') s2++;
    }
    return val;
}

/********************************************************************/
/* A "wrapper" function for memory allocation. Most for debuging /tracing purpose */
void *myrealloc(char *ptr,int newsize)
{
    ptr=realloc(ptr,newsize);
    if(ptr==NULL && newsize>0)
        mexErrMsgTxt("Internal out of memory!");
    return ptr;
}

/********************************************************************/
/* A "wrapper" function for memory allocation. Most for debuging /tracing purpose */
void newbuffsize(io_buff *buff,int newsize)
{
    //    fprintf(stderr,"NEWSIZE:%d\n",newsize);
    if(newsize==-1) {
        free(buff->ptr);
        buff->ptr=NULL;
        return;
    }
    if(newsize<buff->pos)
        newsize=buff->pos;
    if(newsize<256)
        newsize=256;
    if(newsize>buff->len) {  // Grow....
        //	fprintf(stderr,"NEWSIZE UP %d -> %d\n",buff->len,newsize*2);
        buff->ptr=myrealloc(buff->ptr,newsize*2);
        buff->len=newsize*2;
    } else if(newsize*4 < buff->len) { // Decrease...
        //	fprintf(stderr,"NEWSIZE DOWN %d -> %d\n",buff->len,newsize*2);
        buff->ptr=myrealloc(buff->ptr,newsize*2);
        buff->len=newsize*2;
    }
}

/********************************************************************/
mxClassID str2classid(const char *str)
{
    if(myoptstrcmp("CHAR",str)==0)	    return mxCHAR_CLASS;
    if(myoptstrcmp("DOUBLE",str)==0)	return mxDOUBLE_CLASS;
    if(myoptstrcmp("SINGLE",str)==0)	return mxSINGLE_CLASS;
    if(myoptstrcmp("INT8",str)==0)	    return mxINT8_CLASS;
    if(myoptstrcmp("UINT8",str)==0)	    return mxUINT8_CLASS;
    if(myoptstrcmp("INT16",str)==0)	    return mxINT16_CLASS;
    if(myoptstrcmp("UINT16",str)==0)	return mxUINT16_CLASS;
    if(myoptstrcmp("INT32",str)==0)	    return mxINT32_CLASS;
    if(myoptstrcmp("UINT32",str)==0)	return mxUINT32_CLASS;
    return mxCHAR_CLASS; // Default to char;
}

/********************************************************************/
mxClassID classid2size(const mxClassID id)
{
    if(id==mxCHAR_CLASS)	return 1;
    if(id==mxDOUBLE_CLASS)	return sizeof(double);
    if(id==mxSINGLE_CLASS)	return sizeof(float);
    if(id==mxINT8_CLASS)	return 1;
    if(id==mxUINT8_CLASS)	return 1;
    if(id==mxINT16_CLASS)	return 2;
    if(id==mxUINT16_CLASS)	return 2;
    if(id==mxINT32_CLASS)	return 4;
    if(id==mxUINT32_CLASS)	return 4;
    mexErrMsgTxt("Non supported datatype!");
    return 0;
}

/* Windows implementation of perror() function */
#ifdef WIN32
/********************************************************************/
void perror(const char *context )
{
    int wsa_err;
    wsa_err = WSAGetLastError();
    mexPrintf( "[%s]: WSA error: ", context );
    switch ( wsa_err )
    {
    case WSANOTINITIALISED:
        mexPrintf( "WSANOTINITIALISED\n" );
        break;
    case WSAENETDOWN:
        mexPrintf( "WSAENETDOWN      \n" );
        break;
    case WSAEADDRINUSE:
        mexPrintf( "WSAEADDRINUSE    \n" );
        break;
    case WSAEACCES:
        mexPrintf( "WSAEACCES        \n" );
        break;
    case WSAEINTR:
        mexPrintf( "WSAEINTR         \n" );
        break;
    case WSAEINPROGRESS:
        mexPrintf( "WSAEINPROGRESS   \n" );
        break;
    case WSAEALREADY:
        mexPrintf( "WSAEALREADY      \n" );
        break;
    case WSAEADDRNOTAVAIL:
        mexPrintf( "WSAEADDRNOTAVAIL \n" );
        break;
    case WSAEAFNOSUPPORT:
        mexPrintf( "WSAEAFNOSUPPORT  \n" );
        break;
    case WSAEFAULT:
        mexPrintf( "WSAEFAULT        \n" );
        break;
    case WSAENETRESET:
        mexPrintf( "WSAENETRESET     \n" );
        break;
    case WSAENOBUFS:
        mexPrintf( "WSAENOBUFS       \n" );
        break;
    case WSAENOTSOCK:
        mexPrintf( "WSAENOTSOCK      \n" );
        break;
    case WSAEOPNOTSUPP:
        mexPrintf( "WSAEOPNOTSUPP    \n" );
        break;
    case WSAESHUTDOWN:
        mexPrintf( "WSAESHUTDOWN     \n" );
        break;
    case WSAEWOULDBLOCK:
        mexPrintf( "WSAEWOULDBLOCK   \n" );
        break;
    case WSAEMSGSIZE:
        mexPrintf( "WSAEMSGSIZE      \n" );
        break;
    case WSAEHOSTUNREACH:
        mexPrintf( "WSAEHOSTUNREACH  \n" );
        break;
    case WSAEINVAL:
        mexPrintf( "WSAEINVAL        \n" );
        break;

    case WSAECONNREFUSED:
        mexPrintf( "WSAECONNREFUSED  \n" );
        break;
    case WSAECONNABORTED:
        mexPrintf( "WSAECONNABORTED  \n" );
        break;
    case WSAECONNRESET:
        mexPrintf( "WSAECONNRESET    \n" );
        break;
    case WSAEISCONN:
        mexPrintf( "WSAEISCONN       \n" );
        break;
    case WSAENOTCONN:
        mexPrintf( "WSAENOTCONN      \n" );
        break;
    case WSAETIMEDOUT:
        mexPrintf( "WSAETIMEDOUT     \n" );
        break;
    default:
        mexPrintf( "Unknown(%d) error!\n", wsa_err );
        break;
    }
    return;
}
#endif

/********************************************************************/
/*Makes byte swapping, or not depending on the mode argument        */
void byteswapdata(char *ptr,const int elements,const int elementsize,int mode)
{
    // MODE=0 Do nothing, MODE=1 Swap, MODE=2 network byte order, MODE=3 Intel byte order.
#ifndef SWAPDATA
#define SWAPDATA(a,b) { a^=b; b^=a; a^=b; }
#endif
    // A little smart ckeck of byte the machine byte order.
    const int ordertest=1;
    const char *is_intel_order=(const char *)(&ordertest);
    //    fprintf(stderr,"SWAP FUNCTION...E:%d SI:%d\n",elements,elementsize);
    if(elementsize<2) return;
    if(is_intel_order[0]==1 && mode==2)   mode=1;
    if(is_intel_order[0]==0 && mode==3)   mode=1;
    if(mode==1) {
        int e;
        //	fprintf(stderr,"SWAP DATA\n");
        switch(elementsize) {
        case 2:
            for(e=0; e<elements*elementsize; e+=elementsize)
                SWAPDATA(ptr[e],ptr[e+1]) break;
        case 3:
            for(e=0; e<elements*elementsize; e+=elementsize)
                SWAPDATA(ptr[e],ptr[e+2])  break;
        case 4:
            for(e=0; e<elements*elementsize; e+=elementsize)
                SWAPDATA(ptr[e],ptr[e+3]) SWAPDATA(ptr[e+1],ptr[e+2])  break;
        case 5:
            for(e=0; e<elements*elementsize; e+=elementsize)
                SWAPDATA(ptr[e],ptr[e+4]) SWAPDATA(ptr[e+1],ptr[e+3])  break;
        case 6:
            for(e=0; e<elements*elementsize; e+=elementsize)
                SWAPDATA(ptr[e],ptr[e+5]) SWAPDATA(ptr[e+1],ptr[e+4]) SWAPDATA(ptr[e+2],ptr[e+3])  break;
        case 7:
            for(e=0; e<elements*elementsize; e+=elementsize)
                SWAPDATA(ptr[e],ptr[e+6]) SWAPDATA(ptr[e+1],ptr[e+5]) SWAPDATA(ptr[e+2],ptr[e+4])  break;
        case 8:
            for(e=0; e<elements*elementsize; e+=elementsize)
                SWAPDATA(ptr[e],ptr[e+7]) SWAPDATA(ptr[e+1],ptr[e+6]) SWAPDATA(ptr[e+2],ptr[e+5]) SWAPDATA(ptr[e+3],ptr[e+4])  break;
        }
    }
}

/********************************************************************/
/*Makes byte swapping, or not depending on the mode argument        */
void byteswapcopy(char *dest,char *src,const int elements,const int elementsize,int mode)
{
    // MODE=0 Do nothing, MODE=1 Swap, MODE=2 network byte order, MODE=2 Intel byte order.
    // A little smart ckeck of byte the machine byte order.
    const int ordertest=1;
    const char *is_intel_order=(const char *)(&ordertest);
    if(is_intel_order[0]==1 && mode==2)   mode=1;
    if(is_intel_order[0]==0 && mode==3)   mode=1;
    //    fprintf(stderr,"SWAP COPY E:%d SI:%d SWAP:%d\n",elements,elementsize,mode);
    if(mode==1) {
        int e,n;
        //	fprintf(stderr,"SWAP COPY\n");
        for(e=0; e<elements; e++) {
            char *dp=&dest[e*elementsize];
            char *sp=&src[e*elementsize];
            for(n=0; n<elementsize; n++) {
                dp[n]=sp[elementsize-1-n];
                //		fprintf(stderr,"E:%d/%d N:%d/%d\n",e,elements,n,elementsize);
            }
        }
        //	fprintf(stderr,"SWAP COPY END\n");
    }
    else
        memmove(dest,src,elements*elementsize);
}

/********************************************************************/
/* DEBUGING FUNCTION */
void __debug_view_con_status(char *str)
{
    int a;
    mexPrintf("%s\n",str);
    for(a=0; a<5; a++)
    {
        mexPrintf("[%02d] FID:%02d STATUS:%02d WRT.POS:%d RD.POS:%d ",a,con[a].fid,con[a].status,con[a].write.pos,con[a].read.pos);
        if(con[a].read.ptr)
            mexPrintf("RD: %02x %02x %02x %02x %02x %02x %02x %02x ",
                      (int)con[a].read.ptr[0],(int)con[a].read.ptr[1],(int)con[a].read.ptr[2],(int)con[a].read.ptr[3],
                      (int)con[a].read.ptr[4],(int)con[a].read.ptr[5],(int)con[a].read.ptr[6],(int)con[a].read.ptr[7]);
        if(con[a].write.ptr)
            mexPrintf("WR: %02x %02x %02x %02x %02x %02x %02x %02x ",
                      (int)con[a].write.ptr[0],(int)con[a].write.ptr[1],(int)con[a].write.ptr[2],(int)con[a].write.ptr[3],
                      (int)con[a].write.ptr[4],(int)con[a].write.ptr[5],(int)con[a].write.ptr[6],(int)con[a].write.ptr[7]);
        if(a==con_index)
            mexPrintf("<--\n");
        else
            mexPrintf("\n");
    }
    mexPrintf("--------------------\n");
}

/********************************************************************/
/* Portable time function using matlabs NOW                         */
double my_now() {
    double sec;
    double dotimenow;
    static double lastdotime;
    mxArray *plhs[1]= {NULL};
    mxArray *prhs[1]= {NULL};
    mexCallMATLAB(1,plhs,0,prhs,"now");
    sec=mxGetScalar(plhs[0])*60*60*24; //Return time as sec from 1970
    dotimenow=floor(sec*10)/10; // Do calls every 1/10 Sec
    mxDestroyArray(plhs[0]);
    if(lastdotime!=dotimenow) { //Call drawnow once evry X second
        int ret=mexCallMATLAB(0,plhs,0,prhs,"drawnow");
        //mexPrintf("wait... drawnow returns: %d\n",ret);
        lastdotime= dotimenow;
    }
    return sec;
}

/*******************************************************************************/
/* Checks that given index is valid index and set current index, "con_index"   */
/* to that point. If index is CON_FREE (-1) then is search done for a free one */
/* Returns 1 On success. 0 on error                                            */
int move_con(int idx)
{
    if(idx>=MAX_CON)
        mexErrMsgTxt("Unvalid value of handler!\n");
    if(idx>=0)
    {
        con_index=idx;
        if(con[con_index].status==STATUS_FREE)
            mexErrMsgTxt("No valid handler! already closed?");
        return 1;
    }
    debug_view_con_status("START MOVE");
    if(idx==CON_FREE)    /* Move con_index until it find a free non used struct */
    {
        for(con_index=0; con_index<MAX_CON; con_index++)
        {
            debug_view_con_status("STEP MOVE");
            if(con[con_index].status==STATUS_FREE)
                return 1;
        }
        mexErrMsgTxt("To many open connection! Forgot to close old connections?");
    }
    if(idx<0)
        mexErrMsgTxt("Unvalid value of handler!");
    return 0;
}

/**********************************************************************************************/
/* Returns true if specified argument exist                                                  */
int my_mexIsInputArgOK(const int argno)
{
    //    fprintf(stderr,"IS_INPUT_ARG_OK NO:%d of %d\n",argno,gnrhs);        // DEBUG
    if(gnrhs>argno)
        return 1;
    return 0;
}

/******************************************************************************************************/
/* Returns specified input argument as scalar. Global and error tolerant replacement for mxGetScalar */
const mxArray *my_mexInputArg(const int argno)
{
    //    fprintf(stderr,"GET_INPUT_ARG NO:%d\n",argno);                      // DEBUG
    if(!my_mexIsInputArgOK(argno))
        mexErrMsgTxt("Missing input argument.");
    return gprhs[argno];
}

/****************************************************************************************************************/
/* Returns pointer to static buffer (max 80 chars) holding a string in argument argno Global and error tolerant */
const char *my_mexInputOptionString(const int argno)
{
    static char buff[80+1];
    buff[0]=0;
    if(my_mexIsInputArgOK(argno))
        if(mxIsChar(my_mexInputArg(argno)))
            mxGetString(my_mexInputArg(argno),buff,80);
    return buff;
}

/****************************************************************************************************************/
/* Returns pointer to static buffer (max 80 chars) holding the first string with non numeric,nonempty contents  */
const char *my_mexFindInputString(int argno)
{
    const char *str;
    static char buff[1];
    buff[0]=0;
    while(my_mexIsInputArgOK(argno)) {
        str=my_mexInputOptionString(argno);
        if(!isdigit(str[0]) && str[0]!=0 )
            return str;
        argno++;
    }
    return buff;
}

/*********************************************************************/
/* Returns true if if finds option opt at argument argno or later    */
int my_mexFindInputOption(int argno,const char *opt)
{
    char buff[80+1];
    while(my_mexIsInputArgOK(argno))
    {
        buff[0]=0;
        mxGetString(my_mexInputArg(argno),buff,80);
        if(myoptstrcmp(buff,opt)==0)
            return 1;
        argno++;
    }
    return 0;
}

/***********************************************************************************************************/
/* Returns specified input arguments cell as scalar. Global and error tolerant replacement for mxGetScalar */
double my_mexInputCell(const int argno,int cell_no)
{
    if(!mxIsDouble(my_mexInputArg(argno)))
        mexErrMsgTxt("Argument has wrong datatype.");
    if(cell_no>=mxGetNumberOfElements(my_mexInputArg(argno)))
        mexErrMsgTxt("Argument has wrong number of cells");
    return mxGetPr(my_mexInputArg(argno))[cell_no];
}

/******************************************************************************************************/
/* Returns specified input argument as scalar or a scalar product of its elements.                    */
/* Global and error tolerant replacement for mxGetScalar                                              */
int my_mexInputSize(const int argno)
{
    if(!my_mexIsInputArgOK(argno))
        return DEFAULT_INPUT_SIZE;
    if(mxIsChar(my_mexInputArg(argno))) {
        const int ch=(my_mexInputOptionString(argno)[0]);
        if(!isdigit(ch))
            return DEFAULT_INPUT_SIZE;
        else
            return atoi(my_mexInputOptionString(argno));
    }
    if(mxGetNumberOfElements(my_mexInputArg(argno))>1) {
        int val=1,n=0,el=mxGetNumberOfElements(my_mexInputArg(argno));
        for(n=0; n<el; n++)
            val*=(int)my_mexInputCell(argno,n);
        return val;
    } else
        return (int)mxGetScalar(my_mexInputArg(argno));
}

/******************************************************************************************************/
/* Returns specified input argument as scalar. Global and error tolerant replacement for mxGetScalar */
double my_mexInputScalar(const int argno)
{
    if(mxIsChar(my_mexInputArg(argno)))
        return atof(my_mexInputOptionString(argno));
    else
        return mxGetScalar(my_mexInputArg(argno));
}

/********************************************************************/
/* Copys a matlab Char array to a char * string buffer              */
int my_mexInputArray2Buff(const int argno,io_buff *buff)
{
    mxClassID id=mxGetClassID(my_mexInputArg(argno));
    int si=classid2size(id);
    int len=mxGetNumberOfElements(my_mexInputArg(argno));
    int swap=2;
    if(my_mexFindInputOption(argno+1,"NATIVE"))  swap=0;
    if(my_mexFindInputOption(argno+1,"SWAP"))    swap=1;
    if(my_mexFindInputOption(argno+1,"NETWORK")) swap=2;
    if(my_mexFindInputOption(argno+1,"INTEL"))   swap=3;

    newbuffsize(buff,buff->pos+len*si);

    if(id==mxCHAR_CLASS) {
        mxChar *ptr = (mxChar *)mxGetPr(gprhs[argno]);
        int a;
        for(a=0; a<len; a++)
            buff->ptr[buff->pos++]=(char)(unsigned char)ptr[a];
    } else {
        char *ptr = (char *)mxGetPr(gprhs[argno]);
        byteswapcopy(&buff->ptr[buff->pos],ptr,len,si,swap);
        buff->pos+=(len*si);
    }
    return len;
}

/*******************************************************************************/
/* Puts double scalar in matlab variable in the array of return argument for mex*/
void my_mexReturnValue(double val)
{
    if((gret_args>gnlhs) && (gret_args>1))
        return;
    gplhs[gret_args]=mxCreateDoubleMatrix(1,1,mxREAL);
    if(gplhs[gret_args]==NULL)
        mexErrMsgTxt("Matrix creation error! Lack of memory?");
    else
    {
        *mxGetPr(gplhs[gret_args])=val;
        gret_args++;
    }
//    mexPrintf("DEBUG MEX RETURN VALUE:%g\n",val);  // DEBUG
}

/*******************************************************************************/
/* Puts double matrix in matlab variable in the array of return argument for mex*/
void my_mexReturnMatrix(int rows, int cols, double *vals)
{
    double *pr;
    if((gret_args>gnlhs) && (gret_args>1))
        return;
    gplhs[gret_args] = mxCreateDoubleMatrix( rows, cols, mxREAL);
    if( gplhs[gret_args] == NULL)
        mexErrMsgTxt("Matrix creation error");
    pr = (double *)mxGetPr(gplhs[gret_args]);
    memcpy(pr,vals,rows*cols*sizeof(double));
    gret_args++;
}

/********************************************************************************/
/* Puts string as matlab char char variable in array of return argument for mex */
void my_mexReturnString(const char *str)
{
    if(gret_args>gnlhs && gret_args>1)
        return;
    gplhs[gret_args]=mxCreateString(str);
    if(gplhs[gret_args]==NULL)
        mexErrMsgTxt("String creation error! Lack of memory?");
    gret_args++;
}

/******************************************************************************/
/* Puts data from buffer into next MATLAB return Array                        */
void my_mexReturnArrayFromBuff(const int argno,io_buff *buff,const int line)
{
    const int maxelements=my_mexInputSize(argno);
    const mxClassID id=str2classid(my_mexInputOptionString(argno+1));
    mwSize dims[20]= {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 };
    const int si=classid2size(id);
    int returnelements= ( (buff->pos/si)< maxelements )?(buff->pos/si):maxelements;
    int deleteelements=returnelements;
    int swap=2;
    mwSize return_no_dims= my_mexIsInputArgOK(argno) && !mxIsChar(my_mexInputArg(argno))?mxGetNumberOfElements(my_mexInputArg(argno)):1;

    if(my_mexFindInputOption(argno+1,"NATIVE"))  swap=0;
    if(my_mexFindInputOption(argno+1,"SWAP"))    swap=1;
    if(my_mexFindInputOption(argno+1,"NETWORK")) swap=2;
    if(my_mexFindInputOption(argno+1,"INTEL"))   swap=3;
    if(return_no_dims>20)
        mexErrMsgTxt("To many dimentions to return.");
    debug_view_con_status("GET_ARRAY");
    if(line) {
        int n=-7;
        if(id!=mxCHAR_CLASS && return_no_dims)
            mexErrMsgTxt("'READLINE' works only with datatype char and non fixed blocksize");
        for(n=0; n<returnelements; n++)
            if(buff->ptr[n]=='\n')
                break;
        if(n==maxelements)                             // If no new-line found inside limit...
            deleteelements=returnelements=maxelements; // ...return first part of splited line.
        else if(n==returnelements)                     // If new-line not recived inside limit...
            deleteelements=returnelements=0;           // ...return empty string, and delete nothing.
        else if(n>0 && buff->ptr[n-1]=='\r')           // If(*3) new-line, return line of char but not nl chars.
            deleteelements=2+(returnelements=n-1);
        else
            deleteelements=1+(returnelements=n);
        return_no_dims=1;
    }
    if(return_no_dims>1) {                // If shape of return argument is specified.....
        if(returnelements==maxelements) {       // ...then only accept correct shape.
            int n;
            for(n=0; n<return_no_dims; n++)
                dims[n]=(int)my_mexInputCell(argno,n);
        }
    } else if(returnelements>0) {          // else... Just return row of available elements
        dims[0]=1;
        dims[1]=returnelements;
        return_no_dims=2;
    }
    if(! (gret_args>gnlhs && gret_args>1) ) {
        /* Create a 2-Dimensional character mxArray.*/
        if( dims[0]==0)
            gplhs[gret_args] = mxCreateNumericArray(0,dims,id,mxREAL);
        else
            gplhs[gret_args] = mxCreateNumericArray(return_no_dims,dims,id,mxREAL);
        if(gplhs[gret_args] == NULL)
            mexErrMsgTxt("Could not create return array.");
        if(dims[0]!=0) {
            if(id==mxCHAR_CLASS) {
                int i;
                mxChar *p=(mxChar *)mxGetPr(gplhs[gret_args]);
                for(i=0; i<returnelements; i++)
                    p[i]=buff->ptr[i];
            } else {
                char *p=(char *)mxGetPr(gplhs[gret_args]);
                byteswapcopy(p,buff->ptr,returnelements,si,swap);
            }
        }
        gret_args++;
    }
    //    debug_view_con_status("GET_ARRAY N�STAN KLAR");
    // Delete from read buffer if not "VIEW" option and dims filled
    if(my_mexFindInputOption(argno+1,"VIEW")==0 && deleteelements>0 ) {
        buff->pos-=deleteelements*si;
        memmove(buff->ptr,&buff->ptr[deleteelements*si],buff->pos);
        newbuffsize(buff,buff->pos);
    }
    // mexPrintf("DEBUG MEX RETURN ARRAY OF:%d\n",returnelements);  // DEBUG

    //    fprintf(stderr,"DIMS:[%d %d] DEL:%d RET:%d SI:%d POS:%d LEN:%d PTR:%08X\n",
    //	    dims[0],dims[1],deleteelements,returnelements,si,buff->pos,buff->len,buff->ptr);
}


/**********************************************************************/
int ipv4_lookup(const char *hostname,int port)
{
    struct in_addr addr;    /* my address information */
    /* Try IP address */
    addr.s_addr=inet_addr(hostname);
    if (addr.s_addr==INADDR_NONE) {
        /*Can't resolve host string as dot notation IP number...
          try lookup IP from hostname */
        struct hostent *he;
        //	fprintf(stderr,"Trying nameserverlookup:%s\n",hostname);
        he=gethostbyname(hostname);
        if(he==NULL) {
            mexPrintf("\nUNKNOWN HOST:%s\n",hostname);
            return -1;  /* Can not lookup hostname */
        }
        addr = *((struct in_addr *)he->h_addr);
    }
    con[con_index].remote_addr.sin_family=AF_INET;
    con[con_index].remote_addr.sin_port=htons(port);
    con[con_index].remote_addr.sin_addr=addr;
    memset(&con[con_index].remote_addr.sin_zero, 0,8);
    return 0;
}


/**********************************************************************/
/* Writes from specified position (pointer) in buffer of spec. length */
int writedata()
{
    const double timeoutat=my_now()+con[con_index].writetimeout;
    int   len=con[con_index].write.pos;
    const char *ptr=con[con_index].write.ptr;
    const int  fid=con[con_index].fid;
    int sentlen=0;
    int retval=0;
    int lastsize=1000000;
    if(con[con_index].status<STATUS_IO_OK)
        return 0;
    //    if( !IS_STATUS_TCP_CONNECTED(con[con_index].status)  && len>65534 )   // TODO: Ta bort!
    //	len=65534;
    while(sentlen<len)
    {
        if (lastsize<1000) {
            usleep(DEFAULT_USLEEP);
        }
        if (IS_STATUS_UDP_NO_CON(con[con_index].status)) {
            con[con_index].remote_addr.sin_family=AF_INET;
            retval = sendto(fid,&ptr[sentlen],len-sentlen,MSG_NOSIGNAL,
                            (struct sockaddr *)&con[con_index].remote_addr,sizeof(struct sockaddr));
        } else {
            retval=send(fid,&ptr[sentlen],len-sentlen,MSG_NOSIGNAL);
        }
        lastsize=retval>0?retval:0;
        sentlen+=lastsize;
        /*	if( retval==0){
        	mexPrintf("\nREMOTE HOST DISCONNECTED\n");
        	con[con_index].status=STATUS_NOCONNECT;
        	break;
        	}*/
        if(retval<0 && s_errno!=EWOULDBLOCK
                //	   IFWINDOWS( && s_errno!=WSAECONNRESET  )           // DEBUG: REMOVE THIS LINE?
          ) {
            con[con_index].status=STATUS_NOCONNECT;
            perror( "sendto() / send()" );
            mexPrintf("\nREMOTE HOST DISCONNECTED %d\n", s_errno);
            break;
        }
        if( !IS_STATUS_TCP_CONNECTED(con[con_index].status) && sentlen>0 )
            break;
        if(timeoutat<=my_now())
            break;
    }
    con[con_index].write.pos-=sentlen;
    memmove(con[con_index].write.ptr, &con[con_index].write.ptr[sentlen], con[con_index].write.pos);
    newbuffsize(&con[con_index].write,con[con_index].write.pos);
    return sentlen;
}

/********************************************************************/
/* Init current record with values                                  */
void init_con(int fid,int status)
{
    memset(&con[con_index],0,sizeof(con_info));
    con[con_index].fid=fid;
    con[con_index].status=status;
    con[con_index].writetimeout=DEFAULT_WRITETIMEOUT;
    con[con_index].readtimeout=DEFAULT_READTIMEOUT;
}

/********************************************************************/
/* Close con struct                                                 */
void close_con()
{
    if(con[con_index].fid>=0)
        close(con[con_index].fid);
    else
        mexWarnMsgTxt("Closing already closed connection!");
    newbuffsize(&con[con_index].write,-1);
    newbuffsize(&con[con_index].read,-1);
    init_con(-1,STATUS_FREE);
}

/*******************************************************************      */
/* Function to close all still open tcpip connections */
int closeall(void)
{
    int flag=0;
    for(con_index=0; con_index<MAX_CON; con_index++)
        if(con[con_index].fid>=0) {  /* Already closed?? */
            close_con();
            flag=1;
        }
    return flag;
}

/****************************************************************************/
/* This function is called on unloading of mex-file                         */
void CleanUpMex(void)
{
    if(closeall()) /* close all still open connections...*/
        mexWarnMsgTxt("Unloading mex file. Unclosed tcp/udp/ip connections will be closed!");
    IFWINDOWS(   WSACleanup();  );
}

/********************************************************************/
/* Read to fill input buffer with specified length from UDP or TCP network*/
int read2buff(const int len,int newline,int noblock)
{
    const double timeoutat=my_now()+con[con_index].readtimeout;
    int retval=-1;

    if(len<con[con_index].read.pos)              /* If enouth in buffer then return */
        return len;
    if(0==IS_STATUS_IO_OK(con[con_index].status))/* If not read/write fid (broken pipe) then exit.*/
        if(len<con[con_index].read.pos)
            return len;
        else
            return con[con_index].read.pos;

    /* Resize readbuffer to needed size */
    if(con[con_index].read.len<len)
        newbuffsize(&con[con_index].read,len);

    while(1) {
        int readlen=len-con[con_index].read.pos;
//	mexPrintf("DEBUG: READLINE: readlen:%d\n",readlen);
        if(readlen>0) {
            if(IS_STATUS_CONNECTED(con[con_index].status)) {
                retval=recv(con[con_index].fid,&con[con_index].read.ptr[con[con_index].read.pos],readlen,MSG_NOSIGNAL);
            }
            else {
                struct sockaddr_in my_addr;
                int fromlen=sizeof(my_addr);

                // Copy 0.0.0.0 adress and 0 port to remote_addr as init-value.
                memset(&my_addr,0,sizeof(my_addr));
                con[con_index].remote_addr.sin_addr = my_addr.sin_addr;
                con[con_index].remote_addr.sin_port = my_addr.sin_port;
                retval=recvfrom(con[con_index].fid,&con[con_index].read.ptr[con[con_index].read.pos],
                                readlen,MSG_NOSIGNAL,(struct sockaddr *)&my_addr, &fromlen);
                if (retval>0) {
                    con[con_index].remote_addr.sin_addr = my_addr.sin_addr;
                    con[con_index].remote_addr.sin_port = htons((unsigned short int)ntohs(my_addr.sin_port));
                }
            }
            if( retval==0) {
                mexPrintf("\nREMOTE HOST DISCONNECTED\n");
                con[con_index].status=STATUS_NOCONNECT;
                break;
            }
            if(retval<0 && s_errno!=EWOULDBLOCK
                    //	       IFWINDOWS( && s_errno!=WSAECONNRESET )// DEBUG: REMOVE THIS LINE?
              ) {
                con[con_index].status=STATUS_NOCONNECT;
                perror( "recvfrom() or recv()" );
                break;
            }
        }
        //	fprintf(stderr,"RET:%d/%d ",retval,s_errno);
        readlen=retval>0?retval:0;
        con[con_index].read.pos+=readlen;
        if( !IS_STATUS_TCP_CONNECTED(con[con_index].status) && con[con_index].read.pos>0 )
            break;
        if( con[con_index].read.pos>=len )
            break;
        if(timeoutat<=my_now() || noblock)
            break;
        if(newline) {
            int n;
            for(n=0; n<con[con_index].read.pos; n++)
                if(con[con_index].read.ptr[n]=='\n')
                    return con[con_index].read.pos;
        }
        if(readlen<1000)
            usleep(DEFAULT_USLEEP);
    }
    return con[con_index].read.pos;
}

/***************************************************************************/   // BORT???
/* Read specified length & type from UDP or TCP network to input buffer    */
int readtype2buff(int len,mxClassID datatype,int newline,int noblock)
{
    const int si=classid2size(datatype);
    return read2buff(len*si,newline,noblock)/si;
}

/********************************************************************/
/* Function Creating a tcpip connection and returns handler number  */
int tcp_connect(const char *hostname,const int port)
{
    if(ipv4_lookup(hostname,port)==-1)
        return -1;
    con[con_index].fid=socket(AF_INET, SOCK_STREAM, 0);
    if(con[con_index].fid== CON_FREE) {
        /*Can't open socket */
        close_con();
        return -1;
    }
    if(connect(con[con_index].fid,(struct sockaddr *)&con[con_index].remote_addr,sizeof(struct sockaddr)) == -1) {
        /*Can't connect to remote host. */
        close_con();
        return -1;
    }
    con[con_index].status=STATUS_TCP_CLIENT;
    nonblockingsocket(con[con_index].fid); /* Non blocking read! */
    return con_index;
}

/*******************************************************************
 Function Creating a TCP server socket
 or a connectionless UDP client socket.
*/
int tcp_udp_socket(int port,int dgram_f)
{
    int sockfd;
    struct sockaddr_in my_addr;    /* my address information */
    const int on=1;
    if(dgram_f)
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    else
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd==-1)
        return -1;
    my_addr.sin_family = AF_INET;         /* host byte order */
    my_addr.sin_port = htons(port);       /* short, network byte order */
    my_addr.sin_addr.s_addr = INADDR_ANY; /* auto-fill with my IP */
    memset(&(my_addr.sin_zero),0, 8);        /* zero the rest of the struct */
    setsockopt(sockfd,SOL_SOCKET,SO_REUSEADDR,(const char *)&on,sizeof(on));
    if(bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr))== -1)
    {
        close(sockfd);
        return -1;
    }
    listen(sockfd,BACKLOG);
    nonblockingsocket(sockfd);
    return sockfd;
}

/*****************************************************************/
/* Listen to socket and returns connection if there is one...
   else it returns -1 */
int tcpiplisten(int noblock)
{
    const double timeoutat=my_now()+con[con_index].readtimeout;
    int new_fd;
    const int sock_fd= con[con_index].fid;
    int sin_size = sizeof(struct sockaddr_in);
    move_con(CON_FREE);        /* Find a new free con struct for the new connection.... */
    while(1) {
        if ((new_fd = accept(sock_fd, (struct sockaddr *)&con[con_index].remote_addr,&sin_size)) > -1)
            break;
        if(timeoutat<=my_now()|| noblock)
            return -1;
        usleep(DEFAULT_USLEEP);
    }
    nonblockingsocket(new_fd); /* Non blocking read! */
    setsockopt(new_fd,SOL_SOCKET,SO_KEEPALIVE,(void *)1,0); /* realy needed? */
    con[con_index].fid=new_fd;
    con[con_index].status=STATUS_TCP_SERVER;
    return con_index;
}

/*****************************************************************/
/* Make a UDP socket be a "connected" UDP socket                 */
int udp_connect(const char *hostname,int port)
{
    if(con[con_index].status < STATUS_UDP_CLIENT)
        mexErrMsgTxt("Must pass UDP client or server handler!");
    if (port == -1) {
        // use address of last received packet
        con[con_index].remote_addr.sin_family=AF_INET;
    } else if (ipv4_lookup(hostname,port)==-1) {
        return -1;
    }
    mexPrintf("connecting to port %d\n", con[con_index].remote_addr.sin_port);
    if(connect(con[con_index].fid,(struct sockaddr *)&con[con_index].remote_addr,sizeof(struct sockaddr)) == -1) {
        mexErrMsgTxt("connect failed");
        return -1; /*Can't connect to remote host. */
    }
    if(con[con_index].status == STATUS_UDP_CLIENT) {
        con[con_index].status = STATUS_UDP_CLIENT_CONNECT;
    } else {
        con[con_index].status = STATUS_UDP_SERVER_CONNECT;
    }
    nonblockingsocket(con[con_index].fid); /* Non blocking read! */
    return con[con_index].status;
}

/*****************************************************************/
/*                                                               */
/*    ----Main function that is called from matlab--------       */
/*                                                               */
void mexFunction(
    int           nlhs,           /* number of expected outputs */
    mxArray       *plhs[],        /* array of pointers to output arguments */
    int           nrhs,           /* number of inputs */
    const mxArray *prhs[]         /* array of pointers to input arguments */
)
{
    char fun[80+1];
    /* Initialization on first call to the mex file */
    if(mex_call_counter==0)
    {
#ifdef WIN32
        WORD wVersionRequested;
        WSADATA wsaData;
        int wsa_err;
        wVersionRequested = MAKEWORD( 2, 0 );
        wsa_err = WSAStartup( wVersionRequested, &wsaData );
        if (wsa_err)
            mexErrMsgTxt("Error starting WINSOCK32.");
#endif
        Print_Start_Message();

        mexAtExit(CleanUpMex);
        /* Init all connecttions to to free */
        for(con_index=0; con_index<MAX_CON; con_index++)
            init_con(-1,STATUS_FREE);
        con_index=0;
    }
    mex_call_counter++;
    debug_view_con_status("ENTER_MEX");

    /* GLOBAL IN-OUT ARGUMENTS */
    gnlhs=nlhs;       /* number of expected outputs */
    gplhs=plhs;       /* array of pointers to output arguments */
    gnrhs=nrhs;       /* number of inputs */
    gprhs=prhs;       /* array of pointers to input arguments */
    gret_args=0;      /* No return argumens returned */


    if(mxIsChar(my_mexInputArg(0))) {
        /* GET FIRST ARGUMENT -- The "function" name */
        strncpy(fun,my_mexInputOptionString(0),80);
//	mexPrintf("DEBUG MEX(1):[%d] %s\n",con_index,fun);   // DEBUG

        /* Find of the function name corresponds to a non connection associated function */
        if(myoptstrcmp(fun,"CLOSEALL")==0) {
            closeall();
            return;
        }
        if(myoptstrcmp(fun,"TCPCONNECT")==0) {
            const int port=(int)my_mexInputScalar(2);
            const char *hostname=my_mexInputOptionString(1);
            move_con(STATUS_FREE);
            my_mexReturnValue(tcp_connect(hostname,port));
            return;
        }
        if(myoptstrcmp(fun,"TCPSOCKET")==0) {
            const int fd=tcp_udp_socket((int)my_mexInputScalar(1), 0);
            if(fd>=0) {
                move_con(STATUS_FREE);
                init_con(fd,STATUS_TCP_SOCKET);
                my_mexReturnValue(con_index);
            } else
                my_mexReturnValue(-1);
            return;
        }
        if(myoptstrcmp(fun,"UDPSOCKET")==0) {
            const int fd=tcp_udp_socket((int)my_mexInputScalar(1), 1);
            if(fd>=0) {
                move_con(STATUS_FREE);
                init_con(fd,STATUS_UDP_CLIENT);
                my_mexReturnValue(con_index);
            } else
                my_mexReturnValue(-1);
            return;
        }
        if(myoptstrcmp(fun,"DOCALLBACKS")==0) {
            return;
        }
    }
    /* Get connection handler and suppose that it is a connection assosiated function */
    /* Find given handel */
    //       if(strncasecmp(fun,"DEF",3)!=0)
    if(move_con((int)my_mexInputScalar(0))==0)
        mexErrMsgTxt("Unknown connection handler");
    strncpy(fun,my_mexInputOptionString(1),80);
//   mexPrintf("DEBUG MEX(2):[%d] %s\n",con_index,fun);   // DEBUG
    debug_view_con_status("CON_MOVED!!");

    if(myoptstrcmp(fun,"CLOSE")==0) {
        close_con();
        return;
    }
    if(myoptstrcmp(fun,"TCPLISTEN")==0) {
        if(con[con_index].status!=STATUS_TCP_SOCKET)
            mexErrMsgTxt("Invalid socket for LISTEN, Already open, or UDP?...");
        my_mexReturnValue(tcpiplisten(my_mexFindInputOption(2,"noblock")));
        return;
    }
    /* MATLAB called with status = UDP_CONNECT(fid, ip, port) */
    if(myoptstrcmp(fun,"UDPCONNECT")==0) {
        const int port=(int)my_mexInputScalar(3);
        const char *hostname=my_mexInputOptionString(2);
        my_mexReturnValue(udp_connect(hostname,port));
        return;
    }
    if(myoptstrcmp(fun,"WRITE")==0) {
        my_mexInputArray2Buff(2,&con[con_index].write);
        if(IS_STATUS_TCP_CONNECTED(con[con_index].status))
            writedata();
        return;
    }
    if(myoptstrcmp(fun,"PRINTF")==0) {
        mxArray *plhs[1]= {NULL};
        if(gnrhs<3) return;
        mexCallMATLAB(1,plhs, gnrhs-2, (mxArray **)&(gprhs[2]),"sprintf");
        gprhs=(const mxArray **)plhs;
        gnrhs=1; // HACK: Move return arg from sprintf to input arg of this mex.
        my_mexInputArray2Buff(0,&con[con_index].write);
        if(IS_STATUS_TCP_CONNECTED(con[con_index].status))
            writedata();
        return;
    }
    if(myoptstrcmp(fun,"READ")==0) {
        if(IS_STATUS_TCP_CONNECTED(con[con_index].status))
            readtype2buff( (int)my_mexInputSize(2),str2classid(my_mexFindInputString(2)),0,my_mexFindInputOption(2,"noblock"));
        my_mexReturnArrayFromBuff(2,&con[con_index].read,0);
        return;
    }
    if(myoptstrcmp(fun,"READLINE")==0) {
        //	mexPrintf("DEBUG: READLINE....\n");
        if(IS_STATUS_TCP_CONNECTED(con[con_index].status))
            read2buff(my_mexInputSize(2),1,my_mexFindInputOption(2,"noblock"));
        my_mexReturnArrayFromBuff(2,&con[con_index].read,1);
        //	mexPrintf("DEBUG: READLINE END\n");
        return;
    }
    if(myoptstrcmp(fun,"READTOFILE")==0) {
        FILE *f=NULL;
        const int blocklen=my_mexInputSize(2+1);
        int readlen=blocklen;
        int writelen=0;
        if(IS_STATUS_TCP_CONNECTED(con[con_index].status))
            readlen= read2buff(blocklen,0,(int)my_mexFindInputOption(2+1,"noblock"));
        if(readlen>0)
            f=fopen(my_mexInputOptionString(2),(int)my_mexFindInputOption(2+1,"append")?"ab":"wb");
        if(f) {
            writelen=fwrite(con[con_index].read.ptr,1,readlen,f);
            fclose(f);
        }
        // Delete from read buffer if not "VIEW" option and dims filled
        if(my_mexFindInputOption(2+1,"VIEW")==0 ) {
            con[con_index].read.pos-=writelen;
            memmove(con[con_index].read.ptr,&con[con_index].read.ptr[writelen],con[con_index].read.pos);
            newbuffsize(&con[con_index].read,con[con_index].read.pos);
        }
        my_mexReturnValue(writelen);
        return;
    }
    if(myoptstrcmp(fun,"WRITEFROMFILE")==0) {
        int len=1024*1024*1024;
        int start=0;
        FILE *f=fopen(my_mexInputOptionString(2),"rb");
        if(f==NULL)
            my_mexReturnValue(-1);
        if(my_mexIsInputArgOK(2+1))
            start=(int)my_mexInputScalar(2+1);
        if(my_mexIsInputArgOK(2+2))
            len=(int)my_mexInputScalar(2+2);
        else {
            fseek(f,0,SEEK_END);
            len=ftell(f)-start;
        }
        fseek(f,start,SEEK_SET);
        newbuffsize(&con[con_index].write,con[con_index].write.pos+len);
        len=fread(&con[con_index].write.ptr[con[con_index].write.pos],1,len,f);
        con[con_index].write.pos+=len;
        fclose(f);
        if(IS_STATUS_TCP_CONNECTED(con[con_index].status))
            writedata();
        if(len<0)
            my_mexReturnValue(-1);
        else
            my_mexReturnValue(len);
        return;
    }
    if(myoptstrcmp(fun,"READPACKET")==0) {
        con[con_index].read.pos=0;
        my_mexReturnValue(readtype2buff(my_mexInputSize(2),str2classid(my_mexInputOptionString(2+1)),0,
                                        my_mexFindInputOption(2+1,"noblock")));
        return;
    }
    if(myoptstrcmp(fun,"WRITEPACKET")==0) {
        if (my_mexIsInputArgOK(2)) {
            if (IS_STATUS_UDP_NO_CON(con[con_index].status)) {
                ipv4_lookup(my_mexInputOptionString(2),my_mexInputScalar(3));
            }
        }
        my_mexReturnValue(writedata());
        con[con_index].write.pos=0;
        return;
    }
    if(myoptstrcmp(fun,"STATUS")==0) {
        my_mexReturnValue(con[con_index].status);
        return;
    }
    if(myoptstrcmp(fun,"GETHOST")==0) {
        int n;
        double ip_bytes[4] = {0, 0, 0, 0};
        const unsigned char *ipnr=(const unsigned char *)&con[con_index].remote_addr.sin_addr;
        for(n=0; n<4; n++)
            ip_bytes[n] = (double)ipnr[n];
        my_mexReturnMatrix(1,4,ip_bytes);
        my_mexReturnValue((int)ntohs(con[con_index].remote_addr.sin_port));
        return;
    }
    if(myoptstrcmp(fun,"SETCALLBACK")==0) {
        strncpy(con[con_index].callback, my_mexInputOptionString(2), CBNAMELEN);
        return;
    }
    if(myoptstrcmp(fun,"GETCALLBACK")==0) {
        my_mexReturnString(con[con_index].callback);
        return;
    }
    if(myoptstrcmp(fun,"SETWRITETIMEOUT")==0) {
        con[con_index].writetimeout=my_mexInputScalar(2);
        return;
    }
    if(myoptstrcmp(fun,"SETREADTIMEOUT")==0) {
        con[con_index].readtimeout=my_mexInputScalar(2);
        return;
    }
    if(myoptstrcmp(fun,"debug")==0) {
        mexPrintf("     FID:%d\n",con[con_index].fid);
        mexPrintf("  STATUS:%d\n",con[con_index].status);
        mexPrintf("WRITE  TO:%g\n",con[con_index].writetimeout);
        mexPrintf("WRITE PTR:%x\n",(int)con[con_index].write.ptr);
        mexPrintf("WRITE POS:%d\n",con[con_index].write.pos);
        mexPrintf("WRITE LEN:%d\n",con[con_index].write.len);
        mexPrintf("READ  TO:%g\n",con[con_index].readtimeout);
        mexPrintf("READ PTR:%x\n",(int)con[con_index].read.ptr);
        mexPrintf("READ POS:%d\n",con[con_index].read.pos);
        mexPrintf("READ LEN:%d\n",con[con_index].read.len);
        return;
    }
    mexErrMsgTxt("Unknown 'function name' in argument.");
}
