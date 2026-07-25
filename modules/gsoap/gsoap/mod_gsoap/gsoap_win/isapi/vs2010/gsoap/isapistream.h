/** Interface for the isapistream class
  * @file isapistream.h
  * @author Christian Aberger 
  * Copyright (C) 2001 Genivia Inc and WebWare
  */
#ifndef _ISAPISTREAM_H_1DCECA02_E2F0_11d4_83AA_00A0CCD4FF57
#define _ISAPISTREAM_H_1DCECA02_E2F0_11d4_83AA_00A0CCD4FF57

#include <httpext.h>
#include <iostream>

/** a streambuf wrapper for the ISAPI-Extension control block.
 */
class isapistreambuf : public std::streambuf {
public:
    /** constructor */
    isapistreambuf(EXTENSION_CONTROL_BLOCK *pECB);
    /** destructor */
    ~isapistreambuf();

    EXTENSION_CONTROL_BLOCK *ECB();
protected:
    virtual int sync(); ///< flush contents to isapi
    virtual int overflow(int ch); ///< flush contents and write ch
    virtual int underflow();
	std::streamsize showmanyc() {return _cbTotalBytes - _cbRead;}
protected:
    EXTENSION_CONTROL_BLOCK *_pECB; ///< see ISAPI documentation for details
    char _obuf[1024]; ///< output buffer
    char *_ibuf; ///< input buffer
    unsigned int _ibuflen; ///< length of input buffer.
    unsigned int _cbTotalBytes; ///< total input bytes in the request.
    unsigned int _cbRead; ///< total bytes read from ECB already.
};

/** ISAPI input/output stream.
    It can be used anywhere like cout is used, the output goes to the client browser.
 */
class isapistream : public std::iostream {
public:
    typedef std::iostream super;
	/** constructor.
		@param pECB the extension control block received as a parameter of HttpExtensionProc 
	  */
    isapistream(EXTENSION_CONTROL_BLOCK *pECB) : super(&_buf), _buf(pECB) {}

	/**	destructor */
    virtual ~isapistream();

    EXTENSION_CONTROL_BLOCK *ECB(); ///< @return the EXTENSION_CONTROL_BLOCK wrapped by this
protected:
    isapistreambuf _buf;
private:
	isapistream(isapistream&); ///< not implemented.
	isapistream& operator=(const isapistream&); ///< not implemented.
};

#endif //_ISAPISTREAM_H_1DCECA02_E2F0_11d4_83AA_00A0CCD4FF57
