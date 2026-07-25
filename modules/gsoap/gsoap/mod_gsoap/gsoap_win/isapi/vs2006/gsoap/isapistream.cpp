/** Implementation of the isapistream class.
  * @file isapistream.cpp
  * @author Christian Aberger 
  * Copyright (C) 2001 Genivia Inc and WebWare
  */
#include "isapistream.h"
#include <cassert>
using namespace std;

isapistreambuf::isapistreambuf(EXTENSION_CONTROL_BLOCK *pECB)
: _pECB(pECB) 
{
    static const int nMinBufSize = 1024;
    assert(NULL != pECB);
    setp(_obuf, _obuf + sizeof _obuf);

    _cbTotalBytes = pECB->cbTotalBytes;
    _ibuflen = pECB->cbAvailable > nMinBufSize ? pECB->cbAvailable : nMinBufSize;
    _ibuf = new char[_ibuflen + 1]; // we allocate it one byte more, adding a trailing '\0', then it is easier with strings
    memset(_ibuf, 0, _ibuflen + 1);
    memcpy_s(_ibuf, _ibuflen + 1, pECB->lpbData, pECB->cbAvailable);
    _cbRead = pECB->cbAvailable;
    setg(_ibuf, _ibuf, _ibuf + _cbRead);
}
isapistreambuf::~isapistreambuf() {
    delete _ibuf;
}
int isapistreambuf::sync() {
	BOOL bWrite = TRUE;
	if (NULL != _pECB) {
		DWORD dwBytesWritten = pptr() - pbase();
		if (dwBytesWritten) {
			bWrite = _pECB->WriteClient(_pECB->ConnID, (PVOID)_obuf, &dwBytesWritten, 0);
			setp(_obuf, _obuf + sizeof _obuf);
		}
	}
	return bWrite ? 0 : -1;
}
int isapistreambuf::overflow(int ch) {
    if (char_traits<char>::eof() == ch) {
        return streambuf::overflow(ch);
    }
    if (pptr() < epptr()) {
        return sputc(ch);
    }
    int ret = sync();
    if (0 == ret && char_traits<char>::eof() != ch) {
        sputc(ch);
    }
    return ret;
}
int isapistreambuf::underflow() {
    int retval = char_traits<char>::eof();
    if (gptr() < egptr()) {
        //retval = snextc();
		retval = sgetc();
    } else {
        if (_cbRead < _cbTotalBytes) {
            memset(_ibuf, 0, _ibuflen + 1);
            DWORD dwLen = _ibuflen;
            BOOL bRead = (*_pECB->ReadClient)(_pECB->ConnID, _ibuf, &dwLen);
            if (bRead && dwLen > 0) {
                _cbRead += dwLen;
                setg(_ibuf, _ibuf, _ibuf + dwLen);
                //retval = snextc();
				retval = sgetc();
            }
        }
    }
    return retval;
}
isapistream::~isapistream() {
}
EXTENSION_CONTROL_BLOCK *isapistreambuf::ECB() {
    return _pECB;
}
EXTENSION_CONTROL_BLOCK *isapistream::ECB() {
    return _buf.ECB();
}

