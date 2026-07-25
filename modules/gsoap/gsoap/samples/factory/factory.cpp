/*
	factory.cpp

	Remote object factory

	The remote object factory uses a lease-based system. Remote objects
	are purged from the pool when the lease expires. Supports inheritance.

	Compile:
	soapcpp2 factory.h
	c++ -o factory factory.cpp stdsoap2.cpp soapC.cpp soapServer.cpp

	Run (e.g. in the background)
	factory <port>
	where <port> is a available port number, e.g. 18085

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2008, Robert van Engelen, Genivia, Inc. All Rights Reserved.
This software is released under one of the following two licenses:
GPL.
--------------------------------------------------------------------------------
GPL license.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place, Suite 330, Boston, MA 02111-1307 USA

Author contact information:
engelen@genivia.com / engelen@acm.org
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#include <sys/stat.h>	// for open()

#include "soapH.h"
#include "factory.nsmap"

#define POOLSIZE 1000	// max number of remote objects that can be alive at any instance

#define LEASETERM 60	// lease term (in seconds). Also the rate at which objects are purged

////////////////////////////////////////////////////////////////////////////////
//
//  Factory class maintains pool of objects and can act as a simple ORB
//
////////////////////////////////////////////////////////////////////////////////

class Factory
{ public:
  t__root *ref[POOLSIZE];	// pool of objects (t__root is base class)
  unsigned int handle;		// handle generation counter (is allowed to wrap around 32 bits)
  Factory();
  ~Factory();
  unsigned int create(struct soap *soap, enum t__object object, char *name);
  unsigned int lookup(enum t__object object, char *name);
  unsigned int rename(unsigned int handle, char *name);
  void release(unsigned int handle);
  void purge(struct soap* soap);
  t__root *get(unsigned int handle);
  t__root *root(unsigned int handle);
  t__adder *adder(unsigned int handle);
  t__counter *counter(unsigned int handle);
  int save(const char *file);
  int load(const char *file);
};

// Initialize empty pool and set handle generation counter to 0
Factory::Factory()
{ for (int i = 0; i < POOLSIZE; i++)
    ref[i] = NULL;
  handle = 0;
}

// Remove all objects from pool
Factory::~Factory()
{ for (int i = 0; i < POOLSIZE; i++)
    if (ref[i])
      delete ref[i];
}

// Create a new object, place it in the pool, and return handle
unsigned int Factory::create(struct soap *soap, enum t__object object, char *name)
{ (void)soap;
  for (int i = 0; i < POOLSIZE; i++)
    if (!ref[++handle % POOLSIZE])	// succeeds if this slot is available
    { t__root *r = NULL;
      if (!handle)
        handle += POOLSIZE;		// make sure handle is never 0 (0 indicates invalid handle)
      switch (object)			// type of object to instantiate
      { case ROOT:
          r = new t__root();
	  break;
        case ADDER:
          r = new t__adder();
	  break;
        case COUNTER:
          r = new t__counter();
	  break;
      }
      if (r)
      { ref[handle % POOLSIZE] = r;		// add object to the pool
        r->object = object;			// save type
	if (name)				// save name (if assigned)
	  r->name = strdup(name);
	else
	  r->name = NULL;
        r->handle = handle;			// keep handle for verification
        r->renew();				// renew its lease
        return handle;
      }
      return 0;
    }
  return 0;
}

// Lookup the name of an object and return handle
unsigned int Factory::lookup(enum t__object object, char *name)
{ for (int i = 0; i < POOLSIZE; i++)
    if (ref[i] && ref[i]->object == object && ref[i]->name && !strcmp(ref[i]->name, name))
    { ref[i]->renew();
      return ref[i]->handle;
    }
  return 0;
}

// Rename object and return handle if successful
unsigned int Factory::rename(unsigned int handle, char *name)
{ t__root *r = get(handle);
  if (r)
  { size_t l = strlen(name);
    if (r->name)
      free(r->name);
    r->name = (char*)malloc(l + 1);
    soap_strcpy(r->name, l + 1, name);
    r->renew();
    return handle;
  }
  return 0;
}

// get ref to object from handle
t__root *Factory::get(unsigned int handle)
{ t__root *r = ref[handle % POOLSIZE];
  if (r && r->handle == handle)
    return r;
  return NULL;
}

// get ref to root object from handle and renew lease
t__root *Factory::root(unsigned int handle)
{ t__root *r = get(handle);
  if (r)
    r->renew();
  return r;
}

// get ref to adder object from handle and renew lease
t__adder *Factory::adder(unsigned int handle)
{ t__adder *a = (t__adder*)get(handle);
  if (a)
  { if (a->object == ADDER || a->object == COUNTER)
      a->renew();
    else
      a = NULL;
  }
  return a;
}

// get ref to counter object from handle and renew lease
t__counter *Factory::counter(unsigned int handle)
{ t__counter *c = (t__counter*)get(handle);
  if (c)
  { if (c->object == COUNTER)
      c->renew();
    else
      c = NULL;
  }
  return c;
}

// remove all objects from pool whose lease has expired
void Factory::purge(struct soap *soap)
{ time_t t = time(NULL);		// current time
  int flag = 1;
  for (int i = 0; i < POOLSIZE; i++)
  { t__root *r = ref[i];
    if (r && r->lease < t)		// expired?
    { if (flag)
        fprintf(stderr, "\nPurging objects:");
      if (r->name)
        fprintf(stderr, "%s(%u)\n", r->name, r->handle);
      else
        fprintf(stderr, "(%u)\n", r->handle);
      soap_delete(soap, r);
      ref[i] = NULL;
      flag = 0;
    }
  }
}

// remove object from pool and release slot
void Factory::release(unsigned int handle)
{ t__root *r = get(handle);
  if (r)
    ref[handle % POOLSIZE] = NULL;
}

// save object pool to file (or stdout)
int Factory::save(const char *file)
{ struct soap soap;	// use a new local gSOAP environment
  soap_init(&soap);
  soap_begin(&soap);
  if (file)
    soap.sendfd = open(file, O_CREAT|O_TRUNC|O_WRONLY, S_IREAD|S_IWRITE);
  if (soap.sendfd < 0)
    return -1;
  soap_begin_send(&soap);
  for (int i = 0; i < POOLSIZE; i++)
    if (ref[i])
    { ref[i]->soap_serialize(&soap);
      soap_begin_send(&soap);
      ref[i]->soap_put(&soap, "item", NULL);
      soap_end_send(&soap);
    }
  if (file)
    close(soap.sendfd);
  soap_end(&soap);
  soap_done(&soap);
  return 0;
}

// load object pool from file (or stdin)
int Factory::load(const char *file)
{ struct soap soap;
  t__root *r;
  soap_init(&soap);
  if (file)
    soap.recvfd = open(file, O_RDONLY);
  if (soap.recvfd < 0)
    return -1;
  soap_begin_recv(&soap);
  for (int i = 0; i < POOLSIZE; i++)
  { if (ref[i])
    { delete ref[i];
      ref[i] = NULL;
    }
  }
  for (;;)
  { r = soap_in_t__root(&soap, "item", NULL, NULL);	// use the 'in' routine ('get' will also attempt to parse the remaining XML)
    if (r)
      ref[r->handle % POOLSIZE] = r;
    else
      break;
  }
  if (file)
    close(soap.recvfd);
  if (soap.error != SOAP_OK && soap.error != SOAP_EOF)
  { soap_print_fault(&soap, stderr);
    soap_print_fault_location(&soap, stderr);
  }
  soap_free(&soap);	// do not call soap_end: this would remove all deserialized data
  soap_done(&soap);
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Main server program
//
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{ int m, s;
  struct soap soap;
  Factory factory;			// create factory and simple ORB
  soap_init(&soap);
  soap.user = (void*)&factory;		// associate factory with run-time
  soap.accept_timeout = 1;		// check every second, if not too busy for purging objects
  if (argc < 2)
  { factory.load("factory.dat");	// if CGI is used, load the entire pool (not very efficient and there may be a competition for access to this file! This is just to demonstrate load/save of the entire pool)
    factory.purge(&soap);
    soap_serve(&soap);
    factory.save("factory.dat");	// ... and save afterwards
  }
  else
  { m = soap_bind(&soap, NULL, atoi(argv[1]), 100);	// use command line argument as port number
    if (m < 0)
    { soap_print_fault(&soap, stderr);
      exit(1);
    }
    fprintf(stderr, "Socket connection successful %d\n", m);
    for (int i = 1; ; i++)
    { s = soap_accept(&soap);
      if (s < 0)
      { if (soap.errnum)
          soap_print_fault(&soap, stderr);
	else			// errnum is 0, which means a timeout has occurred
	{ factory.purge(&soap);	// purge objects whose lease has ran out
	  continue;
	}
	exit(1);
      }
      fprintf(stderr, "%d: accepted %d IP=%d.%d.%d.%d ... ", i, s, (int)(soap.ip>>24)&0xFF, (int)(soap.ip>>16)&0xFF, (int)(soap.ip>>8)&0xFF, (int)soap.ip&0xFF);
      soap_serve(&soap);
      fprintf(stderr, "served\n");
      soap_end(&soap);		// clean up: this will remove deserialized data
    }
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Remote factory method implementations
//
////////////////////////////////////////////////////////////////////////////////

int ns__create(struct soap *soap, enum t__object object, char *name, enum t__status &status)
{ Factory *factory = (Factory*)soap->user;	// get factory from gSOAP environment
  if (!soap->header)
    soap->header = (struct SOAP_ENV__Header*)soap_malloc(soap, sizeof(struct SOAP_ENV__Header));
  if (soap->header)
  { soap->header->h__handle = factory->create(soap, object, name);
    if (soap->header->h__handle)
      status = FACTORY_OK;
    else
      status = FACTORY_INVALID;
  }
  else
    status = FACTORY_RETRY;
  return SOAP_OK;
}

int ns__lookup(struct soap *soap, enum t__object object, char *name, enum t__status &status)
{ Factory *factory = (Factory*)soap->user;	// get factory from gSOAP environment
  if (!soap->header)
    soap->header = (struct SOAP_ENV__Header*)soap_malloc(soap, sizeof(struct SOAP_ENV__Header));
  if (soap->header)
  { soap->header->h__handle = factory->lookup(object, name);
    if (soap->header->h__handle)
      status = FACTORY_OK;
    else
      status = FACTORY_NOTFOUND;
  }
  else
    status = FACTORY_RETRY;
  return SOAP_OK;
}

int ns__rename(struct soap *soap, char *name, enum t__status &status)
{ Factory *factory = (Factory*)soap->user;	// get factory from gSOAP environment
  if (soap->header)
  { soap->header->h__handle = factory->rename(soap->header->h__handle, name);
    if (soap->header->h__handle)
      status = FACTORY_OK;
    else
      status = FACTORY_INVALID;
  }
  else
    status = FACTORY_INVALID;
  return SOAP_OK;
}

int ns__release(struct soap *soap, enum t__status &status)
{ Factory *factory = (Factory*)soap->user;	// get factory from gSOAP environment
  if (soap->header && soap->header->h__handle)
  { factory->release(soap->header->h__handle);
    status = FACTORY_OK;
  }
  else
    status = FACTORY_INVALID;
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Remote adder method implementations
//
////////////////////////////////////////////////////////////////////////////////

int ns__set(struct soap *soap, double val, enum t__status &status)
{ Factory *factory = (Factory*)soap->user;	// get factory from gSOAP environment
  if (soap->header)
  { t__adder *adder = factory->adder(soap->header->h__handle);
    if (adder)
    { adder->set(val);
      status = FACTORY_OK;
    }
    else
      status = FACTORY_INVALID;
  }
  else
    status = FACTORY_INVALID;
  return SOAP_OK;
}

int ns__get(struct soap *soap, double &val)
{ Factory *factory = (Factory*)soap->user;	// get factory from gSOAP environment
  val = DBL_NAN;
  if (soap->header)
  { t__adder *adder = factory->adder(soap->header->h__handle);
    if (adder)
      val = adder->get();
  }
  return SOAP_OK;
}

int ns__add(struct soap *soap, double val, enum t__status &status)
{ Factory *factory = (Factory*)soap->user;	// get factory from gSOAP environment
  if (soap->header)
  { t__adder *adder = factory->adder(soap->header->h__handle);
    if (adder)
    { adder->add(val);
      status = FACTORY_OK;
    }
    else
      status = FACTORY_INVALID;
  }
  else
    status = FACTORY_INVALID;
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Remote counter method implementations
//
////////////////////////////////////////////////////////////////////////////////

int ns__inc(struct soap *soap, enum t__status &status)
{ Factory *factory = (Factory*)soap->user;	// get factory from gSOAP environment
  if (soap->header)
  { t__counter *counter = factory->counter(soap->header->h__handle);
    if (counter)
    { counter->inc();
      status = FACTORY_OK;
    }
    else
      status = FACTORY_INVALID;
  }
  else
    status = FACTORY_INVALID;
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Server-side base factory class methods
//
////////////////////////////////////////////////////////////////////////////////

t__root::t__root()
{ }

t__root::~t__root()
{ if (name)
    free(name);
}

void t__root::renew()
{ lease = time(NULL) + LEASETERM;	// can adopt a leasing policy per class
}

////////////////////////////////////////////////////////////////////////////////
//
//  Server-side adder class methods
//
////////////////////////////////////////////////////////////////////////////////

void t__adder::set(double val)
{ this->val = val;	// copy data to update state
}

double t__adder::get()
{ return val;
}

void t__adder::add(double val)
{ this->val += val;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Server-side counter class methods
//
////////////////////////////////////////////////////////////////////////////////

void t__counter::inc()
{ add(1.0);
}

