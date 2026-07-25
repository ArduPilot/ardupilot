 /*
        gsoapios.mm
 
        iOS plugin (iPhone and iPad)
 
gSOAP XML Web services tools
Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under one of the following licenses:
GPL, the gSOAP public license, or Genivia's license for commercial use.
-------------------------------------------------------------------------------
gSOAP public license.

The contents of this file are subject to the gSOAP Public License Version 1.3
(the "License"); you may not use this file except in compliance with the
License. You may obtain a copy of the License at
http://www.cs.fsu.edu/~engelen/soaplicense.html
Software distributed under the License is distributed on an "AS IS" basis,
WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
for the specific language governing rights and limitations under the License.

The Initial Developer of the Original Code is Robert A. van Engelen.
Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc., All Rights Reserved.
-------------------------------------------------------------------------------
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

This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
-------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
-------------------------------------------------------------------------------
*/

#import "gsoapios.h"
#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

// -----------------------------------------------------------------------------
// Start of objective-c Class GSoapiOSURLData implementation
// -----------------------------------------------------------------------------

/**
Class GSoapiOSURLData
@brief The class stores the states for connection establishment, buffering soap
request and response 
*/
@implementation GSoapiOSURLData

@synthesize endpoint;           ///< Set or get endpoint
@synthesize host;               ///< Set or get host name
@synthesize soap_action;        ///< Set or get soap action
@synthesize http_method;        ///< Set or get HTTP method
@synthesize content_type;       ///< Set or get content type
@synthesize http_req_msg;       ///< Set or get HTTP request message
@synthesize soap_req_msg;       ///< Set or get SOAP request
@synthesize soap_res_msg;       ///< Set or get SOAP response

@synthesize port;               ///< Set or get port
@synthesize received_length;    ///< Set or get received message length 
@synthesize request_sent;       ///< Set or get the flag for the first time sending
@synthesize req_content_length; ///< Set or get request message length

@synthesize timeout_interval;   ///< Set or get timeout interval
@synthesize cache_policy;       ///< Set or get cache policy

@synthesize user_name;          ///< Set or get user ID for authentication
@synthesize password;           ///< Set or get password for authentication

/**
@fn init
@brief Constructor
*/
- (id) init {
    self = [super init];
    
    http_req_msg = [NSMutableData alloc];
        
    timeout_interval = 60.0; // 60 seconds by default
    cache_policy = NSURLRequestUseProtocolCachePolicy; // The default cache policy

    return self;
}

@end
// -----------------------------------------------------------------------------
// End of objective-c Class GSoapiOSURLData implementation
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// Start of C plugin implementation
// -----------------------------------------------------------------------------


/** plugin identification for plugin registry */
const char soap_ios_id[13] = SOAP_IOS_ID; // Unique plugin ID

static int soap_ios_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src);
static void soap_ios_delete(struct soap *soap, struct soap_plugin *p);
static int soap_ios_init(struct soap *soap, soap_ios_data *data);
static SOAP_SOCKET soap_ios_open( struct soap *soap, const char *endpoint, const char *host, int port);
static int soap_ios_close( struct soap *soap);
static int soap_ios_send( struct soap *soap, const char *buf, size_t len);
static size_t soap_ios_recv(struct soap *soap, char *buf, size_t len);

//void soap_ios_setcachepolicy(struct soap *soap, unsigned int policy);
//void soap_ios_settimeoutinterval(struct soap *soap, double seconds);

/**
@fn int soap_ios(struct soap *soap, struct soap_plugin *p, void *arg);
@brief Save the old callbacks and set new callbacks to handle soap client
applications on iOS platforms (iPhone and iPad)
@param soap  The soap context
@param[in] p The soap plugin data
@param[in] arg The arguments for soap plugin
@return SOAP_OK if registration is successfull; SOAP_EOM otherwise
        
Usage:
@code
struct soap *soap = soap_new();
soap_register_plugin(soap, soap_ios);
@endcode
*/
int soap_ios(struct soap *soap, struct soap_plugin *p, void *arg)
{
  p->id = soap_ios_id; // set a unique plugin id p->data  = new soa
  p->data = static_cast<void*>(new soap_ios_data);  // Allocate memory

  p->fcopy = soap_ios_copy; // set fcopy callback for soap_copy(truct soap*)
  p->fdelete = soap_ios_delete; // set fdeete callback

  if (p->data)
  {
    if (soap_ios_init(soap, static_cast<soap_ios_data*>(p->data)))
    {
      delete static_cast<soap_ios_data*>(p->data); // free resource upon failure to initialize plugin
      return SOAP_EOM;
    }
  }       

  return SOAP_OK; 
}

/**
@fn int soap_ios_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src)
@brief Duplicates the plugin context for the soap context
@param soap The soap context
@param[out] dst The destination plugin data to store to
@param[in] src The sources plugin data to be copied from
@return SOAP_OK if registration is successfull; SOAP_ERR otherwise
*/
static int soap_ios_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src)
{     
  *dst = *src;
  dst->data = static_cast<void*>(new soap_ios_data);
  if (dst->data)
    return soap_ios_init(soap, static_cast<soap_ios_data*>(dst->data));

  return SOAP_ERR;
}

/**
@fn void soap_ios_delete(struct soap *soap, struct soap_plugin *p)
@brief Destroys the plugin context
@param soap context
@param[in] p plugin data
*/
static void soap_ios_delete(struct soap *soap, struct soap_plugin *p)
{
  soap_ios_data *data = static_cast<soap_ios_data*>(p->data);
  if (data && data->url_data)// Free url_data // [data->url_data release]
    delete data; // Free the pointer to url_data
}

/**
@fn int soap_ios_init(struct soap *soap, soap_ios_data *data)
@brief Initializes the plugin data 
@param soap The soap context
@param[out] data The plugin data to be initialized
@return SOAP_OK if initialization is sucessful; SOAP_ERR otherwise
*/
static int soap_ios_init(struct soap *soap, soap_ios_data *data)
{       
  data->url_data = [[GSoapiOSURLData alloc] init];

  if (data->url_data == nil)
    return SOAP_ERR; // failure to init

  data->res_len = 0;
  data->res_msg = "";

  // save and set callbacks
  data->fopen = soap->fopen;   // save old fopen callback
  data->fclose = soap->fclose; // save old fclose callback
  data->fsend = soap->fsend;   // save old fsend callback
  data->frecv = soap->frecv;   // save old frecv callback
  data->buf = nil;             // sets initial value of buf to nil
  data->len = 0;               // sets initial buffer length value to 0

  soap->fopen = soap_ios_open;   // replace open callback
  soap->fclose = soap_ios_close; // replace close callback
  soap->fsend = soap_ios_send;   // replace send callback 
  soap->frecv = soap_ios_recv;   // replace recv callback

  return SOAP_OK;
}

/**
@fn SOAP_SOCKET soap_ios_open( struct soap *soap, const char *endpoint,
const char *host, int port)
@brief Extracts states from soap context and saves states for soap connection  
@param soap The soap context
@param[in] endpoint The Web service's endpoint
@param[in] host The host name
@param[in] port The port numer
@return 1 A valid socket

Note: This open function does not create a real socket for communication. The
actual connection establishment happens in the frecv callback
*/
static SOAP_SOCKET soap_ios_open( struct soap *soap, const char *endpoint, const char *host, int port)
{ 
  // Pull out the pluin data
  soap_ios_data *data = static_cast<soap_ios_data*>(soap_lookup_plugin(soap, soap_ios_id));

  if (data == NULL || data->url_data == nil)
    return SOAP_INVALID_SOCKET;  

  GSoapiOSURLData * url_data = data->url_data;

  [url_data setEndpoint:[NSString stringWithUTF8String:endpoint]];
  [url_data setHost:[NSString stringWithUTF8String:host]];
  [url_data setPort:port];
  [url_data setRequest_sent:false];
  [url_data setReceived_length:0];
  [url_data setSoap_action:[NSString stringWithUTF8String:soap->action]];
  [url_data setReq_content_length:soap->count];

  // Extract a pair of userid and password if present
  if (soap->userid && soap->passwd)
  {
    [url_data setUser_name:[NSString stringWithUTF8String:soap->userid]];
    [url_data setPassword:[NSString stringWithUTF8String:soap->passwd]];
  }
  else
  {
    [url_data setUser_name:nil];
    [url_data setPassword:nil];
  }

  // Extracts and saves http-method
  switch (soap->status)
  {
    case SOAP_POST:
      [url_data setHttp_method:@"POST"];
      break;
    case SOAP_GET:
      [url_data setHttp_method:@"GET"];
      break;  
    case SOAP_DEL:
      [url_data setHttp_method:@"DELETE"];
      break;
    case SOAP_CONNECT:
      [url_data setHttp_method:@"CONNECT"];
      break;
    default:
      [url_data setHttp_method:@"POST"];
      break;                  
  }

  // Extracts and saves http content-type
  if ((soap->mode & SOAP_ENC_DIME) && !(soap->mode & SOAP_ENC_MTOM))
  {
    [url_data setContent_type:@"application/dime"];
  }
  else if (soap->version == 2)
  {
    if (soap->mode & SOAP_ENC_MTOM)
    {
      [url_data setContent_type:@"application/xop+xml; charset=utf-8; \
        type=\"application/soap+xml\""];               
    }
    else
    {
      [url_data setContent_type:@"application/soap+xml; charset=utf-8"];
    }
  }
  else if (soap->mode & SOAP_ENC_MTOM)
  {
    [url_data setContent_type:@"application/xop+xml; charset=utf-8; \
      type=\"text/xml\""];
  }
  else
  {
    [url_data setContent_type:@"text/xml; charset=utf-8"];
  }

  return 1;
}

/**
@fn int soap_ios_close(struct soap *soap)
@param soap The soap context
@return 0

Note: This open function does not create a real socket for communication. The
actual connection establishment happens in the frecv callback
*/
static int soap_ios_close( struct soap *soap)
{
  return 0;
}

/**
@fn int soap_ios_send( struct soap *soap, const char *buf, size_t len)
@brief Buffers the http/soap request in plugin data  
@param soap The soap context
@param[in] buf The buffer holding the serialized http/soap request
@param[in] len The buffer size
@return The number of bytes buffered

Note: This send function does not send any data to its peer. Rather it buffers
the serialied soap request including http header to the plugin data.
The sending happens actually in frecv function.
*/
static int soap_ios_send( struct soap *soap, const char *buf, size_t len)
{

  soap_ios_data *data = static_cast<soap_ios_data*>(soap_lookup_plugin(soap, soap_ios_id));

  GSoapiOSURLData *url_data = data->url_data;

  [url_data.http_req_msg appendBytes:buf length: len];

  return len > 0 ? SOAP_OK : SOAP_EOF;
}

/**
@fn size_t soap_ios_recv( struct soap *soap, char *buf, size_t len)
@brief Sends soap request, buffers soap response and serialize the response
@param soap The soap context
@param[out] buf The buffer for the received data
@param[in] len The number of bytes received
@return The number of bytes received

Note: This recv function sends a synchronous soap request through NSURLRequest,
receives soap response, serializes response.
*/
static size_t soap_ios_recv( struct soap *soap, char *buf, size_t len)
{
  soap_ios_data *data = static_cast<soap_ios_data*>(soap_lookup_plugin(soap, soap_ios_id));

  GSoapiOSURLData *url_data = data->url_data;

  // Handle HTTP basic authentication if present
  if (url_data.user_name && url_data.password)
  {

    NSURLCredential *credential = [NSURLCredential
      credentialWithUser:url_data.user_name
      password:url_data.password
      persistence:NSURLCredentialPersistenceForSession];


    NSURLProtectionSpace * protectionSpace = [[NSURLProtectionSpace alloc]
      initWithHost:url_data.host
      port:url_data.port
      protocol:@"http"
      realm:nil
      authenticationMethod:nil];

    [[NSURLCredentialStorage sharedCredentialStorage]
      setDefaultCredential:credential
      forProtectionSpace:protectionSpace];
  }

  // -------- End of auhentication handling ---------

  // Execute once: sending the request and receiving response
  // Send request through NSURLSessionDataTask and completionHandler
  // and receive response
  if (url_data.request_sent == false)
  { 
    [url_data setRequest_sent:true];

    //int msg_len = [url_data.http_req_msg length];
    NSRange range = NSMakeRange([url_data.http_req_msg length] 
        - url_data.req_content_length, 
        url_data.req_content_length);

    url_data.soap_req_msg = [url_data.http_req_msg subdataWithRange:range];

    NSURL *url = [NSURL URLWithString:url_data.endpoint];           
    NSMutableURLRequest *request = [NSMutableURLRequest
      requestWithURL:url
      cachePolicy:(NSURLRequestCachePolicy)url_data.cache_policy
      timeoutInterval:url_data.timeout_interval];

    [request addValue:url_data.host forHTTPHeaderField:@"HOST"];
    [request addValue:url_data.soap_action forHTTPHeaderField:@"SOAPACTION"];               
    [request addValue:url_data.content_type forHTTPHeaderField:@"Content-Type"];
    [request setHTTPMethod:url_data.http_method];   

    NSString *conten_length = [NSString stringWithFormat:@"%d",
             [url_data.soap_req_msg length]];
    [request addValue:conten_length forHTTPHeaderField:@"Content-Length"];
    [request setHTTPBody:url_data.soap_req_msg];

    dispatch_semaphore_t semaphore = dispatch_semaphore_create(0);          //ensures DataTask is synchronous

    NSURLSessionDataTask *dataTask = [[NSURLSession sharedSession]
      dataTaskWithRequest:request completionHandler:^(NSData *data, NSURLResponse *response, NSError *error)
      {
        // Error handling
        if (error)
        {
          NSLog(@"ERROR:<%@>\n", [error localizedDescription]);
          //const char *fault_string = [[error localizedDescription] UTF8String];

          soap_receiver_fault(soap, [[error localizedDescription ] UTF8String], "");
        } 
        else
        {
          url_data.soap_res_msg = [NSData dataWithData:data];
        }

        dispatch_semaphore_signal(semaphore);
      } ];
    [dataTask resume];
    dispatch_semaphore_wait(semaphore, DISPATCH_TIME_FOREVER);
    //[url_data setRequest_sent:true];
  }

  // may execute multiple times depending on the buffer size and response length
  if (url_data.request_sent)
  {
    if (data->res_msg == "" && data->res_len == 0)
    {
      data->res_msg = (const char*)[url_data.soap_res_msg bytes];  //result message as string
      data->res_len = [url_data.soap_res_msg length];              // response length
    }

    if (data->res_len < len)           // if the # of bytes you neeed to read is < the max num of bytes. Only copy what you
      len = data->res_len;             // need
    memcpy(buf,data->res_msg,len);     // copy to the buffer

    data->res_msg += len ;             // point to the byte you will start copying from on the next call
    data->res_len -= len ;             // update the amount of bytes you still need to copy
  }
  return len;                          // no longer calls function when len = 0
}


/**
@fn void soap_ios_setcachepolicy(struct soap *soap, unsigned int policy)
@brief Sets cache policy
The constants used to specify interaction with the cached responses are:

enum {
       NSURLRequestUseProtocolCachePolicy = 0,
       NSURLRequestReloadIgnoringLocalCacheData = 1,
       NSURLRequestReloadIgnoringCacheData = NSURLRequestReloadIgnoringLocalCacheData,
       NSURLRequestReturnCacheDataElseLoad = 2,
       NSURLRequestReturnCacheDataDontLoad = 3,
       NSURLRequestReloadIgnoringLocalAndRemoteCacheData =4,
       NSURLRequestReloadRevalidatingCacheData = 5
};

@param soap The soap context
@param[in] policy The policy to be specified for he request
*/
void soap_ios_setcachepolicy(struct soap *soap, unsigned int policy)
{
  soap_ios_data *data = static_cast<soap_ios_data*>(soap_lookup_plugin(soap, soap_ios_id));

  data->url_data.cache_policy = policy;
}

/**
@fn soap_ios_settimeoutinterval(struct soap *soap, double seconds)
@brief Sets timeout interval
@param soap The soap context
@param[in] seconds The value for the timeout interval to be specified (in seconds)
*/
void soap_ios_settimeoutinterval(struct soap *soap, double seconds)
{
  soap_ios_data *data = static_cast<soap_ios_data*>(soap_lookup_plugin(soap, soap_ios_id));

  data->url_data.timeout_interval = seconds;
}

// -----------------------------------------------------------------------------
// END of plugin implementation
// -----------------------------------------------------------------------------
