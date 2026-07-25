/*
        gsoapios.h
 
        iOS plugin (iPhone and iPad)
 
gSOAP XML Web services tools
Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under one of the following licenses:
GPL or the gSOAP public license.
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

#ifndef GSOAPIOS_H
#define GSOAPIOS_H

#import "stdsoap2.h"
#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

/**
 @brief stores states for connection establishment, buffering soap request and 
 response 
 */
@interface GSoapiOSURLData : NSObject {
        
@private
        NSString *endpoint;
        NSString *host;
        NSString *soap_action;
        NSString *http_method;
        NSString *content_type;
        NSMutableData *http_req_msg;
        NSData *soap_req_msg;
        NSData *soap_res_msg;
        int port;
        int req_content_length;
        int received_length; // received length of response for deserialization
        bool request_sent; // flag indicates a soap request has been made
        
        // The timeout interval for the new request, in seconds
        double timeout_interval;
        unsigned int cache_policy; // The cache policy for the new request      
        
        NSString *user_name;
        NSString *password;
}

@property(nonatomic, retain) NSString *endpoint; // Web service endpoint
@property(nonatomic, retain) NSString *host; // host name
@property(nonatomic, retain) NSString *soap_action; // soap action
@property(nonatomic, retain) NSString *http_method; // http method
@property(nonatomic, retain) NSString *content_type; // http content type

// http request messgae
@property(nonatomic, retain) NSMutableData *http_req_msg;

// soap request message
@property(nonatomic, retain) NSData *soap_req_msg;

// soap response xml message
@property(nonatomic, retain) NSData *soap_res_msg;
//----------------------------------
@property(nonatomic) int  port; // port
@property(nonatomic) int req_content_length; // request content length

// received length of response for deserialization
@property(nonatomic) int received_length;

// flag indicates a soap request has been made
@property(nonatomic) bool request_sent;

// the timout interval for request
@property(nonatomic) double timeout_interval;
@property(nonatomic) unsigned int cache_policy;
//--------------------------------------------

// user and password: must appear in pair
@property(nonatomic, retain) NSString *user_name; // user name
@property(nonatomic, retain) NSString *password; // pasword

- (id) init;

@end

/** plugin identification for plugin registry */
#define SOAP_IOS_ID "SOAP_IOS-1.0"

#ifdef __cplusplus
extern "C" {
#endif

        /**
         @struct soap_ios_data
         @brief  soap_ios plugin data to override callbacks and store states
         to manipulate SOAP connection using iOS libraries
         */
        struct soap_ios_data {
                
                // Stores states such as endpoint, serialized request and response
                GSoapiOSURLData *url_data;
        
                // Buffer variable
                char* buf;

                // Tracks the size of the buffer
                size_t len;

                // Pointer to the result message
                const char *res_msg;
        
                // The number of bytes the buffer has left to copy from the res_msg
                size_t res_len;
                
                // A callback function to replace the default tcp_connect function
                // Rather than establishment of a tcp connection using socket, this callback
                // extracts and saves states (endpoint, http-method, content-type etc.) and
                // buffers serialized request
                SOAP_SOCKET (*fopen)(struct soap*, const char*, const char*, int);
                
                // A callback function to replace the default tcp_disconnect function
                // Since no connection is established in fopen, this function does noop
                int (*fclose)(struct soap*);
                
                // A callback function to replace the default send function
                // This function buffers serialized request     including HTTP header   and body
                // (soap message)
                int (*fsend)(struct soap*, const char *, size_t);
                
                // A callback function to replace the default send function
                // This function sends a synchrous request to the endpoint and receives 
                // response using iOS SDK class NSURLConnection. The response is then snet
                // to soap for deserialization.
                size_t (*frecv)(struct soap*, char *, size_t);         
        };
        
        /**
        @fn int soap_ios(struct soap *soap, struct soap_plugin *p, void *arg);
        @brief Save the old callbacks and set new callbacks to handle soap
        client applications on iOS platforms (iPhone and iPad)
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
        int soap_ios(struct soap *soap, struct soap_plugin *p, void *arg);
        
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
        void soap_ios_setcachepolicy(struct soap *soap, unsigned int policy);
        
        /**
        @fn soap_ios_settimeoutinterval(struct soap *soap, double seconds)
        @brief Sets timeout interval
        @param soap The soap context
        @param[in] seconds The value for the timeout interval to be specifed (in seconds)
        */
        void soap_ios_settimeoutinterval(struct soap *soap, double seconds);
        
#ifdef __cplusplus
}
#endif  // __cplusplus

#endif

