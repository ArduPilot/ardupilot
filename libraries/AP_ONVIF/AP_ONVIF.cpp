/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Michael Oborne and Siddharth Bharat Purohit, Cubepilot Pty.
 */

#include "AP_ONVIF.h"
#if ENABLE_ONVIF
#include <AP_ONVIF/MediaBinding.nsmap>

#include "onvifhelpers.h"
// For ChibiOS we will use HW RND # generator
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

#if 0
#define DEBUG_PRINT(fmt,args...) do {GCS_SEND_TEXT(MAV_SEVERITY_INFO ,"AP_ONVIF:" fmt "\n", ## args); } while(0)
#else
#define DEBUG_PRINT(fmt,args...)
#endif

#define ERROR_PRINT(fmt,args...) do {GCS_SEND_TEXT(MAV_SEVERITY_ERROR , "AP_ONVIF:" fmt "\n", ## args); } while(0)

const char *wsse_PasswordDigestURI = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordDigest";
const char *wsse_Base64BinaryURI = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-soap-message-security-1.0#Base64Binary";

AP_ONVIF *AP_ONVIF::_singleton;
static AP_ONVIF onvif;

#define DEVICE_ENDPOINT_LOC "/onvif/device_service"
#define MEDIA_ENDPOINT_LOC  "/onvif/media_service"
#define PTZ_ENDPOINT_LOC    "/onvif/ptz_service"
// Default constructor
AP_ONVIF::AP_ONVIF()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_ONVIF must be singleton");
    }
    _singleton = this;
}

// Start ONVIF client with username, password and service host url
bool AP_ONVIF::start(const char *user, const char *pass, const char *hostname)
{
    if (!initialised) {
        soap = soap_new1(SOAP_XML_CANONICAL | SOAP_C_UTFSTRING);
        if (soap == nullptr) {
            ERROR_PRINT("AP_ONVIF: Failed to allocate soap");
            return false;
        }
        soap->connect_timeout = soap->recv_timeout = soap->send_timeout = 30; // 30 sec

        if (proxy_device == nullptr) {
            proxy_device = new DeviceBindingProxy(soap);
        }
        if (proxy_media == nullptr) {
            proxy_media = new MediaBindingProxy(soap);
        }
        if (proxy_ptz == nullptr) {
            proxy_ptz = new PTZBindingProxy(soap);
        }

        if (proxy_device == nullptr ||
            proxy_media == nullptr ||
            proxy_ptz == nullptr) {
            ERROR_PRINT("AP_ONVIF: Failed to allocate gSOAP Proxy objects.");
            return false;
        }
        initialised = true;
    }

    if (strlen(user) > username_len) {
        if (username != nullptr) {
            free(username);
            username = nullptr;
        }
    }
    username_len = strlen(user);
    if (username == nullptr) {
        username = (char*)malloc(username_len + 1);
    }

    if (strlen(pass) > password_len) {
        if (password != nullptr) {
            free(password);
            password = nullptr;
        }
    }
    password_len = strlen(pass);
    if (password == nullptr) {
        password = (char*)malloc(password_len+1);
    }

    if (strlen(hostname) > hostname_len) {
        // free if not nullptr
        if (device_endpoint != nullptr) {
            free(device_endpoint);
            device_endpoint = nullptr;
        }
        if (media_endpoint != nullptr) {
            free(media_endpoint);
            media_endpoint = nullptr;
        }
        if (ptz_endpoint != nullptr) {
            free(ptz_endpoint);
            ptz_endpoint = nullptr;
        }
    }

    hostname_len = strlen(hostname);

    if (device_endpoint == nullptr) {
        device_endpoint = (char*)malloc(hostname_len + strlen(DEVICE_ENDPOINT_LOC) + 1);
    }
    if (media_endpoint == nullptr) {
        media_endpoint = (char*)malloc(hostname_len + strlen(MEDIA_ENDPOINT_LOC) + 1);
    }
    if (ptz_endpoint == nullptr) {
        ptz_endpoint = (char*)malloc(hostname_len + strlen(PTZ_ENDPOINT_LOC) + 1);
    }

    if (device_endpoint == nullptr ||
        media_endpoint == nullptr ||
        ptz_endpoint == nullptr ||
        username == nullptr ||
        password == nullptr) {
        ERROR_PRINT("Failed to Allocate strings");
        return false;
    }

    strcpy(username, user);
    strcpy(password, pass);
    snprintf(device_endpoint, hostname_len + strlen(DEVICE_ENDPOINT_LOC) + 1, "%s" DEVICE_ENDPOINT_LOC, hostname);
    snprintf(media_endpoint, hostname_len + strlen(MEDIA_ENDPOINT_LOC) + 1, "%s" MEDIA_ENDPOINT_LOC, hostname);
    snprintf(ptz_endpoint, hostname_len + strlen(PTZ_ENDPOINT_LOC) + 1, "%s" PTZ_ENDPOINT_LOC, hostname);

    /// TODO: Need to find a way to store this in parameter system
    //  or it could be just storage, we will see
    proxy_device->soap_endpoint = device_endpoint;
    if (!probe_onvif_server()) {
        ERROR_PRINT("Failed to probe onvif server.");
        return false;
    }

    return true;
}

void AP_ONVIF::report_error()
{
    ERROR_PRINT("ONVIF ERROR:");
    if (soap_check_state(soap)) {
        ERROR_PRINT("Error: soap struct state not initialized");
    } else if (soap->error) {
        const char **c, *v = NULL, *s, *d;
        c = soap_faultcode(soap);
        if (!*c) {
            soap_set_fault(soap);
            c = soap_faultcode(soap);
        }
        if (soap->version == 2) {
            v = soap_fault_subcode(soap);
        }
        s = soap_fault_string(soap);
        d = soap_fault_detail(soap);
        ERROR_PRINT("%s%d fault %s [%s]\n%s\nDetail: %s",(soap->version ? "SOAP 1." : "Error "),
                                                      (soap->version ? (int)soap->version : soap->error),
                                                      *c, (v ? v : "no subcode"), (s ? s : "[no reason]"),
                                                      (d ? d : "[no detail]"));
    }
}

// detect onvif server present on the network
bool AP_ONVIF::probe_onvif_server()
{
    {
        _tds__GetDeviceInformation GetDeviceInformation;
        _tds__GetDeviceInformationResponse GetDeviceInformationResponse;
        if (!set_credentials()) {
            ERROR_PRINT("Failed to setup credentials");
            goto err;
        }
        if (proxy_device->GetDeviceInformation(&GetDeviceInformation, GetDeviceInformationResponse)) {
            ERROR_PRINT("Failed to fetch Device Information");
            report_error();
            goto err;
        }

        DEBUG_PRINT("Manufacturer:    %s",GetDeviceInformationResponse.Manufacturer);
        DEBUG_PRINT("Model:           %s",GetDeviceInformationResponse.Model);
        DEBUG_PRINT("FirmwareVersion: %s",GetDeviceInformationResponse.FirmwareVersion);
        DEBUG_PRINT("SerialNumber:    %s",GetDeviceInformationResponse.SerialNumber);
        DEBUG_PRINT("HardwareId:      %s",GetDeviceInformationResponse.HardwareId);
    }

    // get device capabilities and print media
    {
        _tds__GetCapabilities GetCapabilities;
        _tds__GetCapabilitiesResponse GetCapabilitiesResponse;
        if (!set_credentials()) {
            ERROR_PRINT("Failed to setup credentials");
            goto err;
        }
        if (proxy_device->GetCapabilities(&GetCapabilities, GetCapabilitiesResponse)) {
            ERROR_PRINT("Failed to fetch Device Capabilities");
            report_error();
            goto err;
        }

        if (!GetCapabilitiesResponse.Capabilities || !GetCapabilitiesResponse.Capabilities->Media) {
            ERROR_PRINT("Missing device capabilities info");
            goto err;
        } else {
            DEBUG_PRINT("XAddr:        %s", GetCapabilitiesResponse.Capabilities->Media->XAddr);
            if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities) {
                if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTPMulticast) {
                    DEBUG_PRINT("RTPMulticast: %s",(*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTPMulticast ? "yes" : "no"));
                }
                if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORETCP) {
                    DEBUG_PRINT("RTP_TCP:      %s", (*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORETCP ? "yes" : "no"));
                }
                if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORERTSP_USCORETCP) {
                    DEBUG_PRINT("RTP_RTSP_TCP: %s", (*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORERTSP_USCORETCP ? "yes" : "no"));
                }
            }
        }

        // set the Media proxy endpoint to XAddr
        proxy_media->soap_endpoint = media_endpoint;
    }

    // get device profiles
    {
        _trt__GetProfiles GetProfiles;
        _trt__GetProfilesResponse GetProfilesResponse;
        if (!set_credentials()) {
            ERROR_PRINT("Failed to setup credentials");
            goto err;
        }
        if (proxy_media->GetProfiles(&GetProfiles, GetProfilesResponse)){
            ERROR_PRINT("Failed to fetch profiles");
            report_error();
            goto err;
        }
        
        if (GetProfilesResponse.__sizeProfiles > 0) {
            DEBUG_PRINT("Profiles Received %lu", (unsigned long)GetProfilesResponse.__sizeProfiles);
        } else {
            ERROR_PRINT("Error: No Profiles Received");
            goto err;
        }
        
        // for each profile get snapshot
        for (uint32_t i = 0; i < (uint32_t)GetProfilesResponse.__sizeProfiles; i++) {
            DEBUG_PRINT("Profile name: %s", GetProfilesResponse.Profiles[i]->Name);
        }
        
        // Just use first one for now
        if (profile_token_size < (strlen(GetProfilesResponse.Profiles[0]->token) + 1)) {
            if (profile_token != nullptr) {
                free(profile_token);
            }
            profile_token = (char*)malloc(strlen(GetProfilesResponse.Profiles[0]->token) + 1);
            profile_token_size = strlen(GetProfilesResponse.Profiles[0]->token) + 1;
            if (profile_token == nullptr) {
                goto err;
            }
        }

        strcpy(profile_token, GetProfilesResponse.Profiles[0]->token);

        proxy_ptz->soap_endpoint = ptz_endpoint;
    }

    // get PTZ Token
    {
        _tptz__GetConfigurations GetConfigurations;
        _tptz__GetConfigurationsResponse GetConfigurationsResponse;
        if (!set_credentials()) {
            ERROR_PRINT("Failed to setup credentials");
            goto err;
        }
        if (proxy_ptz->GetConfigurations(&GetConfigurations, GetConfigurationsResponse)) {
            ERROR_PRINT("Failed to fetch Configurations");
            report_error();
            goto err;
        }
        
        if (GetConfigurationsResponse.__sizePTZConfiguration > 0) {
            DEBUG_PRINT("PTZ Tokens Received");
        } else {
            ERROR_PRINT("Error: No Profiles Received");
            goto err;
        }
        
        for (uint32_t i = 0; i < (uint32_t)GetConfigurationsResponse.__sizePTZConfiguration; i++) {
            DEBUG_PRINT("PTZ: %s", GetConfigurationsResponse.PTZConfiguration[i]->Name);
        }
        //GetConfigurationsResponse.PTZConfiguration[0]->token
        pan_tilt_limit_max = Vector2f(GetConfigurationsResponse.PTZConfiguration[0]->PanTiltLimits->Range->XRange->Max,
                            GetConfigurationsResponse.PTZConfiguration[0]->PanTiltLimits->Range->YRange->Max);
        pan_tilt_limit_min = Vector2f(GetConfigurationsResponse.PTZConfiguration[0]->PanTiltLimits->Range->XRange->Min,
                            GetConfigurationsResponse.PTZConfiguration[0]->PanTiltLimits->Range->YRange->Min);
        zoom_min = GetConfigurationsResponse.PTZConfiguration[0]->ZoomLimits->Range->XRange->Min;
        zoom_max = GetConfigurationsResponse.PTZConfiguration[0]->ZoomLimits->Range->XRange->Max;

        DEBUG_PRINT("Pan: %f %f Tilt: %f %f", pan_tilt_limit_min.x, pan_tilt_limit_max.x,
                                        pan_tilt_limit_min.y, pan_tilt_limit_max.y);
    }

    // Get PTZ status
    {
        _tptz__GetStatus GetStatus;
        _tptz__GetStatusResponse GetStatusResponse;

        GetStatus.ProfileToken = profile_token;
        if (!set_credentials()) {
            ERROR_PRINT("Failed to setup credentials");
            goto err;
        }
        if (proxy_ptz->GetStatus(&GetStatus, GetStatusResponse)) {
            DEBUG_PRINT("Failed to recieve PTZ status");
            goto err;
        }

        if (GetStatusResponse.PTZStatus->Error) {
            DEBUG_PRINT("ErrorStatus: %s", GetStatusResponse.PTZStatus->Error);
        }
        if (GetStatusResponse.PTZStatus->MoveStatus->PanTilt) {
            DEBUG_PRINT("PTStatus:  %d", *GetStatusResponse.PTZStatus->MoveStatus->PanTilt);
        }
        if (GetStatusResponse.PTZStatus->MoveStatus->Zoom) {
            DEBUG_PRINT("ZoomStatus:  %d", *GetStatusResponse.PTZStatus->MoveStatus->Zoom);
        }
        DEBUG_PRINT("Pan: %f Tilt: %f", GetStatusResponse.PTZStatus->Position->PanTilt->x,
                                GetStatusResponse.PTZStatus->Position->PanTilt->y);
        DEBUG_PRINT("Zoom: %f", GetStatusResponse.PTZStatus->Position->Zoom->x);
    }

    soap_destroy(soap);
    soap_end(soap);
    return true;
err:
    soap_destroy(soap);
    soap_end(soap);
    return false;
}

// Generate Random Nonce value
bool AP_ONVIF::rand_nonce(char *nonce, size_t noncelen)
{
    if (noncelen <= 4) {
        // invalid size
        return false;
    }
    uint32_t r = (uint32_t)(hal.util->get_hw_rtc()/1000000ULL);
    (void)memcpy((void *)nonce, (const void *)&r, 4);
    return hal.util->get_random_vals((uint8_t*)&nonce[4], noncelen - 4);
}

#define TEST_NONCE "LKqI6G/AikKCQrN0zqZFlg=="
#define TEST_TIME "2010-09-16T07:50:45Z"
#define TEST_PASS "userpassword"
#define TEST_RESULT "tuOSpGlFlIXsozq4HFNeeGeFLEI="
#define TEST 0
bool AP_ONVIF::set_credentials()
{
    soap_wsse_delete_Security(soap);
    soap_wsse_add_Timestamp(soap, "Time", 60);

    _wsse__Security *security = soap_wsse_add_Security(soap);
    const char *created = soap_dateTime2s(soap, (time_t)(hal.util->get_hw_rtc()/1000000ULL));
    char HA[SHA1_DIGEST_SIZE] {};
    char HABase64fin[29] {};
    char nonce[16];
    char *nonceBase64enc = nullptr;
    char *nonceBase64fin;
    sha1_ctx ctx;
    uint16_t HABase64len;
    char *HABase64enc = nullptr;
    uint16_t noncelen; 

    /* generate a nonce */
    if (!rand_nonce(nonce, 16)) {
        return false;
    }

    sha1_begin(&ctx);
#if TEST
    char* test_nonce = (char*)base64_decode((const unsigned char*)TEST_NONCE, strlen(TEST_NONCE), &noncelen);
    sha1_hash((const unsigned char*)test_nonce, noncelen, &ctx);
    sha1_hash((const unsigned char*)TEST_TIME, strlen(TEST_TIME), &ctx);
    sha1_hash((const unsigned char*)TEST_PASS, strlen(TEST_PASS), &ctx);
    nonceBase64enc = (char*)base64_encode((unsigned char*)test_nonce, 16, &noncelen); // this call also mallocs
    DEBUG_PRINT("Created:%s Hash64:%s", TEST_TIME, HABase64fin);
#else
    sha1_hash((const unsigned char*)nonce, 16, &ctx);
    sha1_hash((const unsigned char*)created, strlen(created), &ctx);
    sha1_hash((const unsigned char*)password, strlen(password), &ctx);
    nonceBase64enc = (char*)base64_encode((unsigned char*)nonce, 16, &noncelen); // this call also mallocs
#endif
    if (nonceBase64enc == nullptr) {
        return false;
    }
    // move to something we can track
    nonceBase64fin = (char*)soap_malloc(soap, noncelen);
    memcpy(nonceBase64fin, nonceBase64enc, noncelen);
    free(nonceBase64enc);

    sha1_end((unsigned char*)HA, &ctx);
    HABase64enc = (char*)base64_encode((unsigned char*)HA, SHA1_DIGEST_SIZE, &HABase64len);
    if (HABase64enc == nullptr) {
        return false;
    }
    if (HABase64len > 29) {
        //things have gone truly bad time to panic
        ERROR_PRINT("Error: Invalid Base64 Encode!");
        free(HABase64enc);
        return false;
    }

    memcpy(HABase64fin, HABase64enc, HABase64len);
    free(HABase64enc);

    if (soap_wsse_add_UsernameTokenText(soap, "Auth", username, HABase64fin)) {
        report_error();
        return false;
    }
    /* populate the remainder of the password, nonce, and created */
    security->UsernameToken->Password->Type = (char*)wsse_PasswordDigestURI;
    security->UsernameToken->Nonce = (struct wsse__EncodedString*)soap_malloc(soap, sizeof(struct wsse__EncodedString));
    security->UsernameToken->Salt = NULL;
    security->UsernameToken->Iteration = NULL;
    if (!security->UsernameToken->Nonce) {
        ERROR_PRINT("Failed to allocate NONCE");
        return false;
    }
    soap_default_wsse__EncodedString(soap, security->UsernameToken->Nonce);
    security->UsernameToken->Nonce->__item = nonceBase64fin;
    security->UsernameToken->Nonce->EncodingType = (char*)wsse_Base64BinaryURI;
    security->UsernameToken->wsu__Created = soap_strdup(soap, created);
    return true;
}

// Turn ONVIF camera to mentioned pan, tilt and zoom, normalised 
// between limits
bool AP_ONVIF::set_absolutemove(float x, float y, float z)
{
    _tptz__AbsoluteMove AbsoluteMove;
    _tptz__AbsoluteMoveResponse AbsoluteMoveResponse;
    AbsoluteMove.Position = soap_new_tt__PTZVector(soap);
    if (AbsoluteMove.Position == nullptr) {
        ERROR_PRINT("Failed to allocate AbsoluteMove.Position");
        goto err;
    }
    AbsoluteMove.Position->PanTilt = soap_new_tt__Vector2D(soap);
    if (AbsoluteMove.Position->PanTilt == nullptr) {
        ERROR_PRINT("Failed to allocate AbsoluteMove.Position->PanTilt");
        goto err;
    }
    AbsoluteMove.Position->Zoom = soap_new_tt__Vector1D(soap);
    if (AbsoluteMove.Position->Zoom == nullptr) {
        ERROR_PRINT("Failed to allocate AbsoluteMove.Position->Zoom");
        goto err;
    }

    AbsoluteMove.Position->PanTilt->x = constrain_float(x, pan_tilt_limit_min.x, pan_tilt_limit_max.x);
    AbsoluteMove.Position->PanTilt->y = constrain_float(y, pan_tilt_limit_min.y, pan_tilt_limit_max.y);
    AbsoluteMove.Position->Zoom->x = constrain_float(z, zoom_min, zoom_max);
    AbsoluteMove.Speed = NULL;
    AbsoluteMove.ProfileToken = profile_token;
    // DEBUG_PRINT("Setting AbsoluteMove: %f %f %f", AbsoluteMove.Position->PanTilt->x,
                                            // AbsoluteMove.Position->PanTilt->y,
                                            // AbsoluteMove.Position->Zoom->x);
    if (!set_credentials()) {
        ERROR_PRINT("Failed to setup credentials");
        goto err;
    }
    if (proxy_ptz->AbsoluteMove(&AbsoluteMove, AbsoluteMoveResponse)) {
        ERROR_PRINT("Failed to sent AbsoluteMove cmd");
        report_error();
        goto err;
    }
    soap_destroy(soap);
    soap_end(soap);
    return true;
err:
    soap_destroy(soap);
    soap_end(soap);
    return false;
} 
#endif //#if ENABLE_ONVIF
