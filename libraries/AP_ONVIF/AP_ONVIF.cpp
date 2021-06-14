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
#include <stdlib.h> //rand()

#ifndef PRINT
#define PRINT(fmt,args...) do {printf(fmt "\n", ## args); } while(0)
#endif

const char *wsse_PasswordDigestURI = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordDigest";
const char *wsse_Base64BinaryURI = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-soap-message-security-1.0#Base64Binary";

AP_ONVIF *AP_ONVIF::_singleton;
extern const AP_HAL::HAL &hal;
static AP_ONVIF onvif;

// Default constructor
AP_ONVIF::AP_ONVIF()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_ONVIF must be singleton");
    }
    _singleton = this;
}

// Start ONVIF client with username, password and service host url
bool AP_ONVIF::start(const char *user, const char *pass, const char *httphostname)
{
    if (!initialised) {
        srand ((time_t)(hal.util->get_hw_rtc()/1000000ULL));
        soap = soap_new1(SOAP_XML_CANONICAL | SOAP_C_UTFSTRING);
        soap->connect_timeout = soap->recv_timeout = soap->send_timeout = 30; // 30 sec

        proxy_device = new DeviceBindingProxy(soap);
        proxy_media = new MediaBindingProxy(soap);
        proxy_ptz = new PTZBindingProxy(soap);

        if (proxy_device == nullptr ||
            proxy_media == nullptr ||
            proxy_ptz == nullptr) {
            AP_HAL::panic("AP_ONVIF: Failed to allocate gSOAP Proxy objects.");
        }
        initialised = true;
    }

    username = user;
    password = pass;
    hostname = httphostname;
    
    DEVICE_ENDPOINT = hostname + "/onvif/device_service";
    MEDIA_ENDPOINT = hostname + "/onvif/media_service";
    PTZ_ENDPOINT = hostname + "/onvif/ptz_service";
    /// TODO: Need to find a way to store this in parameter system
    //  or it could be just storage, we will see
    proxy_device->soap_endpoint = DEVICE_ENDPOINT.c_str();
    if (!probe_onvif_server()) {
        PRINT("Failed to probe onvif server.");
        return false;
    }

    return true;
}

void AP_ONVIF::report_error()
{
    PRINT("ONVIF ERROR:\n");
    if (soap_check_state(soap)) {
        PRINT("Error: soap struct state not initialized");
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
        PRINT("%s%d fault %s [%s]\n%s\nDetail: %s",(soap->version ? "SOAP 1." : "Error "),
                                                      (soap->version ? (int)soap->version : soap->error),
                                                      *c, (v ? v : "no subcode"), (s ? s : "[no reason]"),
                                                      (d ? d : "[no detail]"));
    }
}

// detect onvif server present on the network
bool AP_ONVIF::probe_onvif_server()
{
    _tds__GetDeviceInformation GetDeviceInformation;
    _tds__GetDeviceInformationResponse GetDeviceInformationResponse;
    set_credentials();
    if (proxy_device->GetDeviceInformation(&GetDeviceInformation, GetDeviceInformationResponse)) {
        PRINT("Failed to fetch Device Information");
        report_error();
        return false;
    }

    PRINT("Manufacturer:    %s",GetDeviceInformationResponse.Manufacturer.c_str());
    PRINT("Model:           %s",GetDeviceInformationResponse.Model.c_str());
    PRINT("FirmwareVersion: %s",GetDeviceInformationResponse.FirmwareVersion.c_str());
    PRINT("SerialNumber:    %s",GetDeviceInformationResponse.SerialNumber.c_str());
    PRINT("HardwareId:      %s",GetDeviceInformationResponse.HardwareId.c_str());

    // get device capabilities and print media
    _tds__GetCapabilities GetCapabilities;
    _tds__GetCapabilitiesResponse GetCapabilitiesResponse;
    set_credentials();
    if (proxy_device->GetCapabilities(&GetCapabilities, GetCapabilitiesResponse)) {
        PRINT("Failed to fetch Device Capabilities");
        report_error();
        return false;
    }

    if (!GetCapabilitiesResponse.Capabilities || !GetCapabilitiesResponse.Capabilities->Media) {
        PRINT("Missing device capabilities info");
    } else {
        PRINT("XAddr:        %s", GetCapabilitiesResponse.Capabilities->Media->XAddr.c_str());
        if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities) {
            if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTPMulticast) {
                PRINT("RTPMulticast: %s",(*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTPMulticast ? "yes" : "no"));
            }
            if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORETCP) {
                PRINT("RTP_TCP:      %s", (*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORETCP ? "yes" : "no"));
            }
            if (GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORERTSP_USCORETCP) {
                PRINT("RTP_RTSP_TCP: %s", (*GetCapabilitiesResponse.Capabilities->Media->StreamingCapabilities->RTP_USCORERTSP_USCORETCP ? "yes" : "no"));
            }
        }
    }

    // set the Media proxy endpoint to XAddr
    proxy_media->soap_endpoint = MEDIA_ENDPOINT.c_str();

    // get device profiles
    _trt__GetProfiles GetProfiles;
    _trt__GetProfilesResponse GetProfilesResponse;
    set_credentials();
    if (proxy_media->GetProfiles(&GetProfiles, GetProfilesResponse)){
        PRINT("Failed to fetch profiles");
        report_error();
        return false;
    }
    
    if (GetProfilesResponse.Profiles.size()) {
        PRINT("Profiles Received %lu", GetProfilesResponse.Profiles.size());
    } else {
        PRINT("Error: No Profiles Received");
        return false;
    }
    
    // for each profile get snapshot
    for (uint32_t i = 0; i < GetProfilesResponse.Profiles.size(); i++) {
        PRINT("Profile name: %s", GetProfilesResponse.Profiles[i]->Name.c_str());
    }
    
    // Just use first one for now
    profile_token = GetProfilesResponse.Profiles[0]->token;

    proxy_ptz->soap_endpoint = PTZ_ENDPOINT.c_str();

    // _tptz__GetServiceCapabilities GetCapabilities;
    // _tptz__GetServiceCapabilitiesResponse GetCapabilitiesResponse;
    // GetCapabilities.Category = tt__CapabilityCategory__PTZ;
    // set_credentials();
    // if (proxy_ptz->GetServiceCapabilities(&GetCapabilities, GetCapabilitiesResponse)) {
    //     PRINT("Failed to fetch PTZ Capabilities");
    //     report_error();
    //     return false;
    // }
    // if (!GetCapabilitiesResponse.Capabilities) {

    // }
    //     GetCapabilitiesResponse.Capabilities->PTZ

    // get PTZ Token
    _tptz__GetConfigurations GetConfigurations;
    _tptz__GetConfigurationsResponse GetConfigurationsResponse;
    set_credentials();
    if (proxy_ptz->GetConfigurations(&GetConfigurations, GetConfigurationsResponse)) {
        PRINT("Failed to fetch Configurations");
        report_error();
        return false;
    }
    
    if (GetConfigurationsResponse.PTZConfiguration.size()) {
        PRINT("PTZ Tokens Received");
    } else {
        PRINT("Error: No Profiles Received");
        return false;
    }
    
    for (uint32_t i = 0; i < GetConfigurationsResponse.PTZConfiguration.size(); i++) {
        PRINT("PTZ: %s", GetConfigurationsResponse.PTZConfiguration[i]->Name.c_str());
    }
    //GetConfigurationsResponse.PTZConfiguration[0]->token
    pan_tilt_limit_max = Vector2f(GetConfigurationsResponse.PTZConfiguration[0]->PanTiltLimits->Range->XRange->Max,
                        GetConfigurationsResponse.PTZConfiguration[0]->PanTiltLimits->Range->YRange->Max);
    pan_tilt_limit_min = Vector2f(GetConfigurationsResponse.PTZConfiguration[0]->PanTiltLimits->Range->XRange->Min,
                        GetConfigurationsResponse.PTZConfiguration[0]->PanTiltLimits->Range->YRange->Min);
    zoom_min = GetConfigurationsResponse.PTZConfiguration[0]->ZoomLimits->Range->XRange->Min;
    zoom_max = GetConfigurationsResponse.PTZConfiguration[0]->ZoomLimits->Range->XRange->Max;

    PRINT("Pan: %f %f Tilt: %f %f", pan_tilt_limit_min.x, pan_tilt_limit_max.x,
                                    pan_tilt_limit_min.y, pan_tilt_limit_max.y);
    // Get PTZ status
    _tptz__GetStatus GetStatus;
    _tptz__GetStatusResponse GetStatusResponse;
    GetStatus.ProfileToken = profile_token;
    set_credentials();
    if (proxy_ptz->GetStatus(&GetStatus, GetStatusResponse)) {
        PRINT("Failed to recieve PTZ status");
        return false;
    }

    if (GetStatusResponse.PTZStatus->Error) {
        PRINT("ErrorStatus: %s", GetStatusResponse.PTZStatus->Error->c_str());
    }
    if (GetStatusResponse.PTZStatus->MoveStatus->PanTilt) {
        PRINT("PTStatus:  %d", *GetStatusResponse.PTZStatus->MoveStatus->PanTilt);
    }
    if (GetStatusResponse.PTZStatus->MoveStatus->Zoom) {
        PRINT("ZoomStatus:  %d", *GetStatusResponse.PTZStatus->MoveStatus->Zoom);
    }
    PRINT("Pan: %f Tilt: %f", GetStatusResponse.PTZStatus->Position->PanTilt->x,
                              GetStatusResponse.PTZStatus->Position->PanTilt->y);
    PRINT("Zoom: %f", GetStatusResponse.PTZStatus->Position->Zoom->x);

    soap_destroy(soap);
    soap_end(soap);
    return true;
}

// Generate Random Nonce value
void AP_ONVIF::rand_nonce(char *nonce, size_t noncelen)
{
    size_t i;
    uint32_t r = (uint32_t)(hal.util->get_hw_rtc()/1000000ULL);
    (void)memcpy((void *)nonce, (const void *)&r, 4);
    for (i = 4; i < noncelen; i += 4)
    {
        r = rand();
        (void)memcpy((void *)(nonce + i), (const void *)&r, 4);
    }
}

#define TEST_NONCE "LKqI6G/AikKCQrN0zqZFlg=="
#define TEST_TIME "2010-09-16T07:50:45Z"
#define TEST_PASS "userpassword"
#define TEST_RESULT "tuOSpGlFlIXsozq4HFNeeGeFLEI="
#define TEST 0
void AP_ONVIF::set_credentials()
{
    soap_wsse_delete_Security(soap);
    soap_wsse_add_Timestamp(soap, "Time", 60);

    _wsse__Security *security = soap_wsse_add_Security(soap);
    const char *created = soap_dateTime2s(soap, (time_t)(hal.util->get_hw_rtc()/1000000ULL));
    char HA[SHA1_DIGEST_SIZE] {};
    char HABase64fin[29] {};
    char nonce[16], *nonceBase64;
    sha1_ctx ctx;
    uint16_t HABase64len;
    char *HABase64enc;
    uint16_t noncelen; 

    /* generate a nonce */
    rand_nonce(nonce, 16);

    sha1_begin(&ctx);
#if TEST
    char* test_nonce = (char*)base64_decode((const unsigned char*)TEST_NONCE, strlen(TEST_NONCE), &noncelen);
    sha1_hash((const unsigned char*)test_nonce, noncelen, &ctx);
    sha1_hash((const unsigned char*)TEST_TIME, strlen(TEST_TIME), &ctx);
    sha1_hash((const unsigned char*)TEST_PASS, strlen(TEST_PASS), &ctx);
    nonceBase64 = (char*)base64_encode((unsigned char*)test_nonce, 16, &noncelen);
    PRINT("Created:%s Hash64:%s", TEST_TIME, HABase64fin);
#else
    sha1_hash((const unsigned char*)nonce, 16, &ctx);
    sha1_hash((const unsigned char*)created, strlen(created), &ctx);
    sha1_hash((const unsigned char*)password.c_str(), strlen(password.c_str()), &ctx);
    nonceBase64 = (char*)base64_encode((unsigned char*)nonce, 16, &noncelen);
#endif
    sha1_end((unsigned char*)HA, &ctx);
    HABase64enc = (char*)base64_encode((unsigned char*)HA, SHA1_DIGEST_SIZE, &HABase64len);
    if (HABase64len > 29) {
        //things have gone truly bad time to panic
        PRINT("Error: Invalid Base64 Encode!");
        free(HABase64enc);
        return;
    }

    memcpy(HABase64fin, HABase64enc, HABase64len);

    if (soap_wsse_add_UsernameTokenText(soap, "Auth", username.c_str(), HABase64fin)) {
        report_error();
    }
    /* populate the remainder of the password, nonce, and created */
    security->UsernameToken->Password->Type = (char*)wsse_PasswordDigestURI;
    security->UsernameToken->Nonce = (struct wsse__EncodedString*)soap_malloc(soap, sizeof(struct wsse__EncodedString));
    security->UsernameToken->Salt = NULL;
    security->UsernameToken->Iteration = NULL;
    if (!security->UsernameToken->Nonce) {
        return;
    }
    soap_default_wsse__EncodedString(soap, security->UsernameToken->Nonce);
    security->UsernameToken->Nonce->__item = nonceBase64;
    security->UsernameToken->Nonce->EncodingType = (char*)wsse_Base64BinaryURI;
    security->UsernameToken->wsu__Created = soap_strdup(soap, created);
}

// Turn ONVIF camera to mentioned pan, tilt and zoom, normalised 
// between limits
bool AP_ONVIF::set_absolutemove(float x, float y, float z)
{
    _tptz__AbsoluteMove AbsoluteMove;
    _tptz__AbsoluteMoveResponse AbsoluteMoveResponse;
    AbsoluteMove.Position = soap_new_tt__PTZVector(soap);
    AbsoluteMove.Position->PanTilt = soap_new_tt__Vector2D(soap);
    AbsoluteMove.Position->Zoom = soap_new_tt__Vector1D(soap);
    AbsoluteMove.Position->PanTilt->x = constrain_float(x, pan_tilt_limit_min.x, pan_tilt_limit_max.x);
    AbsoluteMove.Position->PanTilt->y = constrain_float(y, pan_tilt_limit_min.y, pan_tilt_limit_max.y);
    AbsoluteMove.Position->Zoom->x = constrain_float(z, zoom_min, zoom_max);
    AbsoluteMove.Speed = NULL;
    AbsoluteMove.ProfileToken = profile_token;
    // PRINT("Setting AbsoluteMove: %f %f %f", AbsoluteMove.Position->PanTilt->x,
                                            // AbsoluteMove.Position->PanTilt->y,
                                            // AbsoluteMove.Position->Zoom->x);
    set_credentials();
    if (proxy_ptz->AbsoluteMove(&AbsoluteMove, AbsoluteMoveResponse)) {
        PRINT("Failed to sent AbsoluteMove cmd");
        report_error();
        soap_destroy(soap);
        soap_end(soap);
        return false;
    }
    soap_destroy(soap);
    soap_end(soap);
    return true;
} 
#endif //#if ENABLE_ONVIF
