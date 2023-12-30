// =============================================================================
/**
 * @file CX-GBXXXCtrl.h
 * @brief CX-GBXXX用コントロールクラス ヘッダ
 * @copyright (c) 2022 Xacti Corporation
 */
// =============================================================================

#pragma once

#include <cstdint>

struct uvc_context;
typedef struct uvc_context uvc_context_t;
struct uvc_device;
typedef struct uvc_device uvc_device_t;
struct uvc_device_handle;
typedef struct uvc_device_handle uvc_device_handle_t;
struct uvc_frame;
typedef struct uvc_frame uvc_frame_t;

/*! @brief CCX_GBXXX用コントロールクラス */
class CX_GBXXXCtrl
{
public:
    CX_GBXXXCtrl();
    virtual ~CX_GBXXXCtrl();
    bool Open(const char *serial);
    void Close();

    //! カメラコマンド
    bool SetCameraCtrl(uint8_t unit_id, uint8_t cotrol_id, void *data, int length);
    bool GetCameraCtrl(uint8_t unit_id, uint8_t cotrol_id, void *data, int length);

private:

    uvc_context_t *m_ctx;
    uvc_device_t *m_dev;
    uvc_device_handle_t *m_devh;
};
