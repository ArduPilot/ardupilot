// =============================================================================
/**
 * @file CX-GBXXXCtrl.cpp
 * @brief CX-GBXXX用コントロールクラス ヘッダ
 * @copyright (c) 2022 Xacti Corporation
 */
// =============================================================================

#include "CX-GBXXXCtrl.h"
#include <libusb-1.0/libusb.h>
#include <libuvc/libuvc.h>
#include <assert.h>

// -----------------------------------------------------------------------------
/*!
 * @brief CX_GBXXXCtrlコンストラクタ
 */
// -----------------------------------------------------------------------------
CX_GBXXXCtrl::CX_GBXXXCtrl()
    : m_ctx(NULL), m_dev(NULL), m_devh(NULL)
{
    uvc_error_t res = uvc_init(&m_ctx, NULL);
    if (res < 0)
    {
        uvc_perror(res, "uvc_init");
        assert(res == 0);
    }
}

// -----------------------------------------------------------------------------
/*!
 * @brief CX_GBXXXCtrlデストラクタ
 */
// -----------------------------------------------------------------------------
CX_GBXXXCtrl::~CX_GBXXXCtrl()
{
    uvc_exit(m_ctx);
}

// -----------------------------------------------------------------------------
/*!
 * @brief カメラオープン
 *
 * @param [in] callback  カメラシリアル番号(NULL: Don't Care)
 *
 * @return オープン成功可否
 */
// -----------------------------------------------------------------------------
bool CX_GBXXXCtrl::Open(const char *serial)
{
    uvc_error_t res;
    if ((res = uvc_find_device(
             m_ctx, &m_dev,
             0x296b, 0, serial)) < 0)
    {
        uvc_perror(res, "uvc_find_device"); // CX-GBXXX未接続
        return false;
    }

    if ((res = uvc_open(m_dev, &m_devh)) < 0)
    {
        uvc_perror(res, "uvc_open");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
/*!
 * @brief カメラクローズ
 */
// -----------------------------------------------------------------------------
void CX_GBXXXCtrl::Close()
{
    if (m_devh)
    {
        uvc_close(m_devh);
    }
    if (m_dev)
    {
        uvc_unref_device(m_dev);
    }
}

// -----------------------------------------------------------------------------
/*!
 * @brief カメラ情報設定
 *
 * @param [in] unit_id      ユニットID
 * @param [in] cotrol_id    コントロールID
 * @param [in] data         データバッファ
 * @param [in] length       データサイズ（バイト）
 *
 * @return 成功可否
 */
// -----------------------------------------------------------------------------
bool CX_GBXXXCtrl::SetCameraCtrl(uint8_t unit_id, uint8_t cotrol_id, void *data, int length)
{
    if (!m_devh)
    {
        uvc_perror(UVC_ERROR_INVALID_DEVICE, "SetCameraCtrl");
        return false;
    }

    if (uvc_set_ctrl(m_devh, unit_id, cotrol_id, data, length) != length)
    {
        uvc_perror(UVC_ERROR_OTHER, "SetCameraCtrl");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
/*!
 * @brief カメラ情報取得
 *
 * @param [in] unit_id      ユニットID
 * @param [in] cotrol_id    コントロールID
 * @param [out] data        データバッファ
 * @param [in] length       データサイズ（バイト）
 *
 * @return 成功可否
 */
// -----------------------------------------------------------------------------
bool CX_GBXXXCtrl::GetCameraCtrl(uint8_t unit_id, uint8_t cotrol_id, void *data, int length)
{
    if (!m_devh)
    {
        uvc_perror(UVC_ERROR_INVALID_DEVICE, "GetCameraCtrl");
        return false;
    }

    if (uvc_get_ctrl(m_devh, unit_id, cotrol_id, data, length, UVC_GET_CUR) != length)
    {
        uvc_perror(UVC_ERROR_OTHER, "GetCameraCtrl");
        return false;
    }

    return true;
}
