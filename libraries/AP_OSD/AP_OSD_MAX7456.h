/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_OSD_MAX7456_H__
#define __AP_OSD_MAX7456_H__

#include <AP_HAL.h>
#include <AP_Param.h>
#include "AP_OSD_Stream.h"
#include "AP_OSD_Vars.h"


class AP_OSD_MAX7456 : public AP_OSD_Stream
{
public:
	AP_OSD_MAX7456();

	bool            init();

	//update charsets
	void			write_NVM(uint32_t font_count, uint8_t *character_bitmap);
	//void			read_one_char_from_NVM(uint32_t font_count);

	//update the screen
	void			updateScreen();

	//clear the display screen
	void			clear(void);

	void			setPanel(uint8_t start_col, uint8_t start_row);
	void			openPanel(void);
	void			closePanel(void);
	void			openSingle(uint8_t x, uint8_t y);

	

	//the implementations of AP_OSD_STREAM::write
	virtual size_t write(uint8_t c);

	AP_OSD_Vars *_osd_vars;

	static const struct AP_Param::GroupInfo var_info[];
protected:
	//parameters which can be set from the ground station
	AP_Int8		_bEnableSpeed;				//if show the speed
	AP_Int8		_bEnableAlt;				
	AP_Int8		_bEnableThrottle;
	AP_Int8		_bEnablePitch;
	AP_Int8		_bEnableRoll;
	AP_Int8		_bEnableHome;
	AP_Int8		_bEnableMode;
	AP_Int8		_bEnableTime;
	AP_Int8		_bEnableHorizon;
	AP_Int8		_bEnableGPSSats;
	AP_Int8		_bEnableGPSCoord;
	AP_Int8		_bEnableBattVol;
	AP_Int8		_bEnableBattCur;
	AP_Int8		_bEnableBattPercent;
	AP_Int8		_bEnableWP;
	AP_Int8		_bEnableHead;
	AP_Int8		_bEnableHeadRose;
	AP_Int8		_iMode;
	AP_Int8		_iEnableRSSI;
	AP_Int8		_iEnableCurConsume;

private:
	void showArrow(uint8_t rotate_arrow, uint8_t mode);	
	void showHorizon(uint8_t start_col, uint8_t start_row);
	void printHit(uint8_t col, uint8_t row, uint8_t subval);
	void showAt10HZ();
	void showAt3HZ();
	void showAt1HZ();

	AP_HAL::SPIDeviceDriver *_spi;
	AP_HAL::Semaphore *_spi_sem;
	
	uint8_t start_col, start_row, col, row, _video_mode;
	uint32_t		_startTime;
	uint32_t		_lastUpdate10HZ;
	uint32_t		_lastUpdate3HZ;
	uint32_t		_lastUpdate1HZ;
	uint8_t			_lastHorizonColHit[12];
	uint8_t			_lastHorizonRowHit[12];
	uint8_t			_HorizonHitIndex;
};

#endif //  __AP_OSD_MAX7456_H__
