// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_IMU_Shim.h
/// @brief	IMU shim driver, used when the IMU data is coming from somewhere else.

#ifndef AP_IMU_Shim_h
#define AP_IMU_Shim_h

class AP_IMU_Shim : public IMU
{
public:
	AP_IMU_Shim(void) {}

	/// @name IMU protocol
	//@{
    virtual void	init(Start_style style = COLD_START, void (*callback)(unsigned long t) = delay) {};
	virtual void	init_accel(void (*callback)(unsigned long t) = delay) {};
	virtual void	init_gyro(void (*callback)(unsigned long t) = delay) {};
	virtual bool	update(void) {
		bool updated = _updated;
		_updated = false;
		return updated;
	}
	//@}
	
	float		gx()				{ return 0; }
	float		gy()				{ return 0; }
	float		gz()				{ return 0; }
	float		ax()				{ return 0; }
	float		ay()				{ return 0; }
	float		az()				{ return 0; }

	void		ax(const int v)		{  }
	void		ay(const int v)		{  }
	void		az(const int v)		{  }

	/// Set the gyro vector.  ::update will return
	/// true once after this call.
	///
	/// @param	v		The new gyro vector.
	///
	void		set_gyro(Vector3f v) { _gyro = v; _updated = true; }

	/// Set the accelerometer vector.  ::update will return
	/// true once after this call.
	///
	/// @param	v		The new accelerometer vector.
	///
	void		set_accel(Vector3f v) { _accel = v; _updated = true; }

	// dummy save method
	void		save(void) { }

private:
	/// set true when new data is delivered
	bool		_updated;
};

#endif
