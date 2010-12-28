// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_IMU_Shim.h
/// @brief	IMU shim driver, used when the IMU data is coming from somewhere else.

#ifndef AP_IMU_Shim_h#define AP_IMU_Shim_hclass AP_IMU_Shim : public IMU{public:	AP_IMU_Shim(void) {}	/// @name IMU protocol	//@{	virtual void	init(Start_style style) {}
	virtual void	init_accel(Start_style style) {};
	virtual void	init_gyro(Start_style style) {};
	virtual bool	update(void) {		bool updated = _updated;
		_updated = false;		return updated;	}	//@}	/// Set the gyro vector.  ::update will return	/// true once after this call.	///	/// @param	v		The new gyro vector.	///	void		set_gyro(Vector3f v) { _gyro = v; _updated = true; }	/// Set the accelerometer vector.  ::update will return	/// true once after this call.	///	/// @param	v		The new accelerometer vector.	///	void		set_accel(Vector3f v) { _accel = v; _updated = true; }private:	/// set true when new data is delivered	bool		_updated;};

#endif
