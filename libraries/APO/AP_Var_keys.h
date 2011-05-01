#ifndef AP_Var_keys_H
#define AP_Var_keys_H

enum keys {

	// general
	k_config = 0,
	k_cntrl,
	k_guide,
	k_sensorCalib,

	// radio channels
	k_chMode = 20,
	k_chLeft,
	k_chRight,
	k_chFront,
	k_chBack,
	k_chRoll,
	k_chPitch,
	k_chYaw,
	k_chThr,
	k_chStr,

	// pids
	k_pidPN = 40,
	k_pidPE,
	k_pidPD,
	k_pidThr,
	k_pidStr,
	k_pidRoll,
	k_pidPitch,
	k_pidYawRate,
	k_pidYaw,

	// 200-256 reserved for commands
	k_commands = 200
};

// max 256 keys

#endif
