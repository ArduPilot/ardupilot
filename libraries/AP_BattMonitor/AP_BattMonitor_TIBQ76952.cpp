#include "AP_BattMonitor_config.h"

#if AP_BATTERY_TIBQ76952_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include "AP_BattMonitor_TIBQ76952.h"
#include "stdio.h"

extern const AP_HAL::HAL& hal;

#ifndef AP_BATTMON_TIBQ76952_BUS
#define AP_BATTMON_TIBQ76952_BUS 0
#endif
#ifndef AP_BATTMON_TIBQ76952_ADDR
#define AP_BATTMON_TIBQ76952_ADDR 0x08
#endif

// Data Memory registers Name in TRM
#define TIBQ769x2_Cell1Gain 0x9180          // Calibration:Voltage:Cell 1 Gain
#define TIBQ769x2_Cell2Gain 0x9182          // Calibration:Voltage:Cell 2 Gain
#define TIBQ769x2_Cell3Gain 0x9184          // Calibration:Voltage:Cell 3 Gain
#define TIBQ769x2_Cell4Gain 0x9186          // Calibration:Voltage:Cell 4 Gain
#define TIBQ769x2_Cell5Gain 0x9188          // Calibration:Voltage:Cell 5 Gain
#define TIBQ769x2_Cell6Gain 0x918A          // Calibration:Voltage:Cell 6 Gain
#define TIBQ769x2_Cell7Gain 0x918C          // Calibration:Voltage:Cell 7 Gain
#define TIBQ769x2_Cell8Gain 0x918E          // Calibration:Voltage:Cell 8 Gain
#define TIBQ769x2_Cell9Gain 0x9190          // Calibration:Voltage:Cell 9 Gain
#define TIBQ769x2_Cell10Gain 0x9192         // Calibration:Voltage:Cell 10 Gain
#define TIBQ769x2_Cell11Gain 0x9194         // Calibration:Voltage:Cell 11 Gain
#define TIBQ769x2_Cell12Gain 0x9196         // Calibration:Voltage:Cell 12 Gain
#define TIBQ769x2_Cell13Gain 0x9198         // Calibration:Voltage:Cell 13 Gain
#define TIBQ769x2_Cell14Gain 0x919A         // Calibration:Voltage:Cell 14 Gain
#define TIBQ769x2_Cell15Gain 0x919C         // Calibration:Voltage:Cell 15 Gain
#define TIBQ769x2_Cell16Gain 0x919E         // Calibration:Voltage:Cell 16 Gain
#define TIBQ769x2_PackGain 0x91A0           // Calibration:Voltage:Pack Gain
#define TIBQ769x2_TOSGain 0x91A2            // Calibration:Voltage:TOS Gain
#define TIBQ769x2_LDGain 0x91A4             // Calibration:Voltage:LD Gain
#define TIBQ769x2_ADCGain 0x91A6            // Calibration:Voltage:ADC Gain
#define TIBQ769x2_CCGain 0x91A8             // Calibration:Current:CC Gain
#define TIBQ769x2_CapacityGain 0x91AC       // Calibration:Current:Capacity Gain
#define TIBQ769x2_VcellOffset 0x91B0        // Calibration:Vcell Offset:Vcell Offset
#define TIBQ769x2_VdivOffset 0x91B2         // Calibration:V Divider Offset:Vdiv Offset
#define TIBQ769x2_CoulombCounterOffsetSamples 0x91C6 // Calibration:Current Offset:Coulomb Counter Offset Samples
#define TIBQ769x2_BoardOffset 0x91C8        // Calibration:Current Offset:Board Offset
#define TIBQ769x2_InternalTempOffset 0x91CA // Calibration:Temperature:Internal Temp Offset
#define TIBQ769x2_CFETOFFTempOffset 0x91CB  // Calibration:Temperature:CFETOFF Temp Offset
#define TIBQ769x2_DFETOFFTempOffset 0x91CC  // Calibration:Temperature:DFETOFF Temp Offset
#define TIBQ769x2_ALERTTempOffset 0x91CD    // Calibration:Temperature:ALERT Temp Offset
#define TIBQ769x2_TS1TempOffset 0x91CE      // Calibration:Temperature:TS1 Temp Offset
#define TIBQ769x2_TS2TempOffset 0x91CF      // Calibration:Temperature:TS2 Temp Offset
#define TIBQ769x2_TS3TempOffset 0x91D0      // Calibration:Temperature:TS3 Temp Offset
#define TIBQ769x2_HDQTempOffset 0x91D1      // Calibration:Temperature:HDQ Temp Offset
#define TIBQ769x2_DCHGTempOffset 0x91D2     // Calibration:Temperature:DCHG Temp Offset
#define TIBQ769x2_DDSGTempOffset 0x91D3     // Calibration:Temperature:DDSG Temp Offset
#define TIBQ769x2_IntGain 0x91E2            // Calibration:Internal Temp Model:Int Gain
#define TIBQ769x2_Intbaseoffset 0x91E4      // Calibration:Internal Temp Model:Int base offset
#define TIBQ769x2_IntMaximumAD 0x91E6       // Calibration:Internal Temp Model:Int Maximum AD
#define TIBQ769x2_IntMaximumTemp 0x91E8     // Calibration:Internal Temp Model:Int Maximum Temp
#define TIBQ769x2_T18kCoeffa1 0x91EA        // Calibration:18K Temperature Model:Coeff a1
#define TIBQ769x2_T18kCoeffa2 0x91EC        // Calibration:18K Temperature Model:Coeff a2
#define TIBQ769x2_T18kCoeffa3 0x91EE        // Calibration:18K Temperature Model:Coeff a3
#define TIBQ769x2_T18kCoeffa4 0x91F0        // Calibration:18K Temperature Model:Coeff a4
#define TIBQ769x2_T18kCoeffa5 0x91F2        // Calibration:18K Temperature Model:Coeff a5
#define TIBQ769x2_T18kCoeffb1 0x91F4        // Calibration:18K Temperature Model:Coeff b1
#define TIBQ769x2_T18kCoeffb2 0x91F6        // Calibration:18K Temperature Model:Coeff b2
#define TIBQ769x2_T18kCoeffb3 0x91F8        // Calibration:18K Temperature Model:Coeff b3
#define TIBQ769x2_T18kCoeffb4 0x91FA        // Calibration:18K Temperature Model:Coeff b4
#define TIBQ769x2_T18kAdc0 0x91FE           // Calibration:18K Temperature Model:Adc0
#define TIBQ769x2_T180kCoeffa1 0x9200       // Calibration:180K Temperature Model:Coeff a1
#define TIBQ769x2_T180kCoeffa2 0x9202       // Calibration:180K Temperature Model:Coeff a2
#define TIBQ769x2_T180kCoeffa3 0x9204       // Calibration:180K Temperature Model:Coeff a3
#define TIBQ769x2_T180kCoeffa4 0x9206       // Calibration:180K Temperature Model:Coeff a4
#define TIBQ769x2_T180kCoeffa5 0x9208       // Calibration:180K Temperature Model:Coeff a5
#define TIBQ769x2_T180kCoeffb1 0x920A       // Calibration:180K Temperature Model:Coeff b1
#define TIBQ769x2_T180kCoeffb2 0x920C       // Calibration:180K Temperature Model:Coeff b2
#define TIBQ769x2_T180kCoeffb3 0x920E       // Calibration:180K Temperature Model:Coeff b3
#define TIBQ769x2_T180kCoeffb4 0x9210       // Calibration:180K Temperature Model:Coeff b4
#define TIBQ769x2_T180kAdc0 0x9214          // Calibration:180K Temperature Model:Adc0
#define TIBQ769x2_CustomCoeffa1 0x9216      // Calibration:Custom Temperature Model:Coeff a1
#define TIBQ769x2_CustomCoeffa2 0x9218      // Calibration:Custom Temperature Model:Coeff a2
#define TIBQ769x2_CustomCoeffa3 0x921A      // Calibration:Custom Temperature Model:Coeff a3
#define TIBQ769x2_CustomCoeffa4 0x921C      // Calibration:Custom Temperature Model:Coeff a4
#define TIBQ769x2_CustomCoeffa5 0x921E      // Calibration:Custom Temperature Model:Coeff a5
#define TIBQ769x2_CustomCoeffb1 0x9220      // Calibration:Custom Temperature Model:Coeff b1
#define TIBQ769x2_CustomCoeffb2 0x9222      // Calibration:Custom Temperature Model:Coeff b2
#define TIBQ769x2_CustomCoeffb3 0x9224      // Calibration:Custom Temperature Model:Coeff b3
#define TIBQ769x2_CustomCoeffb4 0x9226      // Calibration:Custom Temperature Model:Coeff b4
#define TIBQ769x2_CustomRc0 0x9228          // Calibration:Custom Temperature Model:Rc0
#define TIBQ769x2_CustomAdc0 0x922A         // Calibration:Custom Temperature Model:Adc0
#define TIBQ769x2_CoulombCounterDeadband 0x922D // Calibration:Current Deadband:Coulomb Counter Deadband
#define TIBQ769x2_CUVThresholdOverride 0x91D4   // Calibration:CUV:CUV Threshold Override
#define TIBQ769x2_COVThresholdOverride 0x91D6   // Calibration:COV:COV Threshold Override
#define TIBQ769x2_MinBlowFuseVoltage 0x9231 // Settings:Fuse:Min Blow Fuse Voltage
#define TIBQ769x2_FuseBlowTimeout 0x9233    // Settings:Fuse:Fuse Blow Timeout
#define TIBQ769x2_PowerConfig 0x9234        // Settings:Configuration:Power Config
#define TIBQ769x2_REG12Config 0x9236        // Settings:Configuration:REG12 Config
#define TIBQ769x2_REG0Config 0x9237         // Settings:Configuration:REG0 Config
#define TIBQ769x2_HWDRegulatorOptions 0x9238    // Settings:Configuration:HWD Regulator Options
#define TIBQ769x2_CommType 0x9239           // Settings:Configuration:Comm Type
#define TIBQ769x2_I2CAddress 0x923A         // Settings:Configuration:I2C Address
#define TIBQ769x2_SPIConfiguration 0x923C   // Settings:Configuration:SPI Configuration
#define TIBQ769x2_CommIdleTime 0x923D       // Settings:Configuration:Comm Idle Time
#define TIBQ769x2_CFETOFFPinConfig 0x92FA   // Settings:Configuration:CFETOFF Pin Config
#define TIBQ769x2_DFETOFFPinConfig 0x92FB   // Settings:Configuration:DFETOFF Pin Config
#define TIBQ769x2_ALERTPinConfig 0x92FC     // Settings:Configuration:ALERT Pin Config
#define TIBQ769x2_TS1Config 0x92FD          // Settings:Configuration:TS1 Config
#define TIBQ769x2_TS2Config 0x92FE          // Settings:Configuration:TS2 Config
#define TIBQ769x2_TS3Config 0x92FF          // Settings:Configuration:TS3 Config
#define TIBQ769x2_HDQPinConfig 0x9300       // Settings:Configuration:HDQ Pin Config
#define TIBQ769x2_DCHGPinConfig 0x9301      // Settings:Configuration:DCHG Pin Config
#define TIBQ769x2_DDSGPinConfig 0x9302      // Settings:Configuration:DDSG Pin Config
#define TIBQ769x2_DAConfiguration 0x9303    // Settings:Configuration:DA Configuration
#define TIBQ769x2_VCellMode 0x9304          // Settings:Configuration:Vcell Mode
#define TIBQ769x2_CC3Samples 0x9307         // Settings:Configuration:CC3 Samples
#define TIBQ769x2_ProtectionConfiguration 0x925F    // Settings:Protection:Protection Configuration
#define TIBQ769x2_EnabledProtectionsA 0x9261    // Settings:Protection:Enabled Protections A
#define TIBQ769x2_EnabledProtectionsB 0x9262    // Settings:Protection:Enabled Protections B
#define TIBQ769x2_EnabledProtectionsC 0x9263    // Settings:Protection:Enabled Protections C
#define TIBQ769x2_CHGFETProtectionsA 0x9265     // Settings:Protection:CHG FET Protections A
#define TIBQ769x2_CHGFETProtectionsB 0x9266     // Settings:Protection:CHG FET Protections B
#define TIBQ769x2_CHGFETProtectionsC 0x9267     // Settings:Protection:CHG FET Protections C
#define TIBQ769x2_DSGFETProtectionsA 0x9269     // Settings:Protection:DSG FET Protections A
#define TIBQ769x2_DSGFETProtectionsB 0x926A     // Settings:Protection:DSG FET Protections B
#define TIBQ769x2_DSGFETProtectionsC 0x926B     // Settings:Protection:DSG FET Protections C
#define TIBQ769x2_BodyDiodeThreshold 0x9273     // Settings:Protection:Body Diode Threshold
#define TIBQ769x2_DefaultAlarmMask 0x926D       // Settings:Alarm:Default Alarm Mask
#define TIBQ769x2_SFAlertMaskA 0x926F           // Settings:Alarm:SF Alert Mask A
#define TIBQ769x2_SFAlertMaskB 0x9270           // Settings:Alarm:SF Alert Mask B
#define TIBQ769x2_SFAlertMaskC 0x9271           // Settings:Alarm:SF Alert Mask C
#define TIBQ769x2_PFAlertMaskA 0x92C4           // Settings:Alarm:PF Alert Mask A
#define TIBQ769x2_PFAlertMaskB 0x92C5           // Settings:Alarm:PF Alert Mask B
#define TIBQ769x2_PFAlertMaskC 0x92C6           // Settings:Alarm:PF Alert Mask C
#define TIBQ769x2_PFAlertMaskD 0x92C7           // Settings:Alarm:PF Alert Mask D
#define TIBQ769x2_EnabledPFA 0x92C0             // Settings:Permanent Failure:Enabled PF A
#define TIBQ769x2_EnabledPFB 0x92C1             // Settings:Permanent Failure:Enabled PF B
#define TIBQ769x2_EnabledPFC 0x92C2             // Settings:Permanent Failure:Enabled PF C
#define TIBQ769x2_EnabledPFD 0x92C3             // Settings:Permanent Failure:Enabled PF D
#define TIBQ769x2_FETOptions 0x9308             // Settings:FET:FET Options
#define TIBQ769x2_ChgPumpControl 0x9309         // Settings:FET:Chg Pump Control
#define TIBQ769x2_PrechargeStartVoltage 0x930A  // Settings:FET:Precharge Start Voltage
#define TIBQ769x2_PrechargeStopVoltage 0x930C   // Settings:FET:Precharge Stop Voltage
#define TIBQ769x2_PredischargeTimeout 0x930E    // Settings:FET:Predischarge Timeout
#define TIBQ769x2_PredischargeStopDelta 0x930F  // Settings:FET:Predischarge Stop Delta
#define TIBQ769x2_DsgCurrentThreshold 0x9310    // Settings:Current Thresholds:Dsg Current Threshold
#define TIBQ769x2_ChgCurrentThreshold 0x9312    // Settings:Current Thresholds:Chg Current Threshold
#define TIBQ769x2_CheckTime 0x9314              // Settings:Cell Open-Wire:Check Time
#define TIBQ769x2_Cell1Interconnect 0x9315      // Settings:Interconnect Resistances:Cell 1 Interconnect
#define TIBQ769x2_Cell2Interconnect 0x9317      // Settings:Interconnect Resistances:Cell 2 Interconnect
#define TIBQ769x2_Cell3Interconnect 0x9319      // Settings:Interconnect Resistances:Cell 3 Interconnect
#define TIBQ769x2_Cell4Interconnect 0x931B      // Settings:Interconnect Resistances:Cell 4 Interconnect
#define TIBQ769x2_Cell5Interconnect 0x931D      // Settings:Interconnect Resistances:Cell 5 Interconnect
#define TIBQ769x2_Cell6Interconnect 0x931F      // Settings:Interconnect Resistances:Cell 6 Interconnect
#define TIBQ769x2_Cell7Interconnect 0x9321      // Settings:Interconnect Resistances:Cell 7 Interconnect
#define TIBQ769x2_Cell8Interconnect 0x9323      // Settings:Interconnect Resistances:Cell 8 Interconnect
#define TIBQ769x2_Cell9Interconnect 0x9325      // Settings:Interconnect Resistances:Cell 9 Interconnect
#define TIBQ769x2_Cell10Interconnect 0x9327     // Settings:Interconnect Resistances:Cell 10 Interconnect
#define TIBQ769x2_Cell11Interconnect 0x9329     // Settings:Interconnect Resistances:Cell 11 Interconnect
#define TIBQ769x2_Cell12Interconnect 0x932B     // Settings:Interconnect Resistances:Cell 12 Interconnect
#define TIBQ769x2_Cell13Interconnect 0x932D     // Settings:Interconnect Resistances:Cell 13 Interconnect
#define TIBQ769x2_Cell14Interconnect 0x932F     // Settings:Interconnect Resistances:Cell 14 Interconnect
#define TIBQ769x2_Cell15Interconnect 0x9331     // Settings:Interconnect Resistances:Cell 15 Interconnect
#define TIBQ769x2_Cell16Interconnect 0x9333     // Settings:Interconnect Resistances:Cell 16 Interconnect
#define TIBQ769x2_MfgStatusInit 0x9343          // Settings:Manufacturing:Mfg Status Init
#define TIBQ769x2_BalancingConfiguration 0x9335 // Settings:Cell Balancing Config:Balancing Configuration
#define TIBQ769x2_MinCellTemp 0x9336            // Settings:Cell Balancing Config:Min Cell Temp
#define TIBQ769x2_MaxCellTemp 0x9337            // Settings:Cell Balancing Config:Max Cell Temp
#define TIBQ769x2_MaxInternalTemp 0x9338        // Settings:Cell Balancing Config:Max Internal Temp
#define TIBQ769x2_CellBalanceInterval 0x9339    // Settings:Cell Balancing Config:Cell Balance Interval
#define TIBQ769x2_CellBalanceMaxCells 0x933A    // Settings:Cell Balancing Config:Cell Balance Max Cells
#define TIBQ769x2_CellBalanceMinCellVCharge 0x933B  // Settings:Cell Balancing Config:Cell Balance Min Cell V (Charge)
#define TIBQ769x2_CellBalanceMinDeltaCharge 0x933D  // Settings:Cell Balancing Config:Cell Balance Min Delta (Charge)
#define TIBQ769x2_CellBalanceStopDeltaCharge 0x933E // Settings:Cell Balancing Config:Cell Balance Stop Delta (Charge)
#define TIBQ769x2_CellBalanceMinCellVRelax 0x933F   // Settings:Cell Balancing Config:Cell Balance Min Cell V (Relax)
#define TIBQ769x2_CellBalanceMinDeltaRelax 0x9341   // Settings:Cell Balancing Config:Cell Balance Min Delta (Relax)
#define TIBQ769x2_CellBalanceStopDeltaRelax 0x9342  // Settings:Cell Balancing Config:Cell Balance Stop Delta (Relax)
#define TIBQ769x2_ShutdownCellVoltage 0x923F        // Power:Shutdown:Shutdown Cell Voltage
#define TIBQ769x2_ShutdownStackVoltage 0x9241       // Power:Shutdown:Shutdown Stack Voltage
#define TIBQ769x2_LowVShutdownDelay 0x9243          // Power:Shutdown:Low V Shutdown Delay
#define TIBQ769x2_ShutdownTemperature 0x9244        // Power:Shutdown:Shutdown Temperature
#define TIBQ769x2_ShutdownTemperatureDelay 0x9245   // Power:Shutdown:Shutdown Temperature Delay
#define TIBQ769x2_FETOffDelay 0x9252                // Power:Shutdown:FET Off Delay
#define TIBQ769x2_ShutdownCommandDelay 0x9253       // Power:Shutdown:Shutdown Command Delay
#define TIBQ769x2_AutoShutdownTime 0x9254           // Power:Shutdown:Auto Shutdown Time
#define TIBQ769x2_RAMFailShutdownTime 0x9255        // Power:Shutdown:RAM Fail Shutdown Time
#define TIBQ769x2_SleepCurrent 0x9248               // Power:Sleep:Sleep Current
#define TIBQ769x2_VoltageTime 0x924A                // Power:Sleep:Voltage Time
#define TIBQ769x2_WakeComparatorCurrent 0x924B      // Power:Sleep:Wake Comparator Current
#define TIBQ769x2_SleepHysteresisTime 0x924D        // Power:Sleep:Sleep Hysteresis Time
#define TIBQ769x2_SleepChargerVoltageThreshold 0x924E   // Power:Sleep:Sleep Charger Voltage Threshold
#define TIBQ769x2_SleepChargerPACKTOSDelta 0x9250   // Power:Sleep:Sleep Charger PACK-TOS Delta
#define TIBQ769x2_ConfigRAMSignature 0x91E0         // System Data:Integrity:Config RAM Signature
#define TIBQ769x2_CUVThreshold 0x9275               // Protections:CUV:Threshold
#define TIBQ769x2_CUVDelay 0x9276                   // Protections:CUV:Delay
#define TIBQ769x2_CUVRecoveryHysteresis 0x927B      // Protections:CUV:Recovery Hysteresis
#define TIBQ769x2_COVThreshold 0x9278               // Protections:COV:Threshold
#define TIBQ769x2_COVDelay 0x9279                   // Protections:COV:Delay
#define TIBQ769x2_COVRecoveryHysteresis 0x927C      // Protections:COV:Recovery Hysteresis
#define TIBQ769x2_COVLLatchLimit 0x927D             // Protections:COVL:Latch Limit
#define TIBQ769x2_COVLCounterDecDelay 0x927E        // Protections:COVL:Counter Dec Delay
#define TIBQ769x2_COVLRecoveryTime 0x927F           // Protections:COVL:Recovery Time
#define TIBQ769x2_OCCThreshold 0x9280               // Protections:OCC:Threshold
#define TIBQ769x2_OCCDelay 0x9281                   // Protections:OCC:Delay
#define TIBQ769x2_OCCRecoveryThreshold 0x9288       // Protections:OCC:Recovery Threshold
#define TIBQ769x2_OCCPACKTOSDelta 0x92B0            // Protections:OCC:PACK-TOS Delta
#define TIBQ769x2_OCD1Threshold 0x9282              // Protections:OCD1:Threshold
#define TIBQ769x2_OCD1Delay 0x9283                  // Protections:OCD1:Delay
#define TIBQ769x2_OCD2Threshold 0x9284              // Protections:OCD2:Threshold
#define TIBQ769x2_OCD2Delay 0x9285                  // Protections:OCD2:Delay
#define TIBQ769x2_SCDThreshold 0x9286               // Protections:SCD:Threshold
#define TIBQ769x2_SCDDelay 0x9287                   // Protections:SCD:Delay
#define TIBQ769x2_SCDRecoveryTime 0x9294            // Protections:SCD:Recovery Time
#define TIBQ769x2_OCD3Threshold 0x928A              // Protections:OCD3:Threshold
#define TIBQ769x2_OCD3Delay 0x928C                  // Protections:OCD3:Delay
#define TIBQ769x2_OCDRecoveryThreshold 0x928D       // Protections:OCD:Recovery Threshold
#define TIBQ769x2_OCDLLatchLimit 0x928F             // Protections:OCDL:Latch Limit
#define TIBQ769x2_OCDLCounterDecDelay 0x9290        // Protections:OCDL:Counter Dec Delay
#define TIBQ769x2_OCDLRecoveryTime 0x9291           // Protections:OCDL:Recovery Time
#define TIBQ769x2_OCDLRecoveryThreshold 0x9292      // Protections:OCDL:Recovery Threshold
#define TIBQ769x2_SCDLLatchLimit 0x9295             // Protections:SCDL:Latch Limit
#define TIBQ769x2_SCDLCounterDecDelay 0x9296        // Protections:SCDL:Counter Dec Delay
#define TIBQ769x2_SCDLRecoveryTime 0x9297           // Protections:SCDL:Recovery Time
#define TIBQ769x2_SCDLRecoveryThreshold 0x9298      // Protections:SCDL:Recovery Threshold
#define TIBQ769x2_OTCThreshold 0x929A               // Protections:OTC:Threshold
#define TIBQ769x2_OTCDelay 0x920B                   // Protections:OTC:Delay
#define TIBQ769x2_OTCRecovery 0x929C                // Protections:OTC:Recovery
#define TIBQ769x2_OTDThreshold 0x929D               // Protections:OTD:Threshold
#define TIBQ769x2_OTDDelay 0x929E                   // Protections:OTD:Delay
#define TIBQ769x2_OTDRecovery 0x929F                // Protections:OTD:Recovery
#define TIBQ769x2_OTFThreshold 0x92A0               // Protections:OTF:Threshold
#define TIBQ769x2_OTFDelay 0x92A1                   // Protections:OTF:Delay
#define TIBQ769x2_OTFRecovery 0x92A2                // Protections:OTF:Recovery
#define TIBQ769x2_OTINTThreshold 0x92A3             // Protections:OTINT:Threshold
#define TIBQ769x2_OTINTDelay 0x92A4                 // Protections:OTINT:Delay
#define TIBQ769x2_OTINTRecovery 0x92A5              // Protections:OTINT:Recovery
#define TIBQ769x2_UTCThreshold 0x92A6               // Protections:UTC:Threshold
#define TIBQ769x2_UTCDelay 0x92A7                   // Protections:UTC:Delay
#define TIBQ769x2_UTCRecovery 0x92A8                // Protections:UTC:Recovery
#define TIBQ769x2_UTDThreshold 0x92A9               // Protections:UTD:Threshold
#define TIBQ769x2_UTDDelay 0x92AA                   // Protections:UTD:Delay
#define TIBQ769x2_UTDRecovery 0x92AB                // Protections:UTD:Recovery
#define TIBQ769x2_UTINTThreshold 0x92AC             // Protections:UTINT:Threshold
#define TIBQ769x2_UTINTDelay 0x92AD                 // Protections:UTINT:Delay
#define TIBQ769x2_UTINTRecovery 0x92AE              // Protections:UTINT:Recovery
#define TIBQ769x2_ProtectionsRecoveryTime 0x92AF    // Protections:Recovery:Time
#define TIBQ769x2_HWDDelay 0x92B2                   // Protections:HWD:Delay
#define TIBQ769x2_LoadDetectActiveTime 0x92B4       // Protections:Load Detect:Active Time
#define TIBQ769x2_LoadDetectRetryDelay 0x92B5       // Protections:Load Detect:Retry Delay
#define TIBQ769x2_LoadDetectTimeout 0x92B6          // Protections:Load Detect:Timeout
#define TIBQ769x2_PTOChargeThreshold 0x92BA         // Protections:PTO:Charge Threshold
#define TIBQ769x2_PTODelay 0x92BC       // Protections:PTO:Delay
#define TIBQ769x2_PTOReset 0x92BE       // Protections:PTO:Reset
#define TIBQ769x2_CUDEPThreshold 0x92C8 // Permanent Fail:CUDEP:Threshold
#define TIBQ769x2_CUDEPDelay 0x92CA     // Permanent Fail:CUDEP:Delay
#define TIBQ769x2_SUVThreshold 0x92CB   // Permanent Fail:SUV:Threshold
#define TIBQ769x2_SUVDelay 0x92CD       // Permanent Fail:SUV:Delay
#define TIBQ769x2_SOVThreshold 0x92CE   // Permanent Fail:SOV:Threshold
#define TIBQ769x2_SOVDelay 0x92D0       // Permanent Fail:SOV:Delay
#define TIBQ769x2_TOSSThreshold 0x92D1  // Permanent Fail:TOS:Threshold
#define TIBQ769x2_TOSSDelay 0x92D3      // Permanent Fail:TOS:Delay
#define TIBQ769x2_SOCCThreshold 0x92D4  // Permanent Fail:SOCC:Threshold
#define TIBQ769x2_SOCCDelay 0x92D6      // Permanent Fail:SOCC:Delay
#define TIBQ769x2_SOCDThreshold 0x92D7  // Permanent Fail:SOCD:Threshold
#define TIBQ769x2_SOCDDelay 0x92D9      // Permanent Fail:SOCD:Delay
#define TIBQ769x2_SOTThreshold 0x92DA   // Permanent Fail:SOT:Threshold
#define TIBQ769x2_SOTDelay 0x92DB       // Permanent Fail:SOT:Delay
#define TIBQ769x2_SOTFThreshold 0x92DC  // Permanent Fail:SOTF:Threshold
#define TIBQ769x2_SOTFDelay 0x92DD      // Permanent Fail:SOTF:Delay
#define TIBQ769x2_VIMRCheckVoltage 0x92DE       // Permanent Fail:VIMR:Check Voltage
#define TIBQ769x2_VIMRMaxRelaxCurrent 0x92E0    // Permanent Fail:VIMR:Max Relax Current
#define TIBQ769x2_VIMRThreshold 0x92E2          // Permanent Fail:VIMR:Threshold
#define TIBQ769x2_VIMRDelay 0x92E4              // Permanent Fail:VIMR:Delay
#define TIBQ769x2_VIMRRelaxMinDuration 0x92E5   // Permanent Fail:VIMR:Relax Min Duration
#define TIBQ769x2_VIMACheckVoltage 0x92E7       // Permanent Fail:VIMA:Check Voltage
#define TIBQ769x2_VIMAMinActiveCurrent 0x92E9   // Permanent Fail:VIMA:Min Active Current
#define TIBQ769x2_VIMAThreshold 0x92EB          // Permanent Fail:VIMA:Threshold
#define TIBQ769x2_VIMADelay 0x92ED              // Permanent Fail:VIMA:Delay
#define TIBQ769x2_CFETFOFFThreshold 0x92EE      // Permanent Fail:CFETF:OFF Threshold
#define TIBQ769x2_CFETFOFFDelay 0x92F0          // Permanent Fail:CFETF:OFF Delay
#define TIBQ769x2_DFETFOFFThreshold 0x92F1      // Permanent Fail:DFETF:OFF Threshold
#define TIBQ769x2_DFETFOFFDelay 0x92F3          // Permanent Fail:DFETF:OFF Delay
#define TIBQ769x2_VSSFFailThreshold 0x92F4      // Permanent Fail:VSSF:Fail Threshold
#define TIBQ769x2_VSSFDelay 0x92F6      // Permanent Fail:VSSF:Delay
#define TIBQ769x2_PF2LVLDelay 0x92F7    // Permanent Fail:2LVL:Delay
#define TIBQ769x2_LFOFDelay 0x92F8      // Permanent Fail:LFOF:Delay
#define TIBQ769x2_HWMXDelay 0x92F9      // Permanent Fail:HWMX:Delay
#define TIBQ769x2_SecuritySettings 0x9256   // Security:Settings:Security Settings
#define TIBQ769x2_UnsealKeyStep1 0x9257     // Security:Keys:Unseal Key Step 1
#define TIBQ769x2_UnsealKeyStep2 0x9259     // Security:Keys:Unseal Key Step 2
#define TIBQ769x2_FullAccessKeyStep1 0x925B // Security:Keys:Full Access Key Step 1
#define TIBQ769x2_FullAccessKeyStep2 0x925D // Security:Keys:Full Access Key Step 2

// Direct Commands
#define TIBQ769x2_ControlStatus 0x00
#define TIBQ769x2_SafetyAlertA 0x02
#define TIBQ769x2_SafetyStatusA 0x03
#define TIBQ769x2_SafetyAlertB 0x04
#define TIBQ769x2_SafetyStatusB 0x05
#define TIBQ769x2_SafetyAlertC 0x06
#define TIBQ769x2_SafetyStatusC 0x07
#define TIBQ769x2_PFAlertA 0x0A
#define TIBQ769x2_PFStatusA 0x0B
#define TIBQ769x2_PFAlertB 0x0C
#define TIBQ769x2_PFStatusB 0x0D
#define TIBQ769x2_PFAlertC 0x0E
#define TIBQ769x2_PFStatusC 0x0F
#define TIBQ769x2_PFAlertD 0x10
#define TIBQ769x2_PFStatusD 0x11
#define TIBQ769x2_BatteryStatus 0x12
#define TIBQ769x2_Cell1Voltage 0x14
#define TIBQ769x2_Cell2Voltage 0x16
#define TIBQ769x2_Cell3Voltage 0x18
#define TIBQ769x2_Cell4Voltage 0x1A
#define TIBQ769x2_Cell5Voltage 0x1C
#define TIBQ769x2_Cell6Voltage 0x1E
#define TIBQ769x2_Cell7Voltage 0x20
#define TIBQ769x2_Cell8Voltage 0x22
#define TIBQ769x2_Cell9Voltage 0x24
#define TIBQ769x2_Cell10Voltage 0x26
#define TIBQ769x2_Cell11Voltage 0x28
#define TIBQ769x2_Cell12Voltage 0x2A
#define TIBQ769x2_Cell13Voltage 0x2C
#define TIBQ769x2_Cell14Voltage 0x2E
#define TIBQ769x2_Cell15Voltage 0x30
#define TIBQ769x2_Cell16Voltage 0x32
#define TIBQ769x2_StackVoltage 0x34
#define TIBQ769x2_PACKPinVoltage 0x36
#define TIBQ769x2_LDPinVoltage 0x38
#define TIBQ769x2_CC2Current 0x3A
#define TIBQ769x2_AlarmStatus 0x62
#define TIBQ769x2_AlarmRawStatus 0x64
#define TIBQ769x2_AlarmEnable 0x66
#define TIBQ769x2_IntTemperature 0x68
#define TIBQ769x2_CFETOFFTemperature 0x6A
#define TIBQ769x2_DFETOFFTemperature 0x6C
#define TIBQ769x2_ALERTTemperature 0x6E
#define TIBQ769x2_TS1Temperature 0x70
#define TIBQ769x2_TS2Temperature 0x72
#define TIBQ769x2_TS3Temperature 0x74
#define TIBQ769x2_HDQTemperature 0x76
#define TIBQ769x2_DCHGTemperature 0x78
#define TIBQ769x2_DDSGTemperature 0x7A
#define TIBQ769x2_FETStatus 0x7F

// Subcommands
#define TIBQ769x2_DEVICE_NUMBER 0x0001
#define TIBQ769x2_FW_VERSION 0x0002
#define TIBQ769x2_HW_VERSION 0x0003
#define TIBQ769x2_IROM_SIG 0x0004
#define TIBQ769x2_STATIC_CFG_SIG 0x0005
#define TIBQ769x2_PREV_MACWRITE 0x0007
#define TIBQ769x2_DROM_SIG 0x0009
#define TIBQ769x2_SECURITY_KEYS 0x0035
#define TIBQ769x2_SAVED_PF_STATUS 0x0053
#define TIBQ769x2_MANUFACTURINGSTATUS 0x0057
#define TIBQ769x2_MANU_DATA 0x0070
#define TIBQ769x2_DASTATUS1 0x0071
#define TIBQ769x2_DASTATUS2 0x0072
#define TIBQ769x2_DASTATUS3 0x0073
#define TIBQ769x2_DASTATUS4 0x0074
#define TIBQ769x2_DASTATUS5 0x0075
#define TIBQ769x2_DASTATUS6 0x0076
#define TIBQ769x2_DASTATUS7 0x0077
#define TIBQ769x2_CUV_SNAPSHOT 0x0080
#define TIBQ769x2_COV_SNAPSHOT 0X0081
#define TIBQ769x2_CB_ACTIVE_CELLS 0x0083
#define TIBQ769x2_CB_SET_LVL 0x0084
#define TIBQ769x2_CBSTATUS1 0x0085
#define TIBQ769x2_CBSTATUS2 0x0086
#define TIBQ769x2_CBSTATUS3 0x0087
#define TIBQ769x2_FET_CONTROL 0x0097
#define TIBQ769x2_REG12_CONTROL 0x0098
#define TIBQ769x2_OTP_WR_CHECK 0x00A0
#define TIBQ769x2_OTP_WRITE 0x00A1
#define TIBQ769x2_READ_CAL1 0xF081
#define TIBQ769x2_CAL_CUV 0xF090
#define TIBQ769x2_CAL_COV 0xF091

// Command Only Subcommands
#define TIBQ769x2_EXIT_DEEPSLEEP 0x000E
#define TIBQ769x2_DEEPSLEEP 0x000F
#define TIBQ769x2_SHUTDOWN 0x0010
#define TIBQ769x2_RESET 0x0012 //"RESET" in documentation
#define TIBQ769x2_PDSGTEST 0x001C
#define TIBQ769x2_FUSE_TOGGLE 0x001D
#define TIBQ769x2_PCHGTEST 0x001E
#define TIBQ769x2_CHGTEST 0x001F
#define TIBQ769x2_DSGTEST 0x0020
#define TIBQ769x2_FET_ENABLE 0x0022
#define TIBQ769x2_PF_ENABLE 0x0024
#define TIBQ769x2_PF_RESET 0x0029
#define TIBQ769x2_SEAL 0x0030
#define TIBQ769x2_RESET_PASSQ 0x0082
#define TIBQ769x2_PTO_RECOVER 0x008A
#define TIBQ769x2_SET_CFGUPDATE 0x0090
#define TIBQ769x2_EXIT_CFGUPDATE 0x0092
#define TIBQ769x2_DSG_PDSG_OFF 0x0093
#define TIBQ769x2_CHG_PCHG_OFF 0x0094
#define TIBQ769x2_ALL_FETS_OFF 0x0095
#define TIBQ769x2_ALL_FETS_ON 0x0096
#define TIBQ769x2_SLEEP_ENABLE 0x0099
#define TIBQ769x2_SLEEP_DISABLE 0x009A
#define TIBQ769x2_OCDL_RECOVER 0x009B
#define TIBQ769x2_SCDL_RECOVER 0x009C
#define TIBQ769x2_LOAD_DETECT_RESTART 0x009D
#define TIBQ769x2_LOAD_DETECT_ON 0x009E
#define TIBQ769x2_LOAD_DETECT_OFF 0x009F
#define TIBQ769x2_CFETOFF_LO 0x2800
#define TIBQ769x2_DFETOFF_LO 0x2801
#define TIBQ769x2_ALERT_LO 0x2802
#define TIBQ769x2_HDQ_LO 0x2806
#define TIBQ769x2_DCHG_LO 0x2807
#define TIBQ769x2_DDSG_LO 0x2808
#define TIBQ769x2_CFETOFF_HI 0x2810
#define TIBQ769x2_DFETOFF_HI 0x2811
#define TIBQ769x2_ALERT_HI 0x2812
#define TIBQ769x2_HDQ_HI 0x2816
#define TIBQ769x2_DCHG_HI 0x2817
#define TIBQ769x2_DDSG_HI 0x2818
#define TIBQ769x2_PF_FORCE_A 0x2857
#define TIBQ769x2_PF_FORCE_B 0x29A3
#define TIBQ769x2_SWAP_COMM_MODE 0x29BC
#define TIBQ769x2_SWAP_TO_I2C 0x29E7
#define TIBQ769x2_SWAP_TO_SPI 0x7C35
#define TIBQ769x2_SWAP_TO_HDQ 0x7C40

// bit masks
#define MfgStatusInit_FET_EN 0x08   //bit 4
#define FET_STATUS_DSG_FET_EN 0x04  //bit 3
#define FET_STATUS_CHG_FET_EN 0x01  //bit 1

// Alarm Status bits
#define ALARM_STATUS_WAKE       (1 << 0)    // device is wakened from sleep mode
#define ALARM_STATUS_ADSCAN     (1 << 1)    // voltage ADC scan complete
#define ALARM_STATUS_CB         (1 << 2)    // cell balancing is active
#define ALARM_STATUS_FUSE       (1 << 3)    // fuse pin driven
#define ALARM_STATUS_SHUTV      (1 << 4)    // stack voltage is below shutdown stack voltage
#define ALARM_STATUS_XDSG       (1 << 5)    // discharge FET is off
#define ALARM_STATUS_XCHG       (1 << 6)    // charge FET is off
#define ALARM_STATUS_FULLSCAN   (1 << 7)    // full voltage scan complete
#define ALARM_STATUS_RSVD       (1 << 8)    // reserved
#define ALARM_STATUS_INITCOMP   (1 << 9)    // initialisation complete
#define ALARM_STATUS_INITSTART  (1 << 10)   // initialisation started
#define ALARM_STATUS_MSK_PFALERT (1 << 11)  // masked safety alerts.  set coresponding to how SF Alert Mask A~C have been set
#define ALARM_STATUS_MSK_SFALERT (1 << 12)  // masked safety alerts.  set coresponding to how SF Alert Mask A~C have been set
#define ALARM_STATUS_PF         (1 << 13)   // permanent fail status
#define ALARM_STATUS_SSA        (1 << 14)   // hardware safety status.  set if a bit in Safety Status A is set
#define ALARM_STATUS_SSBC       (1 << 15)   // safety status.  set if a bit in Safety Status B~C is set

/*
  TI bq76952 register definitions from datasheet SLUUBY2B
  Uses direct commands (7-bit addresses) for voltage readings
  Data stored in little endian byte order
 */
// Direct commands for voltage measurements (from Table 4-1)
#define REG_OTP_CHECK           0xA0    // OTP memory check register (one-time-programmable memory)
#define REG_STACK_VOLTAGE_L     0x34    // Stack (VC16 pin) voltage LSB (µV units)
#define REG_STACK_VOLTAGE_H     0x35    // Stack (VC16 pin) voltage MSB (µV units)
#define REG_PACK_VOLTAGE_L      0x36    // PACK pin voltage LSB (µV units)
#define REG_PACK_VOLTAGE_H      0x37    // PACK pin voltage MSB (µV units)
#define REG_LD_VOLTAGE_L        0x38    // LD pin voltage LSB (µV units)
#define REG_LD_VOLTAGE_H        0x39    // LD pin voltage MSB (µV units)

// Individual cell voltage registers (mV units)
#define REG_CELL1_VOLTAGE_L     0x14    // Cell 1 voltage LSB
#define REG_CELL1_VOLTAGE_H     0x15    // Cell 1 voltage MSB

// BQ76952 Communication Settings
#define DISABLE_TS1 1
#define DISABLE_TS3 1

// battery specific definitions.  These may be overwritten in hwdef.inc
#ifndef AP_BATTMON_CELL_COUNT
#define AP_BATTMON_CELL_COUNT 6
#endif

#define DEBUG_PRINT 1

#if DEBUG_PRINT
 # define Debug(fmt, args ...) do {printf(fmt "\n", ## args);} while(0)
#else
 # define Debug(fmt, args ...)
#endif

// Expected device IDs
#define DEVICE_ID_TIBQ7695 0x7695

// Note: BQ76952 doesn't have a simple device ID register at 0x00
// Device identification requires subcommands, not direct commands

#ifndef HAL_BATTMON_BQ76952_MAX_VOLTAGE
#define HAL_BATTMON_BQ76952_MAX_VOLTAGE 50.0f
#endif

// configuration settings to write during setup
const AP_BattMonitor_TIBQ76952::ConfigurationSetting AP_BattMonitor_TIBQ76952::config_settings[] {

    // 'Power Config' - 0x9234 = 0x2D80
    // Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
    // Set wake speed bits to 00 for best performance
    {TIBQ769x2_PowerConfig, 0x2D80, 2},

    // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
    {TIBQ769x2_REG0Config, 0x01, 1},

    // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
    {TIBQ769x2_REG12Config, 0x0D, 1},

    // Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
    {TIBQ769x2_DFETOFFPinConfig, 0x42, 1},

    // Set up ALERT Pin - 0x92FC = 0x2A
    // This configures the ALERT pin to drive high (REG1 voltage) when enabled.
    // The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
    {TIBQ769x2_ALERTPinConfig, 0x2A, 1},

#if DISABLE_TS1
    {TIBQ769x2_TS1Config, 0x00, 1},
#else
    // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
    {TIBQ769x2_TS1Config, 0x07, 1},
#endif

#if DISABLE_TS3
    {TIBQ769x2_TS3Config, 0x00, 1},
#else
    // Set TS3 to measure FET Temperature - 0x92FF = 0x0F
    {TIBQ769x2_TS3Config, 0x0F, 1},
#endif

    // Set HDQ to measure Cell Temperature - 0x9300 = 0x00 (No thermistor installed on EVM HDQ pin)
    {TIBQ769x2_HDQPinConfig, 0x00, 1},

    // 'VCell Mode' - Enable 16 cells - 0x9304
    // Writing 0x0000 sets the default of 16 cells, but we'll calculate based on actual cell count
    {TIBQ769x2_VCellMode, (1 << AP_BATTMON_CELL_COUNT) - 1, 2},

#if defined(DISABLE_PROTECTION_A)
    {TIBQ769x2_EnabledProtectionsA, 0x00, 1},
#else
    // Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
    // Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
    // COV (over-voltage), CUV (under-voltage)
    {TIBQ769x2_EnabledProtectionsA, 0xBC, 1},
#endif

#if defined(DISABLE_PROTECTION_B)
    {TIBQ769x2_EnabledProtectionsB, 0x00, 1},
#else
    // Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
    // Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
    // OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
    {TIBQ769x2_EnabledProtectionsB, 0xF7, 1},
#endif

    // 'Default Alarm Mask' - 0x926D Enables the FullScan and ADScan bits, default value = 0xF800
    {TIBQ769x2_DefaultAlarmMask, 0xF882, 2},

    // Set up Cell Balancing Configuration - 0x9335 = 0x03 - Automated balancing while in Relax or Charge modes
    {TIBQ769x2_BalancingConfiguration, 0x03, 1},

    // Set up CUV (under-voltage) Threshold - 0x9275 = 0x31 (2479 mV)
    // CUV Threshold is this value multiplied by 50.6mV
    {TIBQ769x2_CUVThreshold, 0x31, 1},

    // Set up COV (over-voltage) Threshold - 0x9278 = 0x55 (4301 mV)
    // COV Threshold is this value multiplied by 50.6mV
    {TIBQ769x2_COVThreshold, 0x55, 1},

    // Set up OCC (over-current in charge) Threshold - 0x9280 = 0x05 (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
    {TIBQ769x2_OCCThreshold, 0x05, 1},

    // Set up OCD1 Threshold - 0x9282 = 0x0A (20 mV = 20A across 1mOhm sense resistor) units of 2mV
    {TIBQ769x2_OCD1Threshold, 0x0A, 1},

    // Set up SCD Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor) 0x05=100mV
    {TIBQ769x2_SCDThreshold, 0x05, 1},

    // Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 μs; min value of 1
    {TIBQ769x2_SCDDelay, 0x03, 1},

    // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
    // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
    {TIBQ769x2_SCDLLatchLimit, 0x01, 1},
};

// initialise the TIBQ76952 battery monitor
void AP_BattMonitor_TIBQ76952::init(void)
{
    dev = hal.i2c_mgr->get_device_ptr(AP_BATTMON_TIBQ76952_BUS, AP_BATTMON_TIBQ76952_ADDR);
    if (dev == nullptr) {
        return;
    }

    // Register periodic callback at 10hz for reading voltage
    dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_TIBQ76952::timer, void));
}

/// read the battery_voltage, should be called at 10hz
void AP_BattMonitor_TIBQ76952::read(void)
{
    // take semaphore before accessing accumulate struct
    WITH_SEMAPHORE(accumulate_sem);

    // exit immediately if no sensor updates
    if (accumulate.count == 0) {
        return;
    }

    // copy accumulated values to state
    _state.last_time_micros = AP_HAL::micros();
    _state.healthy = true;
    _state.voltage = accumulate.voltage / accumulate.count;
    _state.state_of_health_pct = accumulate.health_pct / accumulate.count;
    _state.has_state_of_health_pct = true;
    _state.current_amps = fabsf(accumulate.current / accumulate.count);
    _state.temperature = accumulate.temp / accumulate.count;
    const uint8_t num_cells = MIN(AP_BATTMON_CELL_COUNT, MIN(ARRAY_SIZE(_state.cell_voltages.cells), ARRAY_SIZE(accumulate.cell_voltages_mv)));
    for (uint8_t i = 0; i < num_cells; i++) {
        _state.cell_voltages.cells[i] = accumulate.cell_voltages_mv[i] / accumulate.count;
    }

    // clear accumulate structure
    accumulate = {};
}

// set desired powered state (enabled/disabled) by enabling/disabling discharge FET
void AP_BattMonitor_TIBQ76952::set_powered_state(bool power_on)
{
    if (!configured) {
        return;
    }
    sub_command(power_on ? TIBQ769x2_ALL_FETS_ON : TIBQ769x2_ALL_FETS_OFF);
}

// periodic timer callback
void AP_BattMonitor_TIBQ76952::timer(void)
{
    // configure device if required
    if (!configure()) {
        return;
    }

    // read data from device
    read_voltage_current_temperature();
    read_charging_state();
}

// configure device
bool AP_BattMonitor_TIBQ76952::configure()
{
    // exit immediately if already configured
    if (configured) {
        return true;
    }

    // check device id, exit on failure
    const uint32_t device_number = sub_command_read_4bytes(TIBQ769x2_DEVICE_NUMBER);
    if (device_number != DEVICE_ID_TIBQ7695) {
        Debug("BQ76952: Unknown device detected - ID: 0x%08lX", (unsigned long)device_number);
        return false;
    }

    // check device's firmware and hardware versions
#if DEBUG_PRINT
    const uint32_t fw_version = sub_command_read_4bytes(TIBQ769x2_FW_VERSION);
    const uint32_t hw_version = sub_command_read_4bytes(TIBQ769x2_HW_VERSION);
    Debug("BQ76952 detected, fw: 0x%08lX, hw: 0x%08lX", (unsigned long)fw_version, (unsigned long)hw_version);
#endif

    // enable REG1 3.3v to provide power to the MCU
    set_register(TIBQ769x2_REG12Config, 0x01, 1);

    // wake up device
    sub_command(TIBQ769x2_EXIT_DEEPSLEEP);
    hal.scheduler->delay(10);
    sub_command(TIBQ769x2_SLEEP_DISABLE);
    hal.scheduler->delay(10);

    // clear any remaining permanent failure alerts
    direct_command(TIBQ769x2_PFAlertA, 0xFF, CommandType::WRITE);
    direct_command(TIBQ769x2_PFAlertB, 0xFF, CommandType::WRITE);
    direct_command(TIBQ769x2_PFAlertC, 0xFF, CommandType::WRITE);
    direct_command(TIBQ769x2_PFAlertD, 0xFF, CommandType::WRITE);

    // enter CONFIGUPDATE mode (Subcommand 0x0090) - Required to program device RAM settings
    sub_command(TIBQ769x2_SET_CFGUPDATE);

    // write configuration settings to device registers
    for (uint8_t i = 0; i < ARRAY_SIZE(config_settings); i++) {
        set_register(config_settings[i].reg_addr, config_settings[i].reg_data, config_settings[i].len);
    }

    // exit configuration mode
    sub_command(TIBQ769x2_EXIT_CFGUPDATE);
    sub_command(TIBQ769x2_ALL_FETS_OFF);
    sub_command(TIBQ769x2_FET_ENABLE);

    // mark configuration as complete to prevent repeated attempts
    configured = true;
    Debug("BQ76952: Configuration complete");

    // report success
    return true;
}

// read battery voltage, current and temperature
void AP_BattMonitor_TIBQ76952::read_voltage_current_temperature()
{
    // exit immediately if full voltage scan has not completed (FULL_SCAN bit 7)
    const uint16_t alarm_raw_status = direct_command_read_2bytes(TIBQ769x2_AlarmRawStatus);
    if (alarm_raw_status & ALARM_STATUS_FULLSCAN) {
        return;
    }

    // get semaphore before updating accumulate structure
    WITH_SEMAPHORE(accumulate_sem);

    // read stack voltage (should equal sum of cell voltages)
    // we do not use the package voltage because this is floating when FETs are off
    accumulate.voltage += direct_command_read_2bytes(TIBQ769x2_StackVoltage) * 0.01;

    // read individual cell voltages
    for (uint8_t i = 0; i < AP_BATTMON_CELL_COUNT; i++) {
        const uint16_t cell_voltage_mv = direct_command_read_2bytes(TIBQ769x2_Cell1Voltage + i*2);
        accumulate.cell_voltages_mv[i] += cell_voltage_mv;
    }

    // read current (positive values = discharging, negative = charging)
    const int16_t cc2_current = direct_command_read_2bytes(TIBQ769x2_CC2Current);
    accumulate.current += cc2_current * 0.001f; // convert to Amps

    // read temperature
    const int16_t temp_internal = direct_command_read_2bytes(TIBQ769x2_IntTemperature); // 0.1K
    accumulate.temp += KELVIN_TO_C(temp_internal * 0.1f); // convert to degC

    // increment number of readings
    accumulate.count++;
}

// read battery charging state (e.g. idle, charging, discharging)
void AP_BattMonitor_TIBQ76952::read_charging_state()
{
    AP_BattMonitor::ChargingState new_state = _state.charging_state;

    const uint16_t mfg_status = direct_command_read_2bytes(TIBQ769x2_MfgStatusInit);
    const uint8_t fet_status = direct_command_read_1byte(TIBQ769x2_FETStatus);

    if (mfg_status & MfgStatusInit_FET_EN) {
        new_state = AP_BattMonitor::ChargingState::IDLE;
        if (fet_status & FET_STATUS_CHG_FET_EN && accumulate.current > 0) {
            // Charging if CHG_FET bit is set
            new_state = AP_BattMonitor::ChargingState::CHARGING;
        }
        if (fet_status & FET_STATUS_DSG_FET_EN) {
            // Discharging if DSG_FET bit is set
            new_state = AP_BattMonitor::ChargingState::DISCHARGING;
        }
    }

    _state.charging_state = new_state;
}

// read bytes from a register. returns true on success
bool AP_BattMonitor_TIBQ76952::read_register(uint8_t reg_addr, uint8_t *reg_data, uint8_t len) const
{
    // sanity check device pointer
    if (dev == nullptr) {
        return false;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    // read bytes from device
    return dev->read_registers(reg_addr, reg_data, len);
}

// write a single byte to consecutive registers. returns true on success
bool AP_BattMonitor_TIBQ76952::write_register(uint8_t reg_addr, const uint8_t *reg_data, uint8_t len) const
{
    // sanity check device pointer
    if (dev == nullptr) {
        return false;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    // write each byte
    for (uint8_t i = 0; i < len; i++) {
        if (!dev->write_register(reg_addr + i, reg_data[i])) {
            return false;
        }
    }
    return true;
}

// read or write a direct command. returns true on success
// direct commands are send using a 7-bit command address and may trigger an action or read/write a datavalue
bool AP_BattMonitor_TIBQ76952::direct_command(uint16_t command, uint16_t data, CommandType type, uint8_t *rx_data, uint8_t rx_len) const
{
    // sanity check read buffer
    if (type == CommandType::READ && rx_data == nullptr) {
        return false;
    }

    // prepare transmit buffer
    const uint8_t tx_buffer[2] {LOWBYTE(data), HIGHBYTE(data)};
    switch (type) {
    case CommandType::READ:
        return read_register(command, rx_data, rx_len);
    case CommandType::WRITE:
        return write_register(command, tx_buffer, rx_len);
    }

    return false;
}

// send a direct command to read 1 byte
uint8_t AP_BattMonitor_TIBQ76952::direct_command_read_1byte(uint16_t reg) const
{
    uint8_t rx_data;
    if (direct_command(reg, 0x00, CommandType::READ, &rx_data, 1)) {
        return rx_data;
    }
    return 0;
}

// send a direct command to read 2 bytes
uint16_t AP_BattMonitor_TIBQ76952::direct_command_read_2bytes(uint16_t reg) const
{
    uint8_t rx_data[2];
    if (direct_command(reg, 0x00, CommandType::READ, rx_data, 2)) {
        return UINT16_VALUE(rx_data[1], rx_data[0]);
    }
    return 0;
}

// write 1, 2, or 4 bytes to a RAM data memory register (0x9xxx addresses)
// this includes delays so it should only be called during startup configuration
// uses subcommand protocol: write address+data to 0x3E, then checksum+length to 0x60
void AP_BattMonitor_TIBQ76952::set_register(uint16_t reg_addr, uint32_t reg_data, uint8_t len) const
{
    // sanity check len argument
    if (len != 1 && len != 2 && len != 4) {
        return;
    }

    // prepare transmit data
    const uint8_t tx_reg_data[6] {
        LOWBYTE(reg_addr),
        HIGHBYTE(reg_addr),
        uint8_t((reg_data >> 0) & 0xFF),
        uint8_t((reg_data >> 8) & 0xFF),
        uint8_t((reg_data >> 16) & 0xFF),
        uint8_t((reg_data >> 24) & 0xFF)
    };

    // write sub command address
    write_register(0x3E, tx_reg_data, len + 2);
    hal.scheduler->delay(2);

    // write checksum and length
    const uint8_t tx_buffer[2] {crc_sum_of_bytes(tx_reg_data, 3), uint8_t(len + 4)};
    write_register(0x60, tx_buffer, 2);
    hal.scheduler->delay(2);
}

// send a command-only subcommand (no data payload, no checksum)
// used for commands like ALL_FETS_ON, SLEEP_DISABLE, EXIT_CFGUPDATE, etc.
bool AP_BattMonitor_TIBQ76952::sub_command(uint16_t command) const
{
    const uint8_t tx_buffer[2] {LOWBYTE(command), HIGHBYTE(command)};
    return write_register(0x3E, tx_buffer, 2);
}

// send a subcommand with read or write capability
// this includes delays so it should only be called during startup configuration
// READ: writes command to 0x3E, reads response from 0x40
// WRITE: writes command+data to 0x3E, then checksum+length to 0x60
bool AP_BattMonitor_TIBQ76952::sub_command(uint16_t command, uint16_t data, CommandType type, uint8_t *rx_data, uint8_t rx_len) const
{
    switch (type) {
    case CommandType::READ: {
        // read subcommand
        const uint8_t tx_reg[2] {LOWBYTE(command), HIGHBYTE(command)};
        if (!write_register(0x3E, tx_reg, 2)) {
            return false;
        }
        hal.scheduler->delay(2);
        if (rx_data && rx_len > 0) {
            // read into provided buffer
            if (!read_register(0x40, rx_data, rx_len)) {
                return false;
            }
            return true;
        }
        break;
    }
    case CommandType::WRITE: {
        // write subcommand with 1 byte data (FET_Control, REG12_Control)
        const uint8_t tx_reg[3] {LOWBYTE(command), HIGHBYTE(command), uint8_t(data & 0xFF)};
        if (!write_register(0x3E, tx_reg, 3)) {
            return false;
        }
        hal.scheduler->delay(1);
        const uint8_t tx_buffer[2] {crc_sum_of_bytes(tx_reg, 3), 0x05}; // 0x05 is combined length of registers address and data
        if (!write_register(0x60, tx_buffer, 2)) {
            return false;
        }
        hal.scheduler->delay(1);
        break;
    }
    }

    return true;
}

// subcommands are sent using a 16 bit command address and support block data transfers
uint32_t AP_BattMonitor_TIBQ76952::sub_command_read_4bytes(uint16_t reg) const
{
    uint8_t rx_data[4];
    if (sub_command(reg, 0x00, CommandType::READ, rx_data, 4)) {
        return UINT32_VALUE(rx_data[3], rx_data[2], rx_data[1], rx_data[0]);
    }
    return 0;
}

#endif // AP_BATTERY_TIBQ76952_ENABLED
