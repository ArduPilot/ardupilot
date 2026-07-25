package com.MAVLink.Messages;

import java.util.HashMap;
import java.util.Map;

/**
 * Units enumeration to be used to prevent typos.
 *
 * This enumerated list is a snapshot of the the standard list of units found in the MAVLink Schema.
 *
 * @see <a href="https://github.com/ArduPilot/pymavlink/blob/master/generator/mavschema.xsd#L81">MAVLink Schema</a>
 *
 */
public enum UnitsEnum {
// time
	S("s"), // seconds 
	DS("ds"),// deciseconds 
	CS("cs"),// centiseconds 
	MS("ms"),// milliseconds 
	US("us"),// microseconds 
	HZ("Hz"),// Herz 
	MHZ("MHz"),// Mega-Herz 
	// distance
	KM("km"),// kilometres 
	DAM("dam"),// decametres 
	M("m"),// metres 
	M_PER_SEC("m/s"),// metres per second 
	M_PER_SEC_SEC("m/s/s"),// metres per second per second 
	M_PER_SEC_5("m/s*5"),// metres per second * 5 required from dagar for HIGH_LATENCY2 message 
	DM("dm"),// decimetres 
	DM_PER_S("dm/s"),// decimetres per second 
	CM("cm"),// centimetres 
	CM2("cm^2"),// centimetres squared (typically used in variance) 
	CM_PER_S("cm/s"),// centimetres per second 
	MM("mm"),// millimetres 
	MM_PER_S("mm/s"),// millimetres per second 
	MM_PER_H("mm/h"),// millimetres per hour 
	//temperature 
	K("K"),// Kelvin 
	DEG_C("degC"),// degrees Celsius 
	CDEG_C("cdegC"),// centi degrees Celsius 
	//angle
	RAD("rad"),// radians 
	RAD_PER_S("rad/s"),// radians per second 
	MRAD_PER_S("mrad/s"),// milli-radians per second 
	DEG("deg"),// degrees 
	DEG_OVER_2("deg/2"),// degrees/2 required from dagar for HIGH_LATENCY2 message
	DEG_PER_S("deg/s"),// degrees per second 
	CDEG("cdeg"),// centidegrees 
	CDEG_PER_S("cdeg/s"),// centidegrees per second 
	DEGe5("degE5"),// degrees * 10E5 
	DEGE7("degE7"),// degrees * 10E7 
	RPM("rpm"),// rotations per minute 
	//electricity
	V("V"),// Volt 
	CV("cV"),// centi-Volt 
	MV("mV"),// milli-Volt 
	A("A"),// Ampere 
	CA("cA"),// centi-Ampere 
	MA("mA"),// milli-Ampere 
	MAH("mAh"),// milli-Ampere hour 
	//magnetism
	MT("mT"),// milli-Tesla 
	GAUSS("gauss"),// Gauss 
	MGAUSS("mgauss"),// milli-Gauss 
	// energy 
	HJ("hJ"),// hecto-Joule 
	//power
	W("W"),// Watt 
	//force
	MG("mG"),// milli-G 
	//mass 
	G("g"),// grams 
	KG("kg"),// kilograms 
	//pressure
	PA("Pa"),// Pascal 
	HPA("hPa"),// hecto-Pascal 
	KPA("kPa"),// kilo-Pascal 
	MBAR("mbar"),// millibar 
	//ratio
	PERCENT("%"),// percent 
	DPERCENT("d%"),// decipercent 
	CPERCENT("c%"),// centipercent 
	DB("dB"),// Deci-Bell 
	DBM("dBm"),// Deci-Bell-milliwatts
	//digital
	KIB("KiB"),// Kibibyte (1024 bytes) 
	KIB_PER_S("KiB/s"),// Kibibyte (1024 bytes) per second 
	MIB("MiB"),// Mebibyte (1024*1024 bytes) 
	MIB_PER_S("MiB/s"),// Mebibyte (1024*1024 bytes) per second 
	BYTES("bytes"),// bytes 
	BYTES_PER_S("bytes/s"),// bytes per second
	BITS_PER_S("bits/s"),// bits per second 
	PIX("pix"),// pixels 
	DPIX("dpix"),// decipixels 
	//flow
	G_PER_MIN("g/min"),// grams/minute 
	CM3_PER_MIN("cm^3/min"),// cubic centimetres/minute 
	//volume
	CM3("cm^3"),// cubic centimetres 
	L("l");// liters
	private final String name;
	private static final Map<String, UnitsEnum> toEnum = new HashMap<>();

	static {
		for (UnitsEnum ue : UnitsEnum.values()) {
			toEnum.put(ue.getName(), ue);
		}
	}

	UnitsEnum(String name) {
		this.name = name;
	}

	public String getName() {
		return name;
	}

	@Override
	public String toString() {
		return this.name;
	}

	public static UnitsEnum fromName(String name) {
		return toEnum.get(name);
	}
}