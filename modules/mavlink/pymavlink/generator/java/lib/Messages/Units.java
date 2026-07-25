package com.MAVLink.Messages;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.Field;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
@SuppressWarnings("unchecked")
/**
 * Units annotation to provide programmatic access to the MAVLink units for a given field.
 *
 * The standard list of units can be found in the MAVLink Schema.
 *
 * @see <a href="https://github.com/ArduPilot/pymavlink/blob/master/generator/mavschema.xsd#L81">MAVLink Schema</a>
 *
 * A snapshot of the allowable units is available in the UnitsEnum class to prevent typos
 * @see {@link com.MAVLink.Messages.UnitsEnum}
 * 
 */
public @interface Units {
	/**
	 * The string value of the units. A string was used to prevent brittleness as the list of valid units change  
	 * @return 
	 */
	String value();

	class Test {
		@Units("m/s")
		public float speed;

		public static void main(String[] args) throws Exception {
			Field field = Test.class.getField("speed");
			Units annotation = (Units) field.getAnnotation(Units.class);
			System.out.println(UnitsEnum.fromName(annotation.value()));
		}
	}
}
