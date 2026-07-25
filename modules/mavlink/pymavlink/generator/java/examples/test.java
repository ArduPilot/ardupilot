import com.MAVLink.MAVLinkPacket;
import com.MAVLink.common.*;
import com.MAVLink.Parser;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;

public class test {

	public static void main(String[] args) {
		Parser mavParser = new Parser();
		try {
			FileInputStream tlog = new FileInputStream(args[0]);
			try {
				while(tlog.available() > 0) {
					MAVLinkPacket packet = mavParser.mavlink_parse_char(tlog.read());
					if(packet != null){
						System.out.println("msgid: " + packet.msgid);
					}
				}
				System.out.println("End tlog");
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}	
	}
}
