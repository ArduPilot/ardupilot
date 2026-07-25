using System;

public class Dye {
	private string msg = "";

	public Dye(string _msg) {
		msg = _msg;
	}

	private static string PointerSizeDescription {
		get {
			if (IntPtr.Size==8) {
				return "64-bit";
			} else if (IntPtr.Size==4) {
				return "32-bit";
			}
			return "Unknown";
		}
	}

	public void display() {
		System.Console.WriteLine("{0} ({1})", msg, PointerSizeDescription);
	}
}
