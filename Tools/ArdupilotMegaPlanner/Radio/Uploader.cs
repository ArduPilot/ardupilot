using System;
using System.IO.Ports;
using System.Collections.Generic;

namespace uploader
{
	public class Uploader
	{		
		public event ArdupilotMega._3DRradio.LogEventHandler LogEvent;
        public event ArdupilotMega._3DRradio.ProgressEventHandler ProgressEvent;
		
		private	int bytes_to_process;
		private int bytes_processed;
		public SerialPort port;
		
		private enum Code : byte
		{
			// response codes
			OK				= 0x10,
			FAILED			= 0x11,
			INSYNC			= 0x12,
			
			// protocol commands
			EOC				= 0x20,
			GET_SYNC		= 0x21,
			GET_DEVICE		= 0x22,	// returns DEVICE_ID and FREQ bytes
			CHIP_ERASE		= 0x23,
			LOAD_ADDRESS	= 0x24,
			PROG_FLASH		= 0x25,
			READ_FLASH		= 0x26,
			PROG_MULTI		= 0x27,
			READ_MULTI		= 0x28,
			REBOOT			= 0x30,
			
			// protocol constants
			PROG_MULTI_MAX	= 64,	// maximum number of bytes in a PROG_MULTI command
			READ_MULTI_MAX	= 255,	// largest read that can be requested
			
			// device IDs XXX should come with the firmware image...
			DEVICE_ID_RF50	= 0x4d,
			DEVICE_ID_HM_TRP= 0x4e,
			
			// frequency code bytes XXX should come with the firmware image...
			FREQ_NONE		= 0xf0,
			FREQ_433		= 0x43,
			FREQ_470		= 0x47,
			FREQ_868		= 0x86,
			FREQ_915		= 0x91,
		};
		
		public Uploader ()
		{
		}
		
	
		/// <summary>
		/// Upload the specified image_data.
		/// </summary>
		/// <param name='image_data'>
		/// Image_data to be uploaded.
		/// </param>
		public void upload (SerialPort on_port, IHex image_data)
		{
			progress (0);
			
			port = on_port;
			
			try {
				connect_and_sync ();
				upload_and_verify (image_data);
				cmdReboot ();
			} catch (Exception e) {
				if (port.IsOpen)
					port.Close ();
				throw e;
			}
		}
			
		public void connect_and_sync ()
		{
			// configure the port
			port.ReadTimeout = 2000;			// must be longer than full flash erase time (~1s)
			
			// synchronise with the bootloader
			//
			// The second sync attempt here is mostly laziness, though it does verify that we 
			// can send more than one packet.
			//
			for (int i = 0; i < 3; i++) {
				if (cmdSync ())
					break;
				log (string.Format ("sync({0}) failed\n", i), 1);
			}
			if (!cmdSync ()) {
				log ("FAIL: could not synchronise with the bootloader");
				throw new Exception ("SYNC FAIL");
			}
			checkDevice ();
			
			log ("connected to bootloader\n");
		}
		
		private void upload_and_verify (IHex image_data)
		{
			
			// erase the program area first
			log ("erasing program flash\n");
			cmdErase ();
			
			// progress fractions
			bytes_to_process = 0;
			foreach (byte[] bytes in image_data.Values) {
				bytes_to_process += bytes.Length;
			}
			bytes_to_process *= 2;		// once to program, once to verify
			bytes_processed = 0;
			
			// program the flash blocks
			log ("programming\n");
			foreach (KeyValuePair<UInt32, byte[]> kvp in image_data) {
				// move the program pointer to the base of this block
				cmdSetAddress (kvp.Key);
				log (string.Format ("prog 0x{0:X}/{1}\n", kvp.Key, kvp.Value.Length), 1);

				upload_block_multi (kvp.Value);
			}
			
			// and read them back to verify that they were programmed
			log ("verifying\n");
			foreach (KeyValuePair<UInt32, byte[]> kvp in image_data) {
				// move the program pointer to the base of this block
				cmdSetAddress (kvp.Key);
				log (string.Format ("verf 0x{0:X}/{1}\n", kvp.Key, kvp.Value.Length), 1);
				
				verify_block_multi (kvp.Value);
				bytes_processed += kvp.Value.GetLength (0);
				progress ((double)bytes_processed / bytes_to_process);
			}
			log ("Success\n");
		}
		
		private void upload_block (byte[] data)
		{						
			foreach (byte b in data) {
				cmdProgram (b);
				progress ((double)(++bytes_processed) / bytes_to_process);
			}
		}
		
		private void upload_block_multi (byte[] data)
		{
			int offset = 0;
			int to_send;
			int length = data.GetLength (0);
			
			// Chunk the block in units of no more than what the bootloader
			// will program.
			while (offset < length) {
				to_send = length - offset;
				if (to_send > (int)Code.PROG_MULTI_MAX)
					to_send = (int)Code.PROG_MULTI_MAX;
				
				log (string.Format ("multi {0}/{1}\n", offset, to_send), 1);
				cmdProgramMulti (data, offset, to_send);
				offset += to_send;

				bytes_processed += to_send;
				progress ((double)bytes_processed / bytes_to_process);				
			}			
		}
		
		private void verify_block_multi (byte[] data)
		{
			int offset = 0;
			int to_verf;
			int length = data.GetLength (0);
			
			// Chunk the block in units of no more than what the bootloader
			// will read.
			while (offset < length) {
				to_verf = length - offset;
				if (to_verf > (int)Code.READ_MULTI_MAX)
					to_verf = (int)Code.READ_MULTI_MAX;
				
				log (string.Format ("multi {0}/{1}\n", offset, to_verf), 1);
				cmdVerifyMulti (data, offset, to_verf);
				offset += to_verf;

				bytes_processed += to_verf;
				progress ((double)bytes_processed / bytes_to_process);				
			}			
			
		}
		
		/// <summary>
		/// Requests a sync reply.
		/// </summary>
		/// <returns>
		/// True if in sync, false otherwise.
		/// </returns>
		private bool cmdSync ()
		{
			port.DiscardInBuffer ();
			
			send (Code.GET_SYNC);
			send (Code.EOC);
			
			try {
				getSync ();
			} catch {
				return false;
			}
			
			return true;
		}

		/// <summary>
		/// Erases the device.
		/// </summary>
		private void cmdErase ()
		{
			send (Code.CHIP_ERASE);
			send (Code.EOC);
			
			getSync ();
		}
		
		/// <summary>
		/// Set the address for the next program or read operation.
		/// </summary>
		/// <param name='address'>
		/// Address to be set.
		/// </param>
		private void cmdSetAddress (UInt32 address)
		{
			send (Code.LOAD_ADDRESS);
			send ((UInt16)address);
			send (Code.EOC);
			
			getSync ();
		}	
		
		/// <summary>
		/// Programs a byte and advances the program address by one.
		/// </summary>
		/// <param name='data'>
		/// Data to program.
		/// </param>
		private void cmdProgram (byte data)
		{
			send (Code.PROG_FLASH);
			send (data);
			send (Code.EOC);
			
			getSync ();
		}
		
		private void cmdProgramMulti (byte[] data, int offset, int length)
		{
			send (Code.PROG_MULTI);
			send ((byte)length);
			for (int i = 0; i < length; i++)
				send (data [offset + i]);
			send (Code.EOC);
			
			getSync ();
		}
		
		/// <summary>
		/// Verifies the byte at the current program address.
		/// </summary>
		/// <param name='data'>
		/// Data expected to be found.
		/// </param>
		/// <exception cref='VerifyFail'>
		/// Is thrown when the verify fail.
		/// </exception>
		private void cmdVerify (byte data)
		{
			send (Code.READ_FLASH);
			send (Code.EOC);
			
			if (recv () != data)
				throw new Exception ("flash verification failed");
			
			getSync ();
		}
		
		private void cmdVerifyMulti (byte[] data, int offset, int length)
		{
			send (Code.READ_MULTI);
			send ((byte)length);
			send (Code.EOC);
			
			for (int i = 0; i < length; i++) {
				if (recv () != data [offset + i]) {
					log ("flash verification failed\n");
					throw new Exception ("VERIFY FAIL");
				}
			}

			getSync ();
		}
		
		private void cmdReboot ()
		{
			send (Code.REBOOT);
		}
		
		private void checkDevice ()
		{
			Code id, freq;
			
			send (Code.GET_DEVICE);
			send (Code.EOC);
			
			id = (Code)recv ();
			freq = (Code)recv ();
			
			// XXX should be getting valid board/frequency data from firmware file
			if ((id != Code.DEVICE_ID_HM_TRP) && (id != Code.DEVICE_ID_RF50))
				throw new Exception ("bootloader device ID mismatch");
			
			getSync ();
		}
		
		/// <summary>
		/// Expect the two-byte synchronisation codes within the read timeout.
		/// </summary>
		/// <exception cref='NoSync'>
		/// Is thrown if the wrong bytes are read.
		/// <exception cref='TimeoutException'>
		/// Is thrown if the read timeout expires.
		/// </exception>
		private void getSync ()
		{
			try {
				Code c;
				
				c = (Code)recv ();
				if (c != Code.INSYNC) {
					log (string.Format ("got {0:X} when expecting {1:X}\n", (int)c, (int)Code.INSYNC), 2);
					throw new Exception ("BAD SYNC");
				}
				c = (Code)recv ();
				if (c != Code.OK) {
					log (string.Format ("got {0:X} when expecting {1:X}\n", (int)c, (int)Code.EOC), 2);
					throw new Exception ("BAD STATUS");
				}
			} catch {
				log ("FAIL: lost synchronisation with the bootloader\n");
				throw new Exception ("SYNC LOST");
			}
			log ("in sync\n", 5);
		}
		
		/// <summary>
		/// Send the specified code to the bootloader.
		/// </summary>
		/// <param name='code'>
		/// Code to send.
		/// </param>
		private void send (Code code)
		{
			byte[] b = new byte[] { (byte)code };
			
			log ("send ", 5);
			foreach (byte x in b) {
				log (string.Format (" {0:X}", x), 5);
			}
			log ("\n", 5);
			
			port.Write (b, 0, 1);
		}
		
		/// <summary>
		/// Send the specified byte to the bootloader.
		/// </summary>
		/// <param name='data'>
		/// Data byte to send.
		/// </param>
		private void send (byte data)
		{
			byte[] b = new byte[] { data };
			
			log ("send ", 5);
			foreach (byte x in b) {
				log (string.Format (" {0:X}", x), 5);
			}
			log ("\n", 5);

			port.Write (b, 0, 1);
		}
		
		/// <summary>
		/// Send the specified 16-bit value, LSB first.
		/// </summary>
		/// <param name='data'>
		/// Data value to send.
		/// </param>
		private void send (UInt16 data)
		{
			byte[] b = new byte[2] { (byte)(data & 0xff), (byte)(data >> 8) };
			
			log ("send ", 5);
			foreach (byte x in b) {
				log (string.Format (" {0:X}", x), 5);
			}
			log ("\n", 5);

			port.Write (b, 0, 2);
		}
		
		/// <summary>
		/// Receive a byte.
		/// </summary>
		private byte recv ()
		{
			byte b;

            DateTime Deadline = DateTime.Now.AddMilliseconds(port.ReadTimeout);

            while (DateTime.Now < Deadline && port.BytesToRead == 0)
            {
            }
            if (port.BytesToRead == 0)
                throw new Exception("Timeout");

			b = (byte)port.ReadByte ();
			
			log (string.Format ("recv {0:X}\n", b), 5);
			
			return b;
		}
		
		private void log (string message, int level = 0)
		{
			if (LogEvent != null)
				LogEvent (message, level);
		}
		
		private void progress (double completed)
		{
			if (ProgressEvent != null)
				ProgressEvent (completed);
		}
	}
}

