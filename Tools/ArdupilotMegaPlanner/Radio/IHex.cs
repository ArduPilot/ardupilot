using System;
using System.Collections.Generic;
using System.IO;

namespace uploader
{
	public class IHex : SortedList<UInt32, byte[]>
	{
        public event ArdupilotMega._3DRradio.LogEventHandler LogEvent;

        public event ArdupilotMega._3DRradio.ProgressEventHandler ProgressEvent;
		
		private SortedList<UInt32, UInt32>	merge_index;
		
		public IHex ()
		{
			merge_index = new SortedList<UInt32, UInt32> ();
		}
			
		public void load (string fromPath)
		{
			StreamReader sr = new StreamReader (fromPath);
			UInt32 loadedSize = 0;
			
			// discard anything we might previous have loaded
			Clear ();
			merge_index.Clear ();
			
			log (string.Format ("reading from {0}\n", Path.GetFileName(fromPath)));
			
			while (!sr.EndOfStream) {
				string line = sr.ReadLine ();
				
				// every line must start with a :
				if (!line.StartsWith (":"))
					throw new Exception ("invalid IntelHex file");

                if (ProgressEvent != null)
                    ProgressEvent(sr.BaseStream.Position / (double)sr.BaseStream.Length);
				
				// parse the record type and data length, assume ihex8
				// ignore the checksum
				byte length = Convert.ToByte (line.Substring (1, 2), 16);
				UInt32 address = Convert.ToUInt32 (line.Substring (3, 4), 16);
				byte rtype = Convert.ToByte (line.Substring (7, 2), 16);
				
				// handle type zero (data) records
				if (rtype == 0) {
					byte[] b = new byte[length];
					string hexbytes = line.Substring (9, length * 2);
					
					// convert hex bytes
					for (int i = 0; i < length; i++) {
						b [i] = Convert.ToByte (hexbytes.Substring (i * 2, 2), 16);
					}
	
					log (string.Format ("ihex: 0x{0:X}: {1}\n", address, length), 1);
					loadedSize += length;
					
					// and add to the list of ranges
					insert (address, b);
				}
			}
			if (Count < 1)
				throw new Exception ("no data in IntelHex file");
			log (string.Format ("read {0} bytes from {1}\n", loadedSize, fromPath));
		}
		
		private void log (string message, int level = 0)
		{
			if (LogEvent != null)
				LogEvent (message, level);
		}
		
		private void idx_record (UInt32 start, byte[] data)
		{
			UInt32 len = (UInt32)data.GetLength (0);
			
			merge_index.Add (start + len, start);
		}
		
		private void idx_remove (UInt32 start, byte[] data)
		{
			UInt32 len = (UInt32)data.GetLength (0);

			merge_index.Remove (start + len);
		}
		
		private bool idx_find (UInt32 start, out UInt32 other)
		{
			return merge_index.TryGetValue (start, out other);
		}
		
		public void insert (UInt32 key, byte[] data)
		{
			UInt32 other;
			byte[] mergedata;
			
			// value of the key that would come after this one
			other = key;
			other += (UInt32)data.GetLength (0);
			
			// can we merge with the next block
			if (TryGetValue (other, out mergedata)) {
				int oldlen = data.GetLength (0);
				
				// remove the next entry, we are going to merge with it
				Remove (other);
				
				// remove its index entry as well
				idx_remove (other, mergedata);
				
				log (string.Format ("ihex: merging {0:X}/{1} with next {2:X}/{3}\n", 
					key, data.GetLength (0),
					other, mergedata.GetLength (0)), 1);
				
				// resize the data array and append data from the next block
				Array.Resize (ref data, data.GetLength (0) + mergedata.GetLength (0));
				Array.Copy (mergedata, 0, data, oldlen, mergedata.GetLength (0));
			}
			
			// look up a possible adjacent preceding block in the merge index
			if (idx_find (key, out other)) {
			
				mergedata = this [other];
				int oldlen = mergedata.GetLength (0);
				Remove (other);
				idx_remove (other, mergedata);

				log (string.Format ("ihex: merging {0:X}/{1} with prev {2:X}/{3}\n", 
					key, data.GetLength (0),
					other, mergedata.GetLength (0)), 1);

				Array.Resize (ref mergedata, data.GetLength (0) + mergedata.GetLength (0));
				Array.Copy (data, 0, mergedata, oldlen, data.GetLength (0));
				key = other;
				data = mergedata;
			}
			
			// add the merged block
			Add (key, data);
			idx_record (key, data);
			log (string.Format ("ihex: adding {0:X}/{1}\n", key, data.GetLength (0)), 1);
		}
	}
}

