// Thomas Nagy, 2011

// TODO security
// TODO handle all exceptions properly

import java.util.HashMap;
import java.util.Map;
import java.util.List;
import java.util.Date;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Collections;
import java.lang.Math;
import java.lang.StringBuilder;
import java.io.*;
import java.net.*;
import java.security.*;

public class Netcache implements Runnable, Comparator<Object[]> {
	private static int PORT_UPLOAD   = 11001;
	private static int PORT_DOWNLOAD = 12001;
	private static String CACHEDIR = "/tmp/wafcache/";
	private static long MAX = 10l * 1024l * 1024l * 1024l;
	private static double CLEANRATIO = 0.8;
	private static int BUF = 16 * 8192;

	private final static HashMap<String, Object[]> flist = new HashMap<String, Object[]>();
	private Socket sock = null;
	private int port = 0;

	public Netcache(Socket sock, int port) {
		this.sock = sock;
		this.port = port;
	}

	public void run () {
		try {
			if (sock != null)
			{
				while (true) {
					InputStream in = sock.getInputStream();
					OutputStream out = sock.getOutputStream();

					byte b[] = new byte[128];
					int off = 0;
					while (off < b.length) {
						off += in.read(b, off, b.length - off);
					}

					//System.out.println(new String(b));
					String[] args = new String(b).split(",");
					if (args[0].equals("LST")) {
						lst(args, in, out);
					}
					else if (args[0].equals("PUT") && port == PORT_UPLOAD) {
						put(args, in, out);
					}
					else if (args[0].equals("GET") && port == PORT_DOWNLOAD) {
						get(args, in, out);
					}
					else if (args[0].equals("CLEAN") && port == PORT_UPLOAD) {
						clean(args, in, out);
					}
					else if (args[0].equals("BYE")) {
						sock.close();
						break;
					}
					else {
						System.out.println("Invalid command " + new String(b) + " on port " + this.port);
						sock.close();
						break;
					}
				}
			} else {
				// magic trick to avoid creating a new inner class
				ServerSocket server = new ServerSocket(port);
				server.setReuseAddress(true);
				while(true) {
					Netcache tmp = new Netcache(server.accept(), port);
					Thread t = new Thread(tmp);
					t.start();
				}
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void lst(String[] args, InputStream in, OutputStream out) throws IOException {
		StringBuilder b = new StringBuilder();
		int k = 0;
		synchronized(flist) {
			for (String name : flist.keySet()) {
				b.append(name);
				if (k <= flist.size()) {
					k++;
					b.append("\n");
				}
			}
		}

		byte[] ret = b.toString().getBytes();
		String header = String.format("%-128s", String.format("%d,", ret.length));

		out.write(header.getBytes());
		out.write(ret);
	}

	public void put(String[] args, InputStream in, OutputStream out) throws IOException {
		File cachedir = new File(CACHEDIR);
		File temp = File.createTempFile("foo", ".suffix", cachedir);

		long size = new Long(args[3].trim());

		//System.out.println("" + args[1] + " " + args[2] + " " + args[3] + " " + args.length);

		byte[] buf = new byte[BUF];
		long cnt = 0;
		OutputStream w = new FileOutputStream(temp);
		try {
			while (cnt < size) {
				int c = in.read(buf, 0, (int) Math.min(BUF, size-cnt));
				if (c == 0) {
					throw new RuntimeException("Connection closed too early");
				}
				w.write(buf, 0, c);
				cnt += c;
			}
		} finally {
			w.close();
		}

		/*if (cnt != size) {
		  System.out.println("error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		  }*/

		File parent = new File(new File(new File(CACHEDIR), args[1].substring(0, 2)), args[1]);
		File dest = new File(parent, args[2]);
		try {
			dest.getParentFile().mkdirs();
		} catch (Exception e) {
		}

		if (!temp.renameTo(dest)) {
			throw new RuntimeException("Could not rename the file");
		}

		long total = 0;
		for (File f : parent.listFiles()) {
			total += f.length();
		}

		synchronized(flist) {
			if (flist.containsKey(parent.getName())) {
				flist.get(parent.getName())[0] = parent.lastModified();
			}
			else
			{
				flist.put(parent.getName(), new Object[] {parent.lastModified(), total, parent.getName()});
			}
		}
	}

	public void get(String[] args, InputStream in, OutputStream out) throws IOException {
		File f = new File(new File(new File(new File(CACHEDIR), args[1].substring(0, 2)), args[1]), args[2].trim());
		long fsize = -1;
		try {
			fsize = f.length();
		} catch (Exception e) {
			// return -1 to the client
		}

		String ret = String.format("%-128s", String.format("%d,", fsize));
		out.write(ret.getBytes());

		byte[] buf = new byte[BUF];

		long cnt = 0;
		InputStream s = new FileInputStream(f);
		try {
			while (cnt < fsize) {
				long c = s.read(buf);
				cnt += c;
				out.write(buf, 0, (int) c);
			}
		} finally {
			s.close();
		}

		File parent = f.getParentFile();
		Date d = new Date();
		parent.setLastModified(d.getTime());
		synchronized(flist) {
			flist.get(parent.getName())[0] = parent.lastModified();
		}
	}

	public void clean(String[] args, InputStream in, OutputStream out) throws IOException {
		synchronized(flist) {
			long total = 0;
			for (Map.Entry<String, Object[]> entry : flist.entrySet()) {
				total += (Long) entry.getValue()[1];
			}

			List<Object[]> k = new ArrayList<Object[]>(flist.values());
			Collections.sort(k, this);

			int cur = 0;
			while (total > MAX * CLEANRATIO) {
				Object[] kk = k.get(cur);

				String name = (String) kk[2];
				File f = new File(new File(new File(CACHEDIR), name.substring(0, 2)), name);
				//System.out.println("removing " + cur + " " + kk[0] + " " + kk[1] + " " + f.getAbsolutePath());
				rm(f);

				total -= (Long) kk[1];

				flist.remove(name);
				cur++;
			}
		}
	}

	public static void init_flist() {
		synchronized(flist) {
			flist.clear();
			File dir = new File(CACHEDIR);
			try {
				dir.mkdirs();
			} catch (Exception e) {

			}

			for (File d : dir.listFiles()) {
				if (!d.isDirectory()) continue;
				for (File sd : d.listFiles()) {
					if (!sd.isDirectory()) continue;
					long total = 0;
					for (File f : sd.listFiles()) {
						total += f.length();
					}
					//System.out.println(sd.getName());
					flist.put(sd.getName(), new Object[] {sd.lastModified(), total, sd.getName()});
				}
			}
		}
	}

	public int compare(Object[] a, Object[] b) {
		return ((Long) a[0]).compareTo((Long) b[0]);
	}

	public static void rm(File dir) {
		if (dir.isDirectory()) {
			for (File f: dir.listFiles())
			{
				rm(f);
			}
		}
		dir.delete();
	}

	public static void main(String[] args) {
		init_flist();
		System.out.println("ready (" + flist.keySet().size() + " dirs)");

		// different ports for upload and download, another port could be added for the clean command
		Thread upload = null;
		if (PORT_UPLOAD != PORT_DOWNLOAD) {
			Netcache tmp = new Netcache(null, PORT_UPLOAD);
			upload = new Thread(tmp);
			upload.start();
		}

		Netcache tmp = new Netcache(null, PORT_DOWNLOAD);
		tmp.run();
	}
}

