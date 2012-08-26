using System;
using System.Runtime.InteropServices;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using u32 = System.UInt32;
using u16 = System.UInt16;
using u8 = System.Byte;

/// <summary>
/// based off ftp://pserver.samba.org/pub/unpacked/picturebook/avi.c
/// </summary>

public class AviWriter
{
    /*
avi debug: * LIST-root size:1233440040 pos:0
avi debug:      + RIFF-AVI  size:1233440032 pos:0
avi debug:      |    + LIST-hdrl size:310 pos:12
avi debug:      |    |    + avih size:56 pos:24
avi debug:      |    |    + LIST-strl size:124 pos:88
avi debug:      |    |    |    + strh size:64 pos:100
avi debug:      |    |    |    + strf size:40 pos:172
avi debug:      |    |    + LIST-strl size:102 pos:220
avi debug:      |    |    |    + strh size:64 pos:232
avi debug:      |    |    |    + strf size:18 pos:304
avi debug:      |    + JUNK size:1698 pos:330
avi debug:      |    + LIST-movi size:1232936964 pos:2036
avi debug:      |    + idx1 size:501024 pos:1232939008
avi debug: AVIH: 2 stream, flags  HAS_INDEX 
avi debug: stream[0] rate:1000000 scale:33333 samplesize:0
avi debug: stream[0] video(MJPG) 1280x720 24bpp 30.000300fps
     */


    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct riff_head
    {
        [MarshalAs(
                UnmanagedType.ByValArray,
                SizeConst = 4)]
        public char[] riff; /* "RIFF" */
        public u32 size;
        [MarshalAs(
        UnmanagedType.ByValArray,
        SizeConst = 4)]
        public char[] avistr; /* "AVI " */
    };

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct stream_head
    { /* 56 bytes */
        [MarshalAs(
UnmanagedType.ByValArray,
SizeConst = 4)]
        public char[] strh; /* "strh" */
        public u32 size;
        [MarshalAs(
UnmanagedType.ByValArray,
SizeConst = 4)]
        public char[] vids; /* "vids" */
        [MarshalAs(
UnmanagedType.ByValArray,
SizeConst = 4)]
        public char[] codec; /* codec name */
        public u32 flags;
        public u32 reserved1;
        public u32 initialframes;
        public u32 scale; /* 1 */
        public u32 rate; /* in frames per second */
        public u32 start;
        public u32 length; /* what units?? fps*nframes ?? */
        public u32 suggested_bufsize;
        public u32 quality; /* -1 */
        public u32 samplesize;
        public short l;
        public short t;
        public short r;
        public short b;
    };

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct avi_head
    { /* 64 bytes */
        [MarshalAs(
UnmanagedType.ByValArray,
SizeConst = 4)]
        public char[] avih; /* "avih" */
        public u32 size;
        public u32 time; /* microsec per frame? 1e6 / fps ?? */
        public u32 maxbytespersec;
        public u32 reserved1;
        public u32 flags;
        public u32 nframes;
        public u32 initialframes;
        public u32 numstreams; /* 1 */
        public u32 suggested_bufsize;
        public u32 width;
        public u32 height;
        public u32 scale; /* 1 */
        public u32 rate; /* fps */
        public u32 start;
        public u32 length; /* what units?? fps*nframes ?? */
    };
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct list_head
    { /* 12 bytes */
        [MarshalAs(
UnmanagedType.ByValArray,
SizeConst = 4)]
        public char[] list; /* "LIST" */
        public u32 size;
        [MarshalAs(
UnmanagedType.ByValArray,
SizeConst = 4)]
        public char[] type;
    };
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct db_head
    {
        [MarshalAs(
UnmanagedType.ByValArray,
SizeConst = 4)]
        public char[] db; /* "00db" */
        public u32 size;
    };
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct frame_head
    { /* 48 bytes */
        [MarshalAs(
UnmanagedType.ByValArray,
SizeConst = 4)]
        public char[] strf; /* "strf" */
        public u32 size;
        public UInt32 size2; /* repeat of previous field? */
        public Int32 width;
        public Int32 height;
        public Int16 planes; /* 1 */
        public Int16 bitcount; /* 24 */
        [MarshalAs(
UnmanagedType.ByValArray,
SizeConst = 4)]
        public char[] codec; /* MJPG */
        public UInt32 unpackedsize; /* 3 * w * h */
        public Int32 r1;
        public Int32 r2;
        public UInt32 clr_used;
        public UInt32 clr_important;
    };

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct BITMAPINFOHEADER
    {
        public UInt32 biSize;
        public Int32 biWidth;
        public Int32 biHeight;
        public Int16 biPlanes;
        public Int16 biBitCount;
        public UInt32 biCompression;
        public UInt32 biSizeImage;
        public Int32 biXPelsPerMeter;
        public Int32 biYPelsPerMeter;
        public UInt32 biClrUsed;
        public UInt32 biClrImportant;
    }



    static int nframes;
    static uint totalsize;
    System.IO.BufferedStream fd;
    DateTime start = DateTime.MinValue;
    int targetfps = 10;

    public void avi_close()
    {
        if (fd != null)
            fd.Close();
    }

    /* start writing an AVI file */
    public void avi_start(string filename)
    {
        avi_close();

        fd = new BufferedStream(File.Open(filename, FileMode.Create));

        fd.Seek(2048,SeekOrigin.Begin);

        nframes = 0;
        totalsize = 0;
        start = DateTime.Now;
    }


    /* add a jpeg frame to an AVI file */
    public void avi_add(u8[] buf, uint size)
    {
        uint osize = size;
        Console.WriteLine(DateTime.Now.Millisecond + " avi frame");
        db_head db = new db_head { db = "00dc".ToCharArray(), size = size };
        fd.Write(StructureToByteArray(db), 0, Marshal.SizeOf(db));
        fd.Write(buf, 0, (int)size);
        if (size % 2 == 1)
        {
            size++;
            fd.Seek(1, SeekOrigin.Current);
        }
        nframes++;
        totalsize += size;

        if (((DateTime.Now - start).TotalSeconds * targetfps) > nframes)
        {
            avi_add(buf, osize);
            Console.WriteLine("Extra frame");
        }
    }

    void strcpy(ref char[] to,string orig)
    {
        to = orig.ToCharArray();
    }

    /* finish writing the AVI file - filling in the header */
    public void avi_end(int width, int height, int fps)
    {
        targetfps = fps;

        riff_head rh = new riff_head { riff = "RIFF".ToCharArray(), size = 0, avistr = "AVI ".ToCharArray() };
        list_head lh1 = new list_head { list = "LIST".ToCharArray(), size = 0, type = "hdrl".ToCharArray() };
        avi_head ah = new avi_head();
        list_head lh2 = new list_head { list = "LIST".ToCharArray(), size = 0, type = "strl".ToCharArray() };
        stream_head sh = new stream_head();
        frame_head fh = new frame_head();
        list_head junk = new list_head() { list = "JUNK".ToCharArray(), size = 0 };
        list_head lh3 = new list_head { list = "LIST".ToCharArray(), size = 0, type = "movi".ToCharArray() };

        //bzero(&ah, sizeof(ah));
        strcpy(ref ah.avih, "avih");
        ah.time = (u32)(1e6 / fps);
        ah.numstreams = 1;
        //ah.scale = (u32)(1e6 / fps);
        //ah.rate = (u32)fps;
        //ah.length = (u32)(nframes);
        ah.nframes = (u32)(nframes);
        ah.width = (u32)width;
        ah.height = (u32)height;
        ah.flags = 0;
        ah.suggested_bufsize = (u32)(3 * width * height * fps);
        ah.maxbytespersec = (u32)(3 * width * height * fps);

        //bzero(&sh, sizeof(sh));
        strcpy(ref sh.strh, "strh");
        strcpy(ref sh.vids, "vids");
        strcpy(ref sh.codec, "MJPG");
        sh.scale = (u32)(1e6 / fps);
        sh.rate = (u32)1000000;
        sh.length = (u32)(nframes);
        sh.suggested_bufsize = (u32)(3 * width * height * fps);
        unchecked
        {
            sh.quality = (uint)-1;
        }

        //bzero(&fh, sizeof(fh));
        strcpy(ref fh.strf, "strf");
        fh.width = width;
        fh.height = height;
        fh.planes = 1;
        fh.bitcount = 24;
        strcpy(ref fh.codec, "MJPG");
        fh.unpackedsize = (u32)(3 * width * height);

        rh.size = (u32)(Marshal.SizeOf(lh1) + Marshal.SizeOf(ah) + Marshal.SizeOf(lh2) + Marshal.SizeOf(sh) +
            Marshal.SizeOf(fh) + Marshal.SizeOf(lh3) +
            nframes * Marshal.SizeOf((new db_head())) +
            totalsize);
        lh1.size = (u32)(4 + Marshal.SizeOf(ah) + Marshal.SizeOf(lh2) + Marshal.SizeOf(sh) + Marshal.SizeOf(fh));
        ah.size = (u32)(Marshal.SizeOf(ah) - 8);
        lh2.size = (u32)(4 + Marshal.SizeOf(sh) + Marshal.SizeOf(fh));
        sh.size = (u32)(Marshal.SizeOf(sh) - 8);
        fh.size = (u32)(Marshal.SizeOf(fh) - 8);
        fh.size2 = fh.size;
        lh3.size = (u32)(4 +
            nframes * Marshal.SizeOf((new db_head())) +
            totalsize);
        junk.size = 2048 - lh1.size - 12 - 12 - 12 - 4; // junk head, list head, rif head , 4
        long pos = fd.Position;
        fd.Seek(0, SeekOrigin.Begin);

        fd.Write(StructureToByteArray(rh),0, Marshal.SizeOf(rh));
        fd.Write(StructureToByteArray(lh1), 0, Marshal.SizeOf(lh1));
        fd.Write(StructureToByteArray(ah), 0, Marshal.SizeOf(ah));
        fd.Write(StructureToByteArray(lh2), 0, Marshal.SizeOf(lh2));
        fd.Write(StructureToByteArray(sh), 0, Marshal.SizeOf(sh));
        fd.Write(StructureToByteArray(fh), 0, Marshal.SizeOf(fh));
        fd.Write(StructureToByteArray(junk), 0, Marshal.SizeOf(junk));
        fd.Seek(2036, SeekOrigin.Begin);
        fd.Write(StructureToByteArray(lh3), 0, Marshal.SizeOf(lh3));

        fd.Seek(pos, SeekOrigin.Begin);
    }

    byte[] StructureToByteArray(object obj)
    {

        int len = Marshal.SizeOf(obj);

        byte[] arr = new byte[len];

        IntPtr ptr = Marshal.AllocHGlobal(len);

        Marshal.StructureToPtr(obj, ptr, true);

        Marshal.Copy(ptr, arr, 0, len);

        Marshal.FreeHGlobal(ptr);

        return arr;

    }
}