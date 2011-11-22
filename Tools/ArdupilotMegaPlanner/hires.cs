using System;
using System.Runtime.InteropServices;

namespace hires
{
    public class Stopwatch
    {
        [DllImport("Kernel32.dll")]
        private static extern bool QueryPerformanceCounter(
            out long lpPerformanceCount);

        [DllImport("Kernel32.dll")]
        private static extern bool QueryPerformanceFrequency(
            out long lpFrequency);

        private long start=0;
        private long stop=0;

        // static - so this value used in all instances of 
        private static double frequency = getFrequency();

        // Note this is static- called once, before any constructor!
        private static double getFrequency()
        {
            long tempfrequency;
            QueryPerformanceFrequency(out tempfrequency);
            return tempfrequency; // implicit casting to double from long
        }

        public void Start()
        {
            QueryPerformanceCounter(out start);
        }

        public void Stop()
        {
            QueryPerformanceCounter(out stop);
        }

        public double Elapsed
        {
            get
            {
                return (double)(stop - start) / frequency;
            }
        }
    }
}
