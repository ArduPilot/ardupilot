using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ArdupilotMega
{
    class Delta
    {
        public static void
delta_encode(ref char[] buffer)
        {
            int length = buffer.Length;
            char delta = (char)0;
            char original;
            uint i;
            for (i = 0; i < length; ++i)
            {
                original = buffer[i];
                buffer[i] = (char)(byte)(buffer[i] - delta);
                delta = original;
            }
        }

        public static void
        delta_decode(ref char[] buffer)
        {
            int length = buffer.Length;
            char delta = (char)0;
            uint i;
            for (i = 0; i < length; ++i)
            {
                buffer[i] = (char)(byte)(buffer[i] + delta);
                delta = buffer[i];
            }
        }

    }
}