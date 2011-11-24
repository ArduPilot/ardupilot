using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ArdupilotMega
{
    public class Script
    {
        DateTime timeout = DateTime.Now;

        public enum Conditional
        {
            LT = 0,
            LTEQ,
            EQ,
            GT,
            GTEQ
        }

        public bool WaitFor(string item, Conditional cond,double value ,int timeoutms)
        {
            timeout = DateTime.Now;
            while (DateTime.Now < timeout.AddMilliseconds(timeoutms))
            {
                //if (item)
                {

                }
            }

            return false;
        }

    }
}
