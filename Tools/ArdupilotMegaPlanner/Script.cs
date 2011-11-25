using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ArdupilotMega
{
    public class Script
    {
        DateTime timeout = DateTime.Now;
        List<string> items = new List<string>();

        // keeps history
        MAVLink.__mavlink_rc_channels_override_t rc = new MAVLink.__mavlink_rc_channels_override_t();

        public Script()
        {
            object thisBoxed = MainV2.cs;
            Type test = thisBoxed.GetType();

            foreach (var field in test.GetProperties())
            {
                // field.Name has the field's name.
                object fieldValue;
                try
                {
                    fieldValue = field.GetValue(thisBoxed, null); // Get value
                }
                catch { continue; }

                // Get the TypeCode enumeration. Multiple types get mapped to a common typecode.
                TypeCode typeCode = Type.GetTypeCode(fieldValue.GetType());

                items.Add(field.Name);
            }
        }        

        public enum Conditional
        {
            NONE = 0,
            LT,
            LTEQ,
            EQ,
            GT,
            GTEQ,
            NEQ
        }

        public bool ChangeParam(string param, float value)
        {
            return MainV2.comPort.setParam(param, value);
        }

        public bool ChangeMode(string mode)
        {
            MainV2.comPort.setMode(mode);
            return true;
        }

        public bool WaitFor(string message)
        {
            while (MainV2.cs.message != message) {
                System.Threading.Thread.Sleep(5);
            }

            return true;
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

        public bool sendRC(int channel, ushort pwm)
        {
            switch (channel)
            {
                case 1:
                    MainV2.cs.rcoverridech1 = pwm;
                    rc.chan1_raw = pwm;
                    break;
                case 2:
                    MainV2.cs.rcoverridech2 = pwm;
                    rc.chan2_raw = pwm;
                    break;
                case 3:
                    MainV2.cs.rcoverridech3 = pwm;
                    rc.chan3_raw = pwm;
                    break;
                case 4:
                    MainV2.cs.rcoverridech4 = pwm;
                    rc.chan4_raw = pwm;
                    break;
                case 5:
                    MainV2.cs.rcoverridech5 = pwm;
                    rc.chan5_raw = pwm;
                    break;
                case 6:
                    MainV2.cs.rcoverridech6 = pwm;
                    rc.chan6_raw = pwm;
                    break;
                case 7:
                    MainV2.cs.rcoverridech7 = pwm;
                    rc.chan7_raw = pwm;
                    break;
                case 8:
                    MainV2.cs.rcoverridech8 = pwm;
                    rc.chan8_raw = pwm;
                    break;
            }

            MainV2.comPort.sendPacket(rc);
            System.Threading.Thread.Sleep(20);
            MainV2.comPort.sendPacket(rc);
            MainV2.comPort.sendPacket(rc);

            return true;
        }

        void convertItemtoMessage()
        {

        }

    }
}
