using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using IronPython.Hosting;

namespace ArdupilotMega
{
    public class Script
    {
        DateTime timeout = DateTime.Now;
        List<string> items = new List<string>();
        static Microsoft.Scripting.Hosting.ScriptEngine engine;
        static Microsoft.Scripting.Hosting.ScriptScope scope;

        // keeps history
        MAVLink.__mavlink_rc_channels_override_t rc = new MAVLink.__mavlink_rc_channels_override_t();

        public Script()
        {
            Dictionary<string, object> options = new Dictionary<string, object>();
            options["Debug"] = true;

            if (engine != null)
                engine.Runtime.Shutdown();

            engine = Python.CreateEngine(options);
            scope = engine.CreateScope();

            scope.SetVariable("cs", MainV2.cs);
            scope.SetVariable("Script", this);
            scope.SetVariable("mavutil", this);

            engine.CreateScriptSourceFromString("print 'hello world from python'").Execute(scope);
            engine.CreateScriptSourceFromString("print cs.roll").Execute(scope);


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

        public object mavlink_connection(string device, int baud = 115200, int source_system = 255,
                       bool write = false, bool append = false,
                       bool robust_parsing = true, bool notimestamps = false, bool input = true)
        {

            return null;
        }

        public object recv_match(string condition = null, string type = null, bool blocking = false)
        {

            return null;
        }

        public void Sleep(int ms)
        {
            System.Threading.Thread.Sleep(ms);
        }

        public void runScript(string script)
        {
            try
            {
                engine.CreateScriptSourceFromString(script).Execute(scope);
            }
            catch (Exception e)
            {
                System.Windows.Forms.CustomMessageBox.Show("Error running script " + e.Message);
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

        public float GetParam(string param)
        {
            if (MainV2.comPort.param[param] != null)
                return (float)MainV2.comPort.param[param];

            return 0.0f;
        }

        public bool ChangeMode(string mode)
        {
            MainV2.comPort.setMode(mode);
            return true;
        }

        public bool WaitFor(string message, int timeout)
        {
            int timein = 0;
            while (!MainV2.cs.message.Contains(message))
            {
                System.Threading.Thread.Sleep(5);
                timein += 5;
                if (timein > timeout)
                    return false;
            }

            return true;
        }

        public bool SendRC(int channel, ushort pwm, bool sendnow)
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

            rc.target_component = MainV2.comPort.compid;
            rc.target_system = MainV2.comPort.sysid;

            if (sendnow)
            {
                MainV2.comPort.sendPacket(rc);
                System.Threading.Thread.Sleep(20);
                MainV2.comPort.sendPacket(rc);
                MainV2.comPort.sendPacket(rc);
            }

            return true;
        }
    }
}