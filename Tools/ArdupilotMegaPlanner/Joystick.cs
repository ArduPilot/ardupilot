using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using log4net;
using Microsoft.DirectX.DirectInput;
using System.Reflection;

namespace ArdupilotMega
{
    public class Joystick
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        Device joystick;
        JoystickState state;
        public bool enabled = false;
        byte[] buttonpressed = new byte[128];
        public string name;
        public bool elevons = false;

        public static Joystick self;

        struct JoyChannel
        {
            public int channel;
            public joystickaxis axis;
            public bool reverse;
            public int expo;
        }

        struct JoyButton
        {
            public int buttonno;
            public string mode;
        }

        ~Joystick()
        {
            try
            {
                joystick.Unacquire();
            }
            catch { }
        }

        public Joystick()
        {
            self = this;
        }

        JoyChannel[] JoyChannels = new JoyChannel[9]; // we are base 1
        JoyButton[] JoyButtons = new JoyButton[128]; // base 0

        public static DeviceList getDevices()
        {
            return Manager.GetDevices(DeviceClass.GameControl, EnumDevicesFlags.AttachedOnly);
        }

        public bool start(string name)
        {
            self.name = name;
            DeviceList joysticklist = Manager.GetDevices(DeviceClass.GameControl, EnumDevicesFlags.AttachedOnly);

            bool found = false;

            foreach (DeviceInstance device in joysticklist)
            {
                if (device.ProductName == name)
                {
                    joystick = new Device(device.InstanceGuid);
                    found = true;
                    break;
                }
            }
            if (!found)
                return false;

            joystick.SetDataFormat(DeviceDataFormat.Joystick);

            joystick.Acquire();

            enabled = true;

            System.Threading.Thread t11 = new System.Threading.Thread(new System.Threading.ThreadStart(mainloop)) {
            Name = "Joystick loop",
            Priority = System.Threading.ThreadPriority.AboveNormal,
            IsBackground = true
        };
            t11.Start();

            return true;
        }

        public static joystickaxis getMovingAxis(string name, int threshold)
        {
            self.name = name;
            DeviceList joysticklist = Manager.GetDevices(DeviceClass.GameControl, EnumDevicesFlags.AttachedOnly);

            bool found = false;

            Device joystick = null;

            foreach (DeviceInstance device in joysticklist)
            {
                if (device.ProductName == name)
                {
                    joystick = new Device(device.InstanceGuid);
                    found = true;
                    break;
                }
            }
            if (!found)
                return joystickaxis.ARx;

            joystick.SetDataFormat(DeviceDataFormat.Joystick);

            joystick.Acquire();

            System.Windows.Forms.MessageBox.Show("Please ensure you have calibrated your joystick in Windows first");

            joystick.Poll();

            JoystickState obj = joystick.CurrentJoystickState;
            Hashtable values = new Hashtable();

            Type type = obj.GetType(); 
            PropertyInfo[] properties = type.GetProperties();
            foreach (PropertyInfo property in properties)
            {
                values[property.Name] = int.Parse(property.GetValue(obj, null).ToString());
            }
            values["Slider1"] = obj.GetSlider()[0];
            values["Slider2"] = obj.GetSlider()[1];

            System.Windows.Forms.MessageBox.Show("Please move the joystick axis you want assigned to this function after clicking ok");

            DateTime start = DateTime.Now;

            while (start.AddSeconds(10) > DateTime.Now)
            {
                joystick.Poll();
                JoystickState nextstate = joystick.CurrentJoystickState;

                int[] slider = nextstate.GetSlider();

                type = nextstate.GetType(); 
                properties = type.GetProperties();
                foreach (PropertyInfo property in properties)
                {
                    //Console.WriteLine("Name: " + property.Name + ", Value: " + property.GetValue(obj, null));

                    log.InfoFormat("test name {0} old {1} new {2} ", property.Name, values[property.Name], int.Parse(property.GetValue(nextstate, null).ToString()));
                    log.InfoFormat("{0}  {1}", (int)values[property.Name], (int.Parse(property.GetValue(nextstate, null).ToString()) + threshold));
                    if ((int)values[property.Name] > (int.Parse(property.GetValue(nextstate, null).ToString()) + threshold) ||
                        (int)values[property.Name] < (int.Parse(property.GetValue(nextstate, null).ToString()) - threshold))
                    {
                        log.Info(property.Name);
                        joystick.Unacquire();
                        return (joystickaxis)Enum.Parse(typeof(joystickaxis), property.Name);
                    }
                }

                // slider1
                if ((int)values["Slider1"] > (slider[0] + threshold) ||
                    (int)values["Slider1"] < (slider[0] - threshold))
                {
                    joystick.Unacquire();
                    return joystickaxis.Slider1;
                }

                // slider2
                if ((int)values["Slider2"] > (slider[1] + threshold) ||
                    (int)values["Slider2"] < (slider[1] - threshold))
                {
                    joystick.Unacquire();
                    return joystickaxis.Slider2;
                }
            }

            System.Windows.Forms.MessageBox.Show("No valid option was detected");
            
            return joystickaxis.None;
        }

        public static int getPressedButton(string name)
        {
            self.name = name;
            DeviceList joysticklist = Manager.GetDevices(DeviceClass.GameControl, EnumDevicesFlags.AttachedOnly);

            bool found = false;

            Device joystick = null;

            foreach (DeviceInstance device in joysticklist)
            {
                if (device.ProductName == name)
                {
                    joystick = new Device(device.InstanceGuid);
                    found = true;
                    break;
                }
            }
            if (!found)
                return -1;

            joystick.SetDataFormat(DeviceDataFormat.Joystick);

            joystick.Acquire();

            joystick.Poll();

            System.Windows.Forms.MessageBox.Show("Please press the joystick button you want assigned to this function after clicking ok");

            DateTime start = DateTime.Now;

            while (start.AddSeconds(10) > DateTime.Now)
            {
                joystick.Poll();
                JoystickState nextstate = joystick.CurrentJoystickState;

                byte[] buttons = nextstate.GetButtons();

                for (int a = 0; a < joystick.Caps.NumberButtons; a++)
                {
                    if (buttons[a] > 0)
                        return a;
                }
            }

            System.Windows.Forms.MessageBox.Show("No valid option was detected");

            return -1;
        }

        public void setReverse(int channel, bool reverse)
        {
            JoyChannels[channel].reverse = reverse;
        }

        public void setAxis(int channel, joystickaxis axis)
        {
            JoyChannels[channel].axis = axis;
        }

        public void setChannel(int channel, joystickaxis axis, bool reverse, int expo)
        {
            JoyChannel joy = new JoyChannel();
            joy.axis = axis;
            joy.channel = channel;
            joy.expo = expo;
            joy.reverse = reverse;

            JoyChannels[channel] = joy;
        }

        public void setButton(int arrayoffset,int buttonid,string mode1)
        {
            JoyButtons[arrayoffset] = new JoyButton()
            {
                buttonno = buttonid,
                mode = mode1
            };
        }

        public void changeButton(int buttonid, int newid)
        {
            JoyButtons[buttonid].buttonno = newid;
        }

        int BOOL_TO_SIGN(bool input)
        {
            if (input == true)
            {
                return -1;
            }
            else
            {
                return 1;
            }
        }

        /// <summary>
        /// Updates the rcoverride values and controls the mode changes
        /// </summary>
        void mainloop()
        {
            while (enabled)
            {
                try
                {
                    System.Threading.Thread.Sleep(50);
                    //joystick stuff
                    joystick.Poll();
                    state = joystick.CurrentJoystickState;

                    int[] slider = state.GetSlider();

                    if (elevons)
                    {
//g.channel_roll.set_pwm(BOOL_TO_SIGN(g.reverse_elevons) * (BOOL_TO_SIGN(g.reverse_ch2_elevon) * int(ch2_temp - elevon2_trim) - BOOL_TO_SIGN(g.reverse_ch1_elevon) * int(ch1_temp - elevon1_trim)) / 2 + 1500);
//g.channel_pitch.set_pwm(                                 (BOOL_TO_SIGN(g.reverse_ch2_elevon) * int(ch2_temp - elevon2_trim) + BOOL_TO_SIGN(g.reverse_ch1_elevon) * int(ch1_temp - elevon1_trim)) / 2 + 1500);
                        ushort roll = pickchannel(1, JoyChannels[1].axis, false, JoyChannels[1].expo);
                        ushort pitch = pickchannel(2, JoyChannels[2].axis, false, JoyChannels[2].expo);

                        if (getJoystickAxis(1) != Joystick.joystickaxis.None)
                            MainV2.cs.rcoverridech1 = (ushort)(BOOL_TO_SIGN(JoyChannels[1].reverse) * ((int)(pitch - 1500) - (int)(roll - 1500)) / 2 + 1500);
                        if (getJoystickAxis(2) != Joystick.joystickaxis.None)
                            MainV2.cs.rcoverridech2 = (ushort)(BOOL_TO_SIGN(JoyChannels[2].reverse) * ((int)(pitch - 1500) + (int)(roll - 1500)) / 2 + 1500);
                    }
                    else
                    {
                        if (getJoystickAxis(1) != Joystick.joystickaxis.None)
                            MainV2.cs.rcoverridech1 = pickchannel(1, JoyChannels[1].axis, JoyChannels[1].reverse, JoyChannels[1].expo);//(ushort)(((int)state.Rz / 65.535) + 1000);
                        if (getJoystickAxis(2) != Joystick.joystickaxis.None)
                            MainV2.cs.rcoverridech2 = pickchannel(2, JoyChannels[2].axis, JoyChannels[2].reverse, JoyChannels[2].expo);//(ushort)(((int)state.Y / 65.535) + 1000);
                    }
                    if (getJoystickAxis(3) != Joystick.joystickaxis.None)
                        MainV2.cs.rcoverridech3 = pickchannel(3, JoyChannels[3].axis, JoyChannels[3].reverse, JoyChannels[3].expo);//(ushort)(1000 - ((int)slider[0] / 65.535) + 1000);
                    if (getJoystickAxis(4) != Joystick.joystickaxis.None)
                        MainV2.cs.rcoverridech4 = pickchannel(4, JoyChannels[4].axis, JoyChannels[4].reverse, JoyChannels[4].expo);//(ushort)(((int)state.X / 65.535) + 1000);

                    foreach (JoyButton but in JoyButtons)
                    {
                        if (but.buttonno != -1 && getButtonState(but.buttonno))
                        {
                            string mode = but.mode;
                            MainV2.instance.BeginInvoke((System.Windows.Forms.MethodInvoker)delegate()
                            {
                                try
                                {
                                    MainV2.comPort.setMode(mode); 

                                }
                                catch { System.Windows.Forms.MessageBox.Show("Failed to change Modes"); }
                            });
                        }
                    }

                    //Console.WriteLine("{0} {1} {2} {3}", MainV2.cs.rcoverridech1, MainV2.cs.rcoverridech2, MainV2.cs.rcoverridech3, MainV2.cs.rcoverridech4);
                }
                catch (Exception ex) { log.Info("Joystick thread error "+ex.ToString()); } // so we cant fall out
            }
        }

        public enum joystickaxis
        {
            None,
            Pass,
            ARx,
            ARy,
            ARz,
            AX,
            AY,
            AZ,
            FRx,
            FRy,
            FRz,
            FX,
            FY,
            FZ,
            Rx,
            Ry,
            Rz,
            VRx,
            VRy,
            VRz,
            VX,
            VY,
            VZ,
            X,
            Y,
            Z,
            Slider1,
            Slider2
        }

        const int RESXu = 1024;
        const int RESXul = 1024;
        const int RESXl = 1024;
        const int RESKul = 100;
        /*

        ushort expou(ushort x, ushort k)
        {
          // k*x*x*x + (1-k)*x
          return ((ulong)x*x*x/0x10000*k/(RESXul*RESXul/0x10000) + (RESKul-k)*x+RESKul/2)/RESKul;
        }
        // expo-funktion:
        // ---------------
        // kmplot
        // f(x,k)=exp(ln(x)*k/10) ;P[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
        // f(x,k)=x*x*x*k/10 + x*(1-k/10) ;P[0,1,2,3,4,5,6,7,8,9,10]
        // f(x,k)=x*x*k/10 + x*(1-k/10) ;P[0,1,2,3,4,5,6,7,8,9,10]
        // f(x,k)=1+(x-1)*(x-1)*(x-1)*k/10 + (x-1)*(1-k/10) ;P[0,1,2,3,4,5,6,7,8,9,10]

        short expo(short x, short k)
        {
            if (k == 0) return x;
            short y;
            bool neg = x < 0;
            if (neg) x = -x;
            if (k < 0)
            {
                y = RESXu - expou((ushort)(RESXu - x), (ushort)-k);
            }
            else
            {
                y = expou((ushort)x, (ushort)k);
            }
            return neg ? -y : y;
        }

        */

        public Device AcquireJoystick(string name)
        {
            DeviceList joysticklist = Manager.GetDevices(DeviceClass.GameControl, EnumDevicesFlags.AttachedOnly);

            bool found = false;

            foreach (DeviceInstance device in joysticklist)
            {
                if (device.ProductName == name)
                {
                    joystick = new Device(device.InstanceGuid);
                    found = true;
                    break;
                }
            }

            if (!found)
                return null;

            joystick.SetDataFormat(DeviceDataFormat.Joystick);

            joystick.Acquire();

            return joystick;
        }

        public void UnAcquireJoyStick()
        {
            joystick.Unacquire();
        }

        bool getButtonState(int buttonno)
        {
            byte[] buts = state.GetButtons();

            bool ans = buts[buttonno] > 0 && buttonpressed[buttonno] == 0; // press check + debounce

            buttonpressed[buttonno] = buts[buttonno]; // set only this button

            return ans;
        }

        public int getNumButtons()
        {
            return joystick.Caps.NumberButtons;
        }

        public joystickaxis getJoystickAxis(int channel)
        {
            try
            {
                return JoyChannels[channel].axis;
            }
            catch { return joystickaxis.None; }
        }

        public bool isButtonPressed(int buttonno)
        {
            byte[] buts = state.GetButtons();

            if (buts == null || JoyButtons[buttonno].buttonno < 0)
                return false;

            return buts[JoyButtons[buttonno].buttonno] > 0;
        }

        public ushort getValueForChannel(int channel, string name)
        {
                joystick.Poll();

                state = joystick.CurrentJoystickState;

                ushort ans = pickchannel(channel, JoyChannels[channel].axis, JoyChannels[channel].reverse, JoyChannels[channel].expo);
                log.DebugFormat("{0} = {1} = {2}",channel,ans, state.X);
                return ans;
        }

        ushort pickchannel(int chan, joystickaxis axis, bool rev, int expo)
        {
            int min, max, trim = 0;

            if (MainV2.comPort.param.Count > 0)
            {
                try
                {
                    min = (int)(float)(MainV2.comPort.param["RC" + chan + "_MIN"]);
                    max = (int)(float)(MainV2.comPort.param["RC" + chan + "_MAX"]);
                    trim = (int)(float)(MainV2.comPort.param["RC" + chan + "_TRIM"]);
                }
                catch {
                    min = 1000;
                    max = 2000;
                    trim = 1500;
                }
            }
            else
            {
                min = 1000;
                max = 2000;
                trim = 1500;
            }
            if (chan == 3)
            {
                trim = (min + max) / 2;
//                trim = min; // throttle
            }
            
            int range = Math.Abs(max - min);
            
            int working = 0;

            switch (axis)
            {
                case joystickaxis.None:
                    working = ushort.MaxValue / 2;
                    break;
                case joystickaxis.Pass:
                    working = (int)(((float)(trim - min) / range) * ushort.MaxValue);
                    break;
                case joystickaxis.ARx:
                    working = state.ARx;
                    break;

                case joystickaxis.ARy:
                    working = state.ARy;
                    break;

                case joystickaxis.ARz:
                    working = state.ARz;
                    break;

                case joystickaxis.AX:
                    working = state.AX;
                    break;

                case joystickaxis.AY:
                    working = state.AY;
                    break;

                case joystickaxis.AZ:
                    working = state.AZ;
                    break;

                case joystickaxis.FRx:
                    working = state.FRx;
                    break;

                case joystickaxis.FRy:
                    working = state.FRy;
                    break;

                case joystickaxis.FRz:
                    working = state.FRz;
                    break;

                case joystickaxis.FX:
                    working = state.FX;
                    break;

                case joystickaxis.FY:
                    working = state.FY;
                    break;

                case joystickaxis.FZ:
                    working = state.FZ;
                    break;

                case joystickaxis.Rx:
                    working = state.Rx;
                    break;

                case joystickaxis.Ry:
                    working = state.Ry;
                    break;

                case joystickaxis.Rz:
                    working = state.Rz;
                    break;

                case joystickaxis.VRx:
                    working = state.VRx;
                    break;

                case joystickaxis.VRy:
                    working = state.VRy;
                    break;

                case joystickaxis.VRz:
                    working = state.VRz;
                    break;

                case joystickaxis.VX:
                    working = state.VX;
                    break;

                case joystickaxis.VY:
                    working = state.VY;
                    break;

                case joystickaxis.VZ:
                    working = state.VZ;
                    break;

                case joystickaxis.X:
                    working = state.X;
                    break;

                case joystickaxis.Y:
                    working = state.Y;
                    break;

                case joystickaxis.Z:
                    working = state.Z;
                    break;

                case joystickaxis.Slider1:
                    int[] slider = state.GetSlider();
                    working = slider[0];
                    break;

                case joystickaxis.Slider2:
                    int[] slider1 = state.GetSlider();
                    working = slider1[1];
                    break;
            }
            // between 0 and 65535 - convert to int -500 to 500
            working = (int)(working / 65.535) - 500;

            if (rev)
                working *= -1;

            // calc scale from actualy pwm range
            float scale = range / 1000.0f;

            // save for later
            int raw = working;


            double B = 4 * (expo / 100.0);
            double A = 1 - 0.25*B;

            double t_in = working / 1000.0;
            double t_out = 0;
            double mid = trim / 1000.0;

            t_out = A * (t_in) + B * Math.Pow((t_in), 3);

            t_out = mid + t_out * scale;

//            Console.WriteLine("tin {0} tout {1}",t_in,t_out);

            working = (int)(t_out * 1000);

            if (expo == 0)
            {
                working = (int)(raw) + trim;
            }

            //add limits to movement
            working = Math.Max(min, working);
            working = Math.Min(max, working);

            return (ushort)working;
        }
    }
}
