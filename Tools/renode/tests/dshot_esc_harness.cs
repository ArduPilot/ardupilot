// Drives AP_DShotESC through AM32 behaviours for test_dshot_esc_model.py.
// Usage: harness.exe <scenario>; prints "ok" or throws.
using System;
using Antmicro.Renode.Peripherals.Miscellaneous;

public static class Harness
{
    const double FrameS = 0.001;
    static double now;
    static AP_DShotESC esc;

    static ushort Encode(int input, bool bidirectional)
    {
        var value = input << 1;
        var crc = (value ^ (value >> 4) ^ (value >> 8)) & 0xF;
        if(bidirectional)
        {
            crc = ~crc & 0xF;
        }
        return (ushort)((value << 4) | crc);
    }

    static ushort? Send(int input, bool bidirectional = true)
    {
        now += FrameS;
        return esc.Frame(Encode(input, bidirectional), bidirectional, now);
    }

    static ushort? SendFor(int input, double seconds, bool bidirectional = true)
    {
        ushort? reply = null;
        var end = now + seconds;
        while(now < end)
        {
            reply = Send(input, bidirectional);
        }
        return reply;
    }

    // the reply to the sixth command frame is the one after it takes effect
    static ushort? Command(int command)
    {
        ushort? reply = null;
        for(var i = 0; i < 6; i++)
        {
            reply = Send(command);
        }
        Send(0);
        return reply;
    }

    static string State() => esc.State(now);

    static void Check(bool condition, string message)
    {
        if(!condition)
        {
            throw new Exception(message + " [" + State() + "]");
        }
    }

    static bool Armed => State().Contains("armed=True");

    static double Erpm(ushort? reply)
    {
        Check(reply.HasValue, "no reply");
        var value = reply.Value >> 4;
        Check((reply.Value & 0xF) == (~(value ^ (value >> 4) ^ (value >> 8)) & 0xF), "reply CRC");
        if(value == 0xFFF)
        {
            return 0;
        }
        Check(!IsExtended(value), "EDT frame where eRPM expected");
        var period = (value & 0x1FF) << (value >> 9);
        return 60000000.0 / period;
    }

    static bool IsExtended(int value) => (value & 0x100) == 0 && (value >> 9) != 0;

    static void Arm()
    {
        SendFor(0, 1.2);
        Check(Armed, "did not arm");
    }

    static void Run(string scenario)
    {
        now = 0;
        esc = new AP_DShotESC(14, 1000, 16.8, 30);
        switch(scenario)
        {
        case "arming":
            SendFor(0, 0.9);
            Check(!Armed, "armed before a second of zero input");
            SendFor(0, 0.2);
            Check(Armed, "not armed after a second of zero input");
            esc = new AP_DShotESC(14, 1000, 16.8, 30);
            now = 0;
            Send(0);
            now += 1.0;
            Send(0);
            Check(!Armed, "armed on two zero frames a second apart");
            esc = new AP_DShotESC(14, 1000, 16.8, 30);
            SendFor(13, 1.2);
            Check(Armed, "a command-only stream did not arm");
            esc = new AP_DShotESC(14, 1000, 16.8, 30);
            SendFor(0, 0.8);
            SendFor(500, 0.1);
            SendFor(0, 0.5);
            Check(!Armed, "throttle did not restart the arming wait");
            break;
        case "throttle":
            Arm();
            var erpm = Erpm(SendFor(1047, 0.5));
            // half throttle, 1000KV on 16.8V, 7 pole pairs
            Check(Math.Abs(erpm - 0.5 * 1000 * 16.8 * 7) < 0.02 * 0.5 * 1000 * 16.8 * 7,
                "eRPM " + erpm);
            Check(Erpm(SendFor(0, 0.5)) == 0, "motor did not stop");
            break;
        case "edt":
            Arm();
            var init = Command(13);
            Check(init.HasValue && init.Value >> 4 == 0xE00, "no EDT init frame");
            SendFor(1047, 0.5);
            int current = 0, voltage = 0, temperature = 0;
            var lastExtended = true;
            for(var i = 0; i < 1000; i++)
            {
                var value = Send(1047).Value >> 4;
                var extended = IsExtended(value);
                Check(!(extended && lastExtended), "two EDT frames in a row");
                lastExtended = extended;
                if(!extended)
                {
                    continue;
                }
                switch(value >> 8)
                {
                case 0x6: current = value & 0xFF; break;
                case 0x4: voltage = value & 0xFF; break;
                case 0x2: temperature = value & 0xFF; break;
                }
            }
            Check(current > 0, "no EDT current while spinning");
            Check(Math.Abs(voltage * 0.25 - 16.8) < 0.5, "EDT voltage " + voltage * 0.25);
            Check(temperature >= 25, "EDT temperature " + temperature);
            SendFor(0, 0.5);
            var deinit = Command(14);
            Check(deinit.HasValue && deinit.Value >> 4 == 0xEFF, "no EDT deinit frame");
            break;
        case "commands":
            SendFor(13, 0.2);
            Check(!State().Contains("edt=True"), "command accepted while disarmed");
            Arm();
            for(var i = 0; i < 5; i++)
            {
                Send(13);
            }
            Send(0);
            Check(!State().Contains("edt=True"), "command accepted after five repeats");
            Command(13);
            Check(State().Contains("edt=True"), "command not accepted after six repeats");
            Command(8);
            Command(21);
            Check(State().Contains("reversed=True") && State().Contains("forward=True"),
                "command 21 not relative to the configured direction");
            Command(20);
            Check(State().Contains("forward=False"), "command 20 not relative to the configured direction");
            SendFor(1047, 0.3);
            Command(7);
            Check(State().Contains("reversed=True"), "command accepted while spinning");
            break;
        case "signal-loss":
            Arm();
            Command(13);
            SendFor(1047, 0.3);
            now += 0.6;
            Check(!Armed, "still armed after 0.6s without signal");
            Check(!State().Contains("edt=True"), "EDT survived the reset");
            Check(Erpm(SendFor(1047, 0.3)) == 0, "throttle accepted after reset");
            esc = new AP_DShotESC(14, 1000, 16.8, 30);
            now = 0;
            SendFor(0, 0.5);
            // a frame would re-detect bidirectional mode, so only look
            now += 1.5;
            Check(State().Contains("bidir=True"), "disarmed ESC reset within 2s");
            now += 0.6;
            Check(State().Contains("bidir=False"), "disarmed ESC not reset after 2s");
            break;
        case "settings":
            Arm();
            Command(8);
            Command(10);
            now += 0.6;
            Send(0);
            Check(State().Contains("reversed=False") && State().Contains("3d=False"),
                "unsaved settings survived a reset");
            Arm();
            Command(8);
            Command(10);
            Command(12);
            now += 0.6;
            Send(0);
            Check(State().Contains("reversed=True") && State().Contains("3d=True"),
                "saved settings lost on reset");
            break;
        case "bidirectional-latch":
            SendFor(0, 1.2, false);
            Check(Armed, "normal DShot did not arm");
            Check(!Send(0, true).HasValue, "replied after switching to inverted CRC while armed");
            Check(State().Contains("bidir=False"), "bidirectional mode changed while armed");
            break;
        case "3d":
            Arm();
            Command(10);
            // 48 and 1048 are neutral; 1047 is full reverse and 2047 full forward
            Check(Erpm(SendFor(1049, 0.5)) < 0.01 * 1000 * 16.8 * 7, "near-neutral forward spins fast");
            Check(Erpm(SendFor(50, 0.5)) < 0.01 * 1000 * 16.8 * 7, "near-neutral reverse spins fast");
            Check(State().Contains("forward=False"), "reverse range did not reverse");
            var full = Erpm(SendFor(2047, 0.5));
            Check(full > 0.9 * 1000 * 16.8 * 7, "full forward eRPM " + full);
            Check(State().Contains("forward=True"), "forward range did not run forward");
            // reversing at speed coasts until slow
            Send(1047);
            Check(State().Contains("forward=True"), "reversed while spinning fast");
            var reverse = Erpm(SendFor(1047, 1.0));
            Check(State().Contains("forward=False"), "did not reverse once slow");
            Check(reverse > 0.9 * 1000 * 16.8 * 7, "full reverse eRPM " + reverse);
            break;
        default:
            throw new Exception("unknown scenario " + scenario);
        }
        Console.WriteLine("ok");
    }

    public static int Main(string[] args)
    {
        Run(args[0]);
        return 0;
    }
}
