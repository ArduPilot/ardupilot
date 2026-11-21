namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public interface IAPSigrok
    {
        void ObserveSPI(byte transmitted, byte received);
    }
}
