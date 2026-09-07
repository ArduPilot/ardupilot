namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public interface IAPSigrok
    {
        void ObserveSPI(byte transmitted, byte received);

        // Analytic edge sources, such as timer PWM outputs, reconstruct their
        // pin levels from peripheral state rather than from byte transactions.
        // They add edges only while holding CaptureLock.
        object CaptureLock { get; }
        bool Capturing { get; }
        long NowNs { get; }
        void RegisterEdgeSource(IAPSigrokEdgeSource source);
        void AddEdge(int channel, bool value, long timeNs);
        void CancelEdges(int channel, long fromNs);

        // A PWM-capable pin is routed both to its timer's waveform source and
        // to the GPIO fan-out, because firmware may drive it either way at
        // runtime.  Only one of them may feed a channel at a time, so a
        // source claims the channel while it is actually driving the pin;
        // GPIO transitions are ignored for a claimed channel, and picked up
        // again from the pin's current level when it is released.
        void SetAnalyticOwned(int channel, bool owned);
    }

    public interface IAPSigrokEdgeSource
    {
        // Both are called with the analyser's CaptureLock held. Restart is
        // called when a capture begins and must emit the current level of
        // every channel at nowNs; Extend must emit every edge up to throughNs.
        void Restart(long nowNs);
        void Extend(long throughNs);
    }
}
