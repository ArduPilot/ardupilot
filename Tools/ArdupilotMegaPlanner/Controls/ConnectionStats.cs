using System;
using System.Collections.Generic;
using System.Linq;
using System.Reactive.Disposables;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Threading;
using System.Windows.Forms;

namespace ArdupilotMega.Controls
{
    public partial class ConnectionStats : UserControl
    {
        private readonly IMAVLink _mavlink;
        private CompositeDisposable _subscriptionsDisposable;

        public ConnectionStats(IMAVLink comPort)
            : this()
        {
            _mavlink = comPort;

            this.Load += ConnectionStats_Load;
            this.Disposed += (sender, e) => StopUpdates();
        }

        public ConnectionStats()
        {
            InitializeComponent();
        }

        void ConnectionStats_Load(object sender, EventArgs e)
        {
            _subscriptionsDisposable = new CompositeDisposable();

            var packetsReceivedCount = _mavlink.WhenPacketReceived.Scan(0, (x, y) => x + y);
            var packetsLostCount = _mavlink.WhenPacketLost.Scan(0, (x, y) => x + y);

            var bytesReceivedEverySecond = _mavlink.BytesReceived
                .Buffer(TimeSpan.FromSeconds(1))
                .Select(bytes => bytes.Sum());

            var subscriptions = new List<IDisposable>
                                    {
                                        // Total number of packets received
                                        // but only update the text box at 4Hz
                                        packetsReceivedCount
                                            .Sample(TimeSpan.FromMilliseconds(250))
                                            .SubscribeForTextUpdates(txt_PacketsRx),

                                        packetsLostCount
                                            .Sample(TimeSpan.FromMilliseconds(250))
                                            .SubscribeForTextUpdates(txt_PacketsLost),

                                        // Packets per second = total number of packets received over the
                                        // last 3 seconds divided by three
                                        // Do that every second
                                        _mavlink.WhenPacketReceived
                                            .Buffer(TimeSpan.FromSeconds(3), TimeSpan.FromSeconds(1))
                                            .Select(xs => xs.Sum()/3.0)
                                            .ObserveOn(SynchronizationContext.Current)
                                            .Subscribe(x => this.txt_PacketsPerSecond.Text = x.ToString("0")),

                                        // Link quality is a percentage of the number of good packets received
                                        // to the number of packets missed (detected by mavlink seq no.)
                                        // Calculated as an average over the last 3 seconds (non weighted)
                                        // Calculated every second
                                        CombineWithDefault(_mavlink.WhenPacketReceived, _mavlink.WhenPacketLost, Tuple.Create)
                                            .Buffer(TimeSpan.FromSeconds(3), TimeSpan.FromSeconds(1))
                                            .Select(CalculateAverage)
                                            .ObserveOn(SynchronizationContext.Current)
                                            .Subscribe(x => this.txt_LinkQuality.Text = x.ToString("00%")),
                                        
                                        // Bytes per second is the average number of bytes received every second
                                        // sampled for the last 3 seconds
                                        // updated every second
                                        bytesReceivedEverySecond
                                            .Buffer(3, 1)
                                            .Select(xs => (int) xs.Average())
                                            .Select(ToHumanReadableByteCount)
                                            .SubscribeForTextUpdates(txt_BytesPerSecondRx),

                                        // Total bytes received - just count them up,
                                        // but only update the text box at 4Hz so as not to swamp the UI thread
                                        // Also use a human friendly version e.g '1.3K' not 1345
                                        _mavlink.BytesReceived
                                            .Scan(0, (x, y) => x + y)
                                            .Sample(TimeSpan.FromMilliseconds(250))
                                            .Select(ToHumanReadableByteCount)
                                            .SubscribeForTextUpdates(txt_BytesReceived),

                                        _mavlink.BytesSent
                                            .Scan(0, (x, y) => x + y)
                                            .Sample(TimeSpan.FromMilliseconds(250))
                                            .Select(ToHumanReadableByteCount)
                                            .SubscribeForTextUpdates(txt_BytesSent),

                                        _mavlink.BytesSent
                                            .Buffer(TimeSpan.FromSeconds(5), TimeSpan.FromSeconds(1))
                                            .Select(xs => xs.Any() ? xs.Average() : 0)
                                            .Select(x => ToHumanReadableByteCount((int) x))
                                            .SubscribeForTextUpdates(txt_BytesPerSecondSent),

//                                        Observable.Interval(TimeSpan.FromSeconds(1))
//                                            .Scan(TimeSpan.Zero, (a, _) => a.Add(TimeSpan.FromSeconds(1)))
//                                            .ObserveOn(SynchronizationContext.Current)
//                                            .Subscribe(ts => this.txt_TimeConnected.Text = ts.ToString()),

                                        // The maximum length of time between reception of good packets
                                        // evaluated continuously
                                        _mavlink.WhenPacketReceived
                                            .TimeInterval()
                                            .Select(x => x.Interval.Ticks)
                                            .Scan(0L, Math.Max)
                                            .Select(TimeSpan.FromTicks)
                                            .Select(ts => ts.Milliseconds)
                                            .ObserveOn(SynchronizationContext.Current)
                                            .SubscribeForTextUpdates(txt_MaxPacketInterval),
                                            
                                    };

            subscriptions.ForEach(d => _subscriptionsDisposable.Add(d));
        }

        public void StopUpdates()
        {
            _subscriptionsDisposable.Dispose();
        }

        private static IObservable<TResult> CombineWithDefault<TSource, TResult>(IObservable<TSource> first, Subject<TSource> second, Func<TSource, TSource, TResult> resultSelector)
        {
            return Observable.Defer(() =>
                {
                    var foo = new Subject<TResult>();

                    first.Select(x => resultSelector(x, default(TSource))).Subscribe(foo);
                    second.Select(x => resultSelector(default(TSource), x)).Subscribe(foo);

                    return foo;
                });
        }

        private static double CalculateAverage(IList<Tuple<int, int>> xs)
        {
            var packetsReceived = xs.Sum(t => t.Item1);
            var packetsLost = xs.Sum(t => t.Item2);

            return packetsReceived/(packetsReceived + (double)packetsLost);
        }

        private static string ToHumanReadableByteCount(int i)
        {
            if (i > 1024)
                return string.Format("{0:0.00}K", i/ (float)1024);
            if (i > 1024 * 1024)
                return string.Format("{0:0.00}Mb", i / (float)(1024 * 1024));
            return string.Format("{0:####}",i);
        }
    }




    public static class CompositeDisposableEx
    {
        public static IDisposable SubscribeForTextUpdates<T>(this IObservable<T> source, TextBox txtBox)
        {
            return source
                .ObserveOn(SynchronizationContext.Current)
                .Subscribe(x => txtBox.Text = x.ToString());
        }
    }
}
