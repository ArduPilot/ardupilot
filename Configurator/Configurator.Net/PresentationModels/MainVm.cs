using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO.Ports;
using System.Threading;
using Timer=System.Windows.Forms.Timer;

namespace ArducopterConfigurator.PresentationModels
{
    public class MainVm : NotifyProperyChangedBase, IPresentationModel
    {
        private readonly IComms _session;
        private bool _isConnected;
        private MonitorVm _selectedVm;
        private string _selectedPort;
        private string _apmVersion;
        private Timer _connectionAttemptsTimer;

        private SessionStates _connectionState;


        public MainVm(IComms session)
        {
            _session = session;
            _session.LineOfDataReceived += _session_LineOfDataReceived;

            MonitorVms = new BindingList<MonitorVm>
                             {
                                 new FlightDataVm(session),
                                 new TransmitterChannelsVm(session),
                                 new MotorCommandsVm(session),
                                 new CalibrationOffsetsDataVm(session),
                                 new AcroModeConfigVm(session),
                                 new StableModeConfigVm(session),
                                 new PositionHoldConfigVm(session),
                                 new AltitudeHoldConfigVm(session),
                                 new SerialMonitorVm(session)
                             };

            ConnectCommand = new DelegateCommand(
                _ => Connect(),
                _ => _connectionState==SessionStates.Disconnected && AvailablePorts.Contains(SelectedPort));

            DisconnectCommand = new DelegateCommand(_ => Disconnect(), _ => IsConnected);

            RefreshPortListCommand = new DelegateCommand(_ => RefreshPorts());

            ConnectionState = SessionStates.Disconnected;

            AvailablePorts = new BindingList<string>();
        }

        private void RefreshPorts()
        {
            AvailablePorts.Clear();
            foreach (var c in SerialPort.GetPortNames())
                AvailablePorts.Add(c);

        }

        void _session_LineOfDataReceived(string strRx)
        {
           // If we are waiting for the version string
           if (ConnectionState==SessionStates.Connecting)
           {
               if (strRx.Length > 6)
                   return; // too long for a version string - reject

               // then assume that this is the version string received
               ApmVersion = strRx;
               ConnectionState = SessionStates.Connected;
               _selectedVm.Activate();
           }
        }

        public ICommand ConnectCommand { get; private set; }

        public ICommand DisconnectCommand { get; private set; }

        public ICommand RefreshPortListCommand { get; private set; }

        public BindingList<string> AvailablePorts { get; private set; }

        public enum SessionStates
        {
            Disconnected,
            Connecting,
            Connected,
        }

        public SessionStates ConnectionState
        {
            get { return _connectionState; }
            set
            {
                if (_connectionState != value)
                {
                    _connectionState = value;
                    IsConnected = _connectionState == SessionStates.Connected;
                    FirePropertyChanged("ConnectionState");
                }
            }
        }

        // provided for convenience for binding set from the session state prop
        public bool IsConnected
        {
            get { return _isConnected; }
            private set
            {
                if (_isConnected != value)
                {
                    _isConnected = value;
                    FirePropertyChanged("IsConnected");
                }
            }
        }

        public string SelectedPort
        {
            get { return _selectedPort; }
            set
            {
                if (_selectedPort != value)
                {
                    _selectedPort = value;
                    FirePropertyChanged("SelectedPort");
                }
            }
        }
      
        public void Connect()
        {
            _session.CommPort = SelectedPort;
            
            // Todo: check the status of this call success/failure
            _session.Connect();

            ConnectionState = SessionStates.Connecting;

            _connectionAttemptsTimer = new Timer();
            _connectionAttemptsTimer.Tick += _connectionAttemptsTimer_Tick;
            _connectionAttemptsTimer.Interval = 1000; //milliseconds
            _connectionAttemptsTimer.Start();
        }

        void _connectionAttemptsTimer_Tick(object sender, EventArgs e)
        {
            if (_connectionState != SessionStates.Connecting)
            {
                _connectionAttemptsTimer.Stop();
                return;
            }
            _session.Send("X");

            // once we connected, then get the version string
            _session.Send("!");
        }

        public void Disconnect()
        {
            _session.Send("X");
            _session.DisConnect();
            ConnectionState = SessionStates.Disconnected;
        }

        public string ApmVersion
        {
            get { return _apmVersion; }
            private set
            {
                if (_apmVersion != value)
                {
                    _apmVersion = value;
                    FirePropertyChanged("ApmVersion");
                }
            }
        }

        public BindingList<MonitorVm> MonitorVms { get; private set; }
        
        public string Name
        {
            get { return  "Arducopter Config"; }
        }

        public void Select(MonitorVm vm)
        {
            if (_selectedVm==vm)
                return;

            if (_selectedVm!=null)
                _selectedVm.DeActivate();

            _selectedVm = vm;
            
            _selectedVm.Activate();
        }
    }
}