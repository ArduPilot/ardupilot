using System;
using System.Collections.Generic;
using System.ComponentModel;
using Timer=System.Windows.Forms.Timer;

namespace ArducopterConfigurator.PresentationModels
{
    public class MainVm : NotifyProperyChangedBase, IPresentationModel
    {
        private readonly IComms _comms;
        private bool _isConnected;
        private IPresentationModel _selectedVm;
        private string _selectedPort;
        private int _selectedBaudRate;
        private string _apmVersion;
        private Timer _connectionAttemptsTimer;

        private SessionStates _connectionState;


        public MainVm(IComms session)
        {
            _comms = session;
            _comms.LineOfDataReceived += _session_LineOfDataReceived;

            MonitorVms = new BindingList<IPresentationModel>
                             {
                                 new SensorsVm(),
                                 new TransmitterChannelsVm(),
                                 new FlightControlPidsVm(),
                                 new PositionAltitudePidsVm(),
                                 new GpsStatusVm(),

                                 //new MotorCommandsVm(session),
                                 new SerialMonitorVm(),
                             };

            foreach (var vm in MonitorVms)
            {
                if (vm is ItalksToApm)
                    vm.sendTextToApm += MainVm_sendTextToApm;
            }

            ConnectCommand = new DelegateCommand(
                _ => Connect(),
                _ => _connectionState==SessionStates.Disconnected && AvailablePorts.Contains(SelectedPort));

            DisconnectCommand = new DelegateCommand(_ => Disconnect(), _ => IsConnected);

            RefreshPortListCommand = new DelegateCommand(_ => RefreshPorts());

            RestoreDefaultsCommand = new DelegateCommand(_ => RestoreDefaults(), _ => IsConnected);

            WriteToEepromCommand = new DelegateCommand(_ => WriteToEeprom(), _ => IsConnected);

            ConnectionState = SessionStates.Disconnected;

            AvailablePorts = new BindingList<string>();

            AvailableBaudRates = new BindingList<int>() {115200, 57600, 38400, 9600};
            SelectedBaudRate = 115200;

            RefreshPorts();

            // Initially have selected the last discovered com port.
            // I think this is more likely to be the correct one, as 
            // the built in comports come first, then the usb/serial
            // converter ports
            if (AvailablePorts.Count > 0)
                SelectedPort = AvailablePorts[AvailablePorts.Count-1];
        }



        void MainVm_sendTextToApm(object sender, sendTextToApmEventArgs e)
        {
           if (sender.Equals(_selectedVm)==false)
           {
               Console.WriteLine("Non selected vm wants to send something to the APM");
               return;
           }

            if (_comms.IsConnected)
                _comms.Send(e.textToSend);
         
        }

        private void RefreshPorts()
        {
            AvailablePorts.Clear();
            foreach (var c in _comms.ListCommPorts())
                AvailablePorts.Add(c);
        }

        private void RestoreDefaults()
        {
             if (_comms.IsConnected)
                _comms.Send("Y");
        }

        private void WriteToEeprom()
        {
            if (_comms.IsConnected)
                _comms.Send("W");
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
           else if (ConnectionState == SessionStates.Connected)
           {
               if (_selectedVm is ItalksToApm)
                   (_selectedVm as ItalksToApm).handleLineOfText(strRx);
           }
        }

        public ICommand ConnectCommand { get; private set; }

        public ICommand DisconnectCommand { get; private set; }

        public ICommand RefreshPortListCommand { get; private set; }
        
        public ICommand RestoreDefaultsCommand { get; private set; }

        public ICommand WriteToEepromCommand { get; private set; }

        public BindingList<string> AvailablePorts { get; private set; }

        public BindingList<int> AvailableBaudRates { get; private set; }

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


        public int SelectedBaudRate
        {
            get { return _selectedBaudRate; }
            set
            {
                if (_selectedBaudRate != value)
                {
                    _selectedBaudRate = value;
                    FirePropertyChanged("SelectedBaudRate");
                }
            }
        }
      
        public void Connect()
        {
            _comms.CommPort = SelectedPort;
            _comms.BaudRate = SelectedBaudRate;

            // Todo: check the status of this call success/failure
            if (!_comms.Connect())
            {
                ConnectionState = SessionStates.Disconnected;
                ApmVersion = "Error";
                return;
            }

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
            _comms.Send("X");

            // once we connected, then get the version string
            _comms.Send("!");
        }

        public void Disconnect()
        {
            _comms.Send("X");
            _comms.DisConnect();
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

        public BindingList<IPresentationModel> MonitorVms { get; private set; }
        
        public string Name
        {
            get { return  "Arducopter Config"; }
        }

        public void Activate()
        {
            throw new System.NotImplementedException();
        }

        public void DeActivate()
        {
            throw new System.NotImplementedException();
        }

        public event EventHandler updatedByApm;

        public void Select(IPresentationModel vm)
        {
            if (_selectedVm==vm)
                return;

            if (vm == null)
                throw new ArgumentNullException("vm");
            
            if (_selectedVm!=null)
                _selectedVm.DeActivate();

            _selectedVm = vm;
            _selectedVm.Activate();
        }

        public void handleLineOfText(string strRx)
        {
            throw new System.NotImplementedException();
        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;
    }
}