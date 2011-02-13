using System;

namespace ArducopterConfigurator.PresentationModels
{
    public abstract class DoubleVm<Tvm1,Tvm2> : NotifyProperyChangedBase, IPresentationModel 
        where Tvm1 : IPresentationModel 
        where Tvm2 : IPresentationModel
    {
        protected Tvm1 _vm1;
        protected Tvm2 _vm2;
        protected IPresentationModel _activeVm;


        protected DoubleVm(Tvm1 _vm1, Tvm2 _vm2)
        {
            this._vm1 = _vm1;
            this._vm2 = _vm2;

            _vm1.sendTextToApm += proxyApmTx;
            _vm2.sendTextToApm += proxyApmTx;
        }

        private void proxyApmTx(object sender, sendTextToApmEventArgs e)
        {
            _activeVm = sender as IPresentationModel;

            if (sendTextToApm != null)
                sendTextToApm(this, e);
        }

        public void handleLineOfText(string strRx)
        {
            _activeVm.handleLineOfText(strRx);
        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;


        public abstract string Name { get; }
        
        public void Activate()
        {
            _vm1.updatedByApm += _vm1_updatedByApm;
            _vm1.Activate();
        }

        void _vm1_updatedByApm(object sender, EventArgs e)
        {
            // This is a response to the refresh event being completed on the first
            // vm, so refresh the second. Unsubscribe so that the vms respond
            // individually to refresh commands henceforth
            _vm1.updatedByApm -= _vm1_updatedByApm;
            _vm2.Activate();
        }


        public void DeActivate()
        {
            _vm1.DeActivate();
            _vm2.DeActivate();
        }


        public event EventHandler updatedByApm;

        public Tvm1 Vm1
        {
            get { return _vm1; }
            set
            {
                _vm1 = value;
                FirePropertyChanged("Vm1");
            }
        }


        public Tvm2 Vm2
        {
            get { return _vm2; }
            set
            {
                _vm2 = value;
                FirePropertyChanged("Vm2");
            }
        }
    }
}