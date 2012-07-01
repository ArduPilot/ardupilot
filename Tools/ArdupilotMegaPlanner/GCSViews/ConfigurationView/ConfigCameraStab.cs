using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;
using ArdupilotMega.Presenter;
using Transitions;

namespace ArdupilotMega.GCSViews.ConfigurationView
{
    public partial class ConfigCameraStab : BackStageViewContentPanel
    {
        private ConfigCameraStabPresenter _presenter;
        private Transition[] _ErrorTransition;
        private Transition _NoErrorTransition;

        public ConfigCameraStab()
        {
            InitializeComponent();
            PBOX_WarningIcon.Opacity = 0.0F;
            LBL_Error.Opacity = 0.0F;
        }

        private void ConfigCameraStab_Load(object sender, EventArgs ev)
        {
            _presenter = new ConfigCameraStabPresenter(MainV2.comPort);
            presenterBindingSource.DataSource = _presenter;

            var delay = new Transition(new TransitionType_Linear(2000));
            var fadeIn = new Transition(new TransitionType_Linear(800));
            fadeIn.add(PBOX_WarningIcon, "Opacity", 1.0F);
            fadeIn.add(LBL_Error, "Opacity", 1.0F);
           
            _ErrorTransition = new[] { delay, fadeIn };

            _NoErrorTransition = new Transition(new TransitionType_Linear(10));
            _NoErrorTransition.add(PBOX_WarningIcon, "Opacity", 0.0F);
            _NoErrorTransition.add(LBL_Error, "Opacity", 0.0F);
             
            //setup button actions
            foreach (var btn in Controls.Cast<Control>().OfType<Button>())
                btn.Click += HandleButtonClick;


            _presenter.PropertyChanged += (s, e) =>
                    {
                        if (e.PropertyName == "HasError")
                        {
                            SetErrorMessageOpacity();
                        }
                    };
            
            _presenter.PropertyChanged += CheckCommandStates;
            LNK_wiki.MouseEnter += (s, e) => FadeLinkTo((LinkLabel)s, Color.CornflowerBlue);
            LNK_wiki.MouseLeave += (s, e) => FadeLinkTo((LinkLabel)s, Color.WhiteSmoke);

            SetErrorMessageOpacity();

            // Fix for mono bug where binding sources do not respect INPC notifications on POCOs
            if (MainV2.MONO)
            {
                _presenter.PropertyChanged += (s, e) => presenterBindingSource.ResetBindings(false);
            }

            _presenter.Load();
        }

        private void SetErrorMessageOpacity()
        {
            if (_presenter.HasError)
            {
                // Todo - is this the prob? maybe single log trasition
                var t = new Transition(new TransitionType_Acceleration(1000));
                t.add(PBOX_WarningIcon, "Opacity", 1.0F);
                t.add(LBL_Error, "Opacity", 1.0F);
                t.run();

                //Transition.runChain(_ErrorTransition);
            }
            else
            {
                _NoErrorTransition.run();
            }
        }

        private static void FadeLinkTo(LinkLabel l, Color c)
        {
            var changeColorTransition = new Transition(new TransitionType_Linear(300));
            changeColorTransition.add(l, "LinkColor", c);
            changeColorTransition.run();
        }

        // Common handler for all buttons
        // Will execute an ICommand if one is found on the button Tag
        private static void HandleButtonClick(object sender, EventArgs e)
        {
            if (sender is Button)
            {
                var cmd = (sender as Button).Tag as ICommand;

                if (cmd != null)
                    if (cmd.CanExecute(null))
                        cmd.Execute(null);
            }
        }

        // Something has changed on the presenter - This may be an Icommand
        // enabled state, so update the buttons as appropriate
        void CheckCommandStates(object sender, PropertyChangedEventArgs propertyChangedEventArgs)
        {
            foreach (var btn in Controls.Cast<Control>().OfType<Button>())
            {
                var cmd = btn.Tag as ICommand;
                if (cmd != null)
                    btn.Enabled = cmd.CanExecute(null);
            }
        }

        private void LNK_Wiki_Clicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            Process.Start(new ProcessStartInfo("http://code.google.com/p/arducopter/wiki/AC2_Camera"));
        }
    }
}