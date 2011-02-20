using System;
using System.ComponentModel;
using System.Windows.Forms;
using ArducopterConfigurator.PresentationModels;

namespace ArducopterConfigurator.Views
{
    // cannot be abstract due to vs2008 designer
    public class ViewCommon<T> : UserControl, IView<T> where T : IPresentationModel, INotifyPropertyChanged
    {
        public virtual void SetDataContext(T vm)
        {
        }

        protected void BindButtons(T vm)
        {
            foreach (var c in this.Controls)
                if (c is Button)
                    (c as Button).Click += btn_Click;

            vm.PropertyChanged += CheckCommandStates;
        }

        protected void btn_Click(object sender, EventArgs e)
        {
            var cmd = (sender as Button).Tag as ICommand;

            if (cmd != null)
                if (cmd.CanExecute(null))
                    cmd.Execute(null);
        }

        void CheckCommandStates(object sender, PropertyChangedEventArgs e)
        {
            foreach (var c in this.Controls)
            {
                var btn = c as Button;
                if (btn == null) continue;
                var cmd = btn.Tag as ICommand;
                if (cmd != null)
                    btn.Enabled = cmd.CanExecute(null);
            }
        }

        public Control Control
        {
            get { return this; }
        }
    }
}