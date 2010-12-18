using System;
using System.Windows.Forms;
using ArducopterConfigurator.PresentationModels;

namespace ArducopterConfigurator.Views
{
    // cannot be abstract due to vs2008 designer
    public class ViewCommon<T> : UserControl, IView<T> where T : IPresentationModel
    {
        protected T _vm;

        public virtual void SetDataContext(T model)
        {
        }

        protected void BindButtons()
        {
            foreach (var c in this.Controls)
                if (c is Button)
                    (c as Button).Click += btn_Click;
        }

        protected void btn_Click(object sender, EventArgs e)
        {
            var cmd = (sender as Button).Tag as ICommand;

            if (cmd != null)
                if (cmd.CanExecute(null))
                    cmd.Execute(null);
        }

        public Control Control
        {
            get { return this; }
        }
    }
}