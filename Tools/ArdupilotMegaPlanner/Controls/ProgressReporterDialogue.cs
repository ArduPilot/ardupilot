using System;
using System.ComponentModel;
using System.Threading;
using System.Windows.Forms;
using System.Reflection;
using log4net;

namespace ArdupilotMega.Controls
{
    /// <summary>
    /// Form that is shown to the user during a background operation
    /// </summary>
    /// <remarks>
    /// Performs operation excplicitely on a threadpool thread due to 
    /// Mono not playing nice with the BackgroundWorker
    /// </remarks>
    public partial class ProgressReporterDialogue : Form
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        private Exception workerException;
        public ProgressWorkerEventArgs doWorkArgs;

        internal object locker = new object();
        internal int _progress = -1;
        internal string _status = "";

        public delegate void DoWorkEventHandler(object sender, ProgressWorkerEventArgs e);

        // This is the event that will be raised on the BG thread
        public event DoWorkEventHandler DoWork;

        public ProgressReporterDialogue()
        {
            InitializeComponent();
            doWorkArgs = new ProgressWorkerEventArgs();
            this.btnClose.Visible = false;
        }

        /// <summary>
        /// Called at setup - will kick off the background process on a thread pool thread
        /// </summary>
        public void RunBackgroundOperationAsync()
        {
            ThreadPool.QueueUserWorkItem(RunBackgroundOperation);
            this.ShowDialog();
        }

        private void RunBackgroundOperation(object o)
        {
            log.Info("RunBackgroundOperation");

            try
            {
                Thread.CurrentThread.Name = "ProgressReporterDialogue Background thread";
            }
            catch { } // ok on windows - fails on mono

            // mono fix - ensure the dialog is running
            while (this.IsHandleCreated == false)
            {
                System.Threading.Thread.Sleep(100);
            }

            log.Info("Focus ctl");

            this.Invoke((MethodInvoker)delegate
         {
             // if this windows isnt the current active windows, popups inherit the wrong parent.
             this.Focus();
             Application.DoEvents();
         });

            try
            {
                log.Info("DoWork");
                if (this.DoWork != null) this.DoWork(this, doWorkArgs);
                log.Info("DoWork Done");
            }
            catch(Exception e)
            {
                // The background operation thew an exception.
                // Examine the work args, if there is an error, then display that and the exception details
                // Otherwise display 'Unexpected error' and exception details
                timer1.Stop();
                ShowDoneWithError(e, doWorkArgs.ErrorMessage);
                return;
            }

            // stop the timer
            timer1.Stop();
            // run once more to do final message and progressbar
            this.Invoke((MethodInvoker)delegate
            {
                timer1_Tick(null, null);
            });

            if (doWorkArgs.CancelRequested && doWorkArgs.CancelAcknowledged)
            {
                ShowDoneCancelled();
                return;
            }

            if (!string.IsNullOrEmpty(doWorkArgs.ErrorMessage))
            {
                ShowDoneWithError(null, doWorkArgs.ErrorMessage);
                return;
            }

            if (doWorkArgs.CancelRequested)
            {
                ShowDoneWithError(null, "Operation could not cancel");
                return;
            }

            ShowDone();
        }

        // Called as a possible last operation of the bg thread that was cancelled
        // - Hide progress bar 
        // - Set label text
        private void ShowDoneCancelled()
        {
            this.Invoke((MethodInvoker)delegate
            {
                this.progressBar1.Visible = false;
                this.lblProgressMessage.Text = "Cancelled";
                this.btnClose.Visible = true;
            });
        }

        // Called as a possible last operation of the bg thread
        // - Set progress bar to 100%
        // - Wait a little bit to allow the Aero progress animatiom to catch up
        // - Signal that we can close
        private void ShowDone()
        {
            this.Invoke((MethodInvoker) delegate
                {
                    this.progressBar1.Style = ProgressBarStyle.Continuous;
                    this.progressBar1.Value = 100;
                    this.btnCancel.Visible = false;
                    this.btnClose.Visible = false;
                });
           
            Thread.Sleep(1000);

            this.BeginInvoke((MethodInvoker) this.Close);
        }

        // Called as a possible last operation of the bg thread
        // There was an exception on the worker event, so:
        // - Show the error message supplied by the worker, or a default message
        // - Make visible the error icon
        // - Make the progress bar invisible to make room for:
        // - Add the exception details and stack trace in an expansion panel
        // - Change the Cancel button to 'Close', so that the user can look at the exception message a bit
        private void ShowDoneWithError(Exception exception, string doWorkArgs)
        {
            var errMessage = doWorkArgs ?? "There was an unexpected error";
            
            if (this.InvokeRequired)
            {
                this.Invoke((MethodInvoker) delegate
                                                {
                                                    this.Text = "Error";
                                                    this.lblProgressMessage.Left = 65;
                                                    this.lblProgressMessage.Text = errMessage;
                                                    this.imgWarning.Visible = true;
                                                    this.progressBar1.Visible = false;
                                                    this.btnCancel.Visible = false;
                                                    this.btnClose.Visible = true;
                                                    this.linkLabel1.Visible = exception != null;
                                                    this.workerException = exception;
                                                });
            }

        }


        private void btnCancel_Click(object sender, EventArgs e)
        {
            // User wants to cancel - 
            // * Set the text of the Cancel button to 'Close'
            // * Set the cancel button to disabled, will enable it and let the user dismiss the dialogue
            //      when the async operation is complete
            // * Set the status text to 'Cancelling...'
            // * Set the progress bar to marquee, we don't know how long the worker will take to cancel
            // * Signal the worker.
            this.btnCancel.Visible = false;
            this.lblProgressMessage.Text = "Cancelling...";
            this.progressBar1.Style = ProgressBarStyle.Marquee;

            doWorkArgs.CancelRequested = true;
        }


        private void btn_Close_Click(object sender, EventArgs e)
        {
            // we have already cancelled, and this now a 'close' button
            this.Close();
        }
        
        /// <summary>
        /// Called from the BG thread
        /// </summary>
        /// <param name="progress">progress in %, -1 means inderteminate</param>
        /// <param name="status"></param>
        public void UpdateProgressAndStatus(int progress, string status)
        {
            // we don't let the worker update progress when  a cancel has been
            // requested, unless the cancel has been acknowleged, so we know that
            // this progress update pertains to the cancellation cleanup
            if (doWorkArgs.CancelRequested && !doWorkArgs.CancelAcknowledged)
                return;

            lock (locker)
            {
                _progress = progress;
                _status = status;
            }

        }

        private void linkLabel1_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            var message = this.workerException.Message
                          + Environment.NewLine + Environment.NewLine
                          + this.workerException.StackTrace;

            CustomMessageBox.Show(message,"Exception Details",MessageBoxButtons.OK,MessageBoxIcon.Information);
        }

        /// <summary>
        /// prevent using invokes on main update status call "UpdateProgressAndStatus", as this is slow on mono
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void timer1_Tick(object sender, EventArgs e)
        {
            int pgv = -1;
            lock (locker)
            {
                pgv = _progress;
                lblProgressMessage.Text = _status;
            }
            if (pgv == -1)
            {
                this.progressBar1.Style = ProgressBarStyle.Marquee;
            }
            else
            {
                this.progressBar1.Style = ProgressBarStyle.Continuous;
                try
                {
                    this.progressBar1.Value = pgv;
                } // Exception System.ArgumentOutOfRangeException: Value of '-12959800' is not valid for 'Value'. 'Value' should be between 'minimum' and 'maximum'.
                catch { } // clean fail. and ignore, chances are we will hit this again in the next 100 ms
            }
        }

        private void ProgressReporterDialogue_Load(object sender, EventArgs e)
        {
            this.Focus();
        }

    }

    public class ProgressWorkerEventArgs : EventArgs
    {
        public string ErrorMessage;
        public volatile bool CancelRequested;
        public volatile bool CancelAcknowledged;
    }
}
