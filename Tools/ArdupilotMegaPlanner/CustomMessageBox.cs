using System;
using System.Drawing;
using System.Windows.Forms;
using ArdupilotMega.Controls;

namespace ArdupilotMega
{
    public static class CustomMessageBox
    {
        const int FORM_Y_MARGIN = 10;
        const int FORM_X_MARGIN = 16;

        public static DialogResult Show(string text)
        {
            return Show(text, string.Empty, MessageBoxButtons.OK, MessageBoxIcon.None);
        }

        public static DialogResult Show(string text, string caption)
        {
            return Show(text, caption, MessageBoxButtons.OK, MessageBoxIcon.None);
        }

        public static DialogResult Show(string text, string caption, MessageBoxButtons buttons)
        {
            return Show(text, caption, buttons, MessageBoxIcon.None);
        }

        public static DialogResult Show(string text, string caption, MessageBoxButtons buttons, MessageBoxIcon icon)
        {
            var msgBoxFrm = new Form
                                {
                                    FormBorderStyle = FormBorderStyle.FixedDialog,
                                    ShowInTaskbar = false,
                                    StartPosition = FormStartPosition.CenterScreen,
                                    Text = caption,
                                    MaximizeBox = false,
                                    MinimizeBox = false,
                                    Width = 400,
                                    Height = 170
                                };

            Rectangle screenRectangle = msgBoxFrm.RectangleToScreen(msgBoxFrm.ClientRectangle);
            int titleHeight = screenRectangle.Top - msgBoxFrm.Top;

            var lblMessage = new Label
                                 {
                                     Left = 58,
                                     Top = 15, 
                                     Width = 300,
                                     Text = text,
                                     AutoSize = true,
                                 };

           

            msgBoxFrm.Controls.Add(lblMessage);

            var actualIcon = getMessageBoxIcon(icon);
            
            if (actualIcon == null)
            {
                lblMessage.Location = new Point(FORM_X_MARGIN, FORM_Y_MARGIN);
            }
            else
            {
                var iconPbox = new PictureBox
                                   {
                                       Image = actualIcon.ToBitmap(),
                                       Location = new Point(FORM_X_MARGIN, FORM_Y_MARGIN)
                                   };
                msgBoxFrm.Controls.Add(iconPbox);
            }


            AddButtonsToForm(msgBoxFrm, buttons);

            ThemeManager.ApplyThemeTo(msgBoxFrm);

            msgBoxFrm.ShowDialog();

            return DialogResult.OK;
        }

        private static void AddButtonsToForm(Form msgBoxFrm, MessageBoxButtons buttons)
        {
            Rectangle screenRectangle = msgBoxFrm.RectangleToScreen(msgBoxFrm.ClientRectangle);
            int titleHeight = screenRectangle.Top - msgBoxFrm.Top;

            switch (buttons)
            {
                case MessageBoxButtons.OK:
                    var but = new CustomButton
                                  {
                                      Size = new Size(75, 23),
                                      Text = "OK",
                                      Left = msgBoxFrm.Width - 75 - FORM_X_MARGIN,
                                      Top = msgBoxFrm.Height - 23 - FORM_Y_MARGIN - titleHeight
                                  };

                    but.Click += delegate { msgBoxFrm.Close(); };
                    msgBoxFrm.Controls.Add(but);
                    break;

                default:
                    throw new NotImplementedException("Only MessageBoxButtons.OK supported at this time");
            }
        }

        /// <summary>
        /// Get system icon for MessageBoxIcon.
        /// </summary>
        /// <param name="icon">The MessageBoxIcon value.</param>
        /// <returns>SystemIcon type Icon.</returns>
        private static Icon getMessageBoxIcon(MessageBoxIcon icon)
        {
            switch (icon)
            {
                case MessageBoxIcon.Asterisk:
                    return SystemIcons.Asterisk;
                case MessageBoxIcon.Error:
                    return SystemIcons.Error;
                case MessageBoxIcon.Exclamation:
                    return SystemIcons.Exclamation;
                case MessageBoxIcon.Question:
                    return SystemIcons.Question;
                default:
                    return null;
            }
        }

    }
}