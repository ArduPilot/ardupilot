using System;
using System.Drawing;
using System.Windows.Forms;
using ArdupilotMega.Controls;
using System.Text;
using ArdupilotMega;

namespace System.Windows.Forms
{
    public static class MessageBox
    {
        const int FORM_Y_MARGIN = 10;
        const int FORM_X_MARGIN = 16;

        static DialogResult _state = DialogResult.None;

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
            if (text == null)
                text = "";

            if (caption == null)
                caption = "";

            // ensure we are always in a known state
            _state = DialogResult.None;

            // convert to nice wrapped lines.
            text = AddNewLinesToText(text);
            // get pixel width and height
            Size textSize = TextRenderer.MeasureText(text, SystemFonts.DefaultFont);
            // allow for icon
            if (icon != MessageBoxIcon.None)
                textSize.Width += SystemIcons.Question.Width;

            var msgBoxFrm = new Form
                                {
                                    FormBorderStyle = FormBorderStyle.FixedDialog,
                                    ShowInTaskbar = false,
                                    StartPosition = FormStartPosition.CenterScreen,
                                    Text = caption,
                                    MaximizeBox = false,
                                    MinimizeBox = false,
                                    Width = textSize.Width + 50,
                                    Height = textSize.Height + 100,
                                    TopMost = true,
                                };

            Rectangle screenRectangle = msgBoxFrm.RectangleToScreen(msgBoxFrm.ClientRectangle);
            int titleHeight = screenRectangle.Top - msgBoxFrm.Top;

            var lblMessage = new Label
                                 {
                                     Left = 58,
                                     Top = 15,
                                     Width = textSize.Width + 10,
                                     Height = textSize.Height + 10,
                                     Text = text
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

            // display even if theme fails
            try
            {
                ThemeManager.ApplyThemeTo(msgBoxFrm);
            }
            catch { }

            if (System.Windows.Forms.Application.OpenForms.Count > 0)
            {
                msgBoxFrm.StartPosition = FormStartPosition.Manual;
                Form parentForm = System.Windows.Forms.Application.OpenForms[0];
                // center of first form
                msgBoxFrm.Location = new Point(parentForm.Location.X + parentForm.Width / 2 - msgBoxFrm.Width / 2,
                    parentForm.Location.Y + parentForm.Height / 2 - msgBoxFrm.Height / 2);
                DialogResult test = msgBoxFrm.ShowDialog();
            }
            else
            {
                DialogResult test = msgBoxFrm.ShowDialog();
            }

            DialogResult answer = _state;

            return answer;
        }

        static void msgBoxFrm_FormClosing(object sender, FormClosingEventArgs e)
        {
            throw new NotImplementedException();
        }

        // from http://stackoverflow.com/questions/2512781/winforms-big-paragraph-tooltip/2512895#2512895
        private static int maximumSingleLineTooltipLength = 85;

        private static string AddNewLinesToText(string text)
        {
            if (text.Length < maximumSingleLineTooltipLength)
                return text;
            int lineLength = maximumSingleLineTooltipLength;
            StringBuilder sb = new StringBuilder();
            int currentLinePosition = 0;
            for (int textIndex = 0; textIndex < text.Length; textIndex++)
            {
                // If we have reached the target line length and the next      
                // character is whitespace then begin a new line.   
                if (currentLinePosition >= lineLength &&
                    char.IsWhiteSpace(text[textIndex]))
                {
                    sb.Append(Environment.NewLine);
                    currentLinePosition = 0;
                }
                // If we have just started a new line, skip all the whitespace.    
                if (currentLinePosition == 0)
                    while (textIndex < text.Length && char.IsWhiteSpace(text[textIndex]))
                        textIndex++;
                // Append the next character.     
                if (textIndex < text.Length) sb.Append(text[textIndex]);
                currentLinePosition++;
            }
            return sb.ToString();
        }

        private static void AddButtonsToForm(Form msgBoxFrm, MessageBoxButtons buttons)
        {
            Rectangle screenRectangle = msgBoxFrm.RectangleToScreen(msgBoxFrm.ClientRectangle);
            int titleHeight = screenRectangle.Top - msgBoxFrm.Top;

            var t = Type.GetType("Mono.Runtime");
            if ((t != null))
                titleHeight = 25;

            switch (buttons)
            {
                case MessageBoxButtons.OK:
                    var but = new MyButton
                                  {
                                      Size = new Size(75, 23),
                                      Text = "OK",
                                      Left = msgBoxFrm.Width - 75 - FORM_X_MARGIN,
                                      Top = msgBoxFrm.Height - 23 - FORM_Y_MARGIN - titleHeight
                                  };

                    but.Click += delegate { _state = DialogResult.OK; msgBoxFrm.Close(); };
                    msgBoxFrm.Controls.Add(but);
                    msgBoxFrm.AcceptButton = but;
                    break;

                case MessageBoxButtons.YesNo:

                    if (msgBoxFrm.Width < (75 * 2 + FORM_X_MARGIN * 3))
                        msgBoxFrm.Width = (75 * 2 + FORM_X_MARGIN * 3);

                    var butyes = new MyButton
                    {
                        Size = new Size(75, 23),
                        Text = "Yes",
                        Left = msgBoxFrm.Width - 75 * 2 - FORM_X_MARGIN * 2,
                        Top = msgBoxFrm.Height - 23 - FORM_Y_MARGIN - titleHeight
                    };

                    butyes.Click += delegate { _state = DialogResult.Yes; msgBoxFrm.Close(); };
                    msgBoxFrm.Controls.Add(butyes);
                    msgBoxFrm.AcceptButton = butyes;

                    var butno = new MyButton
                    {
                        Size = new Size(75, 23),
                        Text = "No",
                        Left = msgBoxFrm.Width - 75 - FORM_X_MARGIN,
                        Top = msgBoxFrm.Height - 23 - FORM_Y_MARGIN - titleHeight
                    };

                    butno.Click += delegate { _state = DialogResult.No; msgBoxFrm.Close(); };
                    msgBoxFrm.Controls.Add(butno);
                    msgBoxFrm.CancelButton = butno;
                    break;

                default:
                    throw new NotImplementedException("Only MessageBoxButtons.OK and YesNo supported at this time");
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