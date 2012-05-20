using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;

namespace ArdupilotMega.Controls.BackstageView
{
    public partial class BackstageView : UserControl
    {
        private Color _buttonsAreaBgColor = Color.White;
        private Color _buttonsAreaPencilColor = Color.DarkGray;
        private Color _selectedTextColor = Color.White;
        private Color _unSelectedTextColor = Color.Gray;
        private Color _highlightColor1 = Color.DarkBlue;
        private Color _highlightColor2 = Color.Blue;

        private readonly List<BackstageViewPage> _pages= new List<BackstageViewPage>();
        private BackstageViewPage _activePage;
        private const int ButtonSpacing = 30;
        private const int ButtonHeight = 30;

        public BackstageViewPage SelectedPage { get { return _activePage; } }
        public List<BackstageViewPage> Pages { get { return _pages; } }

        public BackstageView()
        {
            InitializeComponent();

            this.pnlMenu.Height = this.Height;
            this.pnlPages.Height = this.Height;

            pnlMenu.BackColor = _buttonsAreaBgColor;
            pnlMenu.PencilBorderColor = _buttonsAreaPencilColor;
            pnlMenu.GradColor = this.BackColor;
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
        }


        public override Color BackColor
        {
            get
            {
                return base.BackColor;
            }
            set
            {
                base.BackColor = value;
                UpdateButtons();
                pnlMenu.GradColor = this.BackColor;
            }
        }

        [Description("Background pencil line color for the content region"), Category("Appearance")]
        [DefaultValue(typeof(Color),"DarkGray")]
        public Color ButtonsAreaPencilColor
        {
            get { return _buttonsAreaPencilColor; }
            set
            {
                _buttonsAreaPencilColor = value;
                pnlMenu.PencilBorderColor = _buttonsAreaPencilColor;
                pnlMenu.Invalidate();
                UpdateButtons();
                Invalidate();
            }
        }


        [Description("Background color for the buttons region"), Category("Appearance")]
        [DefaultValue(typeof(Color),"White")]
        public Color ButtonsAreaBgColor
        {
            get { return _buttonsAreaBgColor; }
            set
            {
                _buttonsAreaBgColor = value;
                this.pnlMenu.BackColor = _buttonsAreaBgColor;
                pnlMenu.Invalidate();
                Invalidate();
            }
        }

        [Description("Color for the selector buttons text"), Category("Appearance")]
        [DefaultValue(typeof(Color), "White")]
        public Color SelectedTextColor
        {
            get { return _selectedTextColor; }
            set
            {
                _selectedTextColor = value;
                UpdateButtons();
            }
        }

        [Description("Color for the un selected selector buttons text"), Category("Appearance")]
        [DefaultValue(typeof(Color), "Gray")]
        public Color UnSelectedTextColor
        {
            get { return _unSelectedTextColor; }
            set
            {
                _unSelectedTextColor = value;
                UpdateButtons();
                Invalidate();
            }
        }

        [Description("Color selected button background 1"), Category("Appearance")]
        [DefaultValue(typeof(Color), "DarkBlue")]
        public Color HighlightColor1
        {
            get { return _highlightColor1; }
            set
            {
                _highlightColor1 = value;
                UpdateButtons();
                Invalidate();
            }
        }

        [Description("Color selected button background 2"), Category("Appearance")]
        [DefaultValue(typeof(Color), "Blue")]
        public Color HighlightColor2
        {
            get { return _highlightColor2; }
            set
            {
                _highlightColor2 = value;
                UpdateButtons();
                Invalidate();
            }
        }

       
        private void UpdateButtons()
        {
            foreach (var backstageViewButton in pnlMenu.Controls.OfType<BackstageViewButton>())
            {
                backstageViewButton.HighlightColor2 = _highlightColor2;
                backstageViewButton.HighlightColor1 = _highlightColor1;
                backstageViewButton.UnSelectedTextColor = _unSelectedTextColor;
                backstageViewButton.SelectedTextColor = _selectedTextColor;
                backstageViewButton.ContentPageColor = this.BackColor;
                backstageViewButton.PencilBorderColor = _buttonsAreaPencilColor;

                backstageViewButton.Invalidate();
            }
        }

        public void AddPage(BackstageViewPage page)
        {
            page.Page.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;
            page.Page.Location = new Point(pnlMenu.Width, 0);
            page.Page.Dock = DockStyle.Fill;

            _pages.Add(page);
            CreateLinkButton(page);
            this.pnlPages.Controls.Add(page.Page);

            if (_activePage == null)
            {
                _activePage = page;

                ActivatePage(page);
            }
        }

        private void CreateLinkButton(BackstageViewPage page)
        {
            var lnkButton = new BackstageViewButton
                                {
                                    Text = page.LinkText,
                                    Tag = page,
                                    Top = _pages.IndexOf(page) * ButtonSpacing,
                                    Width = this.pnlMenu.Width,
                                    Height = ButtonHeight,
                                    ContentPageColor = this.BackColor,
                                    PencilBorderColor = _buttonsAreaPencilColor,
                                    SelectedTextColor = _selectedTextColor,
                                    UnSelectedTextColor = _unSelectedTextColor,
                                    HighlightColor1 = _highlightColor1,
                                    HighlightColor2 = _highlightColor2
                                };

            pnlMenu.Controls.Add(lnkButton);
            lnkButton.Click += this.ButtonClick;
            lnkButton.DoubleClick += lnkButton_DoubleClick;
        }

        void lnkButton_DoubleClick(object sender, EventArgs e)
        {
            var backstageViewButton = ((BackstageViewButton)sender);
            var associatedPage = backstageViewButton.Tag as BackstageViewPage;

            Form popoutForm = new Form();
            popoutForm.FormClosing += popoutForm_FormClosing;

            int maxright = 0, maxdown = 0;

            foreach (Control ctl in associatedPage.Page.Controls)
            {
                maxright = Math.Max(ctl.Right, maxright);
                maxdown = Math.Max(ctl.Bottom, maxdown);
            }

            // set the height to 0, so we can derive the header height in the next step
            popoutForm.Height = 0;

            popoutForm.Size = new System.Drawing.Size(maxright + 20, maxdown + 20 + popoutForm.Height);
            popoutForm.Controls.Add(associatedPage.Page);
            popoutForm.Tag = associatedPage;

            popoutForm.Text = associatedPage.LinkText;

            popoutForm.BackColor = this.BackColor;
            popoutForm.ForeColor = this.ForeColor;

            popoutForm.Show(this);
        }

        void popoutForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            // get the page back
            var temp = ((Form)sender).Tag as BackstageViewPage;
            // add back to where it belongs
            this.pnlPages.Controls.Add(temp.Page);

            // clear the controls, so we dont dispose the good control
            ((Form)sender).Controls.Clear();
            
        }


        private void ButtonClick(object sender, EventArgs e)
        {
            var backstageViewButton = ((BackstageViewButton) sender);
            var associatedPage = backstageViewButton.Tag as BackstageViewPage;
            this.ActivatePage(associatedPage);
        }

        public void ActivatePage(BackstageViewPage associatedPage)
        {
            // deactivate the old page
            _activePage.Page.Close();
            Pages.ForEach(x =>
            {
                x.Page.Visible = false;
            });

            _activePage.Page.Visible = false;
            var oldButton = this.pnlMenu.Controls.OfType<BackstageViewButton>().Single(b => b.Tag == _activePage);
            oldButton.IsSelected = false;

            associatedPage.Page.Visible = true;
            var newButton = this.pnlMenu.Controls.OfType<BackstageViewButton>().Single(b => b.Tag == associatedPage);
            newButton.IsSelected = true;

            _activePage = associatedPage;

            _activePage.Page.DoLoad(new EventArgs());
        }

        public void Close()
        {
            foreach (BackstageViewPage page in _pages)
            {
                page.Page.Close();
            }
        }

        public class BackstageViewPage
        {            
            public BackstageViewPage(BackStageViewContentPanel page, string linkText)
            {
                Page = page;
                LinkText = linkText;
            }

            public BackStageViewContentPanel Page { get; set; }
            public string LinkText { get; set; }
        }
    }


  
}
