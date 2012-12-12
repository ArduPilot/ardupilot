using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using ArdupilotMega.Controls.BackstageView;
using ArdupilotMega.Utilities;

namespace ArdupilotMega.Controls.BackstageView
{
    /// <summary>
    /// A Control to somewhat emulate the 'backstage view' as in MS Office 2010
    /// </summary>
    /// <remarks>
    /// 'Tabs' are added as a control in a <see cref="BackstageViewPage"/>
    /// </remarks>
    public partial class BackstageView : UserControl
    {
        private Color _buttonsAreaBgColor = Color.White;
        private Color _buttonsAreaPencilColor = Color.DarkGray;
        private Color _selectedTextColor = Color.White;
        private Color _unSelectedTextColor = Color.Gray;
        private Color _highlightColor1 = SystemColors.Highlight;
        private Color _highlightColor2 = SystemColors.MenuHighlight;

        private readonly List<BackstageViewItem> _items = new List<BackstageViewItem>();
        private BackstageViewPage _activePage;
        private const int ButtonSpacing = 30;
        private const int ButtonHeight = 30;

        public BackstageViewPage SelectedPage { get { return _activePage; } }
        
        public List<BackstageViewPage> Pages { get { return _items.OfType<BackstageViewPage>().ToList(); } }

        private BackstageViewPage popoutPage = null;

        private int ButtonTopPos = 0;

        public BackstageView()
        {
            InitializeComponent();

            this.pnlMenu.Height = this.Height;
            this.pnlPages.Height = this.Height;

            pnlMenu.BackColor = _buttonsAreaBgColor;
            pnlMenu.PencilBorderColor = _buttonsAreaPencilColor;
            pnlMenu.GradColor = this.BackColor;
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
                UpdateButtonAppearance();
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
                UpdateButtonAppearance();
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
                UpdateButtonAppearance();
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
                UpdateButtonAppearance();
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
                UpdateButtonAppearance();
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
                UpdateButtonAppearance();
                Invalidate();
            }
        }
          
        /// <summary>
        /// Add a page (tab) to this backstage view. Will be added at the end/bottom
        /// </summary>
        public BackstageViewPage AddPage(UserControl userControl, string headerText, BackstageViewPage Parent)
        {
            var page = new BackstageViewPage(userControl, headerText, Parent)
                           {
                               Page =
                                   {
                                       Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top,
                                       Location = new Point(pnlMenu.Width, 0),
                                       Dock = DockStyle.Fill
                                   }
                           };

            _items.Add(page);
         
            //CreateLinkButton(page);
            DrawMenu(page);

            page.Page.Visible = false;
            
            this.pnlPages.Controls.Add(page.Page);

            if (_activePage == null)
            {
                _activePage = page;

                ActivatePage(page);
            }

            return page;
        }

        /// <summary>
        /// Add a spacer to this backstage view. Will be added at the end/bottom
        /// </summary>
        /// <param name="spacerheight">the amount to space by</param>
        public void AddSpacer(int spacerheight)
        {
            _items.Add(new BackstageViewSpacer(spacerheight));
        }

        private void CreateLinkButton(BackstageViewPage page,bool haschild = false, bool child = false)
        {
            string label = page.LinkText;

            if (haschild)
            {
                label = "> " + label;
            }
            if (child)
            {
                label = "+ " + label;
            }

            var lnkButton = new BackstageViewButton
                                {
                                    Text = label,
                                    Tag = page,
                                    Top = ButtonTopPos,
                       // Top = _items.TakeWhile(i => i != page).Sum(i => i.Spacing),
                                    Width = this.pnlMenu.Width,
                                    Height = ButtonHeight,
                                    ContentPageColor = this.BackColor,
                                    PencilBorderColor = _buttonsAreaPencilColor,
                                    SelectedTextColor = _selectedTextColor,
                                    UnSelectedTextColor = _unSelectedTextColor,
                                    HighlightColor1 = _highlightColor1,
                                    HighlightColor2 = _highlightColor2,
                                    //Dock = DockStyle.Bottom
                                };

            pnlMenu.Controls.Add(lnkButton);
            lnkButton.Click += this.ButtonClick;
            lnkButton.DoubleClick += lnkButton_DoubleClick;

            ButtonTopPos += lnkButton.Height;
        }

        private void DrawMenu(BackstageViewPage CurrentPage)
        {
            if (_activePage == CurrentPage)
                return;

            pnlMenu.Visible = false;
            pnlMenu.SuspendLayout();

            pnlMenu.Controls.Clear();

            // reset back to 0
            ButtonTopPos = 0;

            foreach (BackstageViewItem page in _items) 
            {
                if (page.GetType() == typeof(BackstageViewPage))
                {
                    // its a base item. we want it
                    if (((BackstageViewPage)page).Parent == null)
                    {
                        bool children = PageHasChildren(page);

                        CreateLinkButton((BackstageViewPage)page, children);

                        // check for children
                        foreach (BackstageViewItem parentpage in _items)
                        {
                            if (parentpage.GetType() == typeof(BackstageViewPage))
                            {
                                if (((BackstageViewPage)parentpage).Parent == ((BackstageViewPage)page))
                                {
                                    // draw all the siblings
                                    if (((BackstageViewPage)parentpage).Parent == CurrentPage)
                                    {
                                        CreateLinkButton((BackstageViewPage)parentpage,false,true);
                                    }
                                    if (((BackstageViewPage)parentpage).Parent == CurrentPage.Parent)
                                    {
                                        CreateLinkButton((BackstageViewPage)parentpage,false,true);
                                    }
                                }
                            }
                        }
                        continue;
                    }

                }
                else
                {
                    ButtonTopPos += page.Spacing;
                }
            }

            pnlMenu.ResumeLayout(false);
            pnlMenu.PerformLayout();
            pnlMenu.Visible = true;
        }

        private bool PageHasChildren(BackstageViewItem parent)
        {
            // check for children
            foreach (BackstageViewItem child in _items)
            {
                if (child.GetType() == typeof(BackstageViewPage))
                {
                    if (((BackstageViewPage)child).Parent == parent)
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        private void UpdateButtonAppearance()
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

        /*
         * Experimental - double clicking a button will spawn it out into a new form
         * Care must be given to lifecycle here - two pages can now be interacted with 
         * 'simultaneously'
         */ 
        private void lnkButton_DoubleClick(object sender, EventArgs e)
        {
            var backstageViewButton = ((BackstageViewButton)sender);
            var associatedPage = backstageViewButton.Tag as BackstageViewPage;

            var popoutForm = new Form();
            popoutForm.FormClosing += popoutForm_FormClosing;

            int maxright = 0, maxdown = 0;

            foreach (Control ctl in associatedPage.Page.Controls)
            {
                maxright = Math.Max(ctl.Right, maxright);
                maxdown = Math.Max(ctl.Bottom, maxdown);
            }

            // set the height to 0, so we can derive the header height in the next step
            popoutForm.Height = 0;

            popoutForm.Size = new Size(maxright + 20, maxdown + 20 + popoutForm.Height);
            popoutForm.Controls.Add(associatedPage.Page);
            popoutForm.Tag = associatedPage;

            popoutForm.Text = associatedPage.LinkText;

            popoutPage = associatedPage;

            popoutForm.BackColor = this.BackColor;
            popoutForm.ForeColor = this.ForeColor;

            popoutForm.Show(this);
        }

        private void popoutForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            // get the page back
            var temp = ((Form)sender).Tag as BackstageViewPage;

            // add back to where it belongs
            this.pnlPages.Controls.Add(temp.Page);

            // clear the controls, so we dont dispose the good control when it closes
            ((Form)sender).Controls.Clear();
            popoutPage = null;
        }

        private void ButtonClick(object sender, EventArgs e)
        {
            var backstageViewButton = ((BackstageViewButton) sender);
            var associatedPage = backstageViewButton.Tag as BackstageViewPage;
            this.ActivatePage(associatedPage);
        }

        public void ActivatePage(BackstageViewPage associatedPage)
        {
            DrawMenu(associatedPage);

            // Deactivate old page
            if (_activePage.Page is IDeactivate)
            {
                ((IDeactivate)(_activePage.Page)).Deactivate();
            }

            // deactivate the old page - obsolete way of notifying activation
            //_activePage.Page.Close();

            foreach (var p in Pages)
                p.Page.Visible = false;

            // deactivate button
            _activePage.Page.Visible = false;
            try
            { // if the button was on an expanded tab. when we leave it no longer exits
                var oldButton = this.pnlMenu.Controls.OfType<BackstageViewButton>().Single(b => b.Tag == _activePage);
                oldButton.IsSelected = false;
            }
            catch { }            

            // ensure fields have been init - obsolete way of notifying activation
            //associatedPage.Page.DoLoad(new EventArgs());

            // new way of notifying activation. Goal is to get rid of BackStageViewContentPanel
            // so plain old user controls can be added
            if (associatedPage.Page is IActivate)
            {
                ((IActivate)(associatedPage.Page)).Activate();
            }

            // show it
            associatedPage.Page.Visible = true;
            
            var newButton = this.pnlMenu.Controls.OfType<BackstageViewButton>().Single(b => b.Tag == associatedPage);
            newButton.IsSelected = true;

            _activePage = associatedPage;
        }

        public void Close()
        {
            foreach (var page in _items)
            {
                if (popoutPage != null && popoutPage == page)
                    continue;

                if (page is BackstageViewSpacer)
                    continue;

                if (((BackstageViewPage)page).Page is IDeactivate)
                {
                    ((IDeactivate)((BackstageViewPage)(page)).Page).Deactivate();
                }
                else
                {
                    ((BackstageViewPage)page).Page.Dispose();
                }
            }
        }

        public abstract class BackstageViewItem
        {
            public abstract int Spacing { get; set; }
        }

        /// <summary>
        /// Place-holder for a bit of blank space in a <see cref="BackstageView"/>
        /// Used to visually seperate logically related groups of link buttons
        /// </summary>
        public class BackstageViewSpacer : BackstageViewItem
        {
            private int _spacing;

            public BackstageViewSpacer(int spacerheight)
            {
                _spacing = spacerheight;
            }

            // How much (vertical) space the thing takes up in the button menu
            public override int Spacing
            {
                get { return _spacing; }
                set { _spacing = value; }
            }
        }


        /// <summary>
        /// Data structure to hold information about a 'tab' in the <see cref="BackstageView"/>
        /// </summary>
        public class BackstageViewPage : BackstageViewItem
        {
            public BackstageViewPage(UserControl page, string linkText, BackstageViewPage parent)
            {
                Page = page;
                LinkText = linkText;
                Parent = parent;
            }

            /// <summary>
            /// The user content of the tab
            /// </summary>
            public UserControl Page { get; set; }

            /// <summary>
            /// The text to go in the 'tab header'
            /// </summary>
            public string LinkText { get; set; }

            public BackstageViewPage Parent { get; internal set; }

            private int _spaceoverride = -1;

            public override int Spacing
            {
                get { if (_spaceoverride != -1) return _spaceoverride; return ButtonSpacing; }
                set { _spaceoverride = value; }
            }
        }
    }


  
}
