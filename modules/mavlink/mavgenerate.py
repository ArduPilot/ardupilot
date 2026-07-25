#!/usr/bin/env python
"""\
mavgenerate.py is a GUI front-end for mavgen, a python based MAVLink
header generation tool.

Notes:
-----
* 2012-7-16 -- dagoodman
    Working on Mac 10.6.8 darwin, with Python 2.7.1

* 2012-7-17 -- dagoodman
    Only GUI code working on Mac 10.6.8 darwin, with Python 3.2.3
    Working on Windows 7 SP1, with Python 2.7.3 and 3.2.3
    Mavgen doesn't work with Python 3.x yet

* 2012-9-25 -- dagoodman
    Passing error limit into mavgen to make output cleaner.

Copyright 2012 David Goodman (dagoodman@soe.ucsc.edu)
Released under GNU GPL version 3 or later

"""
import os
import re   
import sys

from tkinter import *
import tkinter.filedialog
import tkinter.messagebox

from pymavlink.generator import mavgen
from pymavlink.generator import mavparse

title = "MAVLink Generator"
error_limit = 5


class Application(Frame):
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.pack_propagate(0)
        self.grid( sticky=N+S+E+W)
        self.createWidgets()

    """\
    Creates the gui and all of its content.
    """
    def createWidgets(self):


        #----------------------------------------
        # Create the XML entry

        self.xml_value = StringVar()
        self.xml_label = Label( self, text="XML" )
        self.xml_label.grid(row=0, column = 0)
        self.xml_entry = Entry( self, width = 26, textvariable=self.xml_value )
        self.xml_entry.grid(row=0, column = 1)
        self.xml_button = Button (self, text="Browse", command=self.browseXMLFile)
        self.xml_button.grid(row=0, column = 2)

        #----------------------------------------
        # Create the Out entry

        self.out_value = StringVar()
        self.out_label = Label( self, text="Out" )
        self.out_label.grid(row=1,column = 0)
        self.out_entry = Entry( self, width = 26, textvariable=self.out_value )
        self.out_entry.grid(row=1, column = 1)
        self.out_button = Button (self, text="Browse", command=self.browseOutDirectory)
        self.out_button.grid(row=1, column = 2)

        #----------------------------------------
        # Create the Lang box

        self.language_value = StringVar()
        self.language_choices = mavgen.supportedLanguages
        self.language_label = Label( self, text="Language" )
        self.language_label.grid(row=2, column=0)
        self.language_menu = OptionMenu(self,self.language_value,*self.language_choices)
        self.language_value.set(mavgen.DEFAULT_LANGUAGE)
        self.language_menu.config(width=10)
        self.language_menu.grid(row=2, column=1,sticky=W)

        #----------------------------------------
        # Create the Protocol box

        self.protocol_value = StringVar()
        self.protocol_choices = [mavparse.PROTOCOL_1_0, mavparse.PROTOCOL_2_0]
        self.protocol_label = Label( self, text="Protocol")
        self.protocol_label.grid(row=3, column=0)
        self.protocol_menu = OptionMenu(self,self.protocol_value,*self.protocol_choices)
        self.protocol_value.set(mavgen.DEFAULT_WIRE_PROTOCOL)
        self.protocol_menu.config(width=10)
        self.protocol_menu.grid(row=3, column=1,sticky=W)

        #----------------------------------------
        # Create the Validate box

        self.validate_value = BooleanVar()
        self.validate_label = Label( self, text="Validate")
        self.validate_label.grid(row=4, column=0)
        self.validate_button = Checkbutton(self, variable=self.validate_value, onvalue=True, offvalue=False)
        self.validate_value.set(mavgen.DEFAULT_VALIDATE)
        self.validate_button.config(width=10)
        self.validate_button.grid(row=4, column=1,sticky=W)

        #----------------------------------------
        # Create the Validate Units box

        self.strict_units_value = BooleanVar()
        self.strict_units_label = Label( self, text="Validate Units")
        self.strict_units_label.grid(row=5, column=0)
        self.strict_units_button = Checkbutton(self, variable=self.strict_units_value, onvalue=True, offvalue=False)
        self.strict_units_value.set(mavgen.DEFAULT_STRICT_UNITS)
        self.strict_units_button.config(width=10)
        self.strict_units_button.grid(row=5, column=1,sticky=W)

        #----------------------------------------
        # Create the generate button

        self.generate_button = Button ( self, text="Generate", command=self.generateHeaders)
        self.generate_button.grid(row=6,column=1)

    """\
    Open a file selection window to choose the XML message definition.
    """
    def browseXMLFile(self):
        # TODO Allow specification of multiple XML definitions
        xml_file = tkinter.filedialog.askopenfilename(parent=self, title='Choose a definition file')
        if xml_file != None:
            self.xml_value.set(xml_file)

    """\
    Open a directory selection window to choose an output directory for
    headers.
    """
    def browseOutDirectory(self):
        mavlinkFolder = os.path.dirname(os.path.realpath(__file__))
        out_dir = tkinter.filedialog.askdirectory(parent=self,initialdir=mavlinkFolder,title='Please select an output directory')
        if out_dir != None:
            self.out_value.set(out_dir)

    """\
    Generates the header files and place them in the output directory.
    """
    def generateHeaders(self):
        # Verify settings
        rex = re.compile(".*\\.xml$", re.IGNORECASE)
        if not self.xml_value.get():
            tkinter.messagebox.showerror('Error Generating Headers','An XML message definition file must be specified.')
            return

        if not self.out_value.get():
            tkinter.messagebox.showerror('Error Generating Headers', 'An output directory must be specified.')
            return


        if os.path.isdir(self.out_value.get()):
            if not tkinter.messagebox.askokcancel('Overwrite Headers?','The output directory \'{0}\' already exists. Headers may be overwritten if they already exist.'.format(self.out_value.get())):
                return

        # Generate headers
        opts = mavgen.Opts(self.out_value.get(), wire_protocol=self.protocol_value.get(), language=self.language_value.get(), validate=self.validate_value.get(), error_limit=error_limit, strict_units=self.strict_units_value.get());
        args = [self.xml_value.get()]
        try:
            mavgen.mavgen(opts,args)
            tkinter.messagebox.showinfo('Successfully Generated Headers', 'Headers generated successfully.')

        except Exception as ex:
            exStr = formatErrorMessage(str(ex));
            tkinter.messagebox.showerror('Error Generating Headers','{0!s}'.format(exStr))
            return

"""\
Format the mavgen exceptions by removing 'ERROR: '.
"""
def formatErrorMessage(message):
    reObj = re.compile(r'^(ERROR):\s+',re.M);
    matches = re.findall(reObj, message);
    prefix = ("An error occurred in mavgen:" if len(matches) == 1 else "Errors occurred in mavgen:\n")
    message = re.sub(reObj, '\n', message);

    return prefix + message


# End of Application class
# ---------------------------------

# ---------------------------------
# Start

if __name__ == '__main__':
  app = Application()
  app.master.title(title)
  app.mainloop()
