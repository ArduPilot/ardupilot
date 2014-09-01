# File: ADNS3080ImageGrabber.py

import serial
import string
import math
import time
from Tkinter import *
from threading import Timer

comPort = 'COM8'  #default com port
comPortBaud = 115200

class App:
    grid_size = 15
    num_pixels = 30
    image_started = FALSE
    image_current_row = 0;
    ser = serial.Serial()
    pixel_dictionary = {}
        
    def __init__(self, master):

        # set main window's title
        master.title("ADNS3080ImageGrabber")

        frame = Frame(master)
        frame.grid(row=0,column=0)

        self.comPortStr = StringVar()
        self.comPort = Entry(frame,textvariable=self.comPortStr)
        self.comPort.grid(row=0,column=0)
        self.comPort.delete(0, END)
        self.comPort.insert(0,comPort)

        self.button = Button(frame, text="Open", fg="red", command=self.open_serial)
        self.button.grid(row=0,column=1)

        self.entryStr = StringVar()
        self.entry = Entry(frame,textvariable=self.entryStr)
        self.entry.grid(row=0,column=2)
        self.entry.delete(0, END)
        self.entry.insert(0,"I")

        self.send_button = Button(frame, text="Send", command=self.send_to_serial)
        self.send_button.grid(row=0,column=3)

        self.canvas = Canvas(master, width=self.grid_size*self.num_pixels, height=self.grid_size*self.num_pixels)
        self.canvas.grid(row=1)

        ## start attempts to read from serial port
        self.read_loop()

    def __del__(self):
        self.stop_read_loop()

    def open_serial(self):
        # close the serial port
        if( self.ser.isOpen() ):
            try:
                self.ser.close()
            except:
                i=i  # do nothing
        # open the serial port
        try:
            self.ser = serial.Serial(port=self.comPortStr.get(),baudrate=comPortBaud, timeout=1)
            print("serial port '" + self.comPortStr.get() + "' opened!")
        except:
            print("failed to open serial port '" + self.comPortStr.get() + "'")

    def send_to_serial(self):
        if self.ser.isOpen():
            self.ser.write(self.entryStr.get())
            print "sent '" + self.entryStr.get() + "' to " + self.ser.portstr
        else:
            print "Serial port not open!"

    def read_loop(self):
        try:
            self.t.cancel()
        except:
            aVar = 1  # do nothing
        #print("reading")
        if( self.ser.isOpen() ) :
            self.read_from_serial();

        self.t = Timer(0.0,self.read_loop)
        self.t.start()

    def stop_read_loop(self):
        try:
            self.t.cancel()
        except:
            print("failed to cancel timer")
            # do nothing

    def read_from_serial(self):
        if( self.ser.isOpen() ):
            while( self.ser.inWaiting() > 0 ):

                self.line_processed = FALSE
                line = self.ser.readline()
                    
                # process the line read

                if( line.find("-------------------------") == 0 ):
                    self.line_processed = TRUE
                    self.image_started = FALSE
                    self.image_current_row = 0
                
                if( self.image_started == TRUE ):
                    if( self.image_current_row >= self.num_pixels ):
                        self.image_started == FALSE
                    else:
                        words = string.split(line,",")
                        if len(words) >= 30:
                            self.line_processed = TRUE
                            x = 0
                            for v in words:
                                try:
                                    colour = int(v)
                                except:
                                    colour = 0;
                                #self.display_pixel(x,self.image_current_row,colour)
                                self.display_pixel(self.num_pixels-1-self.image_current_row,self.num_pixels-1-x,colour)
                                x += 1
                            self.image_current_row += 1
                        else:
                            print("line " + str(self.image_current_row) + "incomplete (" + str(len(words)) + " of " + str(self.num_pixels) + "), ignoring")
                            #print("bad line: " + line);

                if( line.find("image data") >= 0 ):
                    self.line_processed = TRUE
                    self.image_started = TRUE
                    self.image_current_row = 0
                    # clear canvas
                    #self.canvas.delete(ALL) # remove all items

                #display the line if we couldn't understand it
                if( self.line_processed == FALSE ):
                    print( line )
            
    def display_default_image(self):
        # display the grid
        for x in range(0, self.num_pixels-1):
            for y in range(0, self.num_pixels-1):
                colour = x * y / 3.53
                self.display_pixel(x,y,colour)

    def display_pixel(self, x, y, colour):
        if( x >= 0 and x < self.num_pixels and y >= 0 and y < self.num_pixels ) :
            
            #find the old pixel if it exists and delete it
            if self.pixel_dictionary.has_key(x+y*self.num_pixels) :
                self.old_pixel = self.pixel_dictionary[x+y*self.num_pixels]
                self.canvas.delete(self.old_pixel)
                del(self.old_pixel)
                
            fillColour = "#%02x%02x%02x" % (colour, colour, colour)
            #draw a new pixel and add to pixel_array
            self.new_pixel = self.canvas.create_rectangle(x*self.grid_size, y*self.grid_size, (x+1)*self.grid_size, (y+1)*self.grid_size, fill=fillColour)
            self.pixel_dictionary[x+y*self.num_pixels] = self.new_pixel


## main loop ##

root = Tk()
#root.withdraw()
#serPort = SerialHandler(comPort,comPortBaud)

# create main display
app = App(root)
app.display_default_image()

print("entering main loop!")

root.mainloop()

app.stop_read_loop()

print("exiting")
