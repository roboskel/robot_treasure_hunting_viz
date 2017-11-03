#!/usr/bin/env python
import roslib, rospy
import Tkinter as tk
from robot_treasure_area.srv import *


def init():
    global treasure_location_x, treasure_location_y, radiusPublisher

    rospy.init_node('treasure_gui')

    #TreasurePointResponse  -> input
    rospy.Service('gui_service', TreasurePoint, node_callback)

    root = tk.Tk()
    GameGui(root).pack(fill="both", expand=True)
    root.mainloop()

    
    while not rospy.is_shutdown():
        rospy.spin()

def node_callback(req):
    print req.online

    if req.online == True:
        print 'it is True...'
        choice = 1
        return TreasurePointResponse(choice)


class GameGui(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)

        # reset button and a label to ease the human interaction
        self.label = tk.Label(self, text="Which one do you want to find?", font=20).grid(row=0, column=0, sticky="w", padx=50, pady=0)
        self.reset = tk.Button(self, text="Reset", command = self.press_reset, state="disabled")

        # create four buttons for the missing staff
        self.button1 = tk.Button(self, text="Keys", command = self.press_button1)
        self.button2 = tk.Button(self, text="Glasses", command = self.press_button2)
        self.button3 = tk.Button(self, text="Phone", command = self.press_button3)
        self.button4 = tk.Button(self, text="Pills", command = self.press_button4)

        # put characteristic images to the buttons
        image = tk.PhotoImage(file="img/key.png")
        self.button1.config(image=image)
        self.button1.image = image

        image = tk.PhotoImage(file="img/glasses.png")
        self.button2.config(image=image)
        self.button2.image = image

        image = tk.PhotoImage(file="img/phone.png")
        self.button3.config(image=image)
        self.button3.image = image

        image = tk.PhotoImage(file="img/pills.png")
        self.button4.config(image=image)
        self.button4.image = image

        # lay the widgets on the screen in specific grids
        self.reset.grid(row=0, column=1, sticky="e", padx=50, pady=0)

        self.button1.grid(row=1, column=0, padx=50, pady=50, sticky="wens")
        self.button2.grid(row=1, column=1, padx=50, pady=50, sticky="wens")
        self.button3.grid(row=2, column=0, padx=50, pady=50, sticky="wens")
        self.button4.grid(row=2, column=1, padx=50, pady=50, sticky="wens")

        # the same weight to every row and column when the main window expands
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self.rowconfigure(2, weight=1)


    def calculate(self):
        # get the value from the input widget, convert
        # it to an int, and do a calculation
        try:
            i = int(self.entry.get())
            result = "%s*2=%s" % (i, i*2)
        except ValueError:
            result = "Please enter digits only"

        # set the output widget to have our result
        self.output.configure(text=result)

    def press_button1(self):
        self.change_state("disabled")

        self.reset.config(state="active")

    def press_button2(self):
        self.change_state("disabled")

        self.reset.config(state="active")

    def press_button3(self):
        self.change_state("disabled")

        self.reset.config(state="active")

    def press_button4(self):
        self.change_state("disabled")

        self.reset.config(state="active")

    def press_reset(self):
        self.change_state("active")

        self.reset.config(state="disabled")

    def change_state(self, mystate):
        self.button1.config(state=mystate)
        self.button2.config(state=mystate)
        self.button3.config(state=mystate)
        self.button4.config(state=mystate)



# if this is run as a program (versus being imported),
# create a root window and an instance of our example,
# then start the event loop

if __name__ == "__main__":
    init()