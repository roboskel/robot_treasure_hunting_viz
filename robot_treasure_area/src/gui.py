#!/usr/bin/env python
import roslib, rospy
import Tkinter as tk
from robot_treasure_area.srv import ObjectChoice, ObjectChoiceResponse


def init():

    rospy.init_node('treasure_gui')
    imgPath = rospy.get_param('~img_path')

    # Create a root window and an instance of our gui,
    # then start the event loop
    root = tk.Tk()
    GameGui(root, imgPath).pack(fill="both", expand=True)
    root.mainloop()

    while not rospy.is_shutdown():
        rospy.spin()



class GameGui(tk.Frame):
    def __init__(self, parent, imgPath):
        tk.Frame.__init__(self, parent)

        # Build a reset button and a label to ease the human interaction
        self.label = tk.Label(self, text="Which object do you want to find?", font=20)
        self.reset = tk.Button(self, text="Reset", command = self.press_reset, state="disabled")

        # Create four buttons, one for each missing object
        self.button1 = tk.Button(self, text="Keys", command = lambda: self.pressed(1))
        self.button2 = tk.Button(self, text="Glasses", command = lambda: self.pressed(2))
        self.button3 = tk.Button(self, text="Phone", command = lambda: self.pressed(3))
        self.button4 = tk.Button(self, text="Pills", command = lambda: self.pressed(4))

        # Put characteristic images to the buttons
        image = tk.PhotoImage(file=str(imgPath)+"key.png")
        self.button1.config(image=image)
        self.button1.image = image

        image = tk.PhotoImage(file=str(imgPath)+"glasses.png")
        self.button2.config(image=image)
        self.button2.image = image

        image = tk.PhotoImage(file=str(imgPath)+"phone.png")
        self.button3.config(image=image)
        self.button3.image = image

        image = tk.PhotoImage(file=str(imgPath)+"pills.png")
        self.button4.config(image=image)
        self.button4.image = image

        # Lay the widgets on the screen in specific grids
        self.label.grid(row=0, column=0, sticky="w", padx=50, pady=0)
        self.reset.grid(row=0, column=1, sticky="e", padx=50, pady=0)

        self.button1.grid(row=1, column=0, padx=50, pady=50, sticky="wens")
        self.button2.grid(row=1, column=1, padx=50, pady=50, sticky="wens")
        self.button3.grid(row=2, column=0, padx=50, pady=50, sticky="wens")
        self.button4.grid(row=2, column=1, padx=50, pady=50, sticky="wens")

        # The same weight to every row and column when the main window expands
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self.rowconfigure(2, weight=1)

        self.init_service_client();



    """
    Works as a service client, that will give the human's choice to the service provider.
    """
    def init_service_client(self): 
        rospy.wait_for_service('gui_service')

        try:
            self.node_gui_service = rospy.ServiceProxy('gui_service', ObjectChoice)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    """
    When a button is pressed, it sends the respective bid to the service in order to specify the chosen object.
    @bid: represents the button id , i.e. which button is been pressed
    """
    def pressed(self, bid):

        self.change_state("disabled")

        self.reset.config(state="active")

        # send the choice to the service
        resp = self.node_gui_service(bid)

        if resp.status:
            self.label.config(text="Which object do you want to find?")
        else:
            self.label.config(text="An unexpected error occured. Please try again!" , foreground="red")
            self.press_reset()

    """
    When reset button is pressed, all the buttons are active again.
    """
    def press_reset(self):
        self.change_state("active")

        self.reset.config(state="disabled")

    """
    Change the state of each button.
    @mystate: can take values from <disabled, active, normal>
    """
    def change_state(self, mystate):
        self.button1.config(state=mystate)
        self.button2.config(state=mystate)
        self.button3.config(state=mystate)
        self.button4.config(state=mystate)


# Init rosnode and the GUI.
if __name__ == "__main__":
    init()