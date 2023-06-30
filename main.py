from task import BBI_Trainer
from config import OPTIONS as config
import os
from tkinter import *
from tkinter import ttk

def main(config):
    # available rat numbers
    rat_numbers = ['0', '2', '4', '6', '7', '8', '9']

    # create window
    root = Tk()
    root.geometry("300x130")

    def checkInput():
        input = entry.get()
        if input in rat_numbers:
            # Destry GUI
            root.destroy()

            # Create directory
            config["save_dir"] = f"./results/new_protocol/rat{input}"
            os.makedirs(config["save_dir"], exist_ok=False)

            # Start Session
            Task = BBI_Trainer(config)
            Task.start_session()
        
    #Create an Entry widget to accept User Input
    label=Label(root, text="Enter Rat Number")
    label.pack(pady=5)

    #Create an Entry widget to accept User Input
    entry= Entry(root, width= 40)
    entry.focus_set()
    entry.pack(pady=5)

    #Create a Button to validate Entry Widget
    ttk.Button(root, text= "OK",width= 20, command=checkInput).pack(pady=20)
    root.mainloop()

if __name__ == "__main__":
    main(config)