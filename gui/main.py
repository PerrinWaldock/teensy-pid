import os
import tkinter as tk
from controller import PidController, VirtualPidController
from viewmodel.viewmodel import ViewModel
from viewmodel.view import View

def main():
    root = tk.Tk()
    icon_path = os.path.join(os.path.dirname(__file__), 'icon.png')
    icon = tk.PhotoImage(file=icon_path)
    root.iconphoto(True, icon)
    # label = ttk.Label(root)
    root.title("Test GUI")
    # root.geometry('600x400+50+50')
    try:
        pc = PidController()
    except:
        pc = VirtualPidController()
    vm = ViewModel(pc)
    view = View(vm, root)
    root.mainloop()
    
if __name__ == '__main__':
    main()