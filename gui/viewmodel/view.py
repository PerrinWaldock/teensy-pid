import tkinter as tk
try:
    from viewmodel import ViewModel
except ImportError:
    from .viewmodel import ViewModel

class View:
    def __init__(self,  vm: ViewModel, root: tk.Tk):
        self.vm = vm
        self.root = root
        self.parameterNames = []
        
        self.addDoubleEntry("kp")
        self.addDoubleEntry("ki")
        self.addDoubleEntry("kd")
        self.addDoubleEntry("loopFrequency", label="Loop Frequency (Hz)")
        
        self.addDoubleEntries("setPoints", selectionName="activeSetPoint", labelformat="Set Point {0} (V)")
        self.addButton("Push Values", lambda: self.update(), column=0)
        self.addButton("Pull Values", lambda: self.reset(), column=1)
        
        # launch a "stability plot" gui to get parameters?
        # launch a "step response" gui to get parameters?
        
        self.addCheckButton("pidActive", label="Feedback Active")
        self.addButton("Calibrate", lambda: vm.calibrate())
        self.addButton("Save Parameters", lambda: vm.saveParameters())
        self.addButton("Load Parameters", lambda: vm.loadParameters())
        self.addButton("View Feedforward Model", lambda: vm.transferPlot())
        self.addButton("Generate Step Response", lambda: vm.stepResponse())
        self.addButton("Generate Stability Plot", lambda: vm.stabilityPlot()) # TODO spawn separate GUI using TopLevel?
        
    def addButton(self, text: str, command: callable, padx: int=5, pady: int=5, column: int=2):
        button = tk.Button(self.root,
                           text=text,
                           command=command,
                           padx=padx,
                           pady=pady)
        button.grid(row=self.findFirstFreeRow(column=column), column=column, padx=padx, pady=pady)
        setattr(self, text, button)
    
    def addCheckButton(self, name: str, label: str | None=None, padx: int=5, pady: int=5, column: int=2):
        if label is None:
            label = name
        boolobj = tk.BooleanVar(self.root, getattr(self.vm, name))
        setattr(self, name, boolobj)
        button = tk.Checkbutton(self.root,
                                text=label,
                                variable=boolobj,
                                command=lambda: updateViewmodel(self, self.vm, name))
        button.grid(padx=padx, pady=pady, column=column, row=self.findFirstFreeRow(column=column))
        setattr(self, name+'button', button)
        
        self.parameterNames.append(name)
    
    #TODO create a class that handles a grid of these things
    def addDoubleEntry(self, name: str, label: str | None=None, padx: int=5, pady: int=5, column: int=0):
        if label is None:
            label = name
        
        row = max(self.findFirstFreeRow(column=column), self.findFirstFreeRow(column=column+1))
        doubleobj = tk.DoubleVar(self.root, getattr(self.vm, name))
        setattr(self, name, doubleobj)
        
        labelobj = tk.Label(self.root, text=label+":")
        labelobj.grid(padx=padx, pady=pady, column=column, row=row, sticky="E")
        setattr(self, name+'label', labelobj)
        
        entryobj = tk.Entry(self.root, textvariable=doubleobj)
        entryobj.grid(padx=padx, pady=pady, column=column+1, row=row)
        setattr(self, name+'entry', labelobj)
        
        self.parameterNames.append(name)
    
    def addDoubleEntries(self, name: str, selectionName: str=None, labelformat: str | None =None, padx: int=5, pady: int=5, column: int=0):
        if labelformat is None:
            labelformat = name+"{0}"
            
        values = getattr(self.vm, name)
        doubleObjects = []
        if selectionName:
            setattr(self, selectionName, tk.IntVar(self.root, getattr(self.vm, selectionName)))
        for ind, value in enumerate(values):
            label = labelformat.format(ind)
        
            row = max(self.findFirstFreeRow(column=column), self.findFirstFreeRow(column=column+1))
            doubleobj = tk.DoubleVar(self.root, value)
            doubleObjects.append(doubleobj)
            
            labeltext = label+":"
            if selectionName:
                labelobj = tk.Radiobutton(self.root, text=labeltext, value=ind, variable=self.activeSetPoint)
            else:            
                labelobj = tk.Label(self.root, text=labeltext)
            labelobj.grid(padx=padx, pady=pady, column=column, row=row, sticky="E")
            setattr(self, label+'label', labelobj)
            
            entryobj = tk.Entry(self.root, textvariable=doubleobj)
            entryobj.grid(padx=padx, pady=pady, column=column+1, row=row)
            setattr(self, label+'entry', labelobj)
                    
        self.parameterNames.append(name)
        self.parameterNames.append("activeSetPoint")
        setattr(self, name, doubleObjects)
                        
    def findFirstFreeRow(self, column: int=0):
        name = f'rows{column}'
        if not hasattr(self, name):
            setattr(self, name, set())
        rows = getattr(self, name)
        
        row = 0
        while row in rows:
            row += 1
        
        rows.add(row)
        return row
        
    
    def update(self):
        for name in self.parameterNames:
            updateViewmodel(self, self.vm, name)
        
    def reset(self):
        for name in self.parameterNames:
            updateView(self, self.vm, name)
    
def updateViewmodel(v: View, vm: ViewModel, name: str):
    if name in dir(v) and name in dir(vm):
        if isiterable(getattr(v, name)):
            value = [x.get() for x in getattr(v, name)]
        else:
            value = getattr(v, name).get()
            
        if getattr(vm, name) != value:
            setattr(vm, name, value)

def updateView(v: View, vm: ViewModel, name: str):
    if name in dir(v) and name in dir(vm):
        values = getattr(vm, name)
        viewItems = getattr(v, name)
        if not isiterable(values):
            values = [values]
            viewItems = [viewItems]
        for viewItem, value in zip(viewItems, values):
            viewItem.set(value)

def isiterable(x):
    try:
        iter(x)
        return True
    except TypeError:
        return False