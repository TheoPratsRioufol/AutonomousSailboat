import tkinter as tk
from tkinter import ttk

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Utils import *
from config.Config import *


class HScrollPane(tk.Frame):
    def __init__(self, parent, background=''):
        super().__init__(parent)
        self.canv = tk.Canvas(self, background=background)
        self.canv.config(highlightthickness=0)                 

        self.xbar = ttk.Scrollbar(self, orient=tk.HORIZONTAL)
        self.xbar.config(command=self.canv.xview)                   
        
        self.canv.config(xscrollcommand=self.xbar.set)              
        self.xbar.pack(side=tk.BOTTOM, fill=tk.X)                     
        self.canv.pack(side=tk.TOP, expand=True, fill=tk.BOTH)

        self.canv.bind("<Configure>", self.callConfigure)

    def addContent(self, content):
        self.content = content
        self.canv.create_window(0,0,anchor=tk.NW, window=self.content, tags='frame')

    def callConfigure(self, event=None):
        self.canv.config(height=self.content.winfo_height())
        self.canv.configure(scrollregion=self.canv.bbox('all'))
   

