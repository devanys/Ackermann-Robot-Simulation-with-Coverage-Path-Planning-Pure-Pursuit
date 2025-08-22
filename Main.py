import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import ttk

def f(x, y):
    return np.sin(np.sqrt(x**2 + y**2))
def generate_plot():
    try:
        a = float(entry_xmin.get())
        b = float(entry_xmax.get())
        c = float(entry_ymin.get())
        d = float(entry_ymax.get())
        x = np.linspace(a, b, 100)  
        y = np.linspace(c, d, 100)
        X, Y = np.meshgrid(x, y)
        Z = f(X, Y)
        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(X, Y, Z, cmap='rainbow', edgecolor='none', alpha=0.9)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('f(x, y)')
        ax.set_title('3D Visualization of Definite Integral')
        plt.show()
    except ValueError:
        print("Value Error!")
root = tk.Tk()
root.title("Definite Integral 3D Visualization")
ttk.Label(root, text="𝑥_{min}:").grid(row=0, column=0)
entry_xmin = ttk.Entry(root)
entry_xmin.grid(row=0, column=1)
ttk.Label(root, text="𝑥_{max}:").grid(row=1, column=0)
entry_xmax = ttk.Entry(root)
entry_xmax.grid(row=1, column=1)
ttk.Label(root, text="𝑦_{min}:").grid(row=2, column=0)
entry_ymin = ttk.Entry(root)
entry_ymin.grid(row=2, column=1)
ttk.Label(root, text="𝑦_{max}:").grid(row=3, column=0)
entry_ymax = ttk.Entry(root)
entry_ymax.grid(row=3, column=1)
ttk.Button(root, text="Generate", command=generate_plot).grid(row=4, columnspan=2)
root.mainloop()