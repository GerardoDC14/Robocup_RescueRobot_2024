import tkinter as tk
import tk_tools
import threading
import time

def update_gauge():
    value = 0
    while True:
        # dt 
        value = (value + 5) % 100
        gauge.set_value(value)
        time.sleep(1) 

root = tk.Tk()

gauge = tk_tools.RotaryScale(root, max_value=100.0, unit='psi')
gauge.grid()

threading.Thread(target=update_gauge, daemon=True).start()

root.mainloop()
