import sys
print('Python executable:', sys.executable)

# numpy
try:
    import numpy as np
    print('numpy:', np.__version__)
except Exception as e:
    print('numpy import ERROR:', e)

# matplotlib
try:
    import matplotlib
    print('matplotlib:', matplotlib.__version__)
    try:
        print('matplotlib backend:', matplotlib.get_backend())
    except Exception as e:
        print('matplotlib.get_backend ERROR:', e)
except Exception as e:
    print('matplotlib import ERROR:', e)

# Pillow
try:
    from PIL import Image
    print('Pillow: OK')
except Exception as e:
    print('Pillow import ERROR:', e)

# tkinter
try:
    import tkinter
    print('tkinter: OK')
except Exception as e:
    print('tkinter import ERROR:', e)

# simple matplotlib display test (does not show window when run headless)
try:
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot([0,1],[0,1])
    fig.canvas.draw()
    print('matplotlib draw: OK')
except Exception as e:
    print('matplotlib draw ERROR:', e)
