import os, sys, matplotlib
try:
    import tkinter
    print('TK_OK')
except Exception as e:
    print('TK_ERROR', repr(e))
print('BACKEND=' + matplotlib.get_backend())
print('EXE=' + sys.executable)
print('CWD=' + os.getcwd())
print('ARGS=' + ' '.join(sys.argv[1:]))
print('---FILES IN WORKDIR---')
for fn in sorted(os.listdir(os.getcwd())):
    try:
        print(fn, os.path.getsize(os.path.join(os.getcwd(), fn)) if os.path.isfile(os.path.join(os.getcwd(), fn)) else '<dir>')
    except Exception as ee:
        print(fn, '<ERR>', repr(ee))
