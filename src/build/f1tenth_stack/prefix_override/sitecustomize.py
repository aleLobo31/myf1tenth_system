import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jaime/Documents/GitHub/dev_jaime/myf1tenth_system/src/install/f1tenth_stack'
