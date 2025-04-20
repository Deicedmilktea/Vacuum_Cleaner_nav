import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/reeve/Vacuum_Cleaner_nav/install/vacuum_slam'
