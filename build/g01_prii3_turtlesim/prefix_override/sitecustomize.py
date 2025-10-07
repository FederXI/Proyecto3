import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/domenico/Tercero/PRYIII/DISA/g1_Proyecto3_ws/install/g01_prii3_turtlesim'
