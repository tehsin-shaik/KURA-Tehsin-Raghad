import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/booster/Workspace/Booster_T1_3v3_Demo/src/install/sound_play'
