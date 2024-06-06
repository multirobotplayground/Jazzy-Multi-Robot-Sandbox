import os
from glob import glob

package_name = 'multi-robot-simulations'

# make sure that ros is able to see where the python launch files are
# any data file can be added here if needed
#
# Launch file names need to end with _launch.py
#
setup(data_files = [(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))])
