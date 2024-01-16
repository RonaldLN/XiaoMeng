## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['xm_smach' , 'new_lib','smach_compose','smach_common','smach_special'],
    package_dir={'': 'smach_lib'},
)

# setup_args1 = generate_distutils_setup(
#     packages=['xm_arm_nav'],
#     package_dir={'': 'scripts'},
# )
setup(**setup_args)
# setup(**setup_args1)