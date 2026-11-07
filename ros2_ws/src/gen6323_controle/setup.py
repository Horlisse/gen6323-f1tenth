import os
from glob import glob

from setuptools import setup

package_name = 'gen6323_controle'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Équipe GEN6323',
    maintainer_email='equipe@uqo.ca',
    description='Nœud de suivi Pure Pursuit pour F1TENTH',
    license='MIT',
    entry_points={'console_scripts': ['suivi_pp = gen6323_controle.suivi_pp:main']},
)
