from setuptools import setup, find_packages
from glob import glob

package_name = 'pybullet_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('pybullet_sim/urdf/*')),
        ('share/' + package_name + '/meshes', glob('pybullet_sim/meshes/*')),
        ('share/' + package_name + '/launch', glob('pybullet_sim/launch/*')),
        ('share/' + package_name + '/config', glob('pybullet_sim/config/*')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu_email@example.com',
    description='Descripción del paquete',
    license='Licencia',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pybullet_node = pybullet_sim.scripts.pybullet_node:main',  # Agregar el nodo de simulacion
            'controller_node = pybullet_sim.scripts.controller_node:main',  # Agregar el nodo de publicación

        ],
    },
)
