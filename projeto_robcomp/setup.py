from setuptools import find_packages, setup

package_name = 'projeto_robcomp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='borg',
    maintainer_email='luisfpp1@al.insper.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'missao1gabarito = projeto_robcomp.missao1gabarito:main', 
            'mobilenet_detector = projeto_robcomp.mobilenet_detector:main',
            'creeper_pub = projeto_robcomp.creeper_pub:main',
            'aruco_detector = projeto_robcomp.aruco_detector:main',
            'projeto = projeto_robcomp.projeto:main'
        ],
    },
)
