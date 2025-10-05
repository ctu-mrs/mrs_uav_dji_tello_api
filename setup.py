from setuptools import setup

package_name = 'mrs_uav_dji_tello_api'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tomas Baca',
    maintainer_email='klaxalk@gmail.com',
    description='MRS UAV Tello HW Api',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'api = mrs_uav_dji_tello_api.tellopy_wrapper:main'
        ],
    },
)
