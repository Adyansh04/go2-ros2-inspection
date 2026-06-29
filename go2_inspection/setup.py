from setuptools import find_packages, setup
package_name = 'go2_inspection'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EE26 team',
    maintainer_email='manasreddyarumalla@gmail.com',
    description='Go2 autonomous object inspection: zone segmentation + viewpoint/360 spin live YOLOE '
               'detection, depth->map localization, dedup, crops, per-zone + facility reports/maps.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'zone_inspector = go2_inspection.zone_inspector:main',
        'inspection_mission = go2_inspection.inspection_mission:main',
        'mission_control_server = go2_inspection.mission_control_server:main',
    ]},
)
