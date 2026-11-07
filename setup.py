from setuptools import setup
setup(name='thermalnav_demo',version='0.1.0',packages=['thermalnav'],data_files=[('share/ament_index/resource_index/packages',['resource/thermalnav_demo']),('share/thermalnav_demo',['package.xml']),('share/thermalnav_demo/config',['config/demo.rviz'])],entry_points={'console_scripts':['thermalnav = thermalnav.node:main','thermalnav_doctor = thermalnav.doctor:main']})
