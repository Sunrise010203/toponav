from setuptools import setup

setup(name='toponav_bridge',version='0.1.0',packages=['toponav_bridge'],
      data_files=[('share/ament_index/resource_index/packages',['resource/toponav_bridge']),
                  ('share/toponav_bridge',['package.xml']),
                  ('share/toponav_bridge/launch',['launch/continuous_route.launch.py'])],
      install_requires=['setuptools'],zip_safe=True,
      maintainer='sunrise',maintainer_email='sunrise@example.invalid',
      description='Restricted RGB continuous navigation ROS2 simulation bridge',license='Proprietary',
      entry_points={'console_scripts':['bridge_node = toponav_bridge.nodes:main',
                                      'bridge_suite = toponav_bridge.runner:main']})
