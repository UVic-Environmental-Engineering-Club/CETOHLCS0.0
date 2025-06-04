from setuptools import find_packages, setup

package_name = 'hlcs'

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
    maintainer='David Kim',
    maintainer_email='uvicenvironment@gmail.com',
    description='high level control system for UVEEC',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = hlcs.main:main',
            'listener = hlcs.subscriber_member_function:main',
        ],
    },
)
