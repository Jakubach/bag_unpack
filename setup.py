from setuptools import setup

package_name = 'bag_unpack'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your_email@example.com',
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 Bag Reader Node',
    license='Your License',
    tests_require=['pytest'],
entry_points={
    'console_scripts': [
        'bag_unpack = bag_unpack.bag_unpack:main',  # Poprawny import funkcji main
    ],
},
    package_data={
        package_name: [
            'config/*.yaml',  # Dodajemy katalog config i plik yaml
        ],
    },
)

