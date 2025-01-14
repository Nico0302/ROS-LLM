from setuptools import find_packages, setup

package_name = 'llm_tools'

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
    maintainer='robot1',
    maintainer_email='ngres@csuchico.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_tools = llm_tools.llm_tools:main',
            'get_tool_descriptions = llm_tools.llm_tools:run_get_tool_descriptions',
        ],
    },
)
