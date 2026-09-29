"""
安装配置
"""

from setuptools import setup, find_packages
from os import path

here = path.abspath(path.dirname(__file__))

# 读取 README
readme_file = path.join(here, 'README.md')
try:
    with open(readme_file, 'r', encoding='utf-8') as f:
        long_description = f.read()
except:
    long_description = '巨蟹关节电机纯 Python 控制库'

setup(
    name='giantcrab-joint-api',
    version='1.0.0',
    description='巨蟹关节电机纯 Python 控制库',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author='Robot Team',
    python_requires='>=3.6',
    packages=find_packages(exclude=['tests', 'examples']),
    package_data={
        'giantcrab_joint_api': ['../lib/x86_64/*.so', '../lib/aarch64/*.so'],
    },
    include_package_data=True,
    install_requires=[
        # 无额外依赖，仅使用标准库
    ],
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
    ],
    keywords='robotics can motor control joint giantcrab',
)
