from setuptools import setup
from setuptools import find_packages

long_description = '''
'''

setup(name='sparse_rrt',
      version='0.0.1',
      description='Sparse stable trees planner',
      long_description=long_description,
      author='Oleg Sinyavskiy',
      author_email='olegsinyavskiy@gmail.com',
      url='https://github.com/olegsinyavskiy/sparse_rrt',
      download_url='',
      license='BSD',
      install_requires=['numpy>=1.13.3'],
      extras_require={
          'tests': ['pytest>=2.7.2',
                    'pytest-pep8>=1.0.6',
                    'pytest-xdist>=1.13.1',
                    'pytest-cov>=2.1.0'],
      },
      classifiers=[
          # 'Development Status :: 5 - Production/Stable',
          'Intended Audience :: Developers',
          'Intended Audience :: Education',
          'Intended Audience :: Science/Research',
          # 'License :: OSI Approved :: MIT License',
          'Programming Language :: Python :: 2',
          'Programming Language :: Python :: 2.7',
          'Programming Language :: Python :: 3',
          'Programming Language :: Python :: 3.6',
          'Topic :: Software Development :: Libraries',
          'Topic :: Software Development :: Libraries :: Python Modules'
      ],
      packages=find_packages())
