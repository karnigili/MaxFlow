
from setuptools import setup, find_packages
from codecs import open
from os import path

here = path.abspath(path.dirname(__file__))

with open(path.join(here, 'README.rst'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='graph_max_flow',

    version='1.0.0',

    description='A max flow implemetation',
    long_description=long_description,

    url='https://github.com/karnigili/MaxFlow',

    author='Gili Karni',
    author_email='gili.karni@minerva.kgi.edu',

    license='MIT',

    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Topic :: Software Development :: Build Tools',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 2.7'
    ],


    keywords='Max Flow Dinic Push Relabel Edmond Karp',
    packages=find_packages(),

    install_requires=['numpy'],

   
)
