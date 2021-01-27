import setuptools
import os
import shutil

libraries = []
files = os.listdir('ORBSLAM3')
for file in files:
    if os.path.splitext(file)[1] in ['.pyd', '.dll', '.lib', '.so'] or '.so' in file:
        if file.find('ORBSLAM') >= 0:
            if '.so' in file:
                renamed_file = 'ORBSLAM3.so'
            else:
                renamed_file = 'ORBSLAM3' + os.path.splitext(file)[1]
            try:
                shutil.copy(os.path.join('ORBSLAM3', file), os.path.join('ORBSLAM3', renamed_file))
            except shutil.SameFileError as e:
                print(e)
            libraries.append(os.path.join('ORBSLAM3', renamed_file))
        else:
            libraries.append(os.path.join('ORBSLAM3', file))


setuptools.setup(
    name="ORBSLAM3",
    version="0.0.1",
    author="Peter Somers",
    author_email="",
    description="Python bindings for ORBSLAM3",
    packages=['ORBSLAM3'],
    data_files=libraries,
    include_package_data=True,
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
    ],
    python_requires='>=3',
)