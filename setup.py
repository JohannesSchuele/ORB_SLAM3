import setuptools
import os
import shutil

libraries = []
files = os.listdir('ORBSLAM3')
[print(file) for file in files]
for file in files:
    if file[-3:] in ['pyd', 'dll', 'lib']:
        if file.find('ORBSLAM') >= 0:
            renamed_file = 'ORBSLAM3.' + file[-3:]
            try:
                shutil.copyfile(os.path.join('ORBSLAM3', file), os.path.join('ORBSLAM3', renamed_file))
            except shutil.SameFileError as e:
                continue
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