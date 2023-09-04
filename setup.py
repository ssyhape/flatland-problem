import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="piglet",
    version="0.0.1",
    author="Daniel, Mike",
    author_email="dharabo@gmail.com, zhe.chen@monash.edu",
    description="This package provides a variety of flexible searching algorithm implementation.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
    scripts=['piglet.py'],

)