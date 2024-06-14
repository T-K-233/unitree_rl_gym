import setuptools


with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="legged_gym",
    version="0.0.1",
    author="Unitree",
    license="BSD-3-Clause",
    author_email='zlw21gxy@gmail.com',
    description="An Issac simulation example for reinforcement learning, supports Go2 and H1",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/unitreerobotics/unitree_rl_gym",
    install_requires=[],
    packages=setuptools.find_packages(),
)
