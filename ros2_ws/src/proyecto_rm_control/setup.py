from setuptools import setup

package_name = "proyecto_rm_control"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ricardo Lorona",
    maintainer_email="rlorona@espol.edu.ec",
    description="Ackermann trajectory tracking controller for Proyecto_RM.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "trajectory_tracker = proyecto_rm_control.trajectory_tracker:main",
        ],
    },
)
