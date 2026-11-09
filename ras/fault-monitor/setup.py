from setuptools import find_packages, setup

package_name = "ras_fault_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/fault_logging.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ms_kim",
    maintainer_email="206399535+felixmskim@users.noreply.github.com",
    description="ROS 2 nodes for q-vela (RISC-V RERI) kernel fault collection and logging",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "kernel_monitor = ras_fault_monitor.kernel_monitor_node:main",
            "fault_logger = ras_fault_monitor.fault_logger_node:main",
            "fault_standalone = ras_fault_monitor.standalone:main",
        ],
    },
)
