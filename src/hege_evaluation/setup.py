from setuptools import setup


package_name = "hege_evaluation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Oguzhan Enes Isik",
    maintainer_email="ouz.3406@gmail.com",
    description="Quantitative simulation evaluation tools for HEGE",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gps_noise_evaluator = hege_evaluation.gps_noise_evaluator:main",
            "sim_gps_covariance = hege_evaluation.sim_gps_covariance:main",
        ],
    },
)
