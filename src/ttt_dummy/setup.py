from setuptools import setup

package_name = "ttt_dummy"

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
    maintainer="ISDN3000E TA",
    maintainer_email="xingxin.he@connect.ust.hk",
    description="Scripted Python dummy player for end-to-end Tic-Tac-Toe verification",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dummy_player_node = ttt_dummy.dummy_player_node:main",
            "random_player_node = ttt_dummy.random_player_node:main",
            "rule_based_player_node = ttt_dummy.rule_based_player_node:main",
        ],
    },
)
