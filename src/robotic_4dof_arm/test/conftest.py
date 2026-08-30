"""
conftest.py — Pytest configuration for robotic_4dof_arm tests.

Disables ament lint plugins (pep257, flake8, copyright, etc.) that
crash on Python 3.14+ and kill the test subprocess before tests can run.
"""


def pytest_configure(config):
    """Disable ament lint plugins incompatible with Python 3.14."""
    lint_plugins = [
        'ament_pep257',
        'ament_copyright',
        'ament_flake8',
        'ament_xmllint',
        'ament_mypy',
        'ament_lint',
    ]
    for plugin_name in lint_plugins:
        if config.pluginmanager.has_plugin(plugin_name):
            config.pluginmanager.set_blocked(plugin_name)
