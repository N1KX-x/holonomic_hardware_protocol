import unittest

import config


class SafetyTests(unittest.TestCase):
    def test_hardware_execution_flag_is_boolean(self):
        self.assertIsInstance(config.ALLOW_HARDWARE_EXECUTION, bool)

    def test_config_is_consistent(self):
        config.validate()


if __name__ == "__main__":
    unittest.main()
