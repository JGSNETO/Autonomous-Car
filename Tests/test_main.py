import unittest
from unittest.mock import patch
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'Project')))
from Control_vehicle import main

class TestMainFunction(unittest.TestCase):
    @patch('sys.argv', ['Control_vehicle.py', '--verbose', '--sync'])
    def test_main_with_args(self):
        args = main()
        self.assertTrue(args.debug)
        self.assertTrue(args.sync)


if __name__ == '__main__':
    unittest.main()
