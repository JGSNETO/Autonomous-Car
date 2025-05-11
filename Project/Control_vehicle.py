import pygame
import carla
import logging
import time
import numpy as np
import cv2
import open3d as o3d
from matplotlib import cm
import math
import matplotlib
import argparse


def main():
    argparser = argparse.ArgumentParser(
        description = 'CARLA Manual Control Client'
    )
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug information'
    )
    argparser.add_argument(
        '--sync',
        action='store_true',
        help = 'Activate synchronous mode execution'
    )
    args = argparser.parse_args()

    return args

if __name__ == '__main__':
    main()