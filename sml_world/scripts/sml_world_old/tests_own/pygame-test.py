
import sys
sys.path.append('..')
sys.path.append('../RoadModule')

import pygame

import RoadModule
#import time
#import os
#import math

#import bodyclasses
#import dummyvehicle
#import smartvehicle


#import VisualisationModuleReceiver
#import MenuSystem

file_location = "../RoadModule/HighwaySML"
road_module = RoadModule.RoadModule(file_location)

pygame.init()
screen = pygame.display.set_mode((1024, 768))
pg_background = pygame.image.load("../RoadModule/HighwaySML.bmp")
screen.blit(pg_background, (-500,-100))
pygame.display.flip()

while True:
   pass