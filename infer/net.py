#!/usr/bin/env python3
import argparse
import subprocess
import datetime
import yaml
import os

from user import *

if __name__ == '__main__':
    dataset = 'path-to-dataset'
    model   = 'path-to-model'
      # open arch config file
    try:
        print("Opening arch config file from %s" % model)
        ARCH = yaml.safe_load(open(model + "/arch_cfg.yaml", 'r'))
    except Exception as e:
        print(e)
        print("Error opening arch yaml file.")
        quit()

    # open data config file
    try:
        print("Opening data config file from %s" % model)
        DATA = yaml.safe_load(open(model + "/data_cfg.yaml", 'r'))
    except Exception as e:
        print(e)
        print("Error opening data yaml file.")
        quit()
    
    # does model folder exist?
    if os.path.isdir(model):
        print("model folder exists! Using model from %s" % (model))
    else:
        print("model folder doesnt exist! Can't infer...")
        quit()
    user = User(ARCH, DATA, dataset, model)
    user.infer()