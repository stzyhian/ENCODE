#!/usr/bin/env python3
# This file is covered by the LICENSE file in the root of this project.

import imp
import torch
import torch.nn as nn
import torch.nn.functional as F

from backbone.resnet import ResNet18 as Backbone

class Encode(nn.Module):
    def __init__(self, ARCH, path=None):
        super().__init__()
        self.ARCH = ARCH
        self.path = path

        # get the model
        self.backbone = Backbone(params=self.ARCH["backbone"])

        # head
        self.head = nn.Sequential(nn.Dropout(p=ARCH["head"]["dropout"]),
                                  nn.Linear(
                                      in_features=460800, out_features=6, bias=True)
                                  )
        # train backbone?
        if not self.ARCH["backbone"]["train"]:
            for w in self.backbone.parameters():
                w.requires_grad = False

        # train head?
        if not self.ARCH["head"]["train"]:
            for w in self.head.parameters():
                w.requires_grad = False

        # print number of parameters and the ones requiring gradients
        weights_total = sum(p.numel() for p in self.parameters())
        weights_grad = sum(p.numel()
                           for p in self.parameters() if p.requires_grad)
        print("Total number of parameters: ", weights_total)
        print("Total number of parameters requires_grad: ", weights_grad)

        # breakdown by layer
        weights_backbone = sum(p.numel() for p in self.backbone.parameters())
        weights_head = sum(p.numel() for p in self.head.parameters())
        print("Param backbone ", weights_backbone)
        print("Param head ", weights_head)

        # get weights
        if path is not None:
            # try backbone
            try:
                w_dict = torch.load(path + "/backbone",
                                    map_location=lambda storage, loc: storage)
                self.backbone.load_state_dict(w_dict, strict=True)
                print("Successfully loaded model backbone weights")
            except Exception as e:
                print()
                print("Couldn't load backbone, using random weights. Error: ", e)

            # try head
            try:
                w_dict = torch.load(path + "/head",
                                    map_location=lambda storage, loc: storage)
                self.head.load_state_dict(w_dict, strict=True)
                print("Successfully loaded model head weights")
            except Exception as e:
                print("Couldn't load head, using random weights. Error: ", e)

        else:
            print("No path to pretrained, using random init.")

    def forward(self, x0, x1):
        x = torch.cat((x0, x1), 1)
        y = self.backbone(x)
        y = y.view(y.size()[0], -1)
        y = self.head(y)

        return y

    def save_checkpoint(self, logdir, suffix=""):
        # Save the weights
        torch.save(self.backbone.state_dict(), logdir +
                   "/backbone" + suffix)
        torch.save(self.head.state_dict(), logdir +
                   "/head" + suffix)
