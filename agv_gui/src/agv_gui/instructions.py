#!/usr/bin/env python
# -*- coding: utf-8 -*-

from enum import Enum

class Instructions(Enum):
    NoInstruction = 0
    GoalInstruction = 1
    HighwayInstruction = 2
    RestrictedAreaInstruction = 3
