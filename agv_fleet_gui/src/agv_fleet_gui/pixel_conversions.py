#!/usr/bin/env python
# -*- coding: utf-8 -*-

class PixelConversions:
    @staticmethod
    def toWorldX(px, real_x_min, real_x_max, frame_width_max, frame_width_min=0):
        return (((px - frame_width_min) * (real_x_max - real_x_min)) / (frame_width_max - frame_width_min)) + real_x_min

    @staticmethod
    def toWorldY(py, real_y_min, real_y_max, frame_height_min, frame_height_max=0):
        return (((py - frame_height_min) * (real_y_max - real_y_min)) / (frame_height_max - frame_height_min)) + real_y_min

    @staticmethod
    def toPixelX(wx, real_x_min, real_x_max, frame_width_max, frame_width_min=0):
        # return (((wx - real_x_min) * (frame_width_max - frame_width_min)) / (real_x_max - real_x_min)) + frame_width_min
        return int((((wx - real_x_min) * (frame_width_max - frame_width_min)) / (real_x_max - real_x_min)) + frame_width_min)

    @staticmethod
    def toPixelY(wy, real_y_min, real_y_max, frame_height_min, frame_height_max=0):
        # return (((wy - real_y_min) * (frame_height_max - frame_height_min)) / (real_y_max - real_y_min)) + frame_height_min
        return int((((wy - real_y_min) * (frame_height_max - frame_height_min)) / (real_y_max - real_y_min)) + frame_height_min)
