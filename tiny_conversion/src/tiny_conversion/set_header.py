#!/usr/bin/env python3

# import rospy

def set_header_frame_id(m, new_frame_id):
    m.header.frame_id = new_frame_id
    return m
