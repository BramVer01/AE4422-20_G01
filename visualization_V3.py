"""
This file is used for visualization. No big changes should be required to this file.
The only function that can be updated is map_running. You may add additional info to be printed such as a/c in the radar range yourself.
Visualization code in this file is based on graduation work by K.Fines and T. Noortman.
"""

import numpy as np
import pygame as pg
from math import *
import os
import ctypes
import time as timer
import platform  # for checking the operating system

#%% Initialize values
# User inputs (may be changed)
disp_aircaft_id = True  # display aircraft id number
disp_waypoint_id = True  # display waypoint id number
disp_radar_aircraft = False  # display which other aircraft an aircaft sees
disp_time = True  # display current time
disp_waypoint = True  # display dot of the waypoint
disp_vehicles = True

# Lines below are used in visualization, don't change them (line 26-46)
plane_piclist = []  # list to store all plane pics
tug_piclist = []  # list to store all tug pics
plane_rectlist = []  # list to store the rectangular areas of the plane surface
tug_rectlist = []  # list to store the rectangular areas of the tug surface
squared_display = True  # create squared display
boundary_margin = 0.01  # add boundary margin to be sure that points are within boundary
screen_percentage = 0.94  # percentage of pixels used in the critical axis
horizontal_sep = 0.01  # percentage of pixels used in between dual screen

red = (255, 0, 0)  # define colors
lightgreen = (0, 255, 147)
green = (0, 255, 0)
darkgreen = (0, 135, 0)
yellow = (255, 251, 45)
orange = (244, 149, 66)
pink = (255, 192, 203)
purple = (255, 0, 255)

blue = (0, 0, 255)
black = (10, 10, 10)
white = (255, 255, 255)
lightblue = (173, 216, 230)

#%% Functions
def c2m_x(x_coord, min_x, reso_x, x_range, shift=0):
    return int((float(x_coord - min_x) / x_range) * reso_x + shift)

def c2m_y(y_coord, max_y, reso_y, y_range, shift=0):
    return int(((float(y_coord - max_y) / y_range) * reso_y) * -1 - shift)

def plot_vehicle(scr, reso, deg, x, y, x0, y0, x_range, y_range, is_tug=False, x_shift=0, y_shift=0):
    plane_map_x = c2m_x(x, x0, reso[0], x_range, x_shift)
    plane_map_y = c2m_y(y, y0, reso[1], y_range, y_shift)
    
    if is_tug:
        tug_rectlist[deg].centerx = plane_map_x
        tug_rectlist[deg].centery = plane_map_y
        scr.blit(tug_piclist[deg], tug_rectlist[deg])
    else:
        plane_rectlist[deg].centerx = plane_map_x
        plane_rectlist[deg].centery = plane_map_y
        scr.blit(plane_piclist[deg], plane_rectlist[deg])

def plot_line(scr, color_code, reso, radius, coord_1, coord_2, x0, y0, x_range, y_range, x_shift=0, y_shift=0):
    wp_map_x_1 = c2m_x(coord_1[0], x0, reso[0], x_range, x_shift)
    wp_map_y_1 = c2m_y(coord_1[1], y0, reso[1], y_range, y_shift)
    wp_map_x_2 = c2m_x(coord_2[0], x0, reso[0], x_range, x_shift)
    wp_map_y_2 = c2m_y(coord_2[1], y0, reso[1], y_range, y_shift)
    pg.draw.line(scr, color_code, (wp_map_x_1, wp_map_y_1), (wp_map_x_2, wp_map_y_2), radius)

def plot_circle(scr, color_code, reso, radius, coord_1, x0, y0, x_range, y_range, x_shift=0, y_shift=0):
    wp_map_x = c2m_x(coord_1[0], x0, reso[0], x_range, x_shift)
    wp_map_y = c2m_y(coord_1[1], y0, reso[1], y_range, y_shift)
    pg.draw.circle(scr, color_code, (wp_map_x, wp_map_y), radius)

def plot_text(scr, text_str, color_code, fontsize, reso, x, y, x0, y0, x_range, y_range, x_shift=0, y_shift=0):
    font = pg.font.Font(None, fontsize)
    text = font.render(text_str, 1, color_code)
    textpos = text.get_rect()
    wp_map_x = c2m_x(x, x0, reso[0], x_range, x_shift)
    wp_map_y = c2m_y(y, y0, reso[1], y_range, y_shift)
    textpos.centerx = wp_map_x
    textpos.centery = wp_map_y
    scr.blit(text, textpos)

def map_initialization(nodes_dict, edges_dict):
    # Create a dict to store map properties
    map_properties = dict()
    map_get_range(nodes_dict, map_properties)

    # Get correct resolution of screen
    if os.name == 'nt':
        ctypes.windll.user32.SetProcessDPIAware()
        true_res = (ctypes.windll.user32.GetSystemMetrics(0),
                    ctypes.windll.user32.GetSystemMetrics(1))
    else:
        # On Mac/Linux, use Tkinter to determine screen resolution
        try:
            import tkinter as tk
            root = tk.Tk()
            true_res = (root.winfo_screenwidth(), root.winfo_screenheight())
            root.destroy()
        except Exception:
            true_res = (800, 600)  # Fallback resolution

    reso_ratio = map_properties['x_range'] / float(map_properties['y_range'])
    x_pixels = int(screen_percentage * min(true_res[0], true_res[1] * reso_ratio))
    y_pixels = int(x_pixels / reso_ratio)
    outer_reso = (x_pixels, y_pixels)
    inner_reso = (x_pixels, y_pixels)
    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (int(0.5 * true_res[0] - 0.5 * outer_reso[0]),
                                                   int(0.5 * true_res[1] - 0.5 * outer_reso[1]))

    pg.init()
    scr = pg.display.set_mode(outer_reso)
    scrrect = scr.get_rect()
    scr.fill(white)
    
    # Load images using os.path.join for cross-platform compatibility
    plane_pic = pg.image.load(os.path.join(os.getcwd(), "blue-plane-hi.bmp"))
    plane_pic.set_colorkey(pg.Color(255, 255, 255))
    
    tug_pic = pg.image.load(os.path.join(os.getcwd(), "tug.bmp"))
    tug_pic.set_colorkey(pg.Color(255, 255, 255))

    for i in range(0, 360):
        plane_piclist.append(pg.transform.rotozoom(plane_pic, i, (1. / 14.)))
        plane_rectlist.append(plane_piclist[i].get_rect())
        
        tug_piclist.append(pg.transform.rotozoom(tug_pic, i, (1. / 20.)))
        tug_rectlist.append(tug_piclist[i].get_rect())

    map_properties['outer_reso'] = outer_reso
    map_properties['inner_reso'] = inner_reso
    map_properties['scr'] = scr
    map_properties['scrrect'] = scrrect
    map_properties['plane_piclist'] = plane_piclist
    map_properties['plane_rectlist'] = plane_rectlist
    map_properties['tug_piclist'] = tug_piclist
    map_properties['tug_rectlist'] = tug_rectlist
    map_properties['horizontal_sep'] = horizontal_sep

    map_get_background(map_properties, nodes_dict, edges_dict)
    return map_properties

def map_get_range(nodes_dict, map_properties):
    min_x = np.inf
    max_x = 0
    min_y = np.inf
    max_y = 0
    for node in nodes_dict:
       if nodes_dict[node]['x_pos'] < min_x:
           min_x = nodes_dict[node]['x_pos'] - 3 
       if nodes_dict[node]['x_pos'] > max_x:
           max_x = nodes_dict[node]['x_pos'] + 4 
       if nodes_dict[node]['y_pos'] < min_y:
           min_y = nodes_dict[node]['y_pos'] - 2 
       if nodes_dict[node]['y_pos'] > max_y:
           max_y = nodes_dict[node]['y_pos'] + 2 

    x_range = max_x - min_x
    y_range = max_y - min_y

    if squared_display:
        if x_range > y_range:
            min_x -= int(boundary_margin * x_range)
            max_x += int(boundary_margin * x_range)
            diff_range = (max_x - min_x) - y_range
            min_y -= int(0.5 * diff_range)
            max_y += int(0.5 * diff_range)
        elif y_range > x_range:
            min_y -= int(boundary_margin * y_range)
            max_y += int(boundary_margin * y_range)
            diff_range = (max_y - min_y) - x_range
            min_x -= int(0.5 * diff_range)
            max_x += int(0.5 * diff_range)
        else:
            min_x -= int(boundary_margin * x_range)
            max_x += int(boundary_margin * x_range)
            min_y -= int(boundary_margin * y_range)
            max_y += int(boundary_margin * y_range)
    else:
        min_x -= int(boundary_margin * x_range)
        max_x += int(boundary_margin * x_range)
        min_y -= int(boundary_margin * y_range)
        max_y += int(boundary_margin * y_range)

    x_range = max_x - min_x
    y_range = max_y - min_y

    map_properties['min_x'] = min_x
    map_properties['max_x'] = max_x
    map_properties['x_range'] = x_range
    map_properties['min_y'] = min_y
    map_properties['max_y'] = max_y
    map_properties['y_range'] = y_range

def map_get_background(map_properties, nodes_dict, edges_dict):
    reso = map_properties['inner_reso']
    scr = map_properties['scr']
    min_x = map_properties['min_x']
    x_range = map_properties['x_range']
    max_y = map_properties['max_y']
    y_range = map_properties['y_range']

    map_get_layout(scr, nodes_dict, edges_dict, min_x, max_y, reso, x_range, y_range, 0, 0)
    background = pg.image.tostring(scr, "RGB")
    map_properties['background'] = background

def map_get_layout(scr, nodes_dict, edges_dict, min_x, max_y, reso, x_range, y_range, scr_x_shift, scr_y_shift): 
    edges_created = set()
    for edge in edges_dict:
        if edge not in edges_created:
            source = edges_dict[edge]["start_end_pos"][0]
            target = edges_dict[edge]["start_end_pos"][1]
            plot_line(scr, black, reso, 1, source, target, min_x, max_y, x_range, y_range, scr_x_shift, scr_y_shift)
            edges_created.add((edges_dict[edge]["to"], edges_dict[edge]["from"]))
            edges_created.add((edges_dict[edge]["from"], edges_dict[edge]["to"]))
    else:
        pass

    runway_a = [(3, 2.5), (3, 6.5)]
    runway_d = [(1, 3.5), (1, 7.5)]
    plot_line(scr, darkgreen, reso, 5, runway_a[0], runway_a[1], min_x, max_y, x_range, y_range, scr_x_shift, scr_y_shift)
    plot_line(scr, darkgreen, reso, 5, runway_d[0], runway_d[1], min_x, max_y, x_range, y_range, scr_x_shift, scr_y_shift)
    
    for node in nodes_dict:
        wp_coordinate = [nodes_dict[node]['x_pos'], nodes_dict[node]['y_pos']]
        color = blue
        plot_circle(scr, color, reso, 4, wp_coordinate, min_x, max_y, x_range, y_range)
        thisString = str(nodes_dict[node]['id'])
        plot_text(scr, thisString, black, 14, reso, wp_coordinate[0], wp_coordinate[1], min_x, max_y, x_range, y_range, 10, 10)

#%% Update map during running

def map_running(map_properties, current_states, t):
    reso = map_properties['inner_reso']
    scr = map_properties['scr']
    scrrect = map_properties['scrrect']

    min_x = map_properties['min_x']
    x_range = map_properties['x_range']
    max_y = map_properties['max_y']
    y_range = map_properties['y_range']
    background = map_properties['background']

    layout = pg.image.fromstring(background, scrrect.size, "RGB")
    scr.blit(layout, scrrect)
    
    if disp_vehicles:
        for vehicle in current_states.keys():
            heading = int(current_states[vehicle]["heading"])
            x_pos = current_states[vehicle]["xy_pos"][0]
            y_pos = current_states[vehicle]["xy_pos"][1]
            is_tug = not current_states[vehicle].get("has_flight", False)
            plot_vehicle(scr, reso, heading, x_pos, y_pos, min_x, max_y, x_range, y_range, is_tug)
            
    if disp_time:
        plot_text(scr, "timestep", black, 30, reso, min_x + 0.90 * x_range, max_y - 0.03 * y_range, min_x, max_y,
                  x_range, y_range)
        time_str = str(t).zfill(2)
        plot_text(scr, time_str, black, 25, reso, min_x + 0.90 * x_range, max_y - 0.06 * y_range, min_x, max_y, x_range, y_range)

    if disp_aircaft_id:
        for vehicle in current_states.keys():
            label = 'TUG: ' if not current_states[vehicle].get("has_flight", False) else 'AC: '
            id_string = label + str(current_states[vehicle]["ac_id"])
            col = orange if not current_states[vehicle].get("has_flight", False) else red
            plot_text(scr, id_string, col, 14, reso, 
                      current_states[vehicle]["xy_pos"][0], 
                      current_states[vehicle]["xy_pos"][1], 
                      min_x, max_y, x_range, y_range, 0, 25)

    collision = False
    for ac1 in current_states:
        for ac2 in current_states:
            if ac1 != ac2 and current_states[ac1]["xy_pos"] == current_states[ac2]["xy_pos"]:
                collision = True
                print("COLLISION - between", current_states[ac1]["ac_id"], "and", current_states[ac2]["ac_id"], 
                      "at location", current_states[ac1]["xy_pos"], "time", t)
                plot_text(scr, "COLLISION", purple, 16, reso, 
                          current_states[ac1]["xy_pos"][0], 
                          current_states[ac1]["xy_pos"][1]+0.1, 
                          min_x, max_y, x_range, y_range, 0, 25)
   
    pg.display.flip()
    pg.event.pump()
    keys = pg.key.get_pressed()
    
    if collision:
        timer.sleep(0.4)
    
    if keys[pg.K_ESCAPE]:
        escape_pressed = True
        print('Visualization aborted by escape key')
        pg.quit()
    else:
        escape_pressed = False

    if keys[pg.K_p]:
        input('Paused, press enter to continue')
        
    return escape_pressed
