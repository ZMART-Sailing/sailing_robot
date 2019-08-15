#!/usr/bin/env python
import json
import socket
import thread
import time, collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from tcp import server_config

import numpy as np

# color palette definition (V2 from  https://matplotlib.org/users/dflt_style_changes.html#colors-color-cycles-and-color-maps)
C = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']


def find_char(msg, index, char):
    index += 1
    while index < len(msg) and msg[index] != char:
        index += 1
    return index


def find_json(msg, start_index):
    count = 1
    index = start_index + 1
    while index < len(msg):
        if msg[index] == "'" or msg[index] == '"':
            index = find_char(msg, index, msg[index])
        elif msg[index] == '{':
            count += 1
        elif msg[index] == '}':
            count -= 1
            if count == 0:
                return start_index, index + 1
        if index < len(msg):
            index += 1


def update_msg(msg):
    last_msg = None
    index = 0
    while index < len(msg):
        if msg[index] == '{':
            cur_json = find_json(msg, index)
            if cur_json is not None:
                last_msg = cur_json
                index = last_msg[1]
            else:
                index += 1
        else:
            index += 1
    if last_msg is not None:
        last_msg, msg = msg[last_msg[0]:last_msg[1]], msg[last_msg[1]:]
    return last_msg, msg


def update_param(obj):
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mySocket.connect((server_config.server_host, server_config.server_port))
    mySocket.send('Subscriber ' + str(1.0 / obj.rate))
    msg = ''

    try:
        while True:
            cur_msg = mySocket.recv(1024)
            if cur_msg is not None and cur_msg != '':
                msg += cur_msg
                last_msg, msg = update_msg(msg)
                if last_msg is None:
                    continue
                try:
                    print last_msg
                    json_msg = json.loads(last_msg)
                except ValueError as e:
                    print e
                else:
                    for var, value in json_msg.items():
                        setattr(obj, var, value)
                    obj.wind_north = np.radians(obj.heading + obj.wind_direction_apparent)
                    obj.wp_array = np.array(obj.wp_array).T  # [lat, lon]
                    obj.origin = obj.wp_array.mean(axis = 1, keepdims = True)
                    obj.wp_array -= obj.origin
                    if obj.dbg_keep_station_waypoint is not None and obj.dbg_keep_station_waypoint != '':
                        obj.dbg_keep_station_waypoint = np.array(obj.dbg_keep_station_waypoint).T
                        obj.dbg_keep_station_waypoint -= obj.origin
                        # print obj.wp_array
                        # print obj.dbg_keep_station_waypoint
                        obj.maxwpdist = np.abs(
                            np.concatenate((obj.wp_array, obj.dbg_keep_station_waypoint), axis = 0)).max(axis = 1)
                    else:
                        obj.dbg_keep_station_waypoint = None
                        obj.maxwpdist = np.abs(obj.wp_array).max(axis = 1)
                    obj.origin = obj.origin.flatten()
                    obj.position -= obj.origin
                    obj.position_history.append(obj.position)
                    time.sleep(1.0 / obj.rate)
    except KeyboardInterrupt as e:
        print e
    finally:
        mySocket.close()


class Debugging_2D_matplot():

    def __init__(self):

        # init params
        self.sailing_state = 'normal'

        self.wind_direction_apparent = 0
        self.wind_north = 0

        self.heading = 0
        self.goal_heading = 0

        self.longitude = 0
        self.latitude = 0

        self.position_history = collections.deque(maxlen = 500)
        self.position = [1, 1]

        self.wp_array = np.array([[0], [0]])
        self.origin = np.array([0, 0])

        self.window = [0, 0, 0, 0]
        self.maxwpdist = [0, 0]
        self.rate = 10

        self.has_bg_img = False
        self.image = None

        self.dbg_keep_station_waypoint = None

        thread.start_new_thread(update_param, (self,))

        time.sleep(2.0 / self.rate)

        self.fig = plt.figure()
        self.boatline, = plt.plot([], [], c = C[0], label = "Heading")
        plt.plot([], [], c = C[7], label = "Goal heading")
        plt.plot([], [], c = C[1], label = "Wind direction")

        # display Waypoints
        self.wpfig = plt.scatter(self.wp_array[0], self.wp_array[1], c = C[3])
        self.wp_update_fig = None

        plt.tight_layout()
        self.ax = plt.subplot(111)

        self.wind_arrow = self.get_arrow(self.wind_north, C[1], reverse = True)

        self.heading_arrow = self.get_arrow(np.radians(self.heading), C[0])

        self.goal_heading_arrow = self.get_arrow(np.radians(self.goal_heading), C[7])

        arrow_col = C[0]
        if self.sailing_state != 'normal':
            arrow_col = C[1]
        arrow_dx = 0.1 * np.sin(np.radians(self.heading))
        arrow_dy = 0.1 * np.cos(np.radians(self.heading))
        self.boat_arrow = plt.arrow(self.position[0] - arrow_dx, self.position[1] - arrow_dy, arrow_dx, arrow_dy,
                                    head_width = 0.5, head_length = 1., fc = arrow_col, ec = arrow_col)

        self.legend = plt.legend()

        self.update_plot()

    # def get_bg_image(self):
    #
    #     self.has_bg_img = False
    #     self.image = None
    #     side_dist = None
    #     image_origin = None
    #
    #     my_dir = os.path.dirname(__file__)
    #     image_dir = os.path.abspath(os.path.join(my_dir, '../../../utilities/map_bg_images'))
    #
    #     # select the correct image if any
    #     for filename in os.listdir(image_dir):
    #         if not filename.endswith(".png"):
    #             continue
    #
    #         data_filename = filename[:-4].split("_")
    #         current_img_origin = self.nav.latlon_to_utm(float(data_filename[0]), float(data_filename[1]))
    #
    #         dist_to_origin = ((self.origin[0] - current_img_origin[0]) ** 2 + (
    #                 self.origin[1] - current_img_origin[1]) ** 2) ** 0.5
    #
    #         if dist_to_origin < 1000:
    #             image_origin = current_img_origin
    #             side_dist = float(data_filename[2])
    #             self.image = mpimg.imread(os.path.join(image_dir, filename))
    #             self.has_bg_img = True
    #             break
    #
    #     if not self.has_bg_img:
    #         return
    #
    #     minx = image_origin[0] - side_dist - self.origin[0]
    #     maxx = image_origin[0] + side_dist - self.origin[0]
    #     miny = image_origin[1] - side_dist - self.origin[1]
    #     maxy = image_origin[1] + side_dist - self.origin[1]
    #
    #     image_size = (minx, maxx, miny, maxy)
    #     self.image_show = self.ax.imshow(self.image, extent = image_size)

    def calculate_arrow_target(self, angle):
        figsize = self.fig.get_size_inches()
        scale_dx = figsize[0] / np.sqrt(figsize[0] ** 2 + figsize[1] ** 2)
        scale_dy = figsize[1] / np.sqrt(figsize[0] ** 2 + figsize[1] ** 2)

        arrow_ori = (0.88, 0.12)
        arrow_target = (arrow_ori[0] + 0.05 * np.sin(angle) / scale_dx, arrow_ori[1] + 0.05 * np.cos(angle) / scale_dy)
        return arrow_target

    def get_arrow(self, angle, color, reverse = False):
        if reverse:
            style = '<-'
        else:
            style = '->'
        arrow_ori = (0.88, 0.12)
        arrow_target = self.calculate_arrow_target(angle)
        arrow = self.ax.annotate("", xy = arrow_target, xycoords = self.ax.transAxes, xytext = arrow_ori,
                                 textcoords = self.ax.transAxes,
                                 arrowprops = dict(arrowstyle = style, color = color, connectionstyle = "arc3"), )

        return arrow

    def update_arrow(self, arrow, angle):
        arrow.xy = self.calculate_arrow_target(angle)

    def animate(self, i):
        if self.position_history:
            lat, lon = np.array(self.position_history).T
            self.boatline.set_data(lat, lon)

        self.update_arrow(self.wind_arrow, self.wind_north)
        self.update_arrow(self.heading_arrow, np.radians(self.heading))
        self.update_arrow(self.goal_heading_arrow, np.radians(self.goal_heading))

        self.boat_arrow.set_visible(False)
        arrow_col = C[0]
        if self.sailing_state != 'normal':
            arrow_col = C[1]
        arrow_dx = 0.1 * np.sin(np.radians(self.heading))
        arrow_dy = 0.1 * np.cos(np.radians(self.heading))

        self.boat_arrow = plt.arrow(self.position[0] - arrow_dx, self.position[1] - arrow_dy, arrow_dx, arrow_dy,
                                    head_width = 0.5, head_length = 1., fc = arrow_col, ec = arrow_col)

        # display Waypoints
        # print self.dbg_keep_station_waypoint
        if self.dbg_keep_station_waypoint is not None:
            if self.wp_update_fig is not None:
                self.wp_update_fig.set_visible(False)
            self.wp_update_fig = plt.scatter(self.dbg_keep_station_waypoint[0], self.dbg_keep_station_waypoint[1],
                                             c = C[9])

        self.update_window(i)
        if self.has_bg_img:
            return self.image_show, self.boatline, self.wind_arrow, self.boat_arrow, self.wpfig, self.goal_heading_arrow, self.heading_arrow, self.legend, self.wp_update_fig
        else:
            return self.boatline, self.wind_arrow, self.boat_arrow, self.wpfig, self.goal_heading_arrow, self.heading_arrow, self.legend, self.wp_update_fig

    def update_window(self, i):
        # update window only every second
        if not i % 50 == 0:
            return

            # rounding of the window size in m
        rounding = 10.0

        # maximum distance to origin in both direction
        distx = max(self.maxwpdist[0], abs(self.position[0])) + rounding / 2
        disty = max(self.maxwpdist[1], abs(self.position[1])) + rounding / 2

        distx = int(round(distx / rounding) * rounding) + 1
        disty = int(round(disty / rounding) * rounding) + 1

        # scaling to keep x and y orthonormal
        figsize = self.fig.get_size_inches()
        scale_dx = figsize[0] / np.sqrt(figsize[0] ** 2 + figsize[1] ** 2)
        scale_dy = figsize[1] / np.sqrt(figsize[0] ** 2 + figsize[1] ** 2)
        norm = min(scale_dx, scale_dy)
        scale_dx = scale_dx / norm
        scale_dy = scale_dy / norm

        # decide which axis is the limiting one
        if distx * scale_dy > disty * scale_dx:
            dist = distx
        else:
            dist = disty

        minx = - dist * scale_dx
        maxx = + dist * scale_dx
        miny = - dist * scale_dy
        maxy = + dist * scale_dy

        if self.window == [minx, maxx, miny, maxy]:
            return

        self.window = [minx, maxx, miny, maxy]
        self.ax.axis(self.window)

        return

    def update_plot(self):
        line_ani = animation.FuncAnimation(self.fig, self.animate, interval = 1000 / self.rate, blit = False)
        plt.show()


if __name__ == '__main__':
    Debugging_2D_matplot()
