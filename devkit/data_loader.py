import numpy as np
import math
import random
import sys
import pickle
import csv
import os
import time
import matplotlib.pyplot as plt
import cv2
import copy



class DataLoader:

    def __init__(self, base_path, dataset_name):

        self.base_path = base_path
        self.dataset_name = dataset_name
        self.dataset_path = os.path.join(base_path, dataset_name)

        # read calibration params (extrinsic)
        self.read_calib_params()

        # read labeled data
        self.read_labeled_data()

    def read_calib_params(self):

        '''
        We obtained extrinsic params (K, Rt) that defines the relationship between the ego-centric coor. system and
        the left cam coor. system under the assumption that
        1) the center of the vehicle is located at the position where GPS/IMU module is mounted w.r.t Xe, Ye directions
        2) w.r.t Ze direction, the center is located at the ground-plane.

        The vehicle trajectories (in the ego-centric coor. system) obtained from the GPS/IMU is therefore need to be slightly modified
        before drawing them on the left cam image.

        For example, let (X0, Y0, Z0) denote a 3D position in the trajectories. Before transform it into the cam coor. system,
        subtract delta_z (the distance between the location of GPS/IMU and the ground-plane w.r.t Ze direction) such as (X0, Y0, Z0 - delta_z)

        The ground-truth delta_z is 0.35 meter

        '''

        # read calib info
        file_name = self.dataset_name + '_cam0_extcalib.csv'
        file_path = os.path.join(os.path.join(self.dataset_path, 'label'), file_name)

        f = open(file_path)
        reader = csv.reader(f, delimiter=',')
        cnt = 0
        for row in reader:
            if (cnt == 0):
                K = np.array(row).astype('float').reshape(3, 4)
            else:
                Rt = np.array(row).astype('float').reshape(4, 4)
            cnt += 1
        f.close()

        self.K = K
        self.Rt = Rt

    def read_labeled_data(self):

        # read calib info
        file_name = self.dataset_name + '_drvsts_pw.csv'
        file_path = os.path.join(os.path.join(self.dataset_path, 'label'), file_name)


        '''
        ** data format **
        data[:, 0] : frame index 
        data[:, 1] : altitude [meter]
        data[:, 2] : roll [degree]
        data[:, 3] : pitch [degree]
        data[:, 4] : yaw [degree]
        data[:, 5] : heading [degree]
        data[:, 6] : TM X-axis [meter]
        data[:, 7] : TM Y-axis [meter]
        data[:, 8] : img file name
        data[:, 9] : point cloud file name
        data[:, 10] : steering angle [degree]
        data[:, 11] : speed [km/h]
        data[:, 12] : turn signal, non (0), left (1), right (2), both (3)
        data[:, 13] : gps localization error std [meter]
        data[:, 14] : driving action
        '''

        self.data = self.read_csv(file_path)

    def read_csv(self, file_dir):

        return np.genfromtxt(file_dir, delimiter=',')

    def create_pose_matrix(self, alt, roll, pitch, yaw, east, north):

        # translation vector
        t = np.array([east, north, alt]).reshape((3, 1))

        # rotation matrix
        rx = roll
        ry = pitch
        rz = yaw

        # rotation according to roll
        Rx = [1, 0, 0,
              0, math.cos(rx), -1 * math.sin(rx),
              0, math.sin(rx), math.cos(rx)]
        Rx = np.asarray(Rx).reshape((3, 3))

        # rotation according to pitch
        Ry = [math.cos(ry), 0, math.sin(ry),
              0, 1, 0,
              -1 * math.sin(ry), 0, math.cos(ry)]
        Ry = np.asarray(Ry).reshape((3, 3))

        # rotation according to yaw
        Rz = [math.cos(rz), -1 * math.sin(rz), 0,
              math.sin(rz), math.cos(rz), 0,
              0, 0, 1]
        Rz = np.asarray(Rz).reshape((3, 3))

        # transofrmation matrix
        R = np.matmul(np.matmul(Rz, Ry), Rx)

        return np.linalg.inv(self.affinematrix(R, t))

    def read_image(self, file_name, isLeft=True):

        if (isLeft):
            path = os.path.join(self.dataset_path, ('00/%08d.png' % int(file_name)))
        else:
            path = os.path.join(self.dataset_path, ('01/%08d.png' % int(file_name)))
        return cv2.imread(path)

    def read_disparity(self, file_name):
        path = os.path.join(self.dataset_path, ('00_depth/%08d.png' % int(file_name)))

        if (os.path.exists(path) == False):
            print("[error] no such files or directories {" + path + '}')
            sys.exit()

        return cv2.imread(path, -1)

    def read_point_cloud(self, file_name):

        filename = 'bin/%08d.bin' % file_name
        scan = np.fromfile(os.path.join(self.dataset_path, filename), dtype=np.float32)
        return scan.reshape([-1, 4])

    def affinematrix(self, R, t):

        affine = np.concatenate((R, t), axis=1)
        affine = np.concatenate((affine, np.asarray([0, 0, 0, 1]).reshape((1, 4))), axis=0)

        return affine

    def in_range_points(self, points, x, y, z, x_range, y_range, z_range):

        points_select = points[np.logical_and.reduce((x > x_range[0], x < x_range[1], y > y_range[0], y < y_range[1], z > z_range[0], z < z_range[1]))]
        return np.around(points_select, decimals=2)

    def rotate_points_cloud(self, xyz_pts, rotation):

        ry, rx, rz = rotation

        xyz_pts = np.concatenate([xyz_pts, np.ones(shape=(xyz_pts.shape[0], 1))], axis=1)  # N x 4

        # translation vector
        t = np.array([0, 0, 0]).reshape((3, 1))

        # rotation according to roll
        Rx = [1, 0, 0,
              0, math.cos(rx), -1 * math.sin(rx),
              0, math.sin(rx), math.cos(rx)]
        Rx = np.asarray(Rx).reshape((3, 3))

        # rotation according to pitch
        Ry = [math.cos(ry), 0, math.sin(ry),
              0, 1, 0,
              -1 * math.sin(ry), 0, math.cos(ry)]
        Ry = np.asarray(Ry).reshape((3, 3))

        # rotation according to yaw
        Rz = [math.cos(rz), -1 * math.sin(rz), 0,
              math.sin(rz), math.cos(rz), 0,
              0, 0, 1]
        Rz = np.asarray(Rz).reshape((3, 3))

        # transofrmation matrix
        R = np.matmul(np.matmul(Rz, Ry), Rx)
        R = self.affinematrix(R, t)  # 4 x 4

        return np.matmul(R, xyz_pts.T).T

    def draw_traj_on_left_img(self, ego_traj, img_file_name, obs_len, drv_act):

        img = self.read_image(img_file_name)
        for m in range(obs_len, ego_traj.shape[0]):

            A = np.matmul(np.linalg.inv(self.Rt), ego_traj[m, :].reshape(4, 1))
            B = np.matmul(self.K, A)

            x = int(B[0, 0] * 1.0 / B[2, 0])
            y = int(B[1, 0] * 1.0 / B[2, 0])

            if (x < 0 or x > 1280 - 1 or y > 360 - 1):
                continue
            img = cv2.circle(img, (x, y), 3, (0, 0, 255), -1)

        text = self.generate_drvact_text(drv_act)
        cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return img
        # text = dataset_name[p] + '_' + str(int(curr_seq_data[0, 0]))
        # cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # cv2.imshow('test', img)
        # cv2.waitKey(0)

    def draw_traj_on_topivew_img(self, img_file_name, path3d, obs_len):

        # parameters
        rot_angle = -0.0
        scale = 1.0

        # disparity map load
        disparity = self.read_disparity(img_file_name)
        disparity = disparity.astype('float') / 256.0

        # cv2.imshow('test', disparity.astype('uint8'))
        # cv2.waitKey(0)


        # -----------------------------------------
        # image to cam coordinate system
        # -----------------------------------------

        x, y, z, = [], [], []
        b, g, r = [], [], []

        img = self.read_image(img_file_name)
        for u in range(0, 1280, 1):
            for v in range(0, 360, 1):
                z_pos = (0.6 * 806.87) / (disparity[v, u] + 0.000001)
                x_pos = (u - 640) * z_pos / 537.023764
                y_pos = (v - 180) * z_pos / 537.023764

                z.append(z_pos)
                x.append(x_pos)
                y.append(y_pos)

                b.append(img[v, u, 0])
                g.append(img[v, u, 1])
                r.append(img[v, u, 2])

        data_len = len(z)
        z_ = np.array(z).reshape(data_len, 1)
        x_ = np.array(x).reshape(data_len, 1)
        y_ = np.array(y).reshape(data_len, 1)

        b_ = np.array(b).reshape(data_len, 1)
        g_ = np.array(g).reshape(data_len, 1)
        r_ = np.array(r).reshape(data_len, 1)


        # -----------------------------------------
        # cam to vehicle coordinate system
        # -----------------------------------------

        # data_len x 4 (x, y, z, 1)
        cam_points = np.concatenate([x_.reshape(data_len, 1), y_.reshape(data_len, 1), z_.reshape(data_len, 1),
                                     np.ones_like(z_.reshape(data_len, 1))], axis=1)
        veh_points = np.matmul(self.Rt, cam_points.T).T

        y_v = np.copy(veh_points[:, 0])
        x_v = np.copy(veh_points[:, 1])
        z_v = np.copy(veh_points[:, 2])

        data_len = len(z_v)
        yxz_pts = self.rotate_points_cloud(
            np.concatenate([y_v.reshape(data_len, 1), x_v.reshape(data_len, 1), z_v.reshape(data_len, 1)], axis=1),
            (rot_angle, 0, 0))

        y_v = np.copy(yxz_pts[:, 0])
        x_v = np.copy(yxz_pts[:, 1])
        z_v = np.copy(yxz_pts[:, 2])

        # -----------------------------------------
        # 3D view in vehicle coordinate system
        # -----------------------------------------

        x_range = (-70, 70) # lateral
        y_range = (-20, 100) # longitudinal
        z_range = (-5, 5) # vertical

        x_lim = self.in_range_points(x_v, x_v, y_v, z_v, x_range, y_range, z_range)
        y_lim = scale * self.in_range_points(y_v, x_v, y_v, z_v, x_range, y_range, z_range)

        b_lim = self.in_range_points(b_, x_v, y_v, z_v, x_range, y_range, z_range)
        g_lim = self.in_range_points(g_, x_v, y_v, z_v, x_range, y_range, z_range)
        r_lim = self.in_range_points(r_, x_v, y_v, z_v, x_range, y_range, z_range)


        # -----------------------------------------
        # Top view of points in vehicle coordinate system
        # -----------------------------------------

        # for displaying images
        img_size_r = int(1080/4)
        img_size_c = int(1920/4)
        img = np.zeros(shape=(img_size_r, img_size_c, 3))

        row_axis_range = int((y_range[1] - y_range[0]))
        col_axis_range = int((x_range[1] - x_range[0]))

        scale_r = (img_size_r - 1) / row_axis_range
        scale_c = (img_size_c - 1) / col_axis_range

        row_img = -(y_lim * scale_r).astype(np.int32)
        col_img = (x_lim * scale_c).astype(np.int32)


        row_img = row_img + int(np.trunc(y_range[1] * scale_r))
        col_img = col_img - int(np.trunc(x_range[0] * scale_c))

        ch0 = np.copy(img[:, :, 0])
        ch1 = np.copy(img[:, :, 1])
        ch2 = np.copy(img[:, :, 2])

        ch0[row_img.astype('int32'), col_img.astype('int32')] = np.squeeze(b_lim)
        ch1[row_img.astype('int32'), col_img.astype('int32')] = np.squeeze(g_lim)
        ch2[row_img.astype('int32'), col_img.astype('int32')] = np.squeeze(r_lim)

        # array to img
        img[:, :, 0] = ch0.astype('uint8')
        img[:, :, 1] = ch1.astype('uint8')
        img[:, :, 2] = ch2.astype('uint8')

        # cv2.imshow('test', img.astype('uint8'))
        # cv2.waitKey(0)

        # -----------------------------------------
        # path top view
        # -----------------------------------------

        # original path
        path3d_rot = self.rotate_points_cloud(path3d[:, 0:3], (rot_angle, 0, 0))

        y_p = path3d_rot[:, 0].reshape(path3d.shape[0], 1)
        x_p = path3d_rot[:, 1].reshape(path3d.shape[0], 1)

        y_pel = -(scale * y_p * scale_r).astype(np.int32) + int(np.trunc(y_range[1] * scale_r))
        x_pel = (x_p * scale_c).astype(np.int32) - int(np.trunc(x_range[0] * scale_c))

        for i in range(len(x_pel)):
            img = cv2.circle(img, (x_pel[i], y_pel[i]), 1, (0, 0, 255), -1)
            if (i == obs_len - 1):
                img = cv2.circle(img, (x_pel[i], y_pel[i]), 3, (0, 0, 255), 1)


        return np.fliplr(img).astype('uint8')

    def top_view_point_clouds(self, file_name, x_range, y_range, z_range, yaw):

        def rotate_points_cloud(xyz_pts, yaw):

            xyz_pts = np.concatenate([xyz_pts, np.ones(shape=(xyz_pts.shape[0], 1))], axis=1)  # N x 4

            t = np.array([0, 0, 0]).reshape((3, 1))
            # heading = math.radians(self.gpsimu[index, 4])
            yaw_rad = math.radians(yaw)
            Rz = [math.cos(yaw_rad), math.sin(yaw_rad), 0, -1.0 * math.sin(yaw_rad), math.cos(yaw_rad), 0, 0, 0, 1]
            Rz = np.asarray(Rz).reshape((3, 3))
            R = self.affinematrix(Rz, t)  # 4 x 4

            return np.matmul(R, xyz_pts.T).T


        # original points cloud
        points = self.read_point_cloud(file_name)

        # rotate points according to car heading
        # points_rot = rotate_points_cloud(points[:, 0:3], yaw)
        points_rot = np.copy(points[:, 0:3])


        # draw on image
        x = points_rot[:, 0]
        y = points_rot[:, 1]
        z = points_rot[:, 2]

        # extract in-range points
        x_lim = self.in_range_points(x, x, y, z, x_range, y_range, z_range)
        y_lim = self.in_range_points(y, x, y, z, x_range, y_range, z_range)

        # for displaying images
        axis_range = int((y_range[1] - y_range[0]))
        scale = (512 - 1) / axis_range

        # to pixel domain
        col_img = (x_lim * scale).astype(np.int32)
        row_img = -(y_lim * scale).astype(np.int32)

        col_img -= int(np.trunc(y_range[0] * scale))
        row_img += int(np.trunc(y_range[1] * scale))

        img = np.zeros(shape=(512, 512, 3))


        ch0 = np.copy(img[:, :, 0])
        ch1 = np.copy(img[:, :, 1])
        ch2 = np.copy(img[:, :, 2])

        ch0[row_img.astype('int32'), col_img.astype('int32')] = 256 * np.ones_like(x_lim)
        ch1[row_img.astype('int32'), col_img.astype('int32')] = 128 * np.ones_like(x_lim)
        ch2[row_img.astype('int32'), col_img.astype('int32')] = 128 * np.ones_like(x_lim)

        # array to img
        img[:, :, 0] = ch0.astype('uint8')
        img[:, :, 1] = ch1.astype('uint8')
        img[:, :, 2] = ch2.astype('uint8')

        return img

    def draw_point_clouds_on_left_img(self, file_name_img, file_name_pt, x_range, y_range, z_range, yaw):

        # original image (left cam)
        img = self.read_image(file_name_img)

        # original points cloud
        points = self.read_point_cloud(file_name_pt)

        # rotate points according to car heading
        points_rot = self.rotate_points_cloud(points[:, 0:3], (0.01, 0.0, 0.025))

        # draw on image
        x = np.copy(points_rot[:, 0])
        y = np.copy(points_rot[:, 1])
        z = np.copy(points_rot[:, 2])

        # extract in-range points
        x_lim = self.in_range_points(x, x, y, z, x_range, y_range, z_range)
        y_lim = self.in_range_points(y, x, y, z, x_range, y_range, z_range)
        z_lim = self.in_range_points(z, x, y, z, x_range, y_range, z_range)
        dist = np.sqrt(x_lim ** 2 + y_lim ** 2 + z_lim ** 2)
        color = depth_color(dist, 0, 50)

        offset_x = 0.6
        offset_y = -1.5
        offset_z = -1.25

        x_lim += offset_x
        y_lim += offset_y # move to the ego center
        z_lim += offset_z  # move to the bottom


        # homo
        data_len = len(x_lim)
        points_homo = np.concatenate([-1.0*y_lim.reshape(data_len, 1), 1.0*x_lim.reshape(data_len, 1),
                                      z_lim.reshape(data_len, 1), np.ones_like(z_lim.reshape(data_len, 1))], axis=1)

        # cam
        A = np.matmul(np.linalg.inv(self.Rt), points_homo.T)
        B = np.matmul(self.K, A)

        x_pel = (B[0, :] * 1.0 / B[2, :]).astype('int32')
        y_pel = (B[1, :] * 1.0 / B[2, :]).astype('int32')

        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        for i in range(data_len):
            if (x_pel[i] > -1 and x_pel[i] < 1280 and y_pel[i] > -1 and y_pel[i] < 360):
                cv2.circle(img, (x_pel[i], y_pel[i]), 1, (int(color[i]), 255, 255), -1)

        return cv2.cvtColor(img, cv2.COLOR_HSV2BGR)



    def generate_drvact_text(self, drvact):

        text = '[warnings] drvact label is not defined ...'
        if (drvact == 1):
            text = 'Go'
        elif (drvact == 2):
            text = 'Turn Left'
        elif (drvact == 3):
            text = 'Turn Right'
        elif (drvact == 4):
            text = 'U-turn'
        elif (drvact == 5):
            text = 'Left LC'
        elif (drvact == 6):
            text = 'Right LC'
        elif (drvact == 7):
            text = 'Avoidance'
        elif (drvact == 8):
            text = 'Left Way'
        elif (drvact == 9):
            text = 'Right Way'

        return text


def depth_color(val, min_d=0, max_d=120):
    """
        print Color(HSV's H value) corresponding to distance(m)
        close distance = red , far distance = blue
    """
    np.clip(val, 0, max_d, out=val)  # max distance is 120m but usually not usual
    return (((val - min_d) / (max_d - min_d)) * 120).astype(np.uint8)


