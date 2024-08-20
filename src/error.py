#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Error calculated with the methods described from
    D. Prokhorov, D. Zhukov, O. Barinova, A. Vorontsova, and A. Konushin, "Measuring robustness of Visual SLAM,"
    arXiv:1910.04755 [cs], Oct. 2019, Accessed: Apr. 23, 2021. [Online]. Available: http://arxiv.org/abs/1910.04755
"""

from operator import index
import numpy as np
import matplotlib.pyplot as plt
import copy
from rospy.rostime import genpy

from src.trajectory import Trajectory
from src.quaternion import SLERP


class Error:
    lengths = [100, 200, 300, 400, 500, 600, 700, 800]  # Longitudes de segmentos
    num_lengths = len(lengths)

    def __init__(self, reference=None, estimate=None, delta=1):
        """Calculate Error(APE, RPE)
        APE

        Args:
            reference (Trajectory): reference trajectory or ground truth trajectory. Defaults to None.
            estimate  (Trajectory): estimated trajectory for evaluation. Defaults to None.
            delta  (int, optional): local accuracy of the trajectory over a fixed time interval delta(for RPE). Defaults to 1
        """
        self.name = estimate.name
        print("Calculating {}'s Error with respect to Ground Truth Data".format(self.name))
        self.is_short = False

        self.reference, self.estimate = self._post_process(copy.deepcopy(reference), copy.deepcopy(estimate))
        self.time = self.estimate.time

        self.errors = self.calcSequenceErrors(self.reference, self.estimate)

        self.ape_trans, self.ape_rot = self.APE(self.reference, self.estimate)
        self.ape_tans_stat = self._statistics(self.ape_trans)
        self.ape_rot_stat = self._statistics(self.ape_rot)

        self.rpe_trans, self.rpe_rot = self.RPE(self.reference, self.estimate, delta)
        self.rpe_tans_stat = self._statistics(self.rpe_trans)
        self.rpe_rot_stat = self._statistics(self.rpe_rot)
        self.seq_trans_stat = self._kitti_statistics(self.errors, 't_err')
        self.seq_rot_stat = self._kitti_statistics(self.errors, 'r_err')

    def _post_process(self, gt, test):
        orientation, trajectory, dur = [], [], []
        index = []
        for i in range(gt.length):
            time = gt.time[i]
            # print("###########################NEW_TIME###########################")
            for j in range(test.length - 1):
                #print if test.time[j] < time < test.time[j + 1]:
                # print(f"{test.time[j]:.2f} <= {time:.2f} <= {test.time[j + 1]:.2f}")
                if test.time[j] <= time <= test.time[j + 1]:
                    # print("@@@@@@@@@@@@@@@@@@@@@@@@ENTER@@@@@@@@@@@@@@@@@@@@@")
                    alpha = (time - test.time[j]) / (test.time[j + 1] - test.time[j])
                    # Interpolate orientation using SLERP
                    orientation.append(SLERP(test.orientation[j], test.orientation[j + 1], alpha))
                    # Interpolate trajectory using linear interpolation
                    trajectory.append((1 - alpha) * test.trajectory[j] + alpha * test.trajectory[j + 1])
                    dur.append(time)
                    index.append(i)
                    # now delete from 0 to index from the test.time and test.trajectory. This is to reduce the time complexity
                    # test.time = test.time[j:]
                    # test.trajectory = test.trajectory[j:]
                    # test.orientation = test.orientation[j:]
                    # print("trajectory = ", trajectory[-1])
                
                    

                    
                    
                    break
                    
                # else:
                #     print("Error: Time is out of range", i,j)
                #     print("max index: ", test.length)
        index = np.array(index)

        gt.trajectory = gt.trajectory[index]
        gt.orientation = gt.orientation[index]
        gt.time = gt.time[index]
        gt.length = gt.trajectory.shape[0]

        test.trajectory = np.array(trajectory)
        test.orientation = np.array(orientation)
        test.time = np.array(dur)
        test.length = test.trajectory.shape[0]
        # show stats
        print("Ground Truth Data: ", gt.length)
        print("Estimated Data: ", test.length)
        print("test.lenght: ", test.length)

        return gt, test

    def _statistics(self, error):
        std = np.std(error)
        mean = np.mean(error)
        median = np.median(error)
        minimum = np.min(error)
        maximum = np.max(error)
        rmse = np.sqrt((np.asarray(error) ** 2).mean())

        return [mean, std, median, minimum, maximum, rmse]
    def _kitti_statistics(self, errors, key):
        # Extraer los valores usando la clave proporcionada
        values = [error[key] for error in errors]

        # Convertir a un array de NumPy
        values_array = np.asarray(values)

        # Verificar si el array no está vacío
        if values_array.size > 0:
            # Calcular estadísticas si el array tiene elementos
            mean = np.mean(values_array)
            std = np.std(values_array)
            median = np.median(values_array)
            minimum = np.min(values_array)
            maximum = np.max(values_array)
            rmse = np.sqrt((values_array ** 2).mean())
        else:
            # Si el array está vacío, asignar NaN o cualquier otro valor por defecto
            mean = std = median = minimum = maximum = rmse = np.nan

        return [mean, std, median, minimum, maximum, rmse]

    def APE(self, gt, test):
        target_mean = gt.trajectory.mean(0)
        estimate_mean = test.trajectory.mean(0)

        target = gt.trajectory - target_mean
        estimate = test.trajectory - estimate_mean

        W = np.dot(target.T, estimate)
        U, _, V = np.linalg.svd(W, full_matrices=True, compute_uv=True)

        R = np.dot(U, V)
        t = target_mean - np.dot(R, estimate_mean)
        T = np.vstack([np.hstack([R, t.reshape(3, 1)]), np.array([0, 0, 0, 1])])

        ape_trans, ape_rot = [], []
        for i in range(gt.length):
            Q = gt.pose_matrix(i)
            P = test.pose_matrix(i)
            E = np.dot(np.linalg.inv(Q), np.dot(T, P))

            ape_trans.append(np.linalg.norm(E[:3, 3]))
            ape_rot.append(np.arccos((np.trace(E[:3, :3]) - 1) / 2))

        ''' direct comparison (no trajectory matching using Horn's method) 
        for i in range(gt.length):
            translation_error = np.linalg.norm(gt.trajectory[i] - test.trajectory[i])
            rotation_error = np.arccos((gt.orientation[i] ** -1 * test.orientation[i]).w) * 2 - np.pi
            ape_trans.append(translation_error)
            ape_rot.append(rotation_error)
        '''
        return ape_trans, ape_rot

    def RPE(self, gt, test, delta):
        rpe_trans, rpe_rot = [], []
        for i in range(gt.length - delta):
            Q = gt.pose_matrix(i)
            Q_delta = gt.pose_matrix(i+delta)
            Q = np.dot(np.linalg.inv(Q), Q_delta)
            P = test.pose_matrix(i)
            P_delta = test.pose_matrix(i+delta)
            P = np.dot(np.linalg.inv(P), P_delta)

            E = np.dot(np.linalg.inv(Q), P)

            rpe_trans.append(np.linalg.norm(E[:3, 3]))
            cos_angle = (np.trace(E[:3, :3]) - 1) / 2
            
                

            cos_angle_clamped = np.clip(cos_angle, -1, 1)
            rpe_rot.append(np.arccos(cos_angle_clamped))
            if (abs(cos_angle)>1):
                print("i= "+str(i) +" cosangle= "+str(cos_angle)+" clamped angle= "+str(cos_angle_clamped))
            # rpe_rot.append(np.arccos((np.trace(E[:3, :3]) - 1) / 2))
        ''' direct comparison (no conversion to pose matrix)
        for i in range(gt.length-delta):
            translation_error = np.linalg.norm((test.trajectory[i+delta]-test.trajectory[i]) - (gt.trajectory[i+delta] - gt.trajectory[i]))
            rotation_error = np.arccos((gt.orientation[i+delta]**-1 * gt.orientation[i] * test.orientation[i]**-1 * test.orientation[i+delta]).w) * 2 - np.pi
            rpe_trans.append(translation_error)
            rpe_rot.append(rotation_error)
        '''
        return rpe_trans, rpe_rot
    def calcSequenceErrors(self, gt, test):
        dist = self.trajectoryDistances(gt)
        step_size = 10  # every second
        errors = []

        for first_frame in range(0, gt.length, step_size):
            for i in range(self.num_lengths):
                len_segment = self.lengths[i]
                last_frame = self.lastFrameFromSegmentLength(dist, first_frame, len_segment)

                if last_frame == -1:
                    continue

                pose_delta_gt = np.linalg.inv(gt.pose_matrix(first_frame)) @ gt.pose_matrix(last_frame)
                pose_delta_est = np.linalg.inv(test.pose_matrix(first_frame)) @ test.pose_matrix(last_frame)
                pose_error = np.linalg.inv(pose_delta_est) @ pose_delta_gt

                r_err = self.rotationError(pose_error)
                t_err = self.translationError(pose_error)

                num_frames = last_frame - first_frame + 1
                speed = len_segment / (0.1 * num_frames)

                errors.append({
                    'first_frame': first_frame,
                    'r_err': r_err / len_segment,
                    't_err': t_err / len_segment,
                    'len': len_segment,
                    'speed': speed
                })

        return errors

    def trajectoryDistances(self, trajectory):
        dist = [0]
        for i in range(1, trajectory.length):
            dx = trajectory.trajectory[i][0] - trajectory.trajectory[i - 1][0]
            dy = trajectory.trajectory[i][1] - trajectory.trajectory[i - 1][1]
            dz = trajectory.trajectory[i][2] - trajectory.trajectory[i - 1][2]
            dist.append(dist[-1] + np.sqrt(dx**2 + dy**2 + dz**2))
        return dist

    def lastFrameFromSegmentLength(self, dist, first_frame, len_segment):
        for i in range(first_frame, len(dist)):
            if dist[i] > dist[first_frame] + len_segment:
                return i
        return -1

    def rotationError(self, pose_error):
        a = pose_error[0, 0]
        b = pose_error[1, 1]
        c = pose_error[2, 2]
        d = 0.5 * (a + b + c - 1.0)
        return np.arccos(np.clip(d, -1.0, 1.0))

    def translationError(self, pose_error):
        dx = pose_error[0, 3]
        dy = pose_error[1, 3]
        dz = pose_error[2, 3]
        return np.sqrt(dx**2 + dy**2 + dz**2)

def plotAPE(errors):
    plt.figure(figsize=(6, 5))
    plt.figure(figsize=(19.20, 10.80))  # Tamaño de la figura en pulgadas para Full HD
    plt.subplot(2, 1, 1)
    for error in errors:
        plt.plot(error.time, error.ape_trans, label=error.name)
    plt.legend()
    plt.title('APE')
    plt.ylabel('ape[m]')

    plt.subplot(2, 1, 2)
    for error in errors:
        plt.plot(error.time, error.ape_rot, label=error.name)
    plt.legend()
    plt.xlabel('time[nano_sec]')
    plt.ylabel('ape[rad]')

def plotRPE(errors):
    plt.figure(figsize=(6, 5))
    plt.figure(figsize=(19.20, 10.80))  # Tamaño de la figura en pulgadas para Full HD
    plt.subplot(2, 1, 1)
    for error in errors:
        plt.plot(error.time[1:], error.rpe_trans, label=error.name)
    plt.legend()
    plt.title('RPE')
    plt.ylabel('rpe[m]')

    plt.subplot(2, 1, 2)
    for error in errors:
        plt.plot(error.time[1:], error.rpe_rot, label=error.name)
    plt.legend()
    plt.xlabel('time[nano_sec]')
    plt.ylabel('rpe[rad]')

def plotAPEStats(errors):
    import pandas as pd
    index = ['mean', 'std', 'median', 'minimum', 'maximum', 'rmse']
    trans_dic = {}
    rot_dic = {}
    for error in errors:
        trans_dic[error.name] = error.ape_tans_stat
        rot_dic[error.name] = error.ape_rot_stat
    trans_data = pd.DataFrame(trans_dic, index=index)
    rot_data = pd.DataFrame(rot_dic, index=index)
    fig = plt.figure(figsize=(6, 5))

    ax = fig.add_subplot(2, 1, 1)
    trans_data.plot.barh(ax=ax)
    ax.title.set_text('APE Statistics')
    ax.set_xlabel('APE Translation')
    ax = fig.add_subplot(2, 1, 2)
    rot_data.plot.barh(ax=ax)
    ax.set_xlabel('APE Rotation')

def plotRPEStats(errors):
    import pandas as pd
    index = ['mean', 'std', 'median', 'minimum', 'maximum', 'rmse']
    trans_dic = {}
    rot_dic = {}
    for error in errors:
        trans_dic[error.name] = error.rpe_tans_stat
        rot_dic[error.name] = error.rpe_rot_stat
    trans_data = pd.DataFrame(trans_dic, index=index)
    rot_data = pd.DataFrame(rot_dic, index=index)

    fig = plt.figure(figsize=(6, 5))
    ax = fig.add_subplot(2, 1, 1)
    trans_data.plot.barh(ax=ax)
    ax.title.set_text('RPE Statistics')
    ax.set_xlabel('RPE Translation')
    ax = fig.add_subplot(2, 1, 2)
    rot_data.plot.barh(ax=ax)
    ax.set_xlabel('RPE Rotation')












def printSequenceErrors(errors):
    for error in errors:
        lengths = [e['len'] for e in error.errors]
        t_errors = [e['t_err'] for e in error.errors]
        r_errors = [e['r_err'] for e in error.errors]
        print(f"Errors for {error.name}:")
        print(f"Lengths: {lengths}")
        print(f"Translation Errors: {t_errors}")
        print(f"Rotation Errors: {r_errors}")
        print("\n")

def printSequenceErrorStats(errors):
    index = ['mean', 'std', 'median', 'minimum', 'maximum', 'rmse']
    
    # Estadísticas de errores de traslación
    trans_dic = {}
    for error in errors:
        t_errors = [e['t_err'] for e in error.errors]
        trans_dic[error.name] = _statistics(t_errors)
    
    print("Sequence Translation Error Statistics:")
    for name, stats in trans_dic.items():
        print(f"{name}:")
        for i, stat in enumerate(stats):
            print(f"  {index[i]}: {stat}")
        print("\n")
    
    # Estadísticas de errores de rotación
    rot_dic = {}
    for error in errors:
        r_errors = [e['r_err'] for e in error.errors]
        rot_dic[error.name] = _statistics(r_errors)
    
    print("Sequence Rotation Error Statistics:")
    for name, stats in rot_dic.items():
        print(f"{name}:")
        for i, stat in enumerate(stats):
            print(f"  {index[i]}: {stat}")
        print("\n")

def _statistics(error):
    # Convertir error a un array de NumPy para garantizar que todas las operaciones funcionen correctamente
    error_array = np.asarray(error)

    # Verificar si el array no está vacío
    if error_array.size > 0:
        mean = np.mean(error_array)
        std = np.std(error_array)
        median = np.median(error_array)
        minimum = np.min(error_array)
        maximum = np.max(error_array)
        rmse = np.sqrt((error_array ** 2).mean())
    else:
        # Si el array está vacío, asignar NaN o cualquier otro valor por defecto
        mean = std = median = minimum = maximum = rmse = np.nan

    return [mean, std, median, minimum, maximum, rmse]
def _kitti_statistics(errors, key):
    values = [error[key] for error in errors]
    std = np.std(values)
    mean = np.mean(values)
    median = np.median(values)
    minimum = np.min(values)
    maximum = np.max(values)
    rmse = np.sqrt((np.asarray(values) ** 2).mean())
    return [mean, std, median, minimum, maximum, rmse]