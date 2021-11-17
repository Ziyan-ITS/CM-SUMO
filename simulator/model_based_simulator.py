import math
import copy


class simulator:

    def __init__(self, vehicle_set):
        self.time = math.ceil(abs(vehicle_set[-1]['x']) / 20) + 5
        self.reason_fail = {}
        self.vehicle_info = vehicle_set
        self.vehicle_trajectory = {vehicle['vehicle_id']: [vehicle] for vehicle in vehicle_set}

    def run_control_simulation(self, control_vehicle):
        new_vehicle_info = self.vehicle_info
        for t in range(self.time):
            new_vehicle_info = sorted(new_vehicle_info, key=lambda x: x['x'], reverse=True)
            ramp_info = [v_info for v_info in new_vehicle_info if
                         v_info["vehicle_lane"] == 3]

            mainline_info = [v_info for v_info in new_vehicle_info if
                             v_info["vehicle_lane"] == 1]

            merge_section = [v_info for v_info in new_vehicle_info if
                             v_info["vehicle_lane"] == 2]
            new_vehicle_info = []
            All_vehicle = {'merge': merge_section, 'ramp': ramp_info, 'mainline': mainline_info}
            follower_set = set()
            for section, vehicle_info in All_vehicle.items():
                for index, vehicle in enumerate(vehicle_info):
                    if section == 'mainline' and vehicle['vehicle_id'] in follower_set:
                        continue
                    if index != 0:
                        leader = vehicle_info[index - 1]
                        new_speed, acc, distance = self.gipps(vehicle['vehicle_speed'], vehicle['vehicle_dis'],
                                                              leader['vehicle_dis'], leader['vehicle_speed'])
                    elif index == 0 and section == 'merge':
                        new_speed, acc, distance = self.gipps(vehicle['vehicle_speed'], vehicle['vehicle_dis'],
                                                              250, 0)
                    else:
                        new_speed, acc, distance = self.gipps(vehicle['vehicle_speed'], vehicle['vehicle_dis'])
                    new_state = copy.deepcopy(vehicle)
                    new_loc = vehicle['vehicle_dis'] + distance
                    new_state['vehicle_speed'], new_state['vehicle_dis'], new_state[
                        'vehicle_acceleration'] = new_speed, new_loc, acc
                    if section == 'merge':
                        target_location = vehicle['vehicle_dis']
                        mainline_leader, mainline_follower = self.find_vehicle(target_location, All_vehicle['mainline'])
                        if not mainline_leader and not mainline_follower:
                            merge_decision, reason, state_new_alpha, state_new_f = self.lane_changing(None, None, vehicle['vehicle_dis'],None,None,vehicle['vehicle_speed'])
                        elif not mainline_leader:
                            merge_decision, reason, state_new_alpha, state_new_f = self.lane_changing(None, mainline_follower['vehicle_dis'], vehicle['vehicle_dis'], None, mainline_follower['vehicle_speed'], vehicle['vehicle_speed'])
                        elif not mainline_follower:
                            merge_decision, reason, state_new_alpha, state_new_f = self.lane_changing(mainline_leader['vehicle_dis'], None, vehicle['vehicle_dis'], mainline_leader['vehicle_speed'], None, vehicle['vehicle_speed'])
                        else:
                            merge_decision, reason, state_new_alpha, state_new_f = self.lane_changing(mainline_leader['vehicle_dis'], mainline_follower['vehicle_dis'], vehicle['vehicle_dis'], mainline_leader['vehicle_speed'], mainline_follower['vehicle_speed'], vehicle['vehicle_speed'])
                        if merge_decision == 1:
                            new_state['vehicle_lane'] = 1
                            if state_new_alpha:
                                new_loc = vehicle['vehicle_dis'] + state_new_alpha[2]
                                new_state['vehicle_speed'], new_state['vehicle_dis'], new_state[
                                    'vehicle_acceleration'] = state_new_alpha[0], new_loc, state_new_alpha[1]
                            if state_new_f:
                                new_loc = vehicle['vehicle_dis'] + state_new_f[2]
                                mainline_follower['vehicle_speed'], mainline_follower['vehicle_dis'], mainline_follower[
                                    'vehicle_acceleration'] = state_new_f[0], new_loc, state_new_f[1]
                                new_vehicle_info.append(mainline_follower)
                                self.vehicle_trajectory[mainline_follower['vehicle_id']].append(mainline_follower)
                                follower_set.add(mainline_follower['vehicle_id'])
                        else:
                            if vehicle['vehicle_id'] not in self.reason_fail:
                                if mainline_leader and mainline_follower:
                                    self.reason_fail[vehicle['vehicle_id']] = [reason, mainline_leader['vehicle_id'],
                                                                               mainline_follower['vehicle_id'], t]
                                elif not mainline_leader:
                                    self.reason_fail[vehicle['vehicle_id']] = [reason, None,
                                                                               mainline_follower['vehicle_id'], t]
                                else:
                                    self.reason_fail[vehicle['vehicle_id']] = [reason, mainline_leader['vehicle_id'],
                                                                               None, t]
                    elif section == 'ramp' and new_loc > 0:
                        new_state['vehicle_lane'] = 2
                    if vehicle['vehicle_id'] in control_vehicle and t < len(control_vehicle[vehicle['vehicle_id']]):
                        new_state = copy.deepcopy(control_vehicle[vehicle['vehicle_id']][t])
                    new_vehicle_info.append(new_state)
                    self.vehicle_trajectory[new_state['vehicle_id']].append(new_state)

    def run_basic_simulation(self):
        new_vehicle_info = self.vehicle_info
        for t in range(self.time):
            new_vehicle_info = sorted(new_vehicle_info, key=lambda x: x['x'], reverse=True)
            ramp_info = [v_info for v_info in new_vehicle_info if
                         v_info["vehicle_lane"] == 3]

            mainline_info = [v_info for v_info in new_vehicle_info if
                             v_info["vehicle_lane"] == 1]

            merge_section = [v_info for v_info in new_vehicle_info if
                             v_info["vehicle_lane"] == 2]
            new_vehicle_info = []
            All_vehicle = {'merge': merge_section, 'ramp': ramp_info, 'mainline': mainline_info}
            follower_set = set()
            for section, vehicle_info in All_vehicle.items():
                for index, vehicle in enumerate(vehicle_info):
                    if section == 'mainline' and vehicle['vehicle_id'] in follower_set:
                        continue
                    if index != 0:
                        leader = vehicle_info[index - 1]
                        new_speed, acc, distance = self.gipps(vehicle['vehicle_speed'], vehicle['vehicle_dis'],
                                                              leader['vehicle_dis'], leader['vehicle_speed'])
                    elif index == 0 and section == 'merge':
                        new_speed, acc, distance = self.gipps(vehicle['vehicle_speed'], vehicle['vehicle_dis'],
                                                              250, 0)
                    else:
                        new_speed, acc, distance = self.gipps(vehicle['vehicle_speed'], vehicle['vehicle_dis'])
                    new_state = copy.deepcopy(vehicle)
                    new_loc = vehicle['vehicle_dis'] + distance
                    new_state['vehicle_speed'], new_state['vehicle_dis'], new_state[
                        'vehicle_acceleration'] = new_speed, new_loc, acc
                    if section == 'merge':
                        target_location = vehicle['vehicle_dis']
                        mainline_leader, mainline_follower = self.find_vehicle(target_location, All_vehicle['mainline'])
                        if not mainline_leader and not mainline_follower:
                            merge_decision, reason, state_new_alpha, state_new_f = self.lane_changing(None, None, vehicle['vehicle_dis'],None,None,vehicle['vehicle_speed'])
                        elif not mainline_leader:
                            merge_decision, reason, state_new_alpha, state_new_f = self.lane_changing(None, mainline_follower['vehicle_dis'], vehicle['vehicle_dis'], None, mainline_follower['vehicle_speed'], vehicle['vehicle_speed'])
                        elif not mainline_follower:
                            merge_decision, reason, state_new_alpha, state_new_f = self.lane_changing(mainline_leader['vehicle_dis'], None, vehicle['vehicle_dis'], mainline_leader['vehicle_speed'], None, vehicle['vehicle_speed'])
                        else:
                            merge_decision, reason, state_new_alpha, state_new_f = self.lane_changing(mainline_leader['vehicle_dis'], mainline_follower['vehicle_dis'], vehicle['vehicle_dis'], mainline_leader['vehicle_speed'], mainline_follower['vehicle_speed'], vehicle['vehicle_speed'])
                        if merge_decision == 1:
                            new_state['vehicle_lane'] = 1
                            if state_new_alpha:
                                new_loc = vehicle['vehicle_dis'] + state_new_alpha[2]
                                new_state['vehicle_speed'], new_state['vehicle_dis'], new_state[
                                    'vehicle_acceleration'] = state_new_alpha[0], new_loc, state_new_alpha[1]
                            if state_new_f:
                                new_loc = vehicle['vehicle_dis'] + state_new_f[2]
                                mainline_follower['vehicle_speed'], mainline_follower['vehicle_dis'], mainline_follower[
                                    'vehicle_acceleration'] = state_new_f[0], new_loc, state_new_f[1]
                                new_vehicle_info.append(mainline_follower)
                                self.vehicle_trajectory[mainline_follower['vehicle_id']].append(mainline_follower)
                                follower_set.add(mainline_follower['vehicle_id'])
                        else:
                            if vehicle['vehicle_id'] not in self.reason_fail:
                                if mainline_leader and mainline_follower:
                                    self.reason_fail[vehicle['vehicle_id']] = [reason, mainline_leader['vehicle_id'],
                                                                               mainline_follower['vehicle_id'], t]
                                elif not mainline_leader:
                                    self.reason_fail[vehicle['vehicle_id']] = [reason, None,
                                                                               mainline_follower['vehicle_id'], t]
                                else:
                                    self.reason_fail[vehicle['vehicle_id']] = [reason, mainline_leader['vehicle_id'],
                                                                               None, t]
                    elif section == 'ramp' and new_loc > 0:
                        new_state['vehicle_lane'] = 2
                    new_vehicle_info.append(new_state)
                    self.vehicle_trajectory[new_state['vehicle_id']].append(new_state)

    def find_vehicle(self, position, mainline):
        if len(mainline) == 0:
            return None, None
        for index, vehicle in enumerate(mainline):
            if vehicle['vehicle_dis'] > position and index == len(mainline) - 1:
                return vehicle, None
            elif vehicle['vehicle_dis'] < position and index == 0:
                return None, vehicle
            elif vehicle['vehicle_dis'] > position > mainline[index + 1]['vehicle_dis']:
                return vehicle, mainline[index + 1]

    @staticmethod
    def gipps(target_s, target_l, leader_l=None, leader_s=None, Le=6, tau=1, DS=20, a=2, b=2):
        if leader_l:
            V_check = b ** 2 * tau ** 2 + leader_s ** 2 + 2 * b * (abs(target_l - leader_l) - Le)
            if V_check <= 4:
                V_safe = 0
                new_speed = max(0, min([DS, V_safe, target_s + a * tau]))
            else:
                V_safe = -b * tau + math.sqrt(b ** 2 * tau ** 2 + leader_s ** 2 + 2 * b * (abs(target_l - leader_l) - Le))
                new_speed = max(0, min([DS, V_safe, target_s + a * tau]))
        else:
            new_speed = min(DS, target_s + a * tau)
        acc = (new_speed - target_s) / tau
        distance = (target_s + new_speed) / 2 * tau
        return new_speed, acc, distance

    @staticmethod
    def lane_changing(LL_hat=None, FL_hat=None, CL=None, LV_hat=None, FV_hat=None, CV=None, Le=6, Le_f=6, tau=1, DS=20,
                      a=2, b=2,
                      b_safe=6.5, eta1=0.15):
        """
        :param eta1:
        :param b_safe:
        :param LL_hat: mainline leader's location
        :param FL_hat:mainline follower's location
        :param LV_hat:
        :param FV_hat:
        :param Le:
        :param Le_f:
        :param tau:
        :param CL:
        :param CV:
        :param DS:
        :param a:
        :param b:
        :return:
        """
        if CV > 25:
            eta2 = 0.1
        elif CV < 15:
            eta2 = 0.05
        else:
            eta2 = 0.05 + 0.005 * (CV - 15)
        state_new_alpha = tuple()
        state_new_f = []
        reason = 0
        if not FL_hat and not LL_hat:
            merge_decision = 1
            reason = 0
            state_new_alpha = simulator.gipps(CV, CL, LL_hat, LV_hat, Le, tau, DS, a, b)

        elif not FL_hat and LL_hat:
            if LL_hat - CL < Le:
                merge_decision = 0
                if LL_hat - CL < -5:
                    reason = 5
                else:
                    reason = 1
            else:
                state_new_alpha = simulator.gipps(CV, CL, LL_hat, LV_hat, Le, tau, DS, a, b)
                if abs(state_new_alpha[1]) > b_safe:
                    merge_decision = 0
                    reason = 1
                else:
                    Utility = 1 - eta1 * abs(state_new_alpha[1])
                    if Utility >= 0:
                        merge_decision = 1
                    else:
                        merge_decision = 0
                        reason = 3
            if merge_decision == 1:
                reason = 0
        elif FL_hat and not LL_hat:
            if CL - FL_hat < Le:
                merge_decision = 0
                if CL - FL_hat < -5:
                    reason = 6
                else:
                    reason = 2
            else:
                state_new_f = simulator.gipps(FV_hat, FL_hat, CL, CV, Le, tau, DS, a, b)
                if abs(state_new_f[1]) > b_safe:
                    merge_decision = 0
                    reason = 2
                else:
                    Utility = 1 - eta1 * abs(state_new_f[1])
                    if Utility >= 0:
                        merge_decision = 1
                    else:
                        merge_decision = 0.
                        reason = 3
            if merge_decision == 1:
                reason = 0
        else:
            if LL_hat - CL < Le and CL - FL_hat < Le_f:
                merge_decision = 0
                reason = 4
            elif LL_hat - CL < Le and CL - FL_hat >= Le_f:
                merge_decision = 0
                if CL - LL_hat < -5:
                    reason = 5
                else:
                    reason = 1
            elif LL_hat - CL >= Le and CL - FL_hat < Le_f:
                merge_decision = 0
                if FL_hat - CL < -5:
                    reason = 6
                else:
                    reason = 2
            else:
                state_new_alpha = simulator.gipps(CV, CL, LL_hat, LV_hat, Le, tau, DS, a, b)
                state_new_f = simulator.gipps(FV_hat, FL_hat, CL, CV, Le, tau, DS, a, b)
                if abs(state_new_alpha[1]) > b_safe and abs(state_new_f[1]) > b_safe:
                    merge_decision = 0
                    reason = 4
                elif abs(state_new_alpha[1]) > b_safe >= abs(state_new_f[1]):
                    merge_decision = 0
                    reason = 1
                elif abs(state_new_alpha[1]) <= b_safe < abs(state_new_f[1]):
                    merge_decision = 0
                    reason = 2
                else:
                    merge_decision = 1
            if merge_decision == 1:
                reason = 0
        return merge_decision, reason, state_new_alpha, state_new_f
