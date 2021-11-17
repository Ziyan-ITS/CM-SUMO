import numpy as np
from simulator.model_based_simulator import simulator
from algo.SQP import sqp_lower_control


class upper_controller:
    tau = 1

    def __init__(self, vehicle_info):
        self.ramp_vehicle = vehicle_info.ramp_info
        self.vehicle_info = vehicle_info.vehicle_info
        self.Simulator = simulator(self.vehicle_info)
        self.optimal_cav = []

    def run(self):

        self.Simulator.run_basic_simulation()
        v_t = self.Simulator.vehicle_trajectory
        reason_f = self.Simulator.reason_fail
        while True:
            if not reason_f:
                break
            control_vehicle = {}
            for ramp_v in self.ramp_vehicle:
                if ramp_v['vehicle_id'] in reason_f:
                    reason, leader, follower, tf = reason_f[ramp_v['vehicle_id']]
                    if leader and follower:
                        vehicle_set = [v_t[leader], v_t[ramp_v['vehicle_id']], v_t[follower]]
                    elif not leader:
                        vehicle_set = [None, v_t[ramp_v['vehicle_id']], v_t[follower]]
                    else:
                        vehicle_set = [v_t[leader], v_t[ramp_v['vehicle_id']], None]

                    lower = sqp_lower_control(vehicle_set, tf, 0, reason)
                    x = lower.run_optimization()  # x现在数值严重不吻合，需要合适
                    if x:
                        for index, acc in enumerate(x):
                            vs = lower.control_vehicle_set[index]
                            if vs == 'leader':
                                self.optimal_cav.append({leader: acc[0]})
                                control_vehicle.update(self.change_vehicle_state(leader, acc, v_t))
                            elif vs == 'ramp':
                                control_vehicle.update(self.change_vehicle_state(ramp_v['vehicle_id'], acc, v_t))
                                self.optimal_cav.append({ramp_v['vehicle_id']: acc[0]})
                            else:
                                control_vehicle.update(self.change_vehicle_state(follower, acc, v_t))
                                self.optimal_cav.append({follower: acc[0]})
                        break
            reason_f = None
            if control_vehicle:
                self.Simulator.run_control_simulation(control_vehicle)
                v_t = self.Simulator.vehicle_trajectory
                reason_f = self.Simulator.reason_fail

    def change_vehicle_state(self, vehicle_id, acc, v_t):
        change_vehicle = v_t[vehicle_id]
        for index, ac in enumerate(acc):
            change_vehicle[index]['vehicle_acceleration'] = ac
            speed = change_vehicle[index]['vehicle_speed']
            loc = change_vehicle[index]['vehicle_dis']
            change_vehicle[index+1]['vehicle_speed'] = speed + ac
            change_vehicle[index+1]['vehicle_dis'] = loc + (change_vehicle[index+1]['vehicle_speed'] + speed) / 2 * self.tau
            if index == len(acc) - 1 and change_vehicle[index]['vehicle_lane'] == 2:
                change_vehicle[index+1]['vehicle_lane'] = 1
        change_vehicle = change_vehicle[0:len(acc)+1]
        return {vehicle_id: change_vehicle}