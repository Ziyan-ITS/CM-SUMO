"""
vehicle manager是控制车辆的一个类
"""
import traci
import math


class vehicle_manager:
    def __init__(self):
        self.vehicle_info = []
        self.cav_info = []
        self.hv_info = []
        self.ramp_info = []
        self.mainline_info = []
        self.merge_section = []

    def get_vehicle_state(self):
        """
        collect the vehicle state information
        """
        vehicle_id = traci.vehicle.getIDList()
        vehicle_info = []
        for vehicle in vehicle_id:
            vehicle_speed = traci.vehicle.getSpeed(vehicle)
            vehicle_position = traci.vehicle.getPosition(vehicle)
            if (vehicle_position[1] >= -1.6) & (vehicle_position[1] <= 1.6):
                vehicle_lane = 1  # mainline1
            elif (vehicle_position[1] >= -4.8) & (vehicle_position[1] < -1.6):
                vehicle_lane = 2  # ramp
            else:
                vehicle_lane = 3
            #
            vehicle_road = traci.vehicle.getRoadID(vehicle)
            vehicle_color = traci.vehicle.getColor(vehicle)
            if vehicle_color == (0, 0, 255, 255):
                vehicle_type = 'cav'
            else:
                vehicle_type = 'hv'
            vehicle_acceleration = traci.vehicle.getAcceleration(vehicle)
            v_state = {'vehicle_speed': vehicle_speed,
                                 'x': vehicle_position[0],
                                 'y': vehicle_position[1] + 1.6,
                                 'vehicle_road': vehicle_road,
                                 'vehicle_type': vehicle_type,
                                 'vehicle_acceleration': vehicle_acceleration,
                                 'vehicle_lane': vehicle_lane,
                                 'vehicle_id': vehicle}
            if v_state['x'] < 0:
                v_state['vehicle_dis'] = -math.sqrt(vehicle_position[0] ** 2 + (vehicle_position[1] + 1.6) ** 2)
            else:
                v_state['vehicle_dis'] = math.sqrt(vehicle_position[0] ** 2 + (vehicle_position[1] + 1.6) ** 2)
            vehicle_info.append(v_state)
        self.vehicle_info = sorted(vehicle_info, key=lambda x: x['x'], reverse=True)
        self.cav_info = [v_info for v_info in self.vehicle_info if
                            v_info["vehicle_type"] == 'cav']
        self.hv_info = [v_info for v_info in self.vehicle_info if
                            v_info["vehicle_type"] == 'hv']
        self.ramp_info = [v_info for v_info in self.vehicle_info if
                            v_info["vehicle_lane"] == 3]

        self.mainline_info = [v_info for v_info in self.vehicle_info if
                             v_info["vehicle_lane"] == 1]

        self.merge_section = [v_info for v_info in self.vehicle_info if
                             v_info["vehicle_lane"] == 2]

    def change_vehicle_sate(self, action):
        """
        :param action: The dimension of action is the number of CAV, the training sequence is from the leading vehicle to
        the last vehicle. Since we should train two different neural network as the actor.
        :param vehicle_list: we should split the vehicle_list to mainline_vehicle and ramp_vehicle
        :return:
        """
        # This part should be in the agent, 找到这个get_list 的顺序
        # traci.vehicle.setAccel(vehicle, action[i])
        for cav in action:
            traci.vehicle.setSpeed(cav, self.cav_info[cav] + action[cav])

    def __call__(self, *args, **kwargs):
        self.get_vehicle_state()
