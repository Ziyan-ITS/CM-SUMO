import traci
from Vehicle_manager import vehicle_manager
from simulation_generator import generate_route_file


class environment(object):
    def __init__(self, config, mode):
        self.config = config
        self.v_manager = vehicle_manager()
        if mode == 'generate':
            self._generate_xml()

    def _generate_xml(self):
        generate_route_file(self.config["mainline_flow"],
                            self.config["ramp_flow"],
                            self.config["vehicle_numbers"],
                            self.config['cav_penetration'])

    def reset(self, sumo_config):
        """execute the TraCI control loop
            Since we need to get the effective reward at the very begging moment, the start scene is required to be
            stationary traffic flow"""
        traci.start([sumo_config['cmd'], "-c", sumo_config['config_path']])
        while len(traci.vehicle.getIDList()) <= 10:
            traci.simulationStep()

    def change_vehicle_state(self, action):
        self.v_manager.change_vehicle_sate(action)

    def step(self, action=None):
        """execute the TraCI control loop"""
        # change_vehicle_sate(action, CAV_ID)
        # change_vehicle_lateral_sate(change_lane, CAV_change)
        if action:
            self.v_manager.change_vehicle_sate(action)
        traci.simulationStep()
        # 获取环境state
        self.v_manager.get_vehicle_state()

    def __enter__(self, sumo_config):
        self.reset(sumo_config)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        traci.close()

    def traci_close(self):
        return traci.close()

