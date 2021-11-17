from simulation_runner import environment
import json
import argparse
from CDMMT_controller import upper_controller


def load_config(path):
    with open(path, 'r') as f:
        return json.load(f)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Evaluation for MPC-CDMMT evaluation')
    parser.add_argument('-c', '--config', default='cdmmt.json', required=False,
                        help='To generate numerical simulation environment configuration')
    parser.add_argument('-m', '--mode', default='generate', required=False)
    args = parser.parse_args()
    config = load_config(args.config)
    s = environment(config['simulation_setup'], args.mode)
    s.reset(config['sumo_config'])
    t = 1
    while True:
        s.step()
        vehicle_info = s.v_manager
        t += 1
        uc = upper_controller(vehicle_info)
        uc.run()
        if t == 1000:
            break
    s.traci_close()


