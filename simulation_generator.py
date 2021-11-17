from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import random
import numpy as np
import pandas as pd

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


def vehilce_generation(mainline_traffic_flow, ramp_traffic_flow, N):
    """
    vehicel_generation(int, int, int)
    mainline_traffic_flow N means the number of generation vehicle
    """
    mainline_lamda = mainline_traffic_flow / 3600
    ramp_lamda = ramp_traffic_flow / 3600
    mainline_prob = []
    ramp_prob = []
    for time in np.arange(0, 20, 0.1):
        mainline_prob.append(1 - np.exp(-mainline_lamda * time))
        ramp_prob.append(1 - np.exp(-ramp_lamda * time))
    mainline_prob = np.array(mainline_prob)
    mainline_prob[-1] = 1
    ramp_prob = np.array(ramp_prob)
    ramp_prob[-1] = 1
    mainline = []
    ramp = []
    mainline_time = 0
    ramp_time = 0
    for i in range(N):
        main_prob = random.uniform(0, 1)
        ra_prob = random.uniform(0, 1)
        a = np.where((mainline_prob > main_prob))
        b = np.where((ramp_prob > ra_prob))
        a = np.array(a).squeeze(axis=0)
        b = np.array(b).squeeze(axis=0)
        mainline_interval = a[0] / 10
        ramp_interval = b[0] / 10
        if mainline_interval < 1:
            mainline_interval = 1
        if ramp_interval < 1:
            ramp_interval = 1
        mainline_time += mainline_interval
        mainline.append(('mainline', mainline_time))
        ramp_time += ramp_interval
        ramp.append(('ramp', ramp_time))
    mainline = pd.DataFrame(data=mainline, columns=['line', 'time'])
    ramp = pd.DataFrame(data=ramp, columns=['line', 'time'])
    vehicle_sum1 = pd.concat([mainline, ramp], axis=0)
    vehicle_sum1.sort_values('time', inplace=True)
    return vehicle_sum1


def generate_route_file(mainline_flow, ramp_flow, vehicle_numbers, cav_penetration):
    # random.seed(42)  # make tests reproducible
    # demand per second from different directions
    with open("3.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="CAV" accel="2" decel="4" sigma="0.5" length="4.5" minGap="2.5" maxSpeed="20" \
guiShape="passenger" carFollowModel="Wiedemann" />
        <vType id="HV" accel="2" decel="4" sigma="0.5" length="4.5" minGap="3" maxSpeed="20" carFollowModel="Wiedemann"/>

        <route edges="gneE0 gneE2 gneE2.32" color="yellow" id="route_0"/>
        <route edges="gneE1 gneE2 gneE2.32" color="yellow" id="route_1"/>""", file=routes)
        vehNr = 0
        vehicle_sum = vehilce_generation(mainline_flow, ramp_flow, vehicle_numbers)
        for i in range(vehicle_sum.shape[0]):
            if cav_penetration > random.uniform(0, 1):
                vehicle_type = 'CAV'
                vehicle_color = 'Blue'
            else:
                vehicle_type = 'HV'
                vehicle_color = 'Red'
            if vehicle_sum.iloc[i, 0] == 'mainline':
                print('<vehicle id="mainline_%i" type="%s" route="route_0" depart="%f" departSpeed="%f" color="%s"/>'
                      % (
                          vehNr, vehicle_type, vehicle_sum.iloc[i, 1], 20, vehicle_color),
                      file=routes)
            else:
                print(
                    '    <vehicle id="ramp_%i" type="%s" route="route_1" depart="%f" departSpeed="%f" color="%s"/>' % (
                        vehNr, vehicle_type, vehicle_sum.iloc[i, 1], 20, vehicle_color),
                    file=routes)
            vehNr += 1
        print("</routes>", file=routes)