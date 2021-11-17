from abc import ABC, abstractmethod
import copy
import re
import random


def random_sample(current_state, down_sample_rate=0.3):
    """可以random sample， 也可以greedy sample"""
    if len(current_state) < 100:
        return
    pop_list = []
    for key in current_state:
        if random.random() > down_sample_rate:
            pop_list.append(key)
    index = 0
    while len(current_state) > 100 and index < len(pop_list):
        current_state.pop(pop_list[index])
        index = index + 1


def greedy_sample(current_sate, sample_num=100):
    best = sorted(current_sate.items(), key=lambda x: x[1]['value'])
    new_state = {}
    index = 0
    for key, value in best:
        new_state[key] = value
        index = index + 1
        if index == sample_num:
            return new_state
    return new_state


func_name = {'greedy': greedy_sample, 'random': random_sample}


def str_to_tuple(key):
    bay = re.findall(r'\d+', key)
    new_bay = []
    for i in bay:
        new_bay.append(int(i))
    return new_bay


class Dp(ABC):

    tier_reward = {0: 8, 1: 4, 2: 2, 3: 1, 4: 0.5}

    def __init__(self, mainline_vehicle, ramp_vehicle):
        """
        :param step: vessel_bay, containers, vessel_row
        :param yard_bay:list[list[int : container_position or container_id]]
        :param vessel_bay:
        vessel_row: 每一列的限高，限重等信息， 从预配计划中提取
        block_list: 索引对照list，并不需要
        containers 的数据结构可能要做调整
        """
        self.current_state = []
        self.dummy_state = []
        self.mainline_vehicle = mainline_vehicle
        self.ramp_vehicle = ramp_vehicle

    @abstractmethod
    def _initial_state(self):
        """
        GENERATE THE INITIAL STATE OF DP
        :return:
        """
        return NotImplemented

    @abstractmethod
    def _constrains_satisfy(self):
        return NotImplemented

    @abstractmethod
    def _clean_state(self):
        return NotImplemented

    @abstractmethod
    def _new_state_generate(self):
        return NotImplemented

    @abstractmethod
    def _reward_calculate(self, current_key, prev_state, prev_key, re_heave=0.002, limitation=0.01, yc_move=2,
                          diff_block=0.5,
                          re_location=1, yard_car=1):
        """
        caculate the reward
        :return:
        """
        return NotImplemented

    @abstractmethod
    def _check_constraints(self, prev_state, current_state):
        """
        :return:
        """
        return NotImplemented

    @staticmethod
    def state_transition(current_key, current, prev_state, value, prev_key):
        current['value'] = value

        # 更新船贝状态
        current['vessel_bay'] = copy.deepcopy(prev_state['vessel_bay'])
        current['vessel_bay'][current_key[-2]].append(current_key[0:4])

        # 更新连走数量以及上上个贝ID
        if current_key[0:2] == prev_key[0:2]:
            current['continue_num'] = prev_state['continue_num'] + 1
        else:
            current['continue_num'] = 1
            current['last_yard_bay'] = prev_key[0:2]

        # 更新动作集合
        current['action_set'] = copy.deepcopy(prev_state['action_set'])
        current['action_set'][current_key[0:4]] = current_key[-2::]
        # 更新yc位置
        current['yc_loc'][current_key[0]] = current_key[1]

        # 更新堆场状态
        current['yard_con'] = copy.deepcopy(prev_state['yard_con'])
        current['yard_con'][current_key[0]][current_key[1]][current_key[2]].pop(current_key[3])
        if len(current['yard_con'][current_key[0]][current_key[1]][current_key[2]]) == 0:
            current['yard_con'][current_key[0]][current_key[1]].pop(current_key[2])

    def find_optimal(self):
        final_state = self.dummy_state
        best = min(final_state.items(), key=lambda x: x[1]['value'])
        print(best[1]['value'])
        return best[1]['action_set']

    def run_DP(self, func=None, **kwargs):
        """
         DP 主框架
         :return:
         """
        self._new_state_generate()
        self._initial_state()
        # if not self._sequence_possible():
        #     raise LookupError
        f = self._constrains_satisfy()
        if not f:
            print("find_error")
        for i in range(1, self.step):
            print(i)
            # if len(self.dummy_state) < 50:
            #     print(i)
            #     print(self.dummy_state.keys())
            #     print(self.current_state.keys())
            current_state = self.current_state
            # 筛选不满足要求的状态
            pop_state = []
            for current_key in current_state:
                current = current_state[current_key]
                # current_key (i, j, s, t)
                # 这里，这里的状态转移条件应该是上个状态对应的action序列中它的上方集装箱已经被提出来了。
                trans_key = None
                value = current['value']
                for prev_key in self.dummy_state:
                    prev_state = self.dummy_state[prev_key]
                    # 这里应该是(i-1, j)在其中，(s, t-1)在其中。
                    if self._check_constraints(current_key, prev_state, prev_key):
                        reward = self._reward_calculate(current_key, prev_state, prev_key, **kwargs)
                        # 如果新得到的解由于当前解
                        if value > prev_state['value'] + reward:
                            trans_key = prev_key
                            value = prev_state['value'] + reward
                if trans_key:
                    self.state_transition(current_key, current, self.dummy_state[trans_key], value, trans_key)
                else:
                    pop_state.append(current_key)
            for pop_key in pop_state:
                self.current_state.pop(pop_key)

            if func is not None:
                if func == 'random':
                    self.dummy_state = func_name[func](self.current_state, 0.1)
                elif func == 'greedy':
                    self.dummy_state = func_name[func](self.current_state, 2000)
            else:
                self.dummy_state = self.current_state
            # 在new_state_generation上也可以使用一些方法从而对算法提速
            self._clean_state()
            self._new_state_generate()
            if len(self.dummy_state) == 0:
                print(f)
                raise LookupError
