import random
import rospy
from vicon_franka_integration.srv import Strategy, StrategyResponse

i = 0

def get_next_action(data):
    global i

    # Retrieve data
    world_conf = data.world_config
    prev_state = data.prev_state  # will be human state

    print(data, "\n")
    print('i', i)

    # First Execution
    first_initialization = prev_state is ""

    print("hello")
    if i == 0:
        # init_state = dfa_game.get_initial_states()[0][0]

        # _next_state = str_dict.get(init_state)
        # _sys_action_str = dfa_game._graph[__init_state][_next_state][0].get("actions")

        # # parse the output according to the srv file
        # _action_type = get_action_from_causal_graph_edge(_sys_action_str)
        # _box_id, _loc = get_multiple_box_location(_sys_action_str)

        action_type = "transit"
        box_id: int = 0
        to_loc: str = 'l2'
        next_state: str = ''


        i += 1
        return StrategyResponse(action_type, box_id, to_loc, next_state)

    elif i == 1:

        # Get next action
        # TODO:
        # action_type: str = random.choice(["transfer", "transit", "grasp", "place"])
        action_type: str = "grasp"
        box_id: int = 0
        to_loc: str = 'l2'
        next_state: str = ''

        i += 1
        return StrategyResponse(action_type, box_id, to_loc, next_state)

    elif i == 2:

         # Get next action
        # TODO:
        # action_type: str = random.choice(["transfer", "transit", "grasp", "place"])
        action_type: str = "transfer"
        box_id: int = 0
        to_loc: str = 'l8'
        next_state: str = ''

        i += 1
        return StrategyResponse(action_type, box_id, to_loc, next_state)

    elif i == 3:

         # Get next action
        # TODO:
        # action_type: str = random.choice(["transfer", "transit", "grasp", "place"])
        action_type: str = "release"
        box_id: int = 0
        to_loc: str = 'l2'
        next_state: str = ''

        i += 1
        return StrategyResponse(action_type, box_id, to_loc, next_state)


if __name__ == "__main__":
    rospy.init_node("StrategyNode")
    x = rospy.Service("com_node/strategy", Strategy, get_next_action)
    print("Ros Running")
    rospy.spin()
    print("Ros Stop")
