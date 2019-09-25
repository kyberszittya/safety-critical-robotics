import rospy

from geometry_msgs import PoseStamped, Pose

class State(object):

    def __init__(self, state_name):
        self.state_name = state_name

class MissionExecutor(object):
    def execute(self):
        pass

class SimpleSingleMission(MissionExecutor):

    def __init__(self):
        self.state_dict = {}


    def addState(self, state_name):
        self.state_dict[state_name] = State(state_name)


def main():
    mission_executor = SimpleSingleMission()

if __name__=="__main__":
    main()
