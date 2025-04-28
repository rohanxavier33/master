import py_trees as pt
import numpy as np

class GetRelativeError(pt.behaviour.Behaviour):
    def __init__(self, name: str, blackboard, target):
        super().__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.target_color = self.blackboard.read('recognition_colors')[target]

    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())

        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        self.object_id = None

        self.T_0_1 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0.6],
            [0, 0, 0, 1],
        ])

        self.T_1_2 = np.array([
            [1, 0, 0, 0.182],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])

        self.T_2_3 = np.array([
            [1, 0, 0, 0.005],
            [0, 1, 0, 0],
            [0, 0, 1, 0.098],
            [0, 0, 0, 1],
        ])

    def initialise(self):
        print(f'Starting: {self.name}')

        return super().initialise()

    def match_color(self, a, b):
        for i in range(3):
            if a[i] != b[i]:
                return False

        return True

    def update(self):
        # search target object using recognition color
        all_objects = self.camera.getRecognitionObjects()
        objects = [x for x in all_objects if self.match_color(x.getColors(), self.target_color)]

        # when no objects detected, return RUNNING
        if len(objects) == 0:
            self.blackboard.write('target_object_error', None)
            return pt.common.Status.RUNNING

        # remember object_id if first seen
        if self.object_id is None:
            self.object_id = objects[0].getId()

        # filter objects using object id to keep tracking the same object
        for obj in objects:
            if obj.getId() == self.object_id:
                target_object = obj

        # get position and transform to robot coords
        p0 = np.array([*list(target_object.getPosition()), 1])
        self.blackboard.write('target_object_error', p0@self.T_0_1@self.T_1_2@self.T_2_3)

        return pt.common.Status.RUNNING

    def terminate(self, new_status: pt.common.Status):
        pass
