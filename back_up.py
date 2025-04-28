import py_trees as pt

class BackUp(pt.behaviour.Behaviour):
    def __init__(self, name: str, blackboard, dist=10):
        super().__init__(name)
        self.name = name
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.motor_speed = 1.14
        self.timestep_needed = dist / self.motor_speed * 25


    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        self.timestep_count = 0

        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')

    def initialise(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        print(f'Starting: {self.name}')

    def update(self):
        if self.timestep_count >= self.timestep_needed:
            return pt.common.Status.SUCCESS

        self.right_motor.setVelocity(-self.motor_speed)
        self.left_motor.setVelocity(-self.motor_speed)

        self.timestep_count += 1

        return pt.common.Status.RUNNING

    def terminate(self, new_status: pt.common.Status):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

