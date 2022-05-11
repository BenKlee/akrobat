from akrobat.enums import Leg

class Trajectory():

    def __init__(self) -> None:
        self.case_step = [0] * Leg.amount()

        self.tick = 0

        self.initial_amplitude_x = 0
        self.initial_amplitude_y = 0
        self.initial_amplitude_z = 0

        self.amplitude_x = [0] * Leg.amount()
        self.amplitude_y = [0] * Leg.amount()
        self.amplitude_z = [0] * Leg.amount()

    
    def __str__(self) -> str:
        return (
            f'tick:\t\t\t{self.tick}\n' +
            f'initial_amplitude_x:\t{self.initial_amplitude_x}\n' +
            f'initial_amplitude_y:\t{self.initial_amplitude_y}\n' +
            f'initial_amplitude_z:\t{self.initial_amplitude_z}\n' +
            f'case_step:\t\t{self.case_step}\n' +
            f'amplitude_x:\t\t{self.amplitude_x}\n' +
            f'amplitude_y:\t\t{self.amplitude_y}\n' +
            f'amplitude_z:\t\t{self.amplitude_z}\n'
        )