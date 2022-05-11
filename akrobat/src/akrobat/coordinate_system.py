from akrobat.leg import Leg
from akrobat import enums

class CoordinateSystem:

    def __init__(self) -> None:
        self.leg = [Leg() for _ in range(enums.Leg.amount())]
