class Joint():

    def __init__(self, alpha: float, beta: float, gamma: float) -> None:
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma

    @property
    def base_to_coxa(self) -> float:
        return self.alpha

    @property
    def coxa_to_femur(self) -> float:
        return self.beta

    @property
    def femur_to_tibia(self) -> float:
        return self.gamma

    def __str__(self) -> str:
        return f'[{self.alpha}, {self.beta}, {self.gamma}]'