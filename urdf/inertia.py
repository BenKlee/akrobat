import os

DIR = os.path.dirname(os.path.abspath(__file__))

def inertia_cuboid(mass, depth, width, height):
    ixx = mass*(height**2 + depth**2)/12
    iyy = mass*(width**2 + height**2)/12
    izz = mass*(width**2 + depth**2)/12


    return ixx, iyy, izz

coxa    = (.2, 0.077991, 0.050983, 0.050874)
femur   = (.2, 0.123879, 0.047047, 0.040232)
tibia   = (.2, 0.181971, 0.051025, 0.02997)
hexapod = (.2, 0.123748, 0.610616, 0.057466)

links = {
    'coxa_link':     coxa,
    'femur_link':    femur,
    'tibia_link':    tibia,
    'hexapod_link':  hexapod
}

s = ''
for name, values in links.items():
    diagonal = list(inertia_cuboid(*values))

    diagonal.extend([0,0,0])

    s += f'{name}:\n'

    for i, value in zip(('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'), diagonal):
        s += f'  {i}: {value}\n'

    with open(os.path.join(DIR, 'inertia.yaml'), 'w') as f:
        f.write(s)