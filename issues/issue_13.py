from dqrobotics import *

h = DQ([1, 0, 0, 0, 0, 0, 0, 0])

print('Is h unit?', is_unit(h))
print('Is h pure?', is_pure(h))
print('Is h real?', is_real(h))
print('Is h a real number?', is_real_number(h))
print('Is h a quaternion?', is_quaternion(h))
print('Is h a pure quaternion?', is_pure_quaternion(h))
print('Is h a plane?', is_plane(h))

