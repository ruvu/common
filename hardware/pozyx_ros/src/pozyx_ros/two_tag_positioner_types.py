# External interface
from collections import namedtuple

Position = namedtuple('Position', 'x y z')  # [mm, mm, mm]
Velocity2D = namedtuple('Velocity2D', 'x y yaw')  # [mm/s, mm/s, rad/s]
Orientation = namedtuple('Orientation', 'pitch roll yaw')  # [rad, rad, rad]
UWBSettings = namedtuple('UWBSettings', 'channel bitrate prf plen gain_db')
Tag = namedtuple('Tag', 'serial_port position')  # Position w.r.t. robot coordinate frame
Anchor = namedtuple('Anchor', 'network_id position')  # Position w.r.t. global coordinate frame

# Time in seconds since epoch, Velocity w.r.t. robot coordinate frame and the covariance matrix (size=9) as follows:
# [xx,   xy,   xyaw,                [mm^2/s^2, mm^2/s^2, mm*rad/s^2,
#  yx,   yy,   yyaw,    with units   mm^2/s^2, mm^2/s^2, mm*rad/s^2,
#  yawx, yawy, yawyaw]               mm^2/s^2, mm^2/s^2, rad^2/s^2]
Input = namedtuple('Input', 'current_time velocity_time velocity covariance')

# Position and Orientation w.r.t. global coordinate frame and the covariance matrix (size=36) as follows:
# [xx,     xy,     xz,     xroll,     xpitch,     xyaw,                [mm^2,   mm^2,   mm^2,   rad*mm, rad*mm, rad*mm,
#  yx,     yy,     yz,     yroll,     ypitch,     yyaw,                 mm^2,   mm^2,   mm^2,   rad*mm, rad*mm, rad*mm,
#  zx,     zy,     zz,     zroll,     zpitch,     zyaw,     with units  mm^2,   mm^2,   mm^2,   rad*mm, rad*mm, rad*mm,
#  rollx,  rolly,  rollz,  rollroll,  rollpitch,  rollyaw,              mm*rad, mm*rad, mm*rad, rad^2,  rad^2,  rad^2,
#  pitchx, pitchy, pitchz, pitchroll, pitchpitch, pitchyaw,             mm*rad, mm*rad, mm*rad, rad^2,  rad^2,  rad^2,
#  yawx,   yawy,   yawz,   yawroll,   yawpitch,   yawyaw]               mm*rad, mm*rad, mm*rad, rad^2,  rad^2,  rad^2]
Output = namedtuple('Output', 'position orientation covariance')