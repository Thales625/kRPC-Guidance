from time import sleep
from math import sqrt, acos
import krpc

from sys import path
path.append('D:\Codes\www\Vector')
from Vector import Vector3

def climp(value, max_, min_):
    if value >= max_: return max_
    if value <= min_: return min_
    return value

conn = krpc.connect(name='Teste')
space_center = conn.space_center
vessel = space_center.active_vessel
body = vessel.orbit.body
drawing = conn.drawing
a_gravity = body.surface_gravity

body_ref = body.reference_frame
surface_ref = vessel.surface_reference_frame
hybrid_ref = space_center.ReferenceFrame.create_hybrid(
    position=body_ref,
    rotation=surface_ref
)

flight_body = vessel.flight(body_ref)
flight_hybrid = vessel.flight(hybrid_ref)

stream_mass = conn.add_stream(getattr, vessel, "mass")
stream_vel = conn.add_stream(getattr, flight_hybrid, "velocity")
stream_surface_altitude = conn.add_stream(getattr, flight_body, "surface_altitude")
stream_av_thrust = conn.add_stream(getattr, vessel, "available_thrust")

vessel.auto_pilot.reference_frame = body_ref
vessel.auto_pilot.engage()
vessel.auto_pilot.target_direction = space_center.transform_direction((1, 0, 0), surface_ref, body_ref)
vessel.auto_pilot.target_roll = 0


target_pos_body = Vector3(space_center.transform_position((0, 100, 0), surface_ref, body_ref))

drawing.add_line(target_pos_body, target_pos_body + target_pos_body.normalize(), body_ref).color = (1, 0, 0)

line_aim_dir = drawing.add_line((0, 0, 0), (0, 0, 0), surface_ref)
line_aim_dir.color = (0, 0, 1)
text_aim_dir = drawing.add_text('Aim', surface_ref, (0, 0, 0), (0, 0, 0, 1))
text_aim_dir.color = line_aim_dir.color

line_target_dir = drawing.add_line((0, 0, 0), (0, 0, 0), surface_ref)
line_target_dir.color = (1, 0, 0)
text_target_dir = drawing.add_text('Target', surface_ref, (0, 0, 0), (0, 0, 0, 1))
text_target_dir.color = line_target_dir.color

while True:
    # get streams
    mass = stream_mass()
    vel = Vector3(stream_vel())
    alt = stream_surface_altitude()
    mass = stream_mass()
    av_thrust = stream_av_thrust()

    # ---------------------

    a_eng = av_thrust / mass

    delta_pos = Vector3(space_center.transform_position(target_pos_body, body_ref, surface_ref))

    # AIM CONTROL
    aim_dir = Vector3(2, 0, 0)
    max_pitch = acos(a_gravity/a_eng)
    a_max = a_eng - a_gravity
    a_actual = (a_eng * vessel.control.throttle) if vessel.control.throttle > 0 else 0.01
    # Y
    vy_target = a_max * sqrt(abs(delta_pos.y) / a_max)
    if delta_pos.y < 0: vy_target = -vy_target

    dvy = vy_target - vel.y

    pitch_y = climp(dvy*5 / a_actual, 1, -1)
    pitch_y = climp(pitch_y, max_pitch, -max_pitch)
    aim_dir.y = pitch_y / max_pitch


    # Z
    vz_target = a_max * sqrt(abs(delta_pos.z) / a_max)
    if delta_pos.z < 0: vz_target = -vz_target

    dvz = vz_target - vel.z
    
    pitch_z = climp(dvz*5 / a_actual, 1, -1)
    pitch_z = climp(pitch_z, max_pitch, -max_pitch)
    aim_dir.z = pitch_z / max_pitch

    print(tuple(aim_dir))

    aim_dir = aim_dir.normalize()

    # THROTTLE CONTROL
    if delta_pos.x < 0:
        a_net = a_eng - a_gravity
        vy_target = -a_net * sqrt(-delta_pos.x/a_net)
    else:
        vy_target = a_gravity * sqrt(delta_pos.x/a_gravity)
    
    dvy = vy_target - vel.x

    vessel.control.throttle = (dvy*5 + a_gravity) / a_eng

    # aim
    vessel.auto_pilot.target_direction = space_center.transform_direction(aim_dir, surface_ref, body_ref)

    # DRAWING
    line_target_dir.end = delta_pos.normalize() * 10
    line_aim_dir.end = aim_dir * 10
    
    text_target_dir.position = line_target_dir.end
    text_aim_dir.position = line_aim_dir.end

    sleep(.05)