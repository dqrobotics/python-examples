from dqrobotics import *
from dqrobotics.utils import DQ_Geometry
from dqrobotics.interfaces.vrep import DQ_VrepInterface

configuration = {
    "verbose":False,
    "is_line_segment_threshold":DQ_threshold*10e7 # This threshold is used to fix line segments obtained from CoppeliaSim.
}

def check_if_point_is_on_line(point,line):
    l = P(line)
    m = D(line)
    if(cross(point,l) != m):
        print(" Check would fail for point and line with regular DQ_Threshold")
        print(" Error = {0:.32g}".format(norm(m-cross(point,l)).q[0]))


def fix_line_segment_from_coppeliasim(line,p1,p2,threshold):
    """
    When using is_line_segment with elements obtained from CoppeliaSim, the precision is lower
    than what is expected in dqrobotics. This function is used to fix the line segment.
    This function is good enough for the purposes of this example, but do no rely on it for anything else.
    """
    if(not is_line(line)):
        raise RuntimeError("fix_line_segment::Cannot fix line segment if line isn't a line")
    if(not is_pure(p1)):
        raise RuntimeError("fix_line_segment::Cannot fix line segment if p1 isn't a point")
    if(not is_pure(p2)):
        raise RuntimeError("fix_line_segment::Cannot fix line segment if p2 isn't a point")
    if(not DQ_Geometry.is_line_segment(line,p1,p2,threshold)):
        raise RuntimeError("fix_line_segment::Line segment error above acceptable threshold {}. Are you sure \
your input is a line segment?".format(threshold))
    if(not DQ_Geometry.is_line_segment(line,p1,p2)):
        p1_projected = DQ_Geometry.point_projected_in_line(p1,line)
        p2_projected = DQ_Geometry.point_projected_in_line(p2,line)
        return line,p1_projected,p2_projected
    return line,p1,p2        

vi = DQ_VrepInterface()
try:
    vi.connect(19997, 100, 100)
    vi.set_synchronous(True)
    vi.start_simulation()

    print("Move the line segments 'line_segment_1' and 'line_segment_2'" 
    + " on CoppeliaSim and the closest points will be shown.")

    while True:
        x1 = vi.get_object_pose("line1")
        t1 = translation(x1)
        r1 = rotation(x1)
        l1 = Ad(r1, k_)
        m1 = cross(t1, l1)
        line1 = l1 + DQ.E * m1

        line1,line1_p1,line1_p2 = fix_line_segment_from_coppeliasim(line1,
        vi.get_object_translation("line1_p1"),
        vi.get_object_translation("line1_p2"),
        configuration["is_line_segment_threshold"])
        
        if(configuration["verbose"]):
            check_if_point_is_on_line(line1_p1,line1)
            check_if_point_is_on_line(line1_p2,line1)

        x2 = vi.get_object_pose("line2")
        t2 = translation(x2)
        r2 = rotation(x2)
        l2 = Ad(r2, k_)
        m2 = cross(t2, l2)
        line2 = l2 + DQ.E * m2

        line2,line2_p1,line2_p2 = fix_line_segment_from_coppeliasim(line2,
        vi.get_object_translation("line2_p1"),
        vi.get_object_translation("line2_p2"),
        configuration["is_line_segment_threshold"])

        if(configuration["verbose"]):
            check_if_point_is_on_line(line2_p1,line2)
            check_if_point_is_on_line(line2_p2,line2)

        cp_line_1,cp_line_2 = DQ_Geometry.closest_points_between_lines(line1,line2)
        if(configuration["verbose"]):
            print(" Closest point between lines = {}".format((cp_line_1,cp_line_2)))

        vi.set_object_translation("closest_point_line_1",cp_line_1)
        vi.set_object_translation("closest_point_line_2",cp_line_2)

        cp_lineseg_1,cp_lineseg_2 = DQ_Geometry.closest_points_between_line_segments(
            line1,
            line1_p1,
            line1_p2,
            line2,
            line2_p1,
            line2_p2
            )

        if(configuration["verbose"]):
            print(" Closest point between line segments = {}".format((cp_lineseg_1,cp_lineseg_2)))

        vi.set_object_translation("closest_point_line_segment_1",cp_lineseg_1)
        vi.set_object_translation("closest_point_line_segment_2",cp_lineseg_2)

        if(configuration["verbose"]):
            print(" Distance between line segments = {}.".format(
                DQ_Geometry.line_segment_to_line_segment_squared_distance(line1, line1_p1, line1_p2,
                                                                          line2, line2_p1, line2_p2)
            ))

        vi.trigger_next_simulation_step()

except Exception as e:
    print(e)
except KeyboardInterrupt:
    print("Interrupted by user")

vi.stop_simulation()
vi.disconnect_all()
