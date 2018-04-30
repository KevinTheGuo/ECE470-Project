from robot import Robot
#import gripper

from models import *

r = Robot()

TEACH_ZONE = 1
TEACH_PATH = 2
GET_PATH = 3
LIST_ZONES = 4
TO_TAUGHT_ZONE = 5
GRIPPER = 6

def main():

    prompt = "\nChoose an action:\n" \
             "1. teach a zone (save joint and cartesian and name to database)\n" \
             "2. teach a path \n" \
             "3. get a path \n" \
             "4. list all zones \n" \
             "5. to taught zone \n" \
             "6. gripper \n"

    while True:
        resp = raw_input(prompt)

        try:
            action = int(resp)
        except Exception:
            print("Invalid input.")
            continue

        if action == TEACH_ZONE:
            r.get_info()
            print("You are now at cart: {}\n".format(r.ToolPosition))
            print("Joint: {}\n".format(r.JointPosition))
            resp = raw_input("Enter zone name: ")
            create_zone(resp)
            zone = get_zone(resp)
            sp = Safepoint(a1=r.a1, a2=r.a2, a3=r.a3, a4=r.a4, a5=r.a5, a6=r.a6, a7=r.a7, type="joint", zone=zone).save()
            zone.update(add_to_set__safepoints=sp)

        elif action == TEACH_PATH:
            start_zone_name = raw_input("Enter start zone name: ")
            end_zone_name = raw_input("Enter end zone name: ")
            list_waypoint_dict = []
            try:
                while True:
                    resp = raw_input("1. Teach this point, q. quit\n")
                    if resp == "q":
                        raise KeyboardInterrupt
                    elif resp == "1":
                        r.get_info()
                        print("You are now at cart: {}\n".format(r.ToolPosition))
                        print("Joint: {}\n".format(r.JointPosition))
                        while True:
                            try:
                                speed = int(raw_input("Speed for this point"))
                                break
                            except ValueError:
                                continue

                        joint_or_cartesian = int(raw_input("1.Joint 2.Cartesian"))

                        if joint_or_cartesian == 1:
                            list_waypoint_dict.append(dict(a1=r.a1, a2=r.a2, a3=r.a3, a4=r.a4, a5=r.a5, a6=r.a6, a7=r.a7, type="joint", speed=speed))

                        if joint_or_cartesian == 2:
                            lin_or_ptp = int(raw_input("1.lin 2.ptp"))
                            if lin_or_ptp == 1:
                                lin_or_ptp = "lin"
                            else:
                                lin_or_ptp = "ptp"
                            list_waypoint_dict.append(
                                    dict(x=r.x, y=r.y, z=r.z, a=r.a, b=r.b, c=r.c, type="cart", lin_ptp= lin_or_ptp,
                                         speed=speed))


            except KeyboardInterrupt:
                create_path(start_zone_name, end_zone_name, list_waypoint_dict)

        elif action == GET_PATH:
            start_zone_name = raw_input("Enter start zone name: ")
            end_zone_name = raw_input("Enter end zone name: ")
            print(get_waypoints(start_zone_name, end_zone_name))

        elif action == LIST_ZONES:
            zones =  Zone.objects()
            for zone in zones:
                print(zone.name)

        elif action == TO_TAUGHT_ZONE:
            zone_name = raw_input("Enter zone name: ")
            safepoint = get_zone_point(zone_name)
            r.abs_move_joint(safepoint.a1, safepoint.a2, safepoint.a3, safepoint.a4, safepoint.a5, safepoint.a6, safepoint.a7)
            """
            elif action == GET_ZONE:
                point =  where_am_i(dict(x=x_abs, y=y_abs, z=z_abs, w=wrist))
                print(point.x, point.y, point.z)
            """

            """
            elif action == MOVE:
                print "You are now at: x:{}, y:{}, z:{}, w:{}".format(x_abs, y_abs, z_abs, wrist)
                axis = raw_input("Set step axis: ")
                size = int(raw_input("Set step size: "))
                try:
                    while True:
                        if axis != "w":
                            rel_move(**{axis:size})
                        else:
                            rel_move_wrist(size)
                        print "You are now at: x:{}, y:{}, z:{}, w:{}".format(x_abs, y_abs, z_abs, wrist)
                        enter = raw_input()
                        if enter == "q":
                            raise KeyboardInterrupt
                except KeyboardInterrupt:
                    pass
            """

        elif action == GRIPPER:
            resp = raw_input("Grip 1, release 2:")

            if resp == "1":
                gripper_grip()
            elif resp == "2":
                gripper_release()

def move(from_zone_name, to_zone_name):
    waypoints = get_waypoints(from_zone_name, to_zone_name)
    for waypoint in waypoints:
        print "move started!", waypoint.type
        if waypoint.type == "joint":
            r.abs_move_joint(waypoint.a1, waypoint.a2, waypoint.a3, waypoint.a4, waypoint.a5, waypoint.a6, waypoint.a7)
        else:
            r.abs_move_cart(waypoint.x, waypoint.y, waypoint.z, waypoint.a, waypoint.b, waypoint.c, waypoint.lin_ptp)

    save_zone_name(to_zone_name)

if __name__ == "__main__":
    #main()
    move("hotel_nest1_nest", "hotel_nest1_safe")
    move("hotel_nest1_safe", "hotel_nest1_nest")
    #move("hotel_nest1_nest", "hotel_nest1_transit")
    #move("hotel_nest1_transit", "hotel_nest1_nest")
