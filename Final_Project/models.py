from mongoengine import *
from datetime import *
from Queue import Queue

connect(db="iiwarobot",
        host="192.168.31.100")

# {{{ models


class Point(Document):
    x = FloatField()
    y = FloatField()
    z = FloatField()
    a = FloatField()
    b = FloatField()
    c = FloatField()

    px = FloatField()
    py = FloatField()
    pz = FloatField()
    ox = FloatField()
    oy = FloatField()
    oz = FloatField()
    ow = FloatField()

    a1 = FloatField()
    a2 = FloatField()
    a3 = FloatField()
    a4 = FloatField()
    a5 = FloatField()
    a6 = FloatField()
    a7 = FloatField()

    type = StringField(max_length=128)

    meta = {"allow_inheritance": True}


class Safepoint(Point):
    zone = ReferenceField("Zone")


class Waypoint(Point):
    speed = IntField()
    lin_ptp = StringField(max_length=128)
    path = ReferenceField("Path")


class FunctionStep(EmbeddedDocument):
    function = StringField(max_length=128)
    argument = ListField(StringField(max_length=128))


class Function(EmbeddedDocument):
    name = StringField(max_length=128)
    steps = ListField(EmbeddedDocumentField(FunctionStep))


class Zone(Document):
    name = StringField(max_length=128, unique=True)
    date_modified = DateTimeField(default=datetime.now)
    safepoints = ListField(ReferenceField(Safepoint))
    paths = ListField(ReferenceField("Path"))
    neighbors = ListField(ReferenceField("self"))
    functions = EmbeddedDocumentListField(Function)

# for kuka
class PathCommand(Document):
    commannd = StringField(max_length=128)
    path = ReferenceField("Path")

# for kuka
class Path(Document):
    start_zone = ReferenceField(Zone)
    end_zone = ReferenceField(Zone)
    waypoints = ListField(ReferenceField(Waypoint))


class LastPosition(Document):
    x = IntField()
    y = IntField()
    z = IntField()
    w = IntField()
    gripper_status = StringField(max_length=128)
    zone_name = StringField(max_length=128)

# }}}


def create_zone(zone_name):
    try:
        Zone(zone_name).save()
    except NotUniqueError:
        pass


def get_zone(zone_name):
    try:
        return Zone.objects.get(name=zone_name)
    except DoesNotExist:
        raise Exception("Zone {} does not exist".format(zone_name))


def get_zone_point(zone_name):
    zone = get_zone(zone_name)
    return zone.safepoints[0]


def create_path(start_zone_name, end_zone_name, list_pathcommand_dict):
    start_zone = get_zone(start_zone_name)
    end_zone = get_zone(end_zone_name)

    pathcommands = []
    for pathcommand_dict in list_pathcommand_dict:
        pathcommands.append(Waypoint(**pathcommand_dict).save())

    path = Path.objects(start_zone=start_zone,
                        end_zone=end_zone).modify(upsert=True, new=True, set__waypoints=pathcommands)
    start_zone.update(add_to_set__paths=path, add_to_set__neighbors=end_zone)
    end_zone.update(add_to_set__paths=path, add_to_set__neighbors=start_zone)

    for pathcommand in pathcommands:
        pathcommand.update(set__path=path)


def get_path_name(start_zone_name, end_zone_name):
    visited_zone = {}
    queue = Queue()
    path_name = []

    start_zone = get_zone(start_zone_name)
    end_zone = get_zone(end_zone_name)

    queue.put(start_zone)
    while not queue.empty():
        current = queue.get()
        visited_zone.update({current.name: current})
        if current.name == end_zone.name:
            path_name.append(current.name)
            while hasattr(current, "parent"):
                current = current.parent
                path_name.append(current.name)
            return path_name
        for neighbor in current.neighbors:
            if neighbor.name not in visited_zone:
                neighbor.parent = current
                queue.put(neighbor)

    return []


def get_path(start_zone, end_zone):
    try:
        path = Path.objects.get(start_zone=start_zone, end_zone=end_zone)
        path.reversed = False
    except DoesNotExist:
        path = Path.objects.get(start_zone=end_zone, end_zone=start_zone)
        path.reversed = True
    return path


def get_waypoints(start_zone_name, end_zone_name):
    path = get_path_name(start_zone_name, end_zone_name)
    path_zone = []

    waypoints = []

    for zone_name in path:
        path_zone.append(get_zone(zone_name))
    for i in range(len(path_zone)-1):
        path = get_path(path_zone[i], path_zone[i+1])
        if path.reversed:
            path.waypoints.reverse()
            waypoints.extend(path.waypoints)
        else:
            waypoints.extend(path.waypoints)

    waypoints.reverse()
    return waypoints


def where_am_i(point_dict):
    current_point = Point(**point_dict)
    min_distance = float("inf")
    closest_point = None
    for point in Point.objects:
        current_distance = pow((current_point.x - point.x), 2) + pow((current_point.y - point.y), 2) + pow((current_point.z - point.z), 2)
        if current_distance < min_distance:
            min_distance = current_distance
            closest_point = point

    return closest_point


def get_waypoints_to_closest_zone(point_dict):
    my_position = where_am_i(point_dict)
    if isinstance(my_position, Safepoint):
        return []
    elif isinstance(my_position, Waypoint):
        waypoints = my_position.path.waypoints
        for i in range(len(waypoints)):
            if waypoints[i].x == my_position.x and waypoints[i].y == my_position.y and waypoints[i].z == my_position.z:
                if i > len(waypoints) - i:
                    waypoints = waypoints[:i]
                    waypoints.reverse()
                    return waypoints
                else:
                    return waypoints[i:]
            else:
                return []
    return []


def add_function_to_zone(zone_name, function_name, list_function_steps_dict):
    zone = get_zone(zone_name)

    function = Function(name=function_name)

    for function_step_dict in list_function_steps_dict:
        function_step = FunctionStep(**function_step_dict)
        function.steps.append(function_step)

    zone.functions.append(function)
    zone.save()


def get_functions_from_zone(zone_name):
    zone = get_zone(zone_name)
    return zone.functions

def save_last_position(x,y,z,w,gripper):
    LastPosition.objects.modify(upsert=True, set__x=x, set__y=y, set__z=z, set__w=w)

def save_zone_name(zone_name):
    LastPosition.objects.modify(upsert=True, set__zone_name=zone_name)

def get_last_position():
    return LastPosition.objects.get()

if __name__ == "__main__":
    pass
    print get_path_name("asdf", "root-safe1")
    #print Zone.objects.get(name="root-safe").name
    #print get_zone("root-safe")
    #create_zone("root-safe")
    #mywaypoints = dict(x=1,y=1,z=2,w=0)

    #create_path("root-safe", "root-safe1", [mywaypoints, dict(x=3,y=3,z=3,w=3)])

    #waypoints = get_waypoints("root-safe1", "root-safe")

    #for waypoint in waypoints:
    #    print waypoint.w

    #add_function_to_zone("root-safe","put2", [{"function":"robot.move","argument":[]}])
    #print get_functions_from_zone("root-safe")[0].steps[0].function, get_functions_from_zone("root-safe")[1].name
    #save_last_position(1,2,3,4,5,"my_name")
    #print get_last_position().x