from pymongo import MongoClient
from copy import deepcopy
client = MongoClient("mongodb://robot:Foundry4LifeRobot@192.168.31.100/centrifuge_robot?authMechanism=SCRAM-SHA-1")
db = client.centrifuge_robot


def create_zone(name):
    inserted_id = db.zones.insert_one({"name": name, "ancestor_ids": [], "ancestor_names": [], "points": {"transit": [], "nest": [], "safe": []}, "path": [], "speed": []}).inserted_id
    return get_zone_from_id(inserted_id)

def create_zone_matrix(name, row, col):
    zones = []
    for i in range(row):
        for j in range(col):
            zone = create_zone(name + "_" + str(i) + "_" + str(j))
            zones.append(zone)
    return zones


def add_zone_to_parent(zone, parent):
    zone["ancestor_ids"] = [parent["_id"]] + parent["ancestor_ids"]
    zone["ancestor_names"] = [parent["name"]] + parent["ancestor_names"]
    db.zones.update({"_id": zone["_id"]}, zone)

def add_zone_matrix_to_parent(zones, parent):
    for item in zones:
        add_zone_to_parent(item, parent)


def create_point(x, y, z, w, action=None):
    return {"x":x, "y":y, "z":z, "w":w, "action":action}

def create_point_matrix(row, col, ne, nw, se, sw, action=None):
    def interpolate(ipercent, jpercent, dimension):
        return (1-ipercent)*(1-jpercent)*nw[dimension]+(1-ipercent)*(jpercent)*ne[dimension]+(ipercent)*(1-jpercent)*sw[dimension]+(ipercent)*(jpercent)*se[dimension]
    
    points = []
    for i in range(row):
        for j in range(col):
            ipercent = float(i)/row
            jpercent = float(j)/row
            point = create_point(interpolate(ipercent, jpercent, "x"), interpolate(ipercent, jpercent, "y"),interpolate(ipercent, jpercent, "z"), interpolate(ipercent, jpercent, "w"), action)
            points.append(point)
    return points


def add_point_to_zone(point, name, zone):
    if name in zone["points"]:
        zone["points"][name].append(point)
    else:
        zone["points"][name] = [point]
    db.zones.update({"_id": zone["_id"]}, zone)

def add_point_matrix_to_zone_matrix(points, name, zones):
    for i in range(len(points)):
        add_point_to_zone(points[i], name, zones[i])


def create_path(path):
    points = []
    for item in path:
        point = create_point(item[0], item[1], item[2], item[3])
        points.append(point)
    return points

def add_path_to_zone(path, speed, zone):
    if zone["name"] == "root":
        raise Exception
    
    zone["points"]["transit"] = path
    zone["path"] = []
    zone["speed"] = speed
    
    parent = get_zone_from_id(zone["ancestor_ids"][0])
    add_point_to_zone(path[-1], "safe", parent)
    
    db.zones.update({"_id":zone["_id"]}, zone)


def create_customized_path(path):
    points = []
    for item in path:
        point = {"name": item[0], "id": item[1]}
        points.append(point)
    return points

def add_customized_path_to_zone(path, speed, zone):
    if zone["name"] == "root":
        raise Exception

    zone["path"] = path
    zone["speed"] = speed

    parent = get_zone_from_id(zone["ancestor_ids"][0])
    parent_safe_point = zone["points"][path[-1]["name"]][path[-1]["id"]]
    add_point_to_zone(parent_safe_point, "safe", parent)
    
    db.zones.update({"_id":zone["_id"]}, zone)


def get_zone_from_name(name):
    return db.zones.find_one({"name":name})

def get_zone_from_id(id):
    return db.zones.find_one({"_id":id})


def delete_zone(zone):
    zones = [zone["_id"]]
    i = 0
    while i < len(zones):
        cursor = db.zones.find({"ancestor_ids.0": zones[i]})
        for item in cursor:
            zones.append(item["_id"])
        i += 1
    return db.zones.remove({"_id":{"$in": zones}})

def delete_zone_matrix(zones):
    for item in zones:
        delete_zone(item)


def join_path(position1, position2):
    def get_common_ancestor(zone1, zone2):
        ancestor = get_zone_from_name("root")
        reversed_zone1_ancestor_ids = zone1["ancestor_ids"][::-1]
        reversed_zone2_ancestor_ids = zone2["ancestor_ids"][::-1]
        for i in range(min(len(zone1["ancestor_ids"]), len(zone2["ancestor_ids"]))):
            if reversed_zone1_ancestor_ids[i] == reversed_zone2_ancestor_ids[i]:
                ancestor = get_zone_from_id(reversed_zone1_ancestor_ids[i])
        return ancestor
                
    def get_path(zone):
        path = deepcopy(zone["points"]["transit"])
        for i in range(len(path)):
            path[i]["zone"] = zone["name"]
            path[i]["name"] = "transit"
            path[i]["id"] = i
        return path, zone["speed"]

    zone1 = get_zone_from_name(position1["zone"])
    zone2 = get_zone_from_name(position2["zone"])
    
    # case 1
    if position1["zone"] == position2["zone"]:
        movement = position2.copy()
        movement.update(zone2["points"][position2["name"]][position2["id"]])
        return ([movement], ["slow"])

    common_ancestor = get_common_ancestor(zone1, zone2)

    # case 2
    if zone1["_id"] == common_ancestor["_id"]:
        end_point = zone2["points"][position2["name"]][position2["id"]]
        end_point["zone"] = position2["zone"]
        end_point["name"] = position2["name"]
        end_point["id"] = position2["id"]
        
        full_path = get_path(zone2)
        full_path = ([end_point] + full_path[0], ["slow"] + full_path[1] + ["slow"])
        ancestor_index = zone2["ancestor_ids"].index(common_ancestor["_id"])
        for item in zone2["ancestor_ids"][0:ancestor_index]:
            zone = get_zone_from_id(item)
            path = get_path(zone)
            full_path = (full_path[0] + path[0], full_path[1] + path[1] + ["slow"])
        return (full_path[0][::-1], full_path[1][::-1])

    # case 3
    if zone2["_id"] == common_ancestor["_id"]:
        end_point = zone2["points"][position2["name"]][position2["id"]]
        end_point["zone"] = position2["zone"]
        end_point["name"] = position2["name"]
        end_point["id"] = position2["id"]
        
        full_path = get_path(zone1)
        full_path = (full_path[0], ["slow"] + full_path[1])
        ancestor_index = zone1["ancestor_ids"].index(common_ancestor["_id"])
        for item in zone1["ancestor_ids"][0:ancestor_index]:
            zone = get_zone_from_id(item)
            path = get_path(zone)
            full_path = (full_path[0] + path[0], full_path[1] + ["slow"] + path[1] )
        return (full_path[0] + [end_point], full_path[1] + ["slow"])

    # case 4
    end_point = zone2["points"][position2["name"]][position2["id"]]
    end_point["zone"] = position2["zone"]
    end_point["name"] = position2["name"]
    end_point["id"] = position2["id"]

    try:
        full_path1 = get_path(zone1)
        full_path1 = (full_path1[0], ["slow"] + full_path1[1])
        ancestor_index = zone1["ancestor_ids"].index(common_ancestor["_id"])
        for item in zone1["ancestor_ids"][0:ancestor_index]:
            zone = get_zone_from_id(item)
            path = get_path(zone)
            full_path1 = (full_path1[0] + path[0], full_path1[1] + ["slow"] + path[1])

        full_path2 = get_path(zone2)
        full_path2 = ([end_point] + full_path2[0], ["slow"] + full_path2[1] + ["slow"])
        ancestor_index = zone2["ancestor_ids"].index(common_ancestor["_id"])
        for item in zone2["ancestor_ids"][0:ancestor_index]:
            zone = get_zone_from_id(item)
            path = get_path(zone)
            full_path2 = (full_path2[0] + path[0], full_path2[1] + path[1] + ["slow"])
    except ValueError:
        print "they do not share a same ancestor, missing branch in the tree"
        return ([position1], ["slow"])

    return (full_path1[0] + full_path2[0][::-1], full_path1[1] + full_path2[1][::-1])
        
def query_last_position():
    return db.last_position.find_one()

def save_last_position(position):
    return db.last_position.update({}, position, upsert=True)

if not get_zone_from_name("root"):
    root = create_zone("root")
    home = create_zone("home")
    point = create_point(0,0,0,0)
    add_point_to_zone(point, "nest", home)
    save_last_position({"zone": "home", "name": "nest", "id": 0, "action": None, "x": 0, "y": 0, "z": 0, "w": 0})

root = get_zone_from_name("root")
home = get_zone_from_name("home")

if __name__ == "__main__":
    ### example code
    machine_1 = create_zone("machine_1")
    shelf_1 = create_zone("shelf_1")
    add_zone_to_parent(machine_1,root)
    add_zone_to_parent(shelf_1,root)

    #shelfs = create_zone_matrix("shelfs",3,2)

    #add_zone_matrix_to_parent(shelfs, shelf_1)

    point = create_point(700,200,1500,0)
    add_point_to_zone(point, "nest" ,machine_1)
                
    point = create_point(16,17,18,4)
    add_point_to_zone(point, "nest" ,shelf_1)

    path = create_path([[4,5,6,4],[7,8,9,4],[10,11,12,4],[13,14,15,4]])
    speed = ["slow","slow","fast"]

    #add_path_to_zone(path, speed, machine_1)

    path = create_path([[24,25,26,4],[27,28,29,4],[210,211,212,4],[213,214,215,4]])
    speed = ["slow","fast","fast"]

    add_path_to_zone(path, speed, shelf_1)

    result = join_path({"zone":"machine_1", "name":"nest", "id":0}, {"zone":"shelf_1", "name":"nest", "id":0})

    for item in result[0]:
        print item

