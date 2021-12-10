#!/usr/bin/python3
import sys
import xml.etree.cElementTree as xml_tree


def place_model(state, name, pose="0, 0, 0, 0, 0, 0"):
    model_elements = {"pose": pose}
    link_elements = {"pose": pose,
                     "velocity": "0 0 0 0 -0 0",
                     "acceleration": "0 0 0 0 -0 0",
                     "wrench": "0 0 0 0 -0 0"}

    model = xml_tree.SubElement(state, "model", {"name": name})
    for element in model_elements:
        xml_tree.SubElement(model, element).text = model_elements[element]

    link = xml_tree.SubElement(model, "attenuation")
    for element in link_elements:
        xml_tree.SubElement(link, element).text \
            = link_elements[element]


def inlcude_gui(world):
    camera_elements = {"pose": "7.13464 -4.34475 21.0302 0 1.2538 1.30422",
                       "view_controller": "orbit"}
    gui = xml_tree.SubElement(world, "gui", {"fullscreen": "0"})
    camera = xml_tree.SubElement(gui, "camera", {"name": "user_camera"})
    for element in camera_elements:
        xml_tree.SubElement(camera, element).text = camera_elements[element]


def create_model(world, name, width=0, length=0,  height=0, radius=0, shape="box", color="Grey", gravity=1, position="0, 0, 0, 0, 0, 0"):
    wall = xml_tree.SubElement(world, "model", {"name": name})

    pose = xml_tree.SubElement(wall, "pose")
    link = xml_tree.SubElement(wall, "link", {"name": "link"})
    static = xml_tree.SubElement(wall, "static")

    pose.text = position
    static.text = "1"

    inertial = xml_tree.SubElement(link, "inertial")
    collision = xml_tree.SubElement(link, "collision", {"name": "collision"})
    visual = xml_tree.SubElement(link, "visual", {"name": "visual"})
    velocity_decay = xml_tree.SubElement(link, "velocity_decay")

    mass = xml_tree.SubElement(inertial, "mass")
    inertia = xml_tree.SubElement(inertial, "inertia")

    mass.text = "1"
    inertia_elements = {"ixx": "1",
                        "ixy": "0",
                        "ixz": "0",
                        "iyy": "1",
                        "iyz": "0",
                        "izz": "1"}

    for element in inertia_elements:
        xml_tree.SubElement(inertia, element).text \
            = inertia_elements[element]

    geometry = xml_tree.SubElement(collision, "geometry")
    max_contacts = xml_tree.SubElement(collision, "max_contacts")
    surface = xml_tree.SubElement(collision, "surface")

    if shape == "box":
        box = xml_tree.SubElement(geometry, "box")
        size = xml_tree.SubElement(box, "size")
        size.text = str(width) + " " + str(length) + " " + str(height)
    else:
        sphere = xml_tree.SubElement(geometry, "sphere")
        size = xml_tree.SubElement(sphere, "radius")
        size.text = str(radius)

    max_contacts.text = "10"

    xml_tree.SubElement(surface, "bounce")
    friction = xml_tree.SubElement(surface, "friction")
    contact = xml_tree.SubElement(surface, "contact")

    xml_tree.SubElement(friction, "ode")
    xml_tree.SubElement(contact, "ode")

    geometry1 = xml_tree.SubElement(visual, "geometry")
    if shape == "box":
        box1 = xml_tree.SubElement(geometry1, "box")
        size1 = xml_tree.SubElement(box1, "size")
        size1.text = str(width) + " " + str(length) + " " + str(height)
    else:
        sphere1 = xml_tree.SubElement(geometry1, "sphere")
        size1 = xml_tree.SubElement(sphere1, "radius")
        size1.text = str(radius)

    material = xml_tree.SubElement(visual, "material")
    script = xml_tree.SubElement(material, "script")
    uri = xml_tree.SubElement(script, "uri")
    name_script = xml_tree.SubElement(script, "name")
    uri.text = "file://media/materials/scripts/gazebo.material"
    name_script.text = "Gazebo/" + color

    velocity_decay_elements = {"linear": "0",
                               "angular": "0"}

    for element in velocity_decay_elements:
        xml_tree.SubElement(velocity_decay, element).text \
            = velocity_decay_elements[element]

    link_elements = {"self_collide": "0",
                     "kinematic": "0",
                     "gravity": str(gravity)}

    for element in link_elements:
        xml_tree.SubElement(link, element).text \
            = link_elements[element]


def initialize_scene(world):
    physics = xml_tree.SubElement(world, "physics", {"type": "ode"})
    scene = xml_tree.SubElement(world, "scene")
    spherical_coordinates = xml_tree.SubElement(world, "spherical_coordinates")

    physics_elements = {"max_step_size": "0.001",
                        "real_time_factor": "1",
                        "real_time_update_rate": "1000",
                        "gravity": "0 0 -9.8"}
    scene_elements = {"ambient": "0.4 0.4 0.4 1",
                      "background": "0.7 0.7 0.7 1",
                      "shadows": "1"}
    spherical_coordinates_elements = {"surface_model": "EARTH_WGS84",
                                      "latitude_deg": "0",
                                      "longitude_deg": "0",
                                      "elevation": "0",
                                      "heading_deg": "0"}

    for element in physics_elements:
        xml_tree.SubElement(physics, element).text = physics_elements[element]

    for element in scene_elements:
        xml_tree.SubElement(scene, element).text = scene_elements[element]

    for element in spherical_coordinates_elements:
        xml_tree.SubElement(
            spherical_coordinates, element).text = spherical_coordinates_elements[element]


def include_ground(world):
    atributes = {"name": "ground_plane"}
    model_elements = {"static": "1"}
    link_elements = {"self_collide": "0",
                     "kinematic": "0",
                     "gravity": "1"}

    ground = xml_tree.SubElement(world, "model", atributes)
    for element in model_elements:
        xml_tree.SubElement(ground, element).text = model_elements[element]

    link = xml_tree.SubElement(ground, "link", {"name": "link"})
    collision = xml_tree.SubElement(link, "collision", {"name": "collision"})
    geometry = xml_tree.SubElement(collision, "geometry")
    plane_elements = {"normal": "0 0 1",
                      "size": "100 100"}
    plane = xml_tree.SubElement(geometry, "plane")
    for element in plane_elements:
        xml_tree.SubElement(plane, element).text \
            = plane_elements[element]
    surface = xml_tree.SubElement(collision, "surface")
    friction = xml_tree.SubElement(surface, "friction")
    ode_elements = {"mu": "100",
                    "mu2": "50"}
    ode = xml_tree.SubElement(friction, "ode")
    for element in ode_elements:
        xml_tree.SubElement(ode, element).text \
            = ode_elements[element]
    xml_tree.SubElement(surface, "bounce")
    contact = xml_tree.SubElement(surface, "contact")
    xml_tree.SubElement(contact, "ode")
    max_contacts = xml_tree.SubElement(collision, "max_contacts")
    max_contacts.text = "10"

    visual = xml_tree.SubElement(link, "visual", {"name": "visual"})
    cast_shadows = xml_tree.SubElement(visual, "cast_shadows")
    cast_shadows.text = "0"
    geometry1 = xml_tree.SubElement(visual, "geometry")
    plane1 = xml_tree.SubElement(geometry1, "plane")
    for element in plane_elements:
        xml_tree.SubElement(plane1, element).text \
            = plane_elements[element]

    material = xml_tree.SubElement(visual, "material")
    script_elements = {"uri": "file://media/materials/scripts/gazebo.material",
                       "name": "Gazebo/Grey"}
    script = xml_tree.SubElement(material, "script")
    for element in script_elements:
        xml_tree.SubElement(script, element).text \
            = script_elements[element]

    velocity_decay_elements = {"linear": "0",
                               "angular": "0"}
    velocity_decay = xml_tree.SubElement(link, "velocity_decay")
    for element in velocity_decay_elements:
        xml_tree.SubElement(velocity_decay, element).text \
            = velocity_decay_elements[element]
    for element in link_elements:
        xml_tree.SubElement(link, element).text \
            = link_elements[element]


def include_light(world):
    atributes = {"name": "sun", "type": "directional"}
    light_elements = {"cast_shadows": "1",
                      "pose": "0 0 10 0 -0 0",
                      "diffuse": "0.8 0.8 0.8 1",
                      "specular": "0.2 0.2 0.2 1",
                      "direction": "-0.5 0.1 -0.9"}
    attenuation_elements = {"range": "1000",
                            "constant": "0.9",
                            "linear": "0.01",
                            "quadratic": "0.001"}

    light = xml_tree.SubElement(world, "light", atributes)
    for element in light_elements:
        xml_tree.SubElement(light, element).text = light_elements[element]

    attenuation = xml_tree.SubElement(light, "attenuation")
    for element in attenuation_elements:
        xml_tree.SubElement(attenuation, element).text \
            = attenuation_elements[element]


def main():
    if len(sys.argv) != 12:
        print("Wrong number of arguemnts")
        return

    # Initialization
    gazebo = xml_tree.Element("gazebo", {"version": "1.0"})
    world = xml_tree.SubElement(gazebo, "world", {"name": "default"})
    include_light(world)
    include_ground(world)
    initialize_scene(world)

    # Walls model creation:
    create_model(world=world, name="W1", width=12,
                 length=0.2, height=1.0, shape="box",
                 gravity=1, color="Grey", position="0.0 -6.1 0.5 0 0 0")
    create_model(world=world, name="W2", width=0.2,
                 length=11.4, height=1.0, shape="box",
                 gravity=1, color="Grey", position="0.0 -6.1 0.5 0 0 0")
    create_model(world=world, name="W3", width=12,
                 length=0.2, height=1.0, shape="box",
                 gravity=1, color="Grey", position="-6.1 -0.5 0.5 0 0 0")
    create_model(world=world, name="W4", width=0.2,
                 length=11.4, height=1.0, shape="box",
                 gravity=1, color="Grey", position="6.1 -0.5 0.5 0 0 0")

    # Red finish line model creation
    create_model(world=world, name="FINISH", width=1.0,
                 length=11.0, height=0.01, shape="box",
                 gravity=0, color="Red", position="5.5 -0.5 0.0 0 0 0")

    # Spheres creation
    sphere_positions = {
        "S1": "-5.5 -2.5 1.1 0 0 0",
        "S2": "-5.5 1.5 1.1 0 0 0",
        "S3": "-4.5 2.5 1.1 0 0 0",
        "S4": "-3.5 -1.5 1.1 0 0 0",
        "S5": "-2.5 -2.5 1.1 0 0 0",
        "S6": "-2.5 2.5 1.1 0 0 0",
        "S7": "-1.5 -2.5 1.1 0 0 0",
        "S8": "-0.5 2.5 1.1 0 0 0",
        "S9": "0.5 -4.5 1.1 0 0 0",
        "S10": "3.5 -2.5 1.1 0 0 0",
        "S11": "3.5 3.5 1.1 0 0 0"
    }

    color_encoding = {
        "0": "Blue",
        "1": "Red"
    }

    sphere_colors = {
        "S1": color_encoding[sys.argv[1]],
        "S2": color_encoding[sys.argv[2]],
        "S3": color_encoding[sys.argv[3]],
        "S4": color_encoding[sys.argv[4]],
        "S5": color_encoding[sys.argv[5]],
        "S6": color_encoding[sys.argv[6]],
        "S7": color_encoding[sys.argv[7]],
        "S8": color_encoding[sys.argv[8]],
        "S9": color_encoding[sys.argv[9]],
        "S10": color_encoding[sys.argv[10]],
        "S11": color_encoding[sys.argv[11]]
    }

    for sphere in sphere_positions:
        create_model(world=world, name=sphere, radius=0.2, shape="sphere", gravity=0,
                     color=sphere_colors[sphere], position=sphere_positions[sphere])

    # Placing objects into the default state
    state = xml_tree.SubElement(world, "state", {"world_name": "default"})
    state_elements = {"sim_time": "1122 88000000",
                      "real_time": "1125 348952370",
                      "wall_time": "1507096949 25321804"}
    for element in state_elements:
        xml_tree.SubElement(state, element).text \
            = state_elements[element]

    # Place wall models
    place_model(state=state, name="ground_plane", pose="0 0 0 0 -0 0")
    place_model(state=state, name="W1", pose="0.0 -6.1 0.5 0 0 0")
    place_model(state=state, name="W2", pose="-6.1 -0.5 0.5 0 0 0")
    place_model(state=state, name="W3", pose="0.0 5.1 0.5 0 0 0")
    place_model(state=state, name="W4", pose="6.1 -0.5 0.5 0 0 0")

    # Place Finish line model
    place_model(state=state, name="FINISH", pose="5.5 -0.5 0.0 0 0 0")

    # Place spheres
    for sphere in sphere_positions:
        place_model(state=state, name=sphere, pose=sphere_positions[sphere])

    # Final steps
    inlcude_gui(world)
    tree = xml_tree.ElementTree(gazebo)
    tree.write('7x11_grid.world')


if __name__ == '__main__':
    main()
