from lxml import etree
import sys
import os
import pygame

SDF_VERSION = 1.4
POS = "0 0 0"
DIFFUSE = "AS16-110-18026HR-512x512.jpg"
NORMAL = "file://media/textures/flat_normal.png"

AUTHOR_NAME = "Shelby Basco"
AUTHOR_EMAIL = "blicogam@outlook.com"

def create_geometry_map(model_name, dem_file, dem_size):

    # L0 <geometry>
    geometry = etree.Element('geometry')
    # L1 <heightmap>
    heightmap = etree.Element('heightmap')

    # L2 <uri></uri>
    uri = etree.Element('uri')
    uri.text = "model://" + model_name + "/materials/" + dem_file

    # L2 <pos></pos>
    pos = etree.Element('pos')
    pos.text = POS

    # L2 <size></size>
    size = etree.Element('size')
    size.text = str(dem_size) + " " + str(dem_size) + " 50"

    # Add L2 tags + implicit </heightmap>
    heightmap.append(uri)
    heightmap.append(pos)
    heightmap.append(size)

    # Add L1 tags + implicit </geometry>
    geometry.append(heightmap)

    return geometry

# repeated computation from create_geometry_map
def create_visual_map(model_name, dem_file, dem_size):

    # L-1 <visual>
    visual = etree.Element('visual', name="visual")
    # L0 <geometry>
    geometry = etree.Element('geometry')
    # L1 <heightmap>
    heightmap = etree.Element('heightmap')

    # L2 <texture>
    texture = etree.Element('texture')

    # L3 <diffuse></diffuse>
    diffuse = etree.Element('diffuse')
    diffuse.text = "model://" + model_name + "/materials/" + DIFFUSE

    # L3 <normal></normal>
    normal = etree.Element('normal')
    normal.text = NORMAL

    # L3 <size></size>
    size_texture = etree.Element('size')
    size_texture.text = str(2)

    # Add L2 tags + implicit </texture>
    texture.append(diffuse)
    texture.append(normal)
    texture.append(size_texture)

    # L2 <uri></uri>
    uri = etree.Element('uri')
    uri.text = "model://" + model_name + "/materials/" + dem_file

    # L2 <pos></pos>
    pos = etree.Element('pos')
    pos.text = POS

    # L2 <size></size>
    size_dimmension = etree.Element('size')
    size_dimmension.text = str(dem_size) + " " + str(dem_size) + " 50"

    # Add L2 tags + implicit </heightmap>
    heightmap.append(texture)
    heightmap.append(uri)
    heightmap.append(pos)
    heightmap.append(size_dimmension)

    # Add L1 tags + implicit </geometry>
    geometry.append(heightmap)

    # Add L0 tags + implicit </visual>
    visual.append(geometry)

    return visual

def create_model_tag(model_name, dem_file, dem_size, create_world=False):

    # L0 <model>
    model = etree.Element('model', name=model_name)

    # L1 <static>true</static>
    static = etree.Element('static')
    static.text = 'true'

    # L1 <link> (idk why attribute name is also link)
    link = etree.Element('link', name="link")

    # L2 <collision>
    collision = etree.Element('collision', name="collision")

    # create_world flag for blend in both geometry and visual
    # not sure how important is blend or max_contacts
    # L3 <geometry></geometry>
    geometry_map = create_geometry_map(model_name, dem_file, dem_size)

    # L2 <visual></visual>
    visual_map = create_visual_map(model_name, dem_file, dem_size)

    if create_world:
        self_collide = etree.Element('self_collide')
        self_collide.text = str(0)

        kinematic = etree.Element('kinematic')
        kinematic.text = str(0)

        gravity = etree.Element('gravity')
        gravity.text = str(1)

        surface = create_surface_friction()

        max_contacts = etree.Element('max_contacts')
        max_contacts.text = str(10)

        collision.append(surface)
        collision.append(max_contacts)
        link.append(self_collide)
        link.append(kinematic)
        link.append(gravity)

    # Add L3 tags + implicit </collision>
    collision.append(geometry_map)

    # Add L2 tags + implicit </link>
    link.append(collision)
    link.append(visual_map)

    # Add L1 tags + implicit </model>
    model.append(static)
    model.append(link)

    return model

def create_model_sdf(world_name, dem_file, dem_size):
    # create XML

    # L0 <sdf>
    sdf = etree.Element('sdf', version=str(SDF_VERSION))

    # L1 <model>
    model = create_model_tag(world_name, dem_file, dem_size)

    # Add L1 tags + implicit </sdf>
    sdf.append(model)
    # pretty string
    s = etree.tostring(sdf, pretty_print=True, xml_declaration=True)
    return s

def create_global_light():
    include = etree.Element('include')
    uri = etree.Element('uri')
    uri.text = "model://" + "sun"
    include.append(uri)
    return include

def create_place_model(name):
    include = etree.Element('include')

    uri = etree.Element('uri')
    uri.text = "model://" + name

    friction = etree.Element('friction')

    mu = etree.Element('mu')
    mu.text = str(0.01)
    mu2 = etree.Element('mu2')
    mu2.text = str(0.01)

    friction.append(mu)
    friction.append(mu2)

    include.append(uri)
    include.append(friction)

    return include

def create_world_file(world_name, dem_file, dem_size):
    # create XML

    # L0 <sdf>
    sdf = etree.Element('sdf', version=str(SDF_VERSION))
    world = etree.Element('world', name=world_name)

    light = create_global_light()
    place_model = create_place_model(world_name)
    # L1 <model>
    model = create_model_tag(world_name, dem_file, dem_size, True)
    physics = create_physics_map()
    gui = set_gui()

    gravity = etree.Element('gravity')
    gravity.text = "0 0 -1.622"

    magnetic_field = etree.Element('magnetic_field')
    magnetic_field.text = "6e-06 2.3e-05 -4.2e-05"

    atmosphere = etree.Element('atmosphere', type='adiabatic')
    spherical_coordinates = create_spherical_coordinates()

    state = create_state(world_name)

    world.append(light)
    world.append(place_model)
    world.append(model)
    world.append(physics)
    world.append(gravity)
    world.append(magnetic_field)
    world.append(atmosphere)
    world.append(spherical_coordinates)
    world.append(state)
    world.append(gui)
    # Add L1 tags + implicit </sdf>
    sdf.append(world)
    # pretty string
    s = etree.tostring(sdf, pretty_print=True, xml_declaration=True)
    return s
def create_model_config(model_name):

    model = etree.Element('model')

    name = etree.Element('name')
    name.text = model_name
    version = etree.Element('version')
    version.text = str(1.0)
    sdf = etree.Element('sdf', version=str(SDF_VERSION))
    sdf.text = "model.sdf"

    author = etree.Element('author')

    author_name = etree.Element('name')
    author_name.text = AUTHOR_NAME

    author_email = etree.Element('email')
    author_email.text = AUTHOR_EMAIL
    author.append(author_name)
    author.append(author_email)

    description = etree.Element('description')
    description.text = "Auto generated model + world, user please add additional info"
    model.append(name)
    model.append(version)
    model.append(sdf)
    model.append(author)
    model.append(description)
    s = etree.tostring(model, pretty_print=True, xml_declaration=True)
    return s
def create_physics_map():
    physics = etree.Element('physics', name='default_physics', default='0', type='ode')

    max_step_size = etree.Element('max_step_size')
    max_step_size.text = str(0.01)

    real_time_factor = etree.Element('real_time_factor')
    real_time_factor.text = str(1)

    real_time_update_rate = etree.Element('real_time_update_rate')
    real_time_update_rate.text = str(75)

    physics.append(max_step_size)
    physics.append(real_time_factor)
    physics.append(real_time_update_rate)

    return physics

def create_spherical_coordinates():
    spherical_coordinates = etree.Element('spherical_coordinates')

    surface_model = etree.Element('surface_model')
    surface_model.text = "EARTH_WGS84"

    latitude_deg = etree.Element('latitude_deg')
    latitude_deg.text = str(0)

    longitude_deg = etree.Element('longitude_deg')
    longitude_deg.text = str(0)

    elevation= etree.Element('elevation')
    elevation.text = str(0)

    heading_deg = etree.Element('heading_deg')
    heading_deg.text = str(0)

    spherical_coordinates.append(surface_model)
    spherical_coordinates.append(latitude_deg)
    spherical_coordinates.append(longitude_deg)
    spherical_coordinates.append(elevation)
    spherical_coordinates.append(heading_deg)

    return spherical_coordinates

def create_state(model_world_name):
    state = etree.Element('state', world_name=model_world_name)

    sim_time = etree.Element('sim_time')
    sim_time.text = "400 160000000"

    real_time = etree.Element('real_time')
    real_time.text = "311 661025296"

    wall_time = etree.Element('wall_time')
    wall_time.text = "1554701956 216832245"

    iterations = etree.Element('iterations')
    iterations.text = str(30978)

    model = etree.Element('model', name=model_world_name)

    pose_mod = etree.Element('pose', frame='')
    pose_mod.text = "0 0 0 0 -0 0"

    scale = etree.Element('scale')
    scale.text = "1 1 1"

    link = etree.Element('link', name='link')

    pose_lin = etree.Element('pose', frame='')
    pose_lin.text = "0 0 0 0 -0 0"

    velocity = etree.Element('velocity')
    velocity.text = "0 0 0 0 -0 0"

    acceleration = etree.Element('acceleration')
    acceleration.text = "0 0 0 0 -0 0"

    wrench = etree.Element('wrench')
    wrench.text = "0 0 0 0 -0 0"

    link.append(pose_lin)
    link.append(velocity)
    link.append(acceleration)
    link.append(wrench)

    model.append(pose_mod)
    model.append(scale)
    model.append(link)

    state.append(sim_time)
    state.append(real_time)
    state.append(wall_time)
    state.append(iterations)
    state.append(model)

    return state

def set_gui():
    gui = etree.Element('gui', fullscreen='0')

    camera = etree.Element('camera', name='user_camera')

    pose = etree.Element('pose', frame='')
    pose.text = "16.3249 -27.6364 14.6277 -0 0.621796 2.22418"

    view_controller = etree.Element('view_controller')
    view_controller.text = "orbit"

    projection_type = etree.Element('projection_type')
    projection_type.text = "perspective"

    camera.append(pose)
    camera.append(view_controller)
    camera.append(projection_type)

    gui.append(camera)

    return gui

def create_surface_friction():
    surface = etree.Element('surface')
    friction = etree.Element('friction')

    ode_mu = etree.Element('ode')

    mu = etree.Element('mu')
    mu.text = str(0.01)

    torsional = etree.Element('torsional')

    ode_tor = etree.Element('ode')

    contact = etree.Element('contact')

    ode_con = etree.Element('ode')

    soft_cfm = etree.Element('soft_cfm')
    soft_cfm.text = str(1)

    kp = etree.Element('kp')
    kp.text = str(100000)

    kd = etree.Element('kd')
    kd.text = str(1)

    max_vel = etree.Element('max_vel')
    max_vel.text = str(1e-06)

    min_depth = etree.Element('min_depth')
    min_depth.text = str(0.02)

    bounce = etree.Element('bounce')

    ode_mu.append(mu)
    torsional.append(ode_tor)
    friction.append(ode_mu)
    friction.append(torsional)

    ode_con.append(soft_cfm)
    ode_con.append(kp)
    ode_con.append(kd)
    ode_con.append(max_vel)
    ode_con.append(min_depth)
    contact.append(ode_con)

    surface.append(friction)
    surface.append(contact)
    surface.append(bounce)

    return surface

def main():
    dem_file = sys.argv[1]
    world_name = sys.argv[2]
    where = sys.argv[3]
    base = os.path.basename(dem_file)
    img = pygame.image.load(dem_file)
    dem_size = img.get_width()
    #dem_size = sys.argv[5]
    #print(dem_file)
    world_file = open(where + world_name + ".world","w")
    model_sdf = open(where + world_name + "/" + "model.sdf", "w")
    model_config = open(where + world_name + "/" + "model.config", "w")
    world_file.write(create_world_file(world_name, base, dem_size))
    model_sdf.write(create_model_sdf(world_name, base, dem_size))
    model_config.write(create_model_config(world_name))
    world_file.close()
    model_sdf.close()
    model_config.close()

if __name__ == "__main__":
    main()
