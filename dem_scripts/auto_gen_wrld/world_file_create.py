from lxml import etree
import sys
from model_create import replace_diff_mod, replace_ref

def replace_top_name(tag, model_name):
    tag.attrib['name'] = model_name

def world_trav(path_to_file, model_name, dem_name):
    tree = etree.parse(path_to_file)
    tree.xpath("//sdf/world/state")[0].attrib['world_name'] = model_name + "_world"
    tree.xpath("//sdf/world/state/model")[0].attrib['name'] = model_name
    include_tag = tree.xpath("//sdf/world/include")[1].getchildren()[0]
    top_tag = tree.xpath("//sdf/world/model")[0]
    world_tag = tree.xpath("//sdf/world")[0]
    collision_uri_tag = tree.xpath("//sdf/world/model/link/collision/geometry/heightmap/uri")[0]
    visual_uri_tag = tree.xpath("//sdf/world/model/link/visual/geometry/heightmap/uri")[0]
    visual_tex_tag = tree.xpath("//sdf/world/model/link/visual/geometry/heightmap/texture/diffuse")[0]
    print("Attempt to replace collision uri")
    replace_ref(collision_uri_tag, model_name, dem_name)
    print("Attempt to replace visual uri")
    replace_ref(visual_uri_tag, model_name, dem_name)
    print("Attempt to replace texture model uri")
    replace_diff_mod(visual_tex_tag, model_name)
    print("Attempt to replace model name")
    replace_top_name(top_tag, model_name)
    print("Attempt to replace world name")
    replace_top_name(world_tag, model_name + "_world")
    print("Attempt to replace include name")
    replace_diff_mod(include_tag, model_name)
    tree.write(path_to_file, xml_declaration=True)

def main():
    model_name = sys.argv[2]
    dem_img_file = sys.argv[3]
    path_to_world = sys.argv[1]
    world_trav(path_to_world, model_name, dem_img_file)

if __name__ == "__main__":
    main()
