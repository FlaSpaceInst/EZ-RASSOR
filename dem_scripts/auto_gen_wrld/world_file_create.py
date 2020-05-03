#!/usr/bin/env python
from lxml import etree
import sys
import model_create as mc

""" Fill in world template """
def world_trav(path_to_file, model_name, dem_name, w, h, squish_factor):

    tree = etree.parse(path_to_file)
    tree.xpath("//sdf/world/state")[0].attrib['world_name'] = model_name \
                                                            + "_world"
    tree.xpath("//sdf/world/state/model")[0].attrib['name'] = model_name

    include_tag = tree.xpath(
        "//sdf/world/include"
        )[1].getchildren()[0]

    top_tag = tree.xpath(
        "//sdf/world/model"
        )[0]

    world_tag = tree.xpath(
        "//sdf/world"
        )[0]

    collision_uri_tag = tree.xpath(
        "//sdf/world/model/link/collision/geometry/heightmap/uri"
        )[0]

    visual_uri_tag = tree.xpath(
        "//sdf/world/model/link/visual/geometry/heightmap/uri"
        )[0]

    visual_tex_tag = tree.xpath(
        "//sdf/world/model/link/visual/geometry/heightmap/texture/diffuse"
        )[0]

    collision_size_tag = tree.xpath(
        "//sdf/world/model/link/collision/geometry/heightmap/size"
        )[0]

    visual_size_tag = tree.xpath(
        "//sdf/world/model/link/visual/geometry/heightmap/size"
        )[0]

    mc.replace_ref(collision_uri_tag, model_name, dem_name)

    mc.replace_ref(visual_uri_tag, model_name, dem_name)

    mc.replace_diff_mod(visual_tex_tag, model_name)

    mc.replace_top_model_name(top_tag, model_name)

    mc.replace_top_model_name(world_tag, model_name + "_world")

    mc.replace_diff_mod(include_tag, model_name)

    mc.replace_text_field(collision_size_tag, w + " " + h + " " + squish_factor)

    mc.replace_text_field(visual_size_tag, w + " " + h + " " + squish_factor)

    tree.write(path_to_file, xml_declaration=True)

def main():

    if len(sys.argv) < 6:
        print(
            "Not enough arguments, expects: path_to_model, model_name"
            + "dem_img_file, dimmensions_str('w h'), squish_factor"
        )
        return

    path_to_world = sys.argv[1]
    model_name = sys.argv[2]
    dem_img_file = sys.argv[3]
    dimmensions_w = sys.argv[4]
    dimmensions_h = sys.argv[5]
    squish_factor = sys.argv[6]

    world_trav(
        path_to_world, model_name, dem_img_file,
        dimmensions_w, dimmensions_h,
        squish_factor
    )

if __name__ == "__main__":
    main()
