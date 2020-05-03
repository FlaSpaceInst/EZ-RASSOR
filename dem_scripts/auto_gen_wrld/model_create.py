#!/usr/bin/env python
from lxml import etree
import sys

""" Given a uri tag, modify model_name and dem_name parts of the path """
def replace_ref(tag, model_name, dem_name):
    first = True
    tag_text = tag.text
    tag_text = tag_text.split("/")
    tag_text[-1] = dem_name
    tag_text[2] = model_name
    new_tag_text = ""
    for i in tag_text:
        if first:
            new_tag_text += i
            first = False
        else:
            new_tag_text += "/" + i

    tag.text = new_tag_text

""" Given a uri tag, modify model_name part of the path """
def replace_diff_mod(tag, model_name):
    first = True
    tag_text = tag.text
    tag_text = tag_text.split("/")
    tag_text[2] = model_name
    new_tag_text = ""

    for i in tag_text:
        if first:
            new_tag_text += i
            first = False
        else:
            new_tag_text += "/" + i

    tag.text = new_tag_text

""" Update a tag's name attribute with model_name """
def replace_top_model_name(tag, model_name):
    tag.attrib['name'] = model_name

""" Update a tag's text field with string """
def replace_text_field(tag, string):
    tag.text = string

""" Fill in model.sdf template """
def model_trav(path_to_file, model_name, dem_name, w, h, squish_factor):

    tree = etree.parse(path_to_file)

    top_tag = tree.xpath(
                "//sdf/model"
                )[0]

    collision_uri_tag = tree.xpath(
                "//sdf/model/link/collision/geometry/heightmap/uri"
                )[0]
    visual_uri_tag = tree.xpath(
                "//sdf/model/link/visual/geometry/heightmap/uri"
                )[0]
    visual_tex_tag = tree.xpath(
                "//sdf/model/link/visual/geometry/heightmap/texture/diffuse"
                )[0]
    collision_size_tag = tree.xpath(
                "//sdf/model/link/collision/geometry/heightmap/size"
                )[0]
    visual_size_tag = tree.xpath(
                "//sdf/model/link/visual/geometry/heightmap/size"
                )[0]

    replace_ref(collision_uri_tag, model_name, dem_name)

    replace_ref(visual_uri_tag, model_name, dem_name)

    replace_diff_mod(visual_tex_tag, model_name)

    replace_top_model_name(top_tag, model_name)

    replace_text_field(collision_size_tag, w + " " + h + " " + squish_factor)

    replace_text_field(visual_size_tag, w + " " + h + " " + squish_factor)

    tree.write(path_to_file, xml_declaration=True)

def main():

    if len(sys.argv) < 6:
        print(
            "Not enough arguments, expects: path_to_model, model_name"
            + "dem_img_file, dimmensions_str('w h'), squish_factor"
        )
        return

    path_to_model = sys.argv[1]
    model_name = sys.argv[2]
    dem_img_file = sys.argv[3]
    dimmensions_w = sys.argv[4]
    dimmensions_h = sys.argv[5]
    squish_factor = sys.argv[6]

    model_trav(
        path_to_model, model_name, dem_img_file,
        dimmensions_w, dimmensions_h,
        squish_factor
    )

if __name__ == "__main__":
    main()
