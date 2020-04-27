from lxml import etree
import sys

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
    print(new_tag_text)
    tag.text = new_tag_text

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
    print(new_tag_text)
    tag.text = new_tag_text

def replace_top_model_name(tag, model_name):
    tag.attrib['name'] = model_name

def model_trav(path_to_file, model_name, dem_name):
    tree = etree.parse(path_to_file)
    top_tag = tree.xpath("//sdf/model")[0]
    collision_uri_tag = tree.xpath("//sdf/model/link/collision/geometry/heightmap/uri")[0]
    visual_uri_tag = tree.xpath("//sdf/model/link/visual/geometry/heightmap/uri")[0]
    visual_tex_tag = tree.xpath("//sdf/model/link/visual/geometry/heightmap/texture/diffuse")[0]
    print("Attempt to replace collision uri")
    replace_ref(collision_uri_tag, model_name, dem_name)
    print("Attempt to replace visual uri")
    replace_ref(visual_uri_tag, model_name, dem_name)
    print("Attempt to replace texture model uri")
    replace_diff_mod(visual_tex_tag, model_name)
    print("Attempt to replace top model name")
    replace_top_model_name(top_tag, model_name)
    tree.write(path_to_file)

def main():
    path_to_model = sys.argv[1]
    model_name = sys.argv[2]
    dem_img_file = sys.argv[3]
    model_trav(path_to_model, model_name, dem_img_file)

if __name__ == "__main__":
    main()
