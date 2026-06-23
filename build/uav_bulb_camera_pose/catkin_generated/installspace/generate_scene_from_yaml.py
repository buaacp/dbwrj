#!/usr/bin/env python3
import argparse
import math
import os

from uav_bulb_camera_pose.bundle_geometry import load_bundles, matrix_to_pose


def sdf_pose_from_T(T):
    p, q = matrix_to_pose(T)
    x, y, z, w = q
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(min(sinp, 1.0), -1.0))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return '%.6f %.6f %.6f %.6f %.6f %.6f' % (p[0], p[1], p[2], roll, pitch, yaw)


def write_model(model_dir, name, bundle, texture_dir):
    os.makedirs(model_dir, exist_ok=True)
    mesh_dir = os.path.join(model_dir, 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)
    marker_fraction = 0.75
    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write("""<?xml version='1.0'?>
<model><name>{name}</name><version>1.0</version><sdf version='1.6'>model.sdf</sdf></model>
""".format(name=name))
    visuals = []
    for tag in bundle.tags:
        size = tag.size_m
        visual_size = size / marker_fraction
        texture = os.path.abspath(os.path.join(texture_dir, 'tag36h11_%03d.png' % tag.id))
        mesh_path = os.path.abspath(os.path.join(mesh_dir, 'tag_%03d.dae' % tag.id))
        write_tag_dae(mesh_path, visual_size, texture, tag.T_object_tag)
        visuals.append("""
      <visual name='tag_{id}'>
          <geometry><mesh><uri>file://{mesh_path}</uri></mesh></geometry>
      </visual>""".format(id=tag.id, mesh_path=mesh_path))
    body = "<cylinder><radius>0.004</radius><length>0.010</length></cylinder>" if bundle.name == 'socket' else "<sphere><radius>0.004</radius></sphere>"
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
        f.write("""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='{frame}'>
      <visual name='body'><geometry>{body}</geometry><material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
{visuals}
    </link>
  </model>
</sdf>
""".format(name=name, frame=bundle.frame_id, body=body, visuals='\n'.join(visuals)))


def write_tag_dae(path, visual_size, texture_path, T_object_tag):
    s = visual_size / 2.0
    local = [
        [-s, -s, 0.0, 1.0],
        [s, -s, 0.0, 1.0],
        [s, s, 0.0, 1.0],
        [-s, s, 0.0, 1.0],
    ]
    pts = [T_object_tag.dot(p)[:3] for p in local]
    pos = ' '.join('%.9f %.9f %.9f' % (p[0], p[1], p[2]) for p in pts)
    with open(path, 'w') as f:
        f.write("""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_images>
    <image id="tag_image"><init_from>file://{texture}</init_from></image>
  </library_images>
  <library_effects>
    <effect id="tag_effect">
      <profile_COMMON>
        <newparam sid="tag_surface"><surface type="2D"><init_from>tag_image</init_from></surface></newparam>
        <newparam sid="tag_sampler"><sampler2D><source>tag_surface</source></sampler2D></newparam>
        <technique sid="common">
          <lambert><emission><color>1 1 1 1</color></emission><diffuse><texture texture="tag_sampler" texcoord="UVSET0"/></diffuse></lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials><material id="tag_material"><instance_effect url="#tag_effect"/></material></library_materials>
  <library_geometries>
    <geometry id="tag_geometry">
      <mesh>
        <source id="positions">
          <float_array id="positions-array" count="12">{pos}</float_array>
          <technique_common><accessor source="#positions-array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common>
        </source>
        <source id="uvs">
          <float_array id="uvs-array" count="8">0 0 1 0 1 1 0 1</float_array>
          <technique_common><accessor source="#uvs-array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common>
        </source>
        <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
        <triangles material="tag_material" count="2">
          <input semantic="VERTEX" source="#vertices" offset="0"/>
          <input semantic="TEXCOORD" source="#uvs" offset="1" set="0"/>
          <p>0 0 1 1 2 2 0 0 2 2 3 3</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene"><node id="tag"><instance_geometry url="#tag_geometry"><bind_material><technique_common><instance_material symbol="tag_material" target="#tag_material"><bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
""".format(texture=texture_path, pos=pos))


def write_camera_model(model_dir):
    os.makedirs(model_dir, exist_ok=True)
    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write("""<?xml version='1.0'?><model><name>d435i_color_camera_rig</name><version>1.0</version><sdf version='1.6'>model.sdf</sdf></model>""")
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
        f.write("""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='d435i_color_camera_rig'>
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    <link name='camera_color_optical_frame'>
      <sensor name='d435i_color' type='camera'>
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <camera>
          <horizontal_fov>1.204277</horizontal_fov>
          <image><width>1920</width><height>1080</height><format>R8G8B8</format></image>
          <clip><near>0.10</near><far>5.0</far></clip>
        </camera>
        <plugin name='camera_controller' filename='libgazebo_ros_camera.so'>
          <robotNamespace>/camera/camera/color</robotNamespace>
          <cameraName></cameraName>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <frameName>camera_color_optical_frame</frameName>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
""")


def write_occluder_model(model_dir):
    os.makedirs(model_dir, exist_ok=True)
    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write("""<?xml version='1.0'?><model><name>occluder</name><version>1.0</version><sdf version='1.6'>model.sdf</sdf></model>""")
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
        f.write("""<?xml version='1.0'?><sdf version='1.6'><model name='occluder'><static>true</static><link name='link'><visual name='v'><geometry><box><size>0.08 0.08 0.20</size></box></geometry><material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material></visual><collision name='c'><geometry><box><size>0.08 0.08 0.20</size></box></geometry></collision></link></model></sdf>""")


def write_world(path):
    with open(path, 'w') as f:
        f.write("""<?xml version='1.0'?>
<sdf version='1.6'>
  <world name='bulb_pose_test'>
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://d435i_color_camera_rig</uri><pose>0 0 0 0 0 0</pose></include>
    <include><uri>model://socket_tag_ring</uri><pose>0.60 0 -0.10 0 0 0</pose></include>
    <include><uri>model://bulb_tag_ring</uri><pose>0.60 0 0.10 0 0 0</pose></include>
    <include><uri>model://occluder</uri><pose>0.20 0.60 0.68 0 0 0</pose></include>
  </world>
</sdf>
""")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bundle-yaml', required=True)
    parser.add_argument('--output-root', required=True)
    parser.add_argument('--texture-dir', required=True)
    args = parser.parse_args()
    bundles = load_bundles(args.bundle_yaml)
    models = os.path.join(args.output_root, 'models')
    worlds = os.path.join(args.output_root, 'worlds')
    os.makedirs(models, exist_ok=True)
    os.makedirs(worlds, exist_ok=True)
    write_camera_model(os.path.join(models, 'd435i_color_camera_rig'))
    write_model(os.path.join(models, 'socket_tag_ring'), 'socket_tag_ring', bundles['socket'], args.texture_dir)
    write_model(os.path.join(models, 'bulb_tag_ring'), 'bulb_tag_ring', bundles['bulb'], args.texture_dir)
    write_occluder_model(os.path.join(models, 'occluder'))
    write_world(os.path.join(worlds, 'bulb_pose_test.world'))


if __name__ == '__main__':
    main()
