/*
  copyright (c) 2018 georg bartels
  all rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

:- begin_tests(urdf_parser).

:- use_module('urdf_parser').
:- use_module(library('roscpp')).

:- owl_parser:owl_parse('package://knowrob_srdl/owl/urdf.owl').
:- rdf_db:rdf_register_ns(urdf, 'http://knowrob.org/kb/urdf.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

test(load_urdf_file_pr2) :-
  ros_package_path('knowrob_srdl', X),
  atom_concat(X, '/urdf/pr2_for_unit_tests.urdf', Filename),
  load_urdf_file(Filename).

test(load_non_existent_urdf, fail) :-
  load_urdf_file('foo.urdf').

test(robot_name_pr2) :-
  robot_name(pr2).

test(root_link_name_pr2) :-
  root_link_name(base_footprint).

test(joint_names_pr2) :-
  joint_names(Names),
  Names == [base_bellow_joint, base_footprint_joint, base_laser_joint,
            bl_caster_l_wheel_joint, bl_caster_r_wheel_joint, bl_caster_rotation_joint,
            br_caster_l_wheel_joint, br_caster_r_wheel_joint, br_caster_rotation_joint,
            double_stereo_frame_joint, fl_caster_l_wheel_joint, fl_caster_r_wheel_joint,
            fl_caster_rotation_joint, fr_caster_l_wheel_joint, fr_caster_r_wheel_joint,
            fr_caster_rotation_joint, head_mount_joint, head_mount_kinect_ir_joint,
            head_mount_kinect_ir_optical_frame_joint, head_mount_kinect_rgb_joint,
            head_mount_kinect_rgb_optical_frame_joint, head_mount_prosilica_joint,
            head_mount_prosilica_optical_frame_joint, head_pan_joint,
            head_plate_frame_joint, head_tilt_joint, high_def_frame_joint,
            high_def_optical_frame_joint, imu_joint, l_elbow_flex_joint,
            l_forearm_cam_frame_joint, l_forearm_cam_optical_frame_joint,
            l_forearm_joint, l_forearm_roll_joint, l_gripper_joint,
            l_gripper_l_finger_joint, l_gripper_l_finger_tip_joint,
            l_gripper_led_joint, l_gripper_motor_accelerometer_joint,
            l_gripper_motor_screw_joint, l_gripper_motor_slider_joint,
            l_gripper_palm_joint, l_gripper_r_finger_joint,
            l_gripper_r_finger_tip_joint, l_gripper_tool_joint, l_shoulder_lift_joint,
            l_shoulder_pan_joint, l_torso_lift_side_plate_joint, l_upper_arm_joint,
            l_upper_arm_roll_joint, l_wrist_flex_joint, l_wrist_roll_joint,
            laser_tilt_joint, laser_tilt_mount_joint, narrow_stereo_frame_joint,
            narrow_stereo_l_stereo_camera_frame_joint, narrow_stereo_l_stereo_camera_optical_frame_joint,
            narrow_stereo_optical_frame_joint, narrow_stereo_r_stereo_camera_frame_joint,
            narrow_stereo_r_stereo_camera_optical_frame_joint,
            projector_wg6802418_child_frame_joint, projector_wg6802418_frame_joint,
            r_elbow_flex_joint, r_forearm_cam_frame_joint, r_forearm_cam_optical_frame_joint,
            r_forearm_joint, r_forearm_roll_joint, r_gripper_joint, r_gripper_l_finger_joint,
            r_gripper_l_finger_tip_joint, r_gripper_led_joint, r_gripper_motor_accelerometer_joint,
            r_gripper_motor_screw_joint, r_gripper_motor_slider_joint, r_gripper_palm_joint,
            r_gripper_r_finger_joint, r_gripper_r_finger_tip_joint, r_gripper_tool_joint,
            r_shoulder_lift_joint, r_shoulder_pan_joint, r_torso_lift_side_plate_joint,
            r_upper_arm_joint, r_upper_arm_roll_joint, r_wrist_flex_joint,
            r_wrist_roll_joint, sensor_mount_frame_joint, torso_lift_joint,
            torso_lift_motor_screw_joint, wide_stereo_frame_joint,
            wide_stereo_l_stereo_camera_frame_joint, wide_stereo_l_stereo_camera_optical_frame_joint,
            wide_stereo_optical_frame_joint, wide_stereo_r_stereo_camera_frame_joint,
            wide_stereo_r_stereo_camera_optical_frame_joint].

test(link_names_pr2) :-
  link_names(Names),
  Names ==
  [base_bellow_link, base_footprint, base_laser_link, base_link, bl_caster_l_wheel_link,
   bl_caster_r_wheel_link, bl_caster_rotation_link, br_caster_l_wheel_link,
   br_caster_r_wheel_link, br_caster_rotation_link, double_stereo_link, fl_caster_l_wheel_link,
   fl_caster_r_wheel_link, fl_caster_rotation_link, fr_caster_l_wheel_link, fr_caster_r_wheel_link,
   fr_caster_rotation_link, head_mount_kinect_ir_link, head_mount_kinect_ir_optical_frame,
   head_mount_kinect_rgb_link, head_mount_kinect_rgb_optical_frame, head_mount_link,
   head_mount_prosilica_link, head_mount_prosilica_optical_frame, head_pan_link,
   head_plate_frame, head_tilt_link, high_def_frame, high_def_optical_frame,
   imu_link, l_elbow_flex_link, l_forearm_cam_frame, l_forearm_cam_optical_frame,
   l_forearm_link, l_forearm_roll_link, l_gripper_l_finger_link, l_gripper_l_finger_tip_frame,
   l_gripper_l_finger_tip_link, l_gripper_led_frame, l_gripper_motor_accelerometer_link,
   l_gripper_motor_screw_link, l_gripper_motor_slider_link, l_gripper_palm_link,
   l_gripper_r_finger_link, l_gripper_r_finger_tip_link, l_gripper_tool_frame,
   l_shoulder_lift_link, l_shoulder_pan_link, l_torso_lift_side_plate_link,
   l_upper_arm_link, l_upper_arm_roll_link, l_wrist_flex_link, l_wrist_roll_link,
   laser_tilt_link, laser_tilt_mount_link, narrow_stereo_l_stereo_camera_frame,
   narrow_stereo_l_stereo_camera_optical_frame, narrow_stereo_link, narrow_stereo_optical_frame,
   narrow_stereo_r_stereo_camera_frame, narrow_stereo_r_stereo_camera_optical_frame,
   projector_wg6802418_child_frame, projector_wg6802418_frame, r_elbow_flex_link,
   r_forearm_cam_frame, r_forearm_cam_optical_frame, r_forearm_link, r_forearm_roll_link,
   r_gripper_l_finger_link, r_gripper_l_finger_tip_frame, r_gripper_l_finger_tip_link,
   r_gripper_led_frame, r_gripper_motor_accelerometer_link, r_gripper_motor_screw_link,
   r_gripper_motor_slider_link, r_gripper_palm_link, r_gripper_r_finger_link,
   r_gripper_r_finger_tip_link, r_gripper_tool_frame, r_shoulder_lift_link, r_shoulder_pan_link,
   r_torso_lift_side_plate_link, r_upper_arm_link, r_upper_arm_roll_link, r_wrist_flex_link,
   r_wrist_roll_link, sensor_mount_link, torso_lift_link, torso_lift_motor_screw_link,
   wide_stereo_l_stereo_camera_frame, wide_stereo_l_stereo_camera_optical_frame, wide_stereo_link,
   wide_stereo_optical_frame, wide_stereo_r_stereo_camera_frame, wide_stereo_r_stereo_camera_optical_frame].

test(joint_child_parent_torso_lift_joint) :-
  joint_child_parent(torso_lift_joint, torso_lift_link, base_link).

test(joint_child_parent_l_shoulder_pan_joint) :-
  joint_child_parent(l_shoulder_pan_joint, l_shoulder_pan_link, torso_lift_link).

test(joint_child_parent_nonexisting_joint, fail) :-
  joint_child_parent(foo, _,_ ).

test(joint_child_parent_r_elbow_flex_joint) :-
  joint_child_parent(r_elbow_flex_joint, r_elbow_flex_link, r_upper_arm_roll_link).

test(joint_type_pr2_torso_lift_joint) :-
  joint_type(torso_lift_joint, prismatic).

test(joint_type_pr2_l_wrist_roll_joint) :-
  joint_type(l_wrist_roll_joint, continuous).

test(joint_type_pr2_r_shoulder_lift_joint) :-
  joint_type(r_shoulder_lift_joint, revolute).

test(joint_type_pr2_head_plate_frame_joint) :-
  joint_type(head_plate_frame_joint, fixed).

test(joint_axis_pr2_l_shoulder_pan_joint) :-
  joint_axis(l_shoulder_pan_joint, [0,0,1]).

test(joint_axis_pr2_fixed_joint, fail) :-
  joint_axis(head_plate_frame_joint, _).

test(joint_origin_pr2_r_gripper_led_joint) :-
  joint_origin(r_gripper_led_joint, pose([0.0513, 0.0, 0.0244], [0, 0, 0, 1])).

test(joint_origin_pr2_l_foreaem_cam_optical_frame_joint) :-
  joint_origin(l_forearm_cam_optical_frame_joint,
               pose([0,0,0],[-0.5, 0.5, -0.5, 0.5])).

test(joint_pos_limits_l_elbow_flex_joint) :-
  joint_pos_limits(l_elbow_flex_joint, -2.3213, 0.0).

test(joint_pos_limits_l_wrist_roll_joint, fail) :-
  joint_pos_limits(l_wrist_roll_joint, _, _).

test(joint_pos_limits_torso_lift_joint) :-
  joint_pos_limits(torso_lift_joint, 0.33, 0.013).

test(joint_pos_limits_r_forearm_roll_joint, fail) :-
  joint_pos_limits(r_forearm_roll_joint, _, _).

test(joint_kin_limits_r_gripper_joint) :-
  joint_kin_limits(r_gripper_joint, 0.2, 1000.0).

test(joint_kin_limits_l_gripper_motor_screw_joint, fail) :-
  %% NOTE: This is a case of an incorrect urdf. l_gripper_motor_screw_joint
  %% is modeled as a continuous joint, but has not limits specified. This
  %% test makes sure that we fail gracely. There used to be a null pointer exception..
  joint_kin_limits(l_gripper_motor_screw_joint, _, _).

test(joint_kin_limits_pr2_head_pan_joint) :-
  joint_kin_limits(head_pan_joint, 6.0, 2.645).

test(joint_calibration_rising_pr2_fl_caster_rotation_joint) :-
  joint_calibration_rising(fl_caster_rotation_joint, -0.785398163397).

test(joint_calibration_rising_pr2_torso_lift_joint, fail) :-
  joint_calibration_rising(torso_lift_joint, _).

test(joint_calibration_falling_pr2_fl_caster_rotation_joint, fail) :-
  joint_calibration_falling(fl_caster_rotation_joint, _).

test(joint_calibration_falling_pr2_torso_lift_joint) :-
  joint_calibration_falling(torso_lift_joint, 0.00475).

test(joint_dynamics_l_torso_lift_side_plate_joint, fail) :-
  joint_dynamics(l_torso_lift_side_plate_joint, _, _).

test(joint_dynamics_head_pan_joint) :-
  joint_dynamics(head_pan_joint, 0.5, 0.0).

test(joint_mimic_torso_lift_joint, fail) :-
  joint_mimic(torso_lift_joint, _, _, _).

test(joint_mimic_r_gripper_r_finger_joint) :-
  joint_mimic(r_gripper_r_finger_joint, r_gripper_l_finger_joint, 1.0, 0.0).

test(joint_safety_l_upper_arm_joint, fail) :-
  joint_safety(l_upper_arm_joint, _, _, _, _).

test(joint_safety_l_elbow_flex_joint) :-
  joint_safety(l_elbow_flex_joint, -2.1213, -0.15, 100.0, 3.0).

test(link_inertial_pr2_l_gripper_led_frame, fail) :-
  link_inertial(l_gripper_led_frame, _, _, _).

test(link_inertial_pr2_l_elbow_flex_link) :-
  link_inertial(l_elbow_flex_link, pose([0.01014, 0.00032, -0.01211], [0.0, 0.0, 0.0, 1.0]),
                1.90327, [0.00346541989, 0.00004066825, 0.00043171614, 0.00441606455, 0.00003968914, 0.00359156824]).

test(link_num_visuals_pr2_r_gripper_led_frame) :-
  link_num_visuals(r_gripper_led_frame, N),
  N=0.

test(link_num_visuals_pr2_r_gripper_motor_accelerometer_link) :-
  link_num_visuals(r_gripper_motor_accelerometer_link, N),
  N=1.

test(link_visual_shape_r_gripper_motor_accelerometer_link) :-
  link_visual_shape(r_gripper_motor_accelerometer_link, 0, Name, Origin, Geometry),
  Name = '',
  Origin = pose([0.0,0.0,0.0],[0.0, 0.0, 0.0, 1.0]),
  Geometry = box(0.001, 0.001, 0.001).

test(link_visual_shape_r_gripper_motor_slider_link) :-
  link_visual_shape(r_gripper_motor_slider_link, 0, Name, Origin, Geometry),
  Name = '',
  Origin = pose([0.0,0.0,0.0],[0.7071080798594737, 0.0, 0.0, 0.7071054825112364]),
  Geometry = cylinder(0.025, 0.002).

test(link_visual_shape_head_mount_kinect_ir_link) :-
  link_visual_shape(head_mount_kinect_ir_link, 0, Name, Origin, Geometry),
  Geometry = sphere(0.0005),
  Name = '',
  Origin = pose([0.0,0.0,0.0],[0.0, 0.0, 0.0, 1.0]).

test(link_visual_shape_head_mount_link) :-
  link_visual_shape(head_mount_link, 0, _, _, Geometry),
  Geometry = mesh('package://pr2_description/meshes/sensors/kinect_prosilica_v0/115x100_swept_back--coarse.STL', [0.001, 0.001, 0.001]).

test(link_visual_shape_negative_out_of_bounds, fail) :-
  link_visual_shape(head_mount_kinect_ir_link, -1, _, _, _).

test(link_visual_shape_positive_out_of_bounds, fail) :-
  link_visual_shape(head_mount_kinect_ir_link, 1, _, _, _).

test(link_visual_shape_r_gripper_led_frame, fail) :-
  link_visual_shape(r_gripper_led_frame, 0, _, _, _).

test(link_visual_shape_laser_tilt_mount_link) :-
  link_visual_shape(laser_tilt_mount_link, 0, _, _, Geometry),
  Geometry = mesh('package://pr2_description/meshes/tilting_laser_v0/tilting_hokuyo.dae', [1.0, 1.0, 1.0]).

test(link_visual_material_laser_tilt_mount_link) :-
  link_visual_material(laser_tilt_mount_link, 0, Name, Color, Filename),
  Name = 'Red',
  Color = rgba(0.800000011920929, 0.0, 0.0, 1.0),
  Filename = ''.

test(link_visual_material_head_mount_link) :-
  link_visual_material(head_mount_link, 0, Name, Color, Filename),
  Name = gray,
  Color = rgba(0.5, 0.5, 0.5, 1.0),
  Filename = ''.

test(link_visual_material_fl_caster_rotation_link) :-
  link_visual_material(fl_caster_rotation_link, 0, Name, Color, Filename),
  Name = 'Caster',
  Color = rgba(0.0, 0.0, 0.0, 1.0),
  Filename = 'package://pr2_description/materials/textures/pr2_caster_texture.png'.

test(link_num_collisions_pr2_base_link) :-
  link_num_collisions(base_link, N),
  N=1.

test(link_num_collisions_pr2_base_laser_link) :-
  link_num_collisions(base_laser_link, N),
  N=0.

test(link_num_collision_foo_link, fail) :-
  link_num_collisions(foo, _).

test(link_collision_base_laser_link, fail) :-
  link_collision(base_laser_link, 0, _, _, _).

test(link_collision_torso_lift_link) :-
  link_collision(torso_lift_link, 0, Name, pose([Zero, Zero, Zero], [Zero, Zero, Zero, One]),
      mesh(Filename, [One, One, One])),
  Zero = 0.0,
  One = 1.0,
  Filename = 'package://pr2_description/meshes/torso_v0/torso_lift_L.stl',
  Name = torso_lift_collision.

test(link_collision_r_gripper_r_finger_link) :-
  link_collision(r_gripper_r_finger_link, 0, Name, pose([Zero, Zero, Zero], [One, Zero, Zero, Zero]),
      mesh(Filename, [One, One, One])),
  Zero = 0.0,
  One = 1.0,
  Filename = 'package://pr2_description/meshes/gripper_v0/l_finger.stl',
  Name = ''.

test(link_collision_head_mount_prosilica_link) :-
  link_collision(head_mount_prosilica_link, 0, Name, pose([Zero, Zero, Zero], [Zero, Zero, Zero, One]),
      sphere([Radius])),
  Radius = 0.0005,
  Zero = 0.0,
  One = 1.0,
  Name = ''.

test(link_collision_fr_caster_r_wheel_link) :-
  link_collision(fr_caster_r_wheel_link, 0, Name, pose([Zero, Zero, Zero], [Zero, Zero, Zero, One]),
      cylinder([Length, Radius])),
  Zero = 0.0,
  One = 1.0,
  Radius = 0.074792,
  Length = 0.034,
  Name = ''.

test(link_collision_torso_lift_motor_screw_link) :-
  link_collision(torso_lift_motor_screw_link, 0, Name, pose([Zero, Zero, Zero], [Zero, Zero, Zero, One]),
      box([X,Y,Z])),
  X = 0.5,
  Y = 0.7,
  Z = 0.01,
  Zero = 0.0,
  One = 1.0,
  Name = ''.

test(no_owl_robot_in_db, fail) :-
  owl_individual_of(_, urdf:'Robot').

test(no_owl_links_in_db, fail) :-
  owl_individual_of(_, urdf:'Link').

test(no_owl_joints_in_db, fail) :-
  owl_individual_of(_, urdf:'Link').

test(assert_pr2_robot) :-
  assert_robot(_).

test(one_owl_robot_in_db) :-
  findall(Robot, owl_individual_of(Robot, urdf:'Robot'), Robots),
  length(Robots, 1).

test(robot_name_is_pr2) :-
  owl_individual_of(Robot, urdf:'Robot'),
  owl_has(Robot, urdf:'name', literal(type(xsd:string, pr2))), !.
  
test(all_pr2_links_can_be_found_through_robot) :-
  owl_individual_of(Robot, urdf:'Robot'),
  owl_has(Robot, urdf:'name', literal(type(xsd:string, pr2))),!,
  findall(Link, (owl_has(Robot, urdf:'hasLink', Link)), Links),
  length(Links, 95).

test(all_pr2_links_are_of_type_link) :-
  owl_individual_of(Robot, urdf:'Robot'),
  owl_has(Robot, urdf:'name', literal(type(xsd:string, pr2))),!,
  findall(Link, (owl_has(Robot, urdf:'hasLink', Link)), Links),
  forall(member(Link, Links), owl_individual_of(Link, urdf:'Link')).

test(all_pr2_joints_can_be_found_through_robot) :-
  owl_individual_of(Robot, urdf:'Robot'),
  owl_has(Robot, urdf:'name', literal(type(xsd:string, pr2))),!,
  findall(Joint, (owl_has(Robot, urdf:'hasJoint', Joint)), Joints),
  length(Joints, 94).

test(all_pr2_joints_are_of_type_joint) :-
  owl_individual_of(Robot, urdf:'Robot'),
  owl_has(Robot, urdf:'name', literal(type(xsd:string, pr2))),!,
  findall(Joint, (owl_has(Robot, urdf:'hasJoint', Joint)), Joints),
  forall(member(Joint, Joints), owl_individual_of(Joint, urdf:'Joint')).

test(all_pr2_joints_can_be_found_by_name) :-
  joint_names(JointNames),
  forall(member(JointName, JointNames), (
    owl_individual_of(Joint, urdf:'Joint'),
    owl_has(Joint, urdf:'name', literal(type(xsd:string, JointName))))).

test(all_pr2_links_can_be_found_by_name) :-
  link_names(LinkNames),
  forall(member(LinkName, LinkNames), (
    owl_individual_of(Link, urdf:'Link'),
    owl_has(Link, urdf:'name', literal(type(xsd:string, LinkName))))).

test(urdf_owl_joint_type_prismatic) :-
  urdf_owl_joint_type(prismatic, 'http://knowrob.org/kb/urdf.owl#PrismaticJoint').

test(urdf_owl_joint_type_fixed) :-
  urdf_owl_joint_type(fixed, 'http://knowrob.org/kb/urdf.owl#FixedJoint').

test(urdf_owl_joint_type_revolute) :-
  urdf_owl_joint_type(revolute, 'http://knowrob.org/kb/urdf.owl#RevoluteJoint').

test(urdf_owl_joint_type_continuous) :-
  urdf_owl_joint_type(continuous, 'http://knowrob.org/kb/urdf.owl#ContinuousJoint').
 
test(urdf_owl_joint_type_floating) :-
  urdf_owl_joint_type(floating, 'http://knowrob.org/kb/urdf.owl#FloatingJoint').

test(urdf_owl_joint_type_planar) :-
  urdf_owl_joint_type(planar, 'http://knowrob.org/kb/urdf.owl#PlanarJoint').

test(pr2_torso_lift_joint_has_correct_owl_type) :-
  owl_individual_of(Joint, urdf:'PrismaticJoint'),
  owl_has(Joint, urdf:'name', literal(type(xsd:string, torso_lift_joint))),!.

test(pr2_l_shoulder_pan_joint_has_correct_owl_type) :-
  owl_individual_of(Joint, urdf:'RevoluteJoint'),
  owl_has(Joint, urdf:'name', literal(type(xsd:string, l_shoulder_pan_joint))),!.

test(pr2_l_wrist_roll_joint_has_correct_owl_type) :-
  owl_individual_of(Joint, urdf:'ContinuousJoint'),
  owl_has(Joint, urdf:'name', literal(type(xsd:string, l_wrist_roll_joint))),!.

test(pr2_head_plate_frame_joint_has_correct_owl_type) :-
  owl_individual_of(Joint, urdf:'FixedJoint'),
  owl_has(Joint, urdf:'name', literal(type(xsd:string, head_plate_frame_joint))),!.

test(joint_name_torso_lift_joint) :-
  joint_name(_, torso_lift_joint),!.

test(joint_name_foo, fail) :-
  joint_name(_, foo).

test(link_name_l_gripper_tool_frame) :-
  link_name(_, l_gripper_tool_frame),!.

test(link_name_bar, fail) :-
  link_name(_, bar).

test(owl_child_link_pr2_torso_lift_joint) :-
  joint_name(Joint, torso_lift_joint),
  owl_has(Joint, urdf:'hasChildLink', ChildLink),
  link_name(ChildLink, torso_lift_link), !.

test(owl_child_link_pr2_l_shoulder_pan_joint) :-
  joint_name(Joint, l_shoulder_pan_joint),
  owl_has(Joint, urdf:'hasChildLink', ChildLink),
  link_name(ChildLink, l_shoulder_pan_link), !.

test(owl_parent_link_pr2_r_elbow_flex_joint) :-
  joint_name(Joint, r_elbow_flex_joint),
  owl_has(Joint, urdf:'hasParentLink', Link),
  link_name(Link, r_upper_arm_link),!.

test(owl_parent_link_pr2_head_tilt_joint) :-
  joint_name(Joint, head_tilt_joint),
  owl_has(Joint, urdf:'hasParentLink', Link),
  link_name(Link, head_pan_link),!.

test(owl_axis_fl_caster_rotation_joint) :-
  joint_name(Joint, fl_caster_rotation_joint),
  owl_has(Joint, urdf:'hasAxis', Axis),
  owl_has(Axis, urdf:'x', literal(type(xsd:double, 0.0))),
  owl_has(Axis, urdf:'y', literal(type(xsd:double, 0.0))),
  owl_has(Axis, urdf:'z', literal(type(xsd:double, 1.0))),!.

test(owl_axis_r_shoulder_lift_joint) :-
  joint_name(Joint, r_shoulder_lift_joint),
  owl_has(Joint, urdf:'hasAxis', Axis),
  owl_has(Axis, urdf:'x', literal(type(xsd:double, 0.0))),
  owl_has(Axis, urdf:'y', literal(type(xsd:double, 1.0))),
  owl_has(Axis, urdf:'z', literal(type(xsd:double, 0.0))),!.

test(owl_axis_r_forearm_roll_joint) :-
  joint_name(Joint, r_forearm_roll_joint),
  owl_has(Joint, urdf:'hasAxis', Axis),
  owl_has(Axis, urdf:'x', literal(type(xsd:double, 1.0))),
  owl_has(Axis, urdf:'y', literal(type(xsd:double, 0.0))),
  owl_has(Axis, urdf:'z', literal(type(xsd:double, 0.0))),!.

test(owl_axis_r_gripper_motor_slider_joint) :-
  joint_name(Joint, r_gripper_motor_slider_joint),
  owl_has(Joint, urdf:'hasAxis', Axis),
  owl_has(Axis, urdf:'x', literal(type(xsd:double, 1.0))),
  owl_has(Axis, urdf:'y', literal(type(xsd:double, 0.0))),
  owl_has(Axis, urdf:'z', literal(type(xsd:double, 0.0))),!.

test(owl_axis_laser_tilt_joint, fail) :-
  joint_name(Joint, laser_tilt_joint),
  owl_has(Joint, urdf:'hasAxis', _).

test(owl_pos_limits_l_elbow_flex_joint) :-
  joint_name(Joint, l_elbow_flex_joint),
  owl_has(Joint, urdf:'lowerPosLimit', literal(type(xsd:double, -2.3213))),
  owl_has(Joint, urdf:'upperPosLimit', literal(type(xsd:double, 0.0))),!.

test(owl_pos_limits_l_wrist_roll_joint, fail) :-
  joint_name(Joint, l_wrist_roll_joint),
  owl_has(Joint, urdf:'lowerPosLimit', _),
  owl_has(Joint, urdf:'upperPosLimit', _).

test(owl_pos_limits_torso_lift_joint) :-
  joint_name(Joint, torso_lift_joint),
  owl_has(Joint, urdf:'lowerPosLimit', literal(type(xsd:double, 0.0))),
  owl_has(Joint, urdf:'upperPosLimit', literal(type(xsd:double, 0.33))),!.

test(owl_kin_limits_r_shoulder_lift_joint) :-
  joint_name(Joint, r_shoulder_lift_joint),
  owl_has(Joint, urdf:'velocityLimit', literal(type(xsd:double, 2.082))),
  owl_has(Joint, urdf:'effortLimit', literal(type(xsd:double, 30.0))),!.

test(owl_kin_limits_r_upper_arm_joint, fail) :-
  joint_name(Joint, r_upper_arm_joint),
  owl_has(Joint, urdf:'velocityLimit', _),
  owl_has(Joint, urdf:'effortLimit', _).

test(owl_kin_limits_head_pan_joint) :-
  joint_name(Joint, head_pan_joint),
  owl_has(Joint, urdf:'velocityLimit', literal(type(xsd:double, 6.0))),
  owl_has(Joint, urdf:'effortLimit', literal(type(xsd:double, 2.645))),!.

test(owl_origin_base_bellow_joint) :-
  joint_name(Joint, base_bellow_joint),
  owl_has(Joint, urdf:'hasOrigin', Origin),
  owl_has(Origin, urdf:'hasPosition', Position),
  owl_has(Origin, urdf:'hasOrientation', Orientation),
  owl_has(Position, urdf:'x', literal(type(xsd:double, -0.29))),
  owl_has(Position, urdf:'y', literal(type(xsd:double, 0.0))),
  owl_has(Position, urdf:'z', literal(type(xsd:double, 0.8))),
  owl_has(Orientation, urdf:'x', literal(type(xsd:double, 0.0))),
  owl_has(Orientation, urdf:'y', literal(type(xsd:double, 0.0))),
  owl_has(Orientation, urdf:'z', literal(type(xsd:double, 0.0))),
  owl_has(Orientation, urdf:'w', literal(type(xsd:double, 1.0))),!.

test(owl_dynamics_fr_caster_r_wheel_joint) :-
  joint_name(Joint, fr_caster_r_wheel_joint),
  owl_has(Joint, urdf:'damping', literal(type(xsd:double, 1.0))),
  owl_has(Joint, urdf:'friction', literal(type(xsd:double, 0.0))),!.

test(owl_dynamics_torso_lift_joint) :-
  joint_name(Joint, torso_lift_joint),
  owl_has(Joint, urdf:'damping', literal(type(xsd:double, 20000.0))),
  owl_has(Joint, urdf:'friction', literal(type(xsd:double, 0.0))),!.

test(owl_dynamics_r_forearm_cam_optical_frame_joint, fail) :-
  joint_name(Joint, r_forearm_cam_optical_frame_joint),
  owl_has(Joint, urdf:'damping', _),
  owl_has(Joint, urdf:'friction', _).

test(owl_safety_r_gripper_joint) :-
  joint_name(Joint, r_gripper_joint),
  owl_has(Joint, urdf:'hasSafetyController', Safety),
  owl_has(Safety, urdf:'softLowerLimit', literal(type(xsd:double, -0.01))),
  owl_has(Safety, urdf:'softUpperLimit', literal(type(xsd:double, 0.088))),
  owl_has(Safety, urdf:'kPosition', literal(type(xsd:double, 20.0))),
  owl_has(Safety, urdf:'kVelocity', literal(type(xsd:double, 5000.0))),!.

test(owl_safety_l_upper_arm_joint, fail) :-
  joint_name(Joint, l_upper_arm_joint),
  owl_has(Joint, urdf:'hasSafetyController', _).

test(owl_calibration_rising_fl_caster_rotation_joint) :-
  joint_name(Joint, fl_caster_rotation_joint),
  owl_has(Joint, urdf:'risingCalibrationPos', literal(type(xsd:double, -0.785398163397))),!.

test(owl_calibration_rising_torso_lift_joint, fail) :-
  joint_name(Joint, torso_lift_joint),
  owl_has(Joint, urdf:'risingCalibrationPos', _).

test(owl_calibration_falling_torso_lift_joint) :-
  joint_name(Joint, torso_lift_joint),
  owl_has(Joint, urdf:'fallingCalibrationPos', literal(type(xsd:double, 0.00475))),!.

test(owl_calibration_falling_fl_caster_rotation_joint, fail) :-
  joint_name(Joint, fl_caster_rotation_joint),
  owl_has(Joint, urdf:'fallingCalibrationPos', _).

test(owl_calibration_rising_head_plate_frame_joint, fail) :-
  joint_name(Joint, head_plate_frame_joint),
  owl_has(Joint, urdf:'risingCalibrationPos', _).

test(owl_calibration_falling_head_plate_frame_joint, fail) :-
  joint_name(Joint, head_plate_frame_joint),
  owl_has(Joint, urdf:'fallingCalibrationPos', _).

test(owl_mimic_torso_lift_joint, fail) :-
  joint_name(Joint, torso_lift_joint),
  owl_has(Joint, urdf:'hasMimicProperties', _).

test(owl_mimic_r_gripper_r_finger_joint) :-
  joint_name(Joint, r_gripper_r_finger_joint),
  owl_has(Joint, urdf:'hasMimicProperties', MimicProps),
  owl_has(MimicProps, urdf:'hasMimickedJoint', MimickedJoint),
  joint_name(MimickedJoint, r_gripper_l_finger_joint),
  owl_has(MimicProps, urdf:'mimicFactor', literal(type(xsd:double, 1.0))),
  owl_has(MimicProps, urdf:'mimicOffset', literal(type(xsd:double, 0.0))),!.

test(owl_inertial_props_base_link) :-
  link_name(Link, base_link),
  owl_has(Link, urdf:'hasInertialProperties', InertialProps),
  owl_has(InertialProps, urdf:'mass', literal(type(xsd:double, 116.0))),
  owl_has(InertialProps, urdf:'hasOrigin', Origin),
  owl_has(InertialProps, urdf:'hasInertiaMatrix', InertiaMat),
  owl_has(Origin, urdf:'hasPosition', Position),
  owl_has(Origin, urdf:'hasOrientation', Orientation),
  owl_has(Position, urdf:'x', literal(type(xsd:double, -0.061))),
  owl_has(Position, urdf:'y', literal(type(xsd:double, 0.0))),
  owl_has(Position, urdf:'z', literal(type(xsd:double, 0.1465))),
  owl_has(Orientation, urdf:'x', literal(type(xsd:double, 0.0))),
  owl_has(Orientation, urdf:'y', literal(type(xsd:double, 0.0))),
  owl_has(Orientation, urdf:'z', literal(type(xsd:double, 0.0))),
  owl_has(Orientation, urdf:'w', literal(type(xsd:double, 1.0))),
  owl_has(InertiaMat, urdf:'ixx', literal(type(xsd:double, 5.652232699207))),
  owl_has(InertiaMat, urdf:'ixy', literal(type(xsd:double, -0.009719934438))),
  owl_has(InertiaMat, urdf:'ixz', literal(type(xsd:double, 1.293988226423))),
  owl_has(InertiaMat, urdf:'iyy', literal(type(xsd:double, 5.669473158652))),
  owl_has(InertiaMat, urdf:'iyz', literal(type(xsd:double, -0.007379583694))),
  owl_has(InertiaMat, urdf:'izz', literal(type(xsd:double, 3.683196351726))),!.

test(owl_visual_props_head_mount_link) :-
  link_name(Link, head_mount_link),

  % check that we have really only 1 visual element
  findall(Visual, owl_has(Link, urdf:'hasVisualProperties', Visual), Visuals),
  length(Visuals, 1),
  last(Visuals, Visual),
  owl_individual_of(Visual, urdf:'VisualProperties'),!,

  % verify origin of visual element
  owl_has(Visual, urdf:'hasOrigin', Origin),
  owl_has(Origin, urdf:'hasPosition', Position),
  owl_has(Origin, urdf:'hasOrientation', Orientation),
  owl_individual_of(Origin, urdf:'Transform'),!,
  owl_individual_of(Position, urdf:'Vector3d'),!,
  owl_individual_of(Orientation, urdf:'Quaternion'),!,
  owl_has(Position, urdf:'x', literal(type(xsd:double, 0.0))),
  owl_has(Position, urdf:'y', literal(type(xsd:double, 0.0))),
  owl_has(Position, urdf:'z', literal(type(xsd:double, 0.0))),
  owl_has(Orientation, urdf:'x', literal(type(xsd:double, 0.0))),
  owl_has(Orientation, urdf:'y', literal(type(xsd:double, 0.0))),
  owl_has(Orientation, urdf:'z', literal(type(xsd:double, 0.0))),
  owl_has(Orientation, urdf:'w', literal(type(xsd:double, 1.0))),

  % verify geometry of visual element
  owl_has(Visual, urdf:'hasGeometry', Geometry),
  owl_individual_of(Geometry, urdf:'Mesh'),!,
  owl_has(Geometry, urdf:'filename', literal(type(xsd:string, 'package://pr2_description/meshes/sensors/kinect_prosilica_v0/115x100_swept_back--coarse.STL'))),
  owl_has(Geometry, urdf:'hasScale', Scale),
  owl_individual_of(Scale, urdf:'Vector3d'),!,
  owl_has(Scale, urdf:'x', literal(type(xsd:double, 0.001))),
  owl_has(Scale, urdf:'y', literal(type(xsd:double, 0.001))),
  owl_has(Scale, urdf:'z', literal(type(xsd:double, 0.001))),

  % verify material of visual element
  owl_has(Visual, urdf:'hasMaterialProperties', Material),
  owl_individual_of(Material, urdf:'MaterialProperties'),!,
  owl_has(Material, urdf:'name', literal(type(xsd:string, 'gray'))), % TODO: figure out way this breaks
  owl_has(Material, urdf:'hasColor', Color),
  owl_has(Color, urdf:'r', literal(type(xsd:double, 0.5))),
  owl_has(Color, urdf:'g', literal(type(xsd:double, 0.5))),
  owl_has(Color, urdf:'b', literal(type(xsd:double, 0.5))),
  owl_has(Color, urdf:'a', literal(type(xsd:double, 1.0))),!.

test(test_owl_visual_props_fl_caster_rotation_link) :-
  link_name(Link, fl_caster_rotation_link),
  owl_has(Link, urdf:'hasVisualProperties', Visual),
  owl_has(Visual, urdf:'hasMaterialProperties', Material),
  owl_has(Material, urdf:'name', literal(type(xsd:string, 'Caster'))),
  owl_has(Material, urdf:'filename', literal(type(xsd:string, 'package://pr2_description/materials/textures/pr2_caster_texture.png'))),!.

test(owl_collision_props_base_footprint) :-
  link_name(Link, base_footprint),
  owl_has(Link, urdf:'hasCollisionProperties', Collision),
  owl_has(Collision, urdf:'hasGeometry', Geometry),
  owl_has(Collision, urdf:'hasOrigin', Origin),
  owl_has(Origin, urdf:'hasPosition', Position),
  owl_has(Origin, urdf:'hasOrientation', Orientation),
  owl_individual_of(Collision, urdf:'CollisionProperties'),
  owl_individual_of(Origin, urdf:'Transform'),
  owl_individual_of(Geometry, urdf:'Box'),
  owl_individual_of(Orientation, urdf:'Quaternion'),
  owl_individual_of(Position, urdf:'Vector3d'),
  owl_has(Geometry, urdf:'x', literal(type(xsd:double, 0.001))),
  owl_has(Geometry, urdf:'y', literal(type(xsd:double, 0.001))),
  owl_has(Geometry, urdf:'z', literal(type(xsd:double, 0.001))),
  owl_has(Position, urdf:'x', literal(type(xsd:double, 0.0))),
  owl_has(Position, urdf:'y', literal(type(xsd:double, 0.0))),
  owl_has(Position, urdf:'z', literal(type(xsd:double, 0.071))),!.

test(owl_collision_props_fl_caster_rotation_link) :-
  link_name(Link, fl_caster_rotation_link),!,
  owl_has(Link, urdf:'hasCollisionProperties', Collision),
  owl_has(Collision, urdf:'hasGeometry', Geometry),
  owl_individual_of(Collision, urdf:'CollisionProperties'),!,
  owl_individual_of(Geometry, urdf:'Mesh'),!,
  owl_has(Geometry, urdf:'filename', literal(type(xsd:string, 'package://pr2_description/meshes/base_v0/caster_L.stl'))),
  owl_has(Geometry, urdf:'hasScale', Scale),
  owl_individual_of(Scale, urdf:'Vector3d'),
  owl_has(Scale, urdf:'x', literal(type(xsd:double, 1.0))),
  owl_has(Scale, urdf:'y', literal(type(xsd:double, 1.0))),
  owl_has(Scale, urdf:'z', literal(type(xsd:double, 1.0))),!.

:- end_tests(urdf_parser).
