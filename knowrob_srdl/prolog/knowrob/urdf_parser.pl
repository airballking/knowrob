/*
  Copyright (C) 2018 Georg Bartels
  All rights reserved.

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

:- module(urdf_parser,
    [
      load_urdf_file/1,
      load_urdf_param/1,
      load_urdf_string/1,
      robot_name/1,
      root_link_name/1,
      link_names/1,
      link_parent_joint/2,
      link_child_joints/2,
      link_inertial_origin/2,
      link_inertial_mass/2,
      link_inertial_inertia/2,
      link_num_visuals/2,
      link_visual_type/3,
      link_visual_origin/3,
      link_visual_geometry/3,
      link_material_name/3,
      link_material_color/3,
      link_material_texture/3,
      link_num_collisions/2,
      link_collision_type/3,
      link_collision_name/3,
      link_collision_origin/3,
      link_collision_geometry/3,
      joint_names/1,
      joint_type/2,
      joint_child_link/2,
      joint_parent_link/2,
      joint_axis/2,
      joint_origin/2,
      joint_lower_pos_limit/2,
      joint_upper_pos_limit/2,
      joint_velocity_limit/2,
      joint_effort_limit/2,
      joint_calibration_rising/2,
      joint_calibration_falling/2,
      joint_dynamics_damping/2,
      joint_dynamics_friction/2,
      joint_mimic_joint_name/2,
      joint_mimic_multiplier/2,
      joint_mimic_offset/2,
      joint_safety_lower_limit/2,
      joint_safety_upper_limit/2,
      joint_safety_kp/2,
      joint_safety_kv/2,
      urdf_owl_joint_type/2,
      link_name/2,
      joint_name/2,
      joint_has_kin_limits/1,
      joint_has_dynamics/1,
      joint_has_safety_controller/1,
      joint_has_calibration/1,
      assert_urdf_file/2, 
      assert_robot/1, 
      assert_robot_individual/1,
      assert_robot_links/1,
      assert_robot_joints/1,
      assert_joint_properties/1,
      assert_child_link/1,
      assert_parent_link/1,
      assert_axis/1,
      assert_pos_limits/1,
      assert_kin_limits/1,
      assert_origin/1,
      assert_dynamics/1,
      assert_safety_controller/1,
      assert_calibration/1,
      assert_link_properties/1
    ]).

/** <module> Prolog-wrapping for the C++ URDF Parser.

@author Georg Bartels
@license BSD
*/

:- use_foreign_library('liburdf_parser.so').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/owl')).

:- rdf_db:rdf_register_ns(urdf, 'http://knowrob.org/kb/urdf.owl#', [keep(true)]).

:- rdf_meta
        urdf_owl_joint_type(r,r),
        assert_robot_links(r),
        assert_robot_joints(r).

%% ros_param_get_string(+Key,-Value) is semidet.
%
% Read parameter from ROS parameter server.

%% ros_info(+Msg) is det.
%
% Debug via ROS master.

%% load_urdf_file(+Filename) is semidet.
%
% Load URDF from disc using global filename.

%% load_urdf_param(+Param) is semidet.
%
% Load URDF from ROS parameter server using public parameter name.

%% load_urdf_string(+String) is semidet.
%
% Load URDF from given XML string.

%% robot_name(-Name) is semidet.
%
% Get the name of the currently loaded robot.

%% root_link_name(-Name) is semidet.
%
% Get the name of the root link of the robot.

%% link_names(-Names) is semidet.
%
% Get the list of all link names of this robot.

%% link_parent_joint(+LinkName, -JointName) is semidet.
%
% Get the name of the parent joint of a link.

%% link_child_joints(+LinkName, -JointNames) is semidet.
%
% Get the list of joint names of all child joints of a link.

%% link_inertial_origin(+LinkName, -Origin) is semidet.
%
% Get the inertial origin of a link as a pose w.r.t.
% the origin frame of a link.
%
% Poses are coded as a compound term: pose([X,Y,Z],[QX,QY,QZ,QW]),
% with the orientation represented as Quaternion.

%% link_inertial_mass(+LinkName, -Mass) is semidet.
%
% Get the inertial mass of a link in kg.

%% link_inertial_inertia(+LinkName, -InertiaMat) is semidet.
%
% Get the relevant parts of a links inertia matrix w.r.t.
% the origin frame of a link.
%
% Inertia matrices are coded as a list:
% [XX, XY, XZ, YY, YZ, ZZ].
% For an explanation, visit: http://wiki.ros.org/urdf/XML/link

%% link_num_visuals(+LinkName, -Num) is semidet.
%
% Get the number of visual elements of a link.

%% link_visual_type(+LinkName, +Index, -Type) is semidet.
%
% Get the type of a particular visual element of a link.
% Possible types: box, sphere, cylinder, mesh
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_visual_origin(+LinkName, +Index, -Origin) is semidet.
%
% Get the origin of a particular visual element of a link
% as a pose w.r.t. to the link frame.
%
% Poses are coded as a compound term: pose([X,Y,Z],[QX,QY,QZ,QW]),
% with the orientation represented as Quaternion.
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_visual_geometry(+LinkName, +Index, -Geometry) is semidet.
%
% Get the geometry of a particular visual element of a link.
%
% Depending on the type of visual element, the geometry is
% coded differently:
% - SPHERE: sphere([Radius])
% - CYLINDER: cylinder([Radius, Length])
% - BOX: box([X, Y, Z])
% - MESH: mesh(Filename, [ScaleX, ScaleY, ScaleZ])
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_material_name(+LinkName, +Index, -Name) is semidet.
%
% Get the name of a material of a particular visual element
% of a link.
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_material_color(+LinkName, +Index, -Color) is semidet.
%
% Get the color of a material of a particular visual element
% of a link.
%
% Colors are coded as compound terms: rgba([R, G, B, A]).
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_material_texture(+LinkName, +Index, -FileName) is semidet.
%
% Get the filename of a texture of a particular visual element
% of a link.
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_num_collisions(+LinkName, -Num) is semidet.
%
% Get the number of collision elements of a link.

%% link_collision_type(+LinkName, +Index, -Type) is semidet.
%
% Get the type of a particular collision element of a link.
% Possible types: box, sphere, cylinder, and mesh.
%
% Note: Links can have several collision elements, the first
% one has index 0.

%% link_collision_name(+LinkName, +Index, -Name) is semidet.
%
% Get the name of a particular collision element of a link.
%
% Note: Links can have several collision elements, the first
% one has index 0.

%% link_collision_origin(+LinkName, +Index, -Origin) is semidet.
%
% Get the origin of a particular collision element of a link,
% expressed as a pose w.r.t. to the link frame.
%
% Poses are coded as a compound term: pose([X,Y,Z],[QX,QY,QZ,QW]),
% with the orientation represented as Quaternion.
%
% Note: Links can have several collision elements, the first
% one has index 0.

%% link_collision_geometry(+LinkName, +Index, -Geometry) is semidet.
%
% Get the geometry of a particular collision element of a link.
%
% Depending on the type of collision element, the geometry is
% coded differently:
% - SPHERE: sphere([Radius])
% - CYLINDER: cylinder([Radius, Length])
% - BOX: box([X, Y, Z])
% - MESH: mesh(Filename, [ScaleX, ScaleY, ScaleZ])
%
% Note: Links can have several collision elements, the first
% one has index 0.

%% joint_names(-Names) is semidet.
%
% Get the list of joint names of the currently loaded robot.

%% joint_type(+JointName, Type) is semidet.
%
% Get the type of a joint.
% Possible types: revolute, prismatic, continuous, fixed,
% floating, planar, and unknown.

%% joint_child_link(+JointName, -LinkName) is semidet.
%
% Get the name of the link of a joint.

%% joint_parent_link(+JointName, -LinkName) is semidet.
%
% Get the name the parent link of a joint.

%% joint_axis(+JointName, -Axis) is semidet.
%
% Get the axis of a joint, expressed as a list [X, Y, Z].

%% joint_origin(+JointName, -Origin) is semidet.
%
% Get the origin of a joint, expressed as a pose
% w.r.t. the link frame of its parent link.
%
% Poses are coded as a compound term: pose([X,Y,Z],[QX,QY,QZ,QW]),
% with the orientation represented as Quaternion.

%% joint_lower_pos_limit(+JointName, -Lower) is semidet.
%
% Read the lower position limit of a joint.
%
% Note: Only valid for prismatic and revolute joints.


%% joint_upper_pos_limit(+JointName, -Upper) is semidet.
%
% Read the upper position limit of a joint.
%
% Note: Only valid for prismatic and revolute joints.

%% joint_velocity_limit(+JointName, -VelLimit) is semidet.
%
% Read the velocity limit of a joint.
%
% Note: Only valid for prismatic, revolute, and continuous joints.

%% joint_effort_limit(+JointName, -EffLimit) is semidet.
%
% Read the effort limit of a joint.
%
% Note: Only valid for prismatic, revolute, and continuous joints.

%% joint_calibration_rising(+JointName, -Rising) is semidet.
%
% Read the rising reference position of a joint.

%% joint_calibration_falling(+JointName, -Falling) is semidet.
%
% Read the falling reference position of a joint.

%% joint_dynamics_damping(+JointName, -Damping) is semidet.
%
% Read the damping value of a joint.

%% joint_dynamics_friction(+JointName, -Friction) is semidet.
%
% Get the static friction value of a joint.

%% joint_mimic_joint_name(+JointName, -MimickedJointName) ist semidet.
%
% Get the name of a joint that a mimic joint mimicks.

%% joint_mimic_multiplier(+JointName, -Multiplier) is semidet.
%
% Get the multiplication factor of a mimic joint.

%% joint_mimic_offset(+JointName, -Offset) is semidet.
%
% Get the offset value of a mimic joint.

%% joint_safety_lower_limit(+JointName, -Lower) is semidet.
%
% Get the lower position limit of the safety controller
% of a joint.

%% joint_safety_upper_limit(+JointName, -Upper) is semidet.
%
% Get the upper position limit of the safety controller
% of a joint.

%% joint_safety_kp(+JointName, -Kp) is semidet.
%
% Get the relation between position and velocity
% limits of the safety controller of a joint. For
% more details, visit:
% http://wiki.ros.org/pr2_controller_manager/safety_limits

%% joint_safety_kv(+JointName, -Kv) is semidet.
%
% Get the relation between position and velocity
% limits of the safety controller of a joint. For
% more details, visit:
% http://wiki.ros.org/pr2_controller_manager/safety_limits
%

urdf_owl_joint_type(prismatic, urdf:'PrismaticJoint').

urdf_owl_joint_type(fixed, urdf:'FixedJoint').

urdf_owl_joint_type(revolute, urdf:'RevoluteJoint').

urdf_owl_joint_type(continuous, urdf:'ContinuousJoint').

urdf_owl_joint_type(planar, urdf:'PlanarJoint').

urdf_owl_joint_type(floating, urdf:'FloatingJoint').

joint_name(Joint, Name) :-
  owl_individual_of(Joint, urdf:'Joint'),
  owl_has(Joint, urdf:'name', literal(type(xsd:string, Name))).

link_name(Link, Name) :-
  owl_individual_of(Link, urdf:'Link'),
  owl_has(Link, urdf:'name', literal(type(xsd:string, Name))).

assert_urdf_file(FileName, Robot) :-
  load_urdf_file(FileName),
  assert_robot(Robot).

assert_robot(Robot) :-
  assert_robot_individual(Robot),
  assert_robot_links(Robot),
  assert_robot_joints(Robot),
  assert_joint_properties(Robot),
  assert_link_properties(Robot).

assert_robot_individual(Robot) :-
  owl_instance_from_class(urdf:'Robot', Robot),
  robot_name(RobotName),
  rdf_assert(Robot, urdf:'name', literal(type(xsd:string, RobotName))).

assert_robot_links(Robot) :-
  link_names(LinkNames),
  forall(member(LinkName, LinkNames), (   
    owl_instance_from_class(urdf:'Link', Link),
    rdf_assert(Robot, urdf:'hasLink', Link),
    rdf_assert(Link, urdf:'name', literal(type(xsd:string, LinkName))))).

assert_robot_joints(Robot) :-
  joint_names(JointNames),
  forall(member(JointName, JointNames), (   
    joint_type(JointName, UrdfJointType),
    urdf_owl_joint_type(UrdfJointType, OwlJointType),
    owl_instance_from_class(OwlJointType, Joint),
    rdf_assert(Robot, urdf:'hasJoint', Joint),
    rdf_assert(Joint, urdf:'name', literal(type(xsd:string, JointName))))).

joint_has_kin_limits(Joint) :-
  owl_individual_of(Joint, urdf:'JointWithKinematicLimits'),
  joint_name(Joint, JointName),
  joint_velocity_limit(JointName, _),
  joint_effort_limit(JointName, _).

joint_has_dynamics(Joint) :-
  joint_name(Joint, JointName),
  joint_dynamics_damping(JointName, _),
  joint_dynamics_friction(JointName, _).

joint_has_safety_controller(Joint) :-
  joint_name(Joint, JointName),!,
  joint_safety_kp(JointName, _).

joint_has_calibration(Joint) :-
  joint_name(Joint, JointName),
  (joint_calibration_rising(JointName, _); joint_calibration_falling(JointName, _)).

joint_has_mimic_props(Joint) :-
  joint_name(Joint, JointName),
  joint_mimic_joint_name(JointName, _).

assert_joint_properties(Robot) :-
  owl_individual_of(Robot, urdf:'Robot'),!,
  forall(owl_has(Robot, urdf:'hasJoint', Joint), 
    assert_joint_properties(Joint)).

assert_joint_properties(Joint) :-
  owl_individual_of(Joint, urdf:'Joint'),!,
  assert_parent_link(Joint),
  assert_child_link(Joint),
  assert_origin(Joint),
  ((owl_individual_of(Joint, urdf:'JointWithAxis')) -> (assert_axis(Joint)) ; (true)),
  ((joint_has_kin_limits(Joint)) -> (assert_kin_limits(Joint)) ; (true)),
  ((owl_individual_of(Joint, urdf:'JointWithPositionLimits')) -> (assert_pos_limits(Joint)) ; (true)),
  ((joint_has_dynamics(Joint)) -> (assert_dynamics(Joint)) ; (true)),
  ((joint_has_safety_controller(Joint)) -> (assert_safety_controller(Joint)) ; (true)),
  ((joint_has_calibration(Joint)) -> (assert_calibration(Joint)) ; (true)),
  ((joint_has_mimic_props(Joint)) -> (assert_mimic_props(Joint)) ; (true)).

assert_parent_link(Joint) :-
  joint_name(Joint, JointName),!, 
  joint_parent_link(JointName, ParentLinkName),
  link_name(ParentLink, ParentLinkName),!,
  rdf_assert(Joint, urdf:'hasParentLink', ParentLink).

assert_child_link(Joint) :-
  joint_name(Joint, JointName),!,
  joint_child_link(JointName, ChildLinkName),
  link_name(ChildLink, ChildLinkName),!,
  rdf_assert(Joint, urdf:'hasChildLink', ChildLink).

assert_axis(Joint) :-
  joint_name(Joint, JointName),!,
  joint_axis(JointName, [X,Y,Z]),
  owl_individual_of(Joint, urdf:'JointWithAxis'),!,
  owl_instance_from_class(urdf:'Vector3d', Axis),
  rdf_assert(Joint, urdf:'hasAxis', Axis),
  rdf_assert(Axis, urdf:'x', literal(type(xsd:double, X))),
  rdf_assert(Axis, urdf:'y', literal(type(xsd:double, Y))),
  rdf_assert(Axis, urdf:'z', literal(type(xsd:double, Z))).

assert_pos_limits(Joint) :-
  joint_name(Joint, JointName),!,
  joint_lower_pos_limit(JointName, Lower),
  joint_upper_pos_limit(JointName, Upper),
  owl_individual_of(Joint, urdf:'JointWithPositionLimits'),!,
  rdf_assert(Joint, urdf:'lowerPosLimit', literal(type(xsd:double, Lower))),
  rdf_assert(Joint, urdf:'upperPosLimit', literal(type(xsd:double, Upper))).

assert_kin_limits(Joint) :-
  joint_name(Joint, JointName),!,
  owl_individual_of(Joint, urdf:'JointWithKinematicLimits'),!,
  joint_velocity_limit(JointName, VelocityLimit),
  joint_effort_limit(JointName, EffortLimit),
  rdf_assert(Joint, urdf:'velocityLimit', literal(type(xsd:double, VelocityLimit))),
  rdf_assert(Joint, urdf:'effortLimit', literal(type(xsd:double, EffortLimit))).

assert_origin(Joint) :-
  joint_name(Joint, JointName),!,
  joint_origin(JointName, pose([X,Y,Z],[QX,QY,QZ,QW])),
  owl_instance_from_class(urdf:'Vector3d', Position),
  rdf_assert(Position, urdf:'x', literal(type(xsd:double, X))),
  rdf_assert(Position, urdf:'y', literal(type(xsd:double, Y))),
  rdf_assert(Position, urdf:'z', literal(type(xsd:double, Z))),
  owl_instance_from_class(urdf:'Quaternion', Orientation),
  rdf_assert(Orientation, urdf:'x', literal(type(xsd:double, QX))),
  rdf_assert(Orientation, urdf:'y', literal(type(xsd:double, QY))),
  rdf_assert(Orientation, urdf:'z', literal(type(xsd:double, QZ))),
  rdf_assert(Orientation, urdf:'w', literal(type(xsd:double, QW))),
  owl_instance_from_class(urdf:'Transform', Origin),
  rdf_assert(Origin, urdf:'hasPosition', Position),
  rdf_assert(Origin, urdf:'hasOrientation', Orientation),
  rdf_assert(Joint, urdf:'hasOrigin', Origin).

assert_dynamics(Joint) :-
  joint_name(Joint, JointName),!,
  joint_dynamics_friction(JointName, Friction),
  joint_dynamics_damping(JointName, Damping),
  rdf_assert(Joint, urdf:'damping', literal(type(xsd:double, Damping))),
  rdf_assert(Joint, urdf:'friction', literal(type(xsd:double, Friction))).

assert_safety_controller(Joint) :-
  joint_name(Joint, JointName),!,
  joint_safety_lower_limit(JointName, Lower),
  joint_safety_upper_limit(JointName, Upper),
  joint_safety_kp(JointName, Kp),
  joint_safety_kv(JointName, Kv),
  owl_instance_from_class(urdf:'SafetyController', Safety),
  rdf_assert(Joint, urdf:'hasSafetyController', Safety),
  rdf_assert(Safety, urdf:'softLowerLimit', literal(type(xsd:double, Lower))),
  rdf_assert(Safety, urdf:'softUpperLimit', literal(type(xsd:double, Upper))),
  rdf_assert(Safety, urdf:'kPosition', literal(type(xsd:double, Kp))),
  rdf_assert(Safety, urdf:'kVelocity', literal(type(xsd:double, Kv))).

assert_calibration(Joint) :-
  joint_name(Joint, JointName),
  ((joint_calibration_rising(JointName, Rising)) -> (rdf_assert(Joint, urdf:'risingCalibrationPos', literal(type(xsd:double, Rising)))) ; (true)),
  ((joint_calibration_falling(JointName, Falling)) -> (rdf_assert(Joint, urdf:'fallingCalibrationPos', literal(type(xsd:double, Falling)))) ; (true)).

assert_mimic_props(Joint) :-
  joint_name(Joint, JointName),
  joint_mimic_joint_name(JointName, MimickedJointName),
  joint_mimic_multiplier(JointName, MimicFactor),
  joint_mimic_offset(JointName, MimicOffset),
  joint_name(MimickedJoint, MimickedJointName),
  owl_instance_from_class(urdf:'MimicProperties', MimicProps),
  rdf_assert(Joint, urdf:'hasMimicProperties', MimicProps),
  rdf_assert(MimicProps, urdf:'hasMimickedJoint', MimickedJoint),
  rdf_assert(MimicProps, urdf:'mimicFactor', literal(type(xsd:double, MimicFactor))),
  rdf_assert(MimicProps, urdf:'mimicOffset', literal(type(xsd:double, MimicOffset))).

assert_link_properties(Robot) :-
  owl_individual_of(Robot, urdf:'Robot'),!,
  forall(owl_has(Robot, urdf:'hasLink', Link), 
    assert_link_properties(Link)).

assert_link_properties(Link) :-
  owl_individual_of(Link, urdf:'Link'),!,
  %% TODO: complete me
  true.




