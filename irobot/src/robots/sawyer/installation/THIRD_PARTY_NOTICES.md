# Wire schema sources

`wire_schema.json` is generated metadata derived from ROS 1 message, service
and action definitions. It preserves original definitions and records SHA-256
checksums for Intera and standard input files. The Python codec and TCPROS implementation
are project code; they do not embed the ROS client libraries.

- `intera_core_msgs` and `intera_motion_msgs`: Rethink Robotics,
  Copyright (c) 2013–2018, Rethink Robotics Inc., Apache License 2.0.
  The upstream license is included as `INTERA_LICENSE.txt`.
  Source: <https://github.com/RethinkRobotics/intera_common>.
- Standard definitions: `std_msgs` 0.5.14 and `geometry_msgs`, `sensor_msgs`,
  `diagnostic_msgs`, `actionlib_msgs` 1.13.2, identified as BSD-licensed by
  their package manifests.
  Authors credited there include Morgan Quigley, Ken Conley, Jeremy Leibs,
  Vijay Pradeep, and Tully Foote. Sources: <https://github.com/ros/std_msgs> and
  <https://github.com/ros/common_msgs>.
- `tf2_msgs/TFMessage`: the canonical ROS 1 single-field definition from
  <https://github.com/ros/geometry2>, BSD license. Its dependencies use the
  pinned ROS 1 `geometry_msgs`/`std_msgs` definitions above; ROS 2 `Header`
  definitions must not be substituted. Reference MD5:
  `94810edda583a504dfda3829e70d7eec`.

The input baseline is the local Intera SDK 5.3.0 / robot 5.3.1 deployment.
`genmsg` is used only to regenerate schemas during development. It and `genpy`
are not runtime dependencies of this driver.
