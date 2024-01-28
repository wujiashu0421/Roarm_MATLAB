=====================
Using URDF with SPART
=====================

SPART can also use the popular Unified Robot Description Format (URDF) file. More information about how to create this type of robot descriptions can be found in the `ROS wiki`_. Particularly clarifying are the `XML field descriptions`_.

.. _ROS wiki: http://wiki.ros.org/urdf
.. _XML field descriptions: http://wiki.ros.org/urdf/XML

SPART includes several URDF models that can be found in the ``URDF_models`` folder. A list of these models can be found in :ref:`URDF-Models`.

Once you have your URDF description it has to be converted into the robot model. This can be easily done as follows:

.. code-block:: matlab

	%URDF filename
	filename='kuka_lwr/kuka.urdf';

	%Create robot model
	[robot,robot_keys] = urdf2robot(filename);

The ``robot_keys`` variables contains the original URDF names of the joints and links in three Matlab containers.

To get the id of a link:

.. code-block:: matlab

	robot_keys.link_id('Link_name')

To get the id of a joint:

.. code-block:: matlab

	robot_keys.joint_id('Joint_name')

To get the id of a variable ``qm`` (moving joint):

.. code-block:: matlab

	robot_keys.q_id('Joint_name')

And if you want to know the names of the available links, all joints, and moving joints:

.. code-block:: matlab

	link_names=keys(robot_keys.link_id);
	joint_names=keys(robot_keys.joint_id);
	qm_names=keys(robot_keys.q_id);


Limitations
===========

There are several known limitations with how SPART models parses URDF models.

- Revolute joint limits are not enforced. Revolute and Continuous joints are treated equally.
- Planar and Floating joints are not supported. Please redefine them as a combination of Revolute/Continuous and Prismatic joints.

Virtual World in Simulink Using URDF
====================================

One advantage of using URDF is that it can be imported into a Simulink virtual world (since Matlab 2016b). To do so follow the `MathWorks documentation`_.

.. _Mathworks documentation: https://www.mathworks.com/help/sl3d/import-visual-representations-of-robot-models.html