=====
SPART
=====

SPART is an open-source modeling and control toolkit for mobile-base robotic multibody systems with kinematic tree topologies (`i.e.`, open-loop multi-branched systems).
SPART is MATLAB-based and ROS-compatible, allowing to prototype in simulation and deploy to hardware controllers for robotic systems.

Given a URDF description of a multibody system, SPART computes the system's:

* Kinematics -- pose of the links and joints (`i.e.`, rotation matrices and position vectors).
* Differential kinematics -- operational space velocities and accelerations, as well as the geometric Jacobians and their time derivatives.
* Dynamics -- generalized inertia and convective inertia matrices.
* Forward/Inverse dynamics -- solves both problems, including the floating-base case.

SPART supports symbolic computation and analytic expressions for all kinematic and dynamic quantities can be obtained.

Contents
========

.. toctree::
   :maxdepth: 2

   Installation
   Tutorial_Intro
   Tutorial_Robot
   Tutorial_Kinematics
   Tutorial_Dynamics
   Functions
   URDF_Models
   Cite
   Help


License
=======

SPART is released under the `LGPLv3 <https://www.gnu.org/licenses/lgpl.html>`_ license.

.. image:: https://www.gnu.org/graphics/lgplv3-147x51.png
   :height: 51px
   :width: 147px
   :scale: 50 %
   :alt: LGPLv3

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

