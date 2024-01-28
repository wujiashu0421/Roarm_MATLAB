================
Installing SPART
================

First you need to download the SPART toolkit from the `SPART Github repository <https://github.com/NPS-SRL/SPART/archive/master.zip>`_:

.. code:: sh

 git clone https://github.com/NPS-SRL/SPART.git

Once you have the SPART source code, navigate to the SPART root folder and execute the following command in the MATLAB command window:

.. code:: matlab
	
	SPART2path

This adds all the required SPART folders into your MATLAB path and saves it.

.. note::

   In Linux, running MATLAB with root privileges may be required (`i.e.`, ``sudo``) to permanently save the MATLAB path.


Dependencies
============

Some MATLAB toolboxes are required to fully exploit SPART:

* The `Robotics System Toolbox <https://www.mathworks.com/products/robotics.html>`_ is required to interface MATLAB with ROS.
* The `Matlab Coder <https://www.mathworks.com/products/matlab-coder.html>`_ and the `Simulink Coder <https://www.mathworks.com/products/simulink-coder.html>`_ are required to generate standalone ROS nodes or portable C/C++ code from SPART MATLAB code or Simulink models.
* The `Symbolic Math Toolbox <https://www.mathworks.com/products/symbolic.html>`_ is required to obtain analytic expressions of the kinematic and dynamic quantities.
* The `Simulink 3D Animation <https://www.mathworks.com/products/3d-animation.html>`_ is useful to visualize the robotic systems.

	


