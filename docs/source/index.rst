.. mpu9250 documentation master file, created by
   sphinx-quickstart on Fri Jun 22 11:27:23 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Drivers for MPU9250 IMU Sensor
==============================
These drivers support the `MPU9250 Inertial Measurement Unit <https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf>`_.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

Prerequisites
-------------
- Linux device with an MPU9250 connected via I2C, such as any version of `Forward Loop Zero <https://forward-loop.com/developer.html>`_ with the optional inertial measurement unit.
- (Optional) Docker on the Linux device. We recommend you check out `floop <https://github.com/ForwardLoopLLC/floopcli>`_, our simple, open-source tool for working with embedded Docker on multiple devices at the same time.

Install
-------

.. tabs::

    .. group-tab:: Using floop

        In order to use the drivers with floop, you configure the drivers and your application on a host device then push the source code and build instructions to each target device using floop.

        On your host, install floop using pip:

        .. code-block:: bash

            pip install --upgrade floopcli

        Clone the driver repository:
        
        .. code-block:: bash

            git clone --recursive https://github.com/ForwardLoopLLC/mpu9250

        The clone needs to be recursive in order to clone the appropriate `I2C drivers <https://github.com/ForwardLoopLLC/i2c>`_.

        Change directory into driver directory:

        .. code-block:: bash

            cd mlx90614

        Configure floop to allow access to I2C devices on the target device. For example, if you have one target device available over SSH at the address 192.168.1.100, your **floop.json** in the **mlx90614** directory could be:

        .. literalinclude:: ../../floop.json.example

        Notice that *privileged* is true, so floop has access to target device hardware.

        In order to check that installation succeeded, you can run the simple example included in the driver repository. You will need to tell the driver which bus holds the MPU9250. 
        
        Edit the file **run.sh.linux**:
        
        .. literalinclude:: ../../run.sh.linux
       
        Make sure to change the value of `BUS` to match the bus to which your MPU9250 is connected. The value of `BUS` is the same as the integer that corresponds to the `/dev/i2c-*` entry for your device.

        Now run the example:

        .. code-block:: bash

            floop run -v

        If the example returns with no error, then the installation succeeded. 

    .. group-tab:: Header Files 

        You can install the C++ header files for use in your own applications.  

        In order to use the drivers with Linux, you need to install some I2C dependencies:

        .. code-block:: bash

            sudo apt-get install -y python-smbus python-dev i2c-tools libi2c-dev 

        You need to install the MPU9250 and I2C headers, then include them during compilation of your application. Inside of your application folder, you can install both headers at the same time:

        .. code-block:: bash

            mkdir -p ./floop/i2c/ && \
            mkdir -p ./floop/mpu9250/ && \
            wget -O ./floop/i2c/i2c.h https://github.com/ForwardLoopLLC/i2c/blob/master/i2c/i2c.h && \
            wget -O ./floop/mpu9250/mpu9250.h https://github.com/ForwardLoopLLC/mpu9250/blob/master/mpu9250/mpu9250.h && \
            wget -O ./floop/mpu9250/mpu9250.h https://github.com/ForwardLoopLLC/mpu9250/blob/master/mpu9250/quaternion.h
            # optional quaternion functionality

        When you compile your application, make sure to include the path to the driver headers. For example, add `-I./floop/` during compilation. You can then access the drivers by including the mpu9250header(s):

        .. code-block:: c++

            #include "mpu9250/mpu9250.h"
            #include "mpu9250/quaternion.h" //optional

        Note that you can also use these steps to install the drivers inside of a Docker container.

Example
-------

The example shows the major functionality of the drivers.

.. literalinclude:: ../../example/main.cpp
    :language: c++

Related Documentation
---------------------
:doc:`cpp/api`
    Driver interface for MLX90614

`Source Code <https://github.com/ForwardLoopLLc/mpu9250>`_
    Our MIT license lets you use, change, and sell the drivers for free
 
`Forward Loop Docs <https://docs.forward-loop.com>`_
    Check out other drivers, develop tools, and code
