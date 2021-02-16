First, Let’s define how we want to use offline mode:

# Handled in Fastcat
1. Modify a line in the top of the yaml config file
2. Modify the bus type “jsd_bus” -> “offline_bus” **
3. Program passes command line argument to fastcat-lib **
4. input yaml file contains many buses and API calls are used to enable only certain buses (Joseph's suggestion) **
* ** allows for interspersing online and offline buses

# Handled in JSD
1. Run a different “offline” program (app links to jsd-lib-offline instead of jsd-lib and make 2 programs)

# Other Questions
* Do we ever want to implement a simulation mode in fastcat?
    * Probably better handled at “actuator” level and not “motor controller” level
    * How best to plug into Gazebo or Darts?
* Do we want to ever run online and offline devices together?
  * YES! but this will complicate builds that do not have JSD/SOEM installed


# Pros and Cons:
Pros for Fastcat Handling:
* No JSD code change
* No JSD/SOEM dependency at all for offline mode

Cons for Fastcat Handling:
* Apps that use JSD and not Fastcat will have to make an offline mode
* Complexity of Fastcat code change probably > complexity of JSD code change
* Fastcat is still a bit immature still, JSD more stable


Pros for JSD Handling:
* No Fastcat C++ code change
* Can manage builds that don’t have SOEM installed
* Same yaml config file is used for both
* Fastcat is still a bit immature still, JSD more stable

Cons for JSD Handling:
* Will have to implement jsd_read() and jsd_write() functions
* makes concurrent online/offline bus handling more difficult

# Conclusions (5/18/2020)
* Implement solution that allows changing jsd_bus from "jsd_bus" -> "offline_bus" and vice versa
* Force Fastcat to have required dependency on JSD (for now) can work that out later
* Revisit closer to mature 1.0.0 release
