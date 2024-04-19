yarp_prepare_plugin(CarlWbEPuck
                    CATEGORY device
                    TYPE CarlWbEPuck
                    INCLUDE CarlWbEPuck.h
                    DEFAULT ON)

if (NOT SKIP_CarlWbEPuck)

  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  yarp_add_plugin(yarp_CarlWbEPuck)
  
  find_package(Boost 1.75 REQUIRED COMPONENTS date_time container )  
  include_directories(${Boost_INCLUDE_DIRS})
   
  find_package(CARLcpp REQUIRED)
  include_directories(${CARLCPP_INCLUDE_DIR}) 
  include_directories(${libYARP_carl_SOURCE_DIR}/include)
   
  target_compile_definitions(yarp_CarlWbEPuck PRIVATE YARP_HAS_ACE)
  list(APPEND yarp_CarlWbEPuck ACE)  
 
  include_directories("../ConnectorCPP/include"
	$ENV{WEBOTS_HOME}/include/controller/c 
	$ENV{WEBOTS_HOME}/include/controller/cpp
	)   

  target_sources(yarp_CarlWbEPuck PRIVATE   CarlWbEPuck.cpp   # Yarp Device for E-Puck (parameterization, communication)
                                          CarlWbEPuck.h 
										                      CarlWbEPuckRobot.cpp			# The actual instanciation of the robot E-Puck2, which behavior and defaults 
                                          CarlWbEPuckRobot.h 

		$ENV{WEBOTS_HOME}/src/controller/cpp/Accelerometer.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Altimeter.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Brake.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Camera.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Compass.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Connector.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Device.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Display.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/DistanceSensor.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Emitter.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Field.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/GPS.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Gyro.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/InertialUnit.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Joystick.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Keyboard.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/LED.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Lidar.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/LightSensor.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Motion.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Motor.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Mouse.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Node.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Pen.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/PositionSensor.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Radar.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/RangeFinder.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Receiver.cpp

		#$ENV{WEBOTS_HOME}/src/controller/cpp/Robot.cpp
		webots_patches/src/controller/cpp/Robot.cpp

		$ENV{WEBOTS_HOME}/src/controller/cpp/Skin.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Speaker.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/Supervisor.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/TouchSensor.cpp
		$ENV{WEBOTS_HOME}/src/controller/cpp/VacuumGripper.cpp
   )
 
 
  #link_directories($ENV{WEBOTS_HOME}/lib/controller)
  #link_directories("C:/Program Files/Webots/lib/controller")
  
  target_link_directories(yarp_CarlWbEPuck PRIVATE 
	${CMAKE_INSTALL_PREFIX}/lib
	$ENV{WEBOTS_HOME}/lib/controller
	)
  
  target_link_libraries(yarp_CarlWbEPuck PRIVATE  ACE::ACE
                                                  YARP::YARP_os
                                                  YARP::YARP_sig
                                                  YARP::YARP_dev
                                                  YARP::YARP_carl   
												  
												  Controller

												  )
                                                   
  yarp_install(TARGETS yarp_CarlWbEPuck
               EXPORT YARP_${YARP_PLUGIN_MASTER}
               COMPONENT ${YARP_PLUGIN_MASTER}
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET yarp_CarlWbEPuck PROPERTY FOLDER "Plugins/Device/CarlWbEPuck")
                
endif ()
