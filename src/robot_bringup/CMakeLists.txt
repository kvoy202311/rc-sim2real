cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config systemd DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/kvoy_bringup_autostart.sh
  scripts/install_user_service.sh
  scripts/install_gamesir_watchdog_service.sh
  scripts/bringup_restart_supervisor.py
  scripts/startup_sound_node.py
  scripts/gamesir_usb_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
