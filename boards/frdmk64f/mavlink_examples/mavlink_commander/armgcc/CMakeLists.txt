INCLUDE(CMakeForceCompiler)

# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

 
# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})


SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -DDEBUG")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -D__STARTUP_CLEAR_BSS")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -g")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mcpu=cortex-m4")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -Wall")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mfloat-abi=hard")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mfpu=fpv4-sp-d16")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mthumb")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -fno-common")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -ffunction-sections")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -fdata-sections")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -ffreestanding")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -fno-builtin")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -mapcs")

SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -std=gnu99")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -DNDEBUG")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -D__STARTUP_CLEAR_BSS")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mcpu=cortex-m4")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -Wall")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mfloat-abi=hard")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mfpu=fpv4-sp-d16")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mthumb")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -fno-common")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -ffunction-sections")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -fdata-sections")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -ffreestanding")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -fno-builtin")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mapcs")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -std=gnu99")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DCPU_MK64FN1M0VLL12")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DPRINTF_FLOAT_ENABLE=0")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DSCANF_FLOAT_ENABLE=0")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DPRINTF_ADVANCED_ENABLE=0")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DSCANF_ADVANCED_ENABLE=0")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DFRDM_K64F")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DFREEDOM")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DSERIAL_PORT_TYPE_UART=1")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mcpu=cortex-m4")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mfloat-abi=hard")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mfpu=fpv4-sp-d16")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mthumb")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -MMD")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -MP")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-common")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -ffunction-sections")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fdata-sections")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -ffreestanding")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-builtin")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mapcs")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -std=gnu99")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DNDEBUG")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DCPU_MK64FN1M0VLL12")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DPRINTF_FLOAT_ENABLE=0")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DSCANF_FLOAT_ENABLE=0")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DPRINTF_ADVANCED_ENABLE=0")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DSCANF_ADVANCED_ENABLE=0")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DFRDM_K64F")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DFREEDOM")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DSERIAL_PORT_TYPE_UART=1")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Os")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mcpu=cortex-m4")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mfloat-abi=hard")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mfpu=fpv4-sp-d16")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mthumb")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -MMD")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -MP")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fno-common")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -ffunction-sections")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fdata-sections")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -ffreestanding")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fno-builtin")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mapcs")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -std=gnu99")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DCPU_MK64FN1M0VLL12")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DSERIAL_PORT_TYPE_UART=1")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mcpu=cortex-m4")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mfloat-abi=hard")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mfpu=fpv4-sp-d16")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mthumb")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -MMD")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -MP")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-common")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ffunction-sections")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fdata-sections")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ffreestanding")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-builtin")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mapcs")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-rtti")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-exceptions")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DCPU_MK64FN1M0VLL12")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DSERIAL_PORT_TYPE_UART=1")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Os")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mcpu=cortex-m4")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mfloat-abi=hard")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mfpu=fpv4-sp-d16")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mthumb")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -MMD")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -MP")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-common")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ffunction-sections")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fdata-sections")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ffreestanding")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-builtin")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mapcs")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-rtti")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-exceptions")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mcpu=cortex-m4")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Wall")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mfloat-abi=hard")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mfpu=fpv4-sp-d16")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --specs=nano.specs")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --specs=nosys.specs")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fno-common")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -ffunction-sections")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fdata-sections")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -ffreestanding")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fno-builtin")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mthumb")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mapcs")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --gc-sections")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -static")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -z")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} muldefs")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -Map=output.map")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mcpu=cortex-m4")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Wall")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mfloat-abi=hard")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mfpu=fpv4-sp-d16")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --specs=nano.specs")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --specs=nosys.specs")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -fno-common")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -ffunction-sections")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -fdata-sections")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -ffreestanding")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -fno-builtin")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mthumb")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mapcs")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --gc-sections")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -static")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -z")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} muldefs")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Xlinker")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Map=output.map")

include_directories(${ProjDirPath}/../../../../src/mavlink_examples/mavlink_commander)

include_directories(${ProjDirPath}/..)

include_directories(${ProjDirPath}/../../..)

include_directories(${ProjDirPath}/../../../../src/mavlink_examples/mavlink_commander)

include_directories(${ProjDirPath}/..)

include_directories(${ProjDirPath}/../../..)

include_directories(${ProjDirPath}/..)

include_directories(${ProjDirPath}/../../..)

include_directories(${ProjDirPath}/../../../../../CMSIS/Include)

include_directories(${ProjDirPath}/../../../../../devices)

include_directories(${ProjDirPath}/../../../../../devices/MK64F12/drivers)

include_directories(${ProjDirPath}/../../../../../middleware/mavlink)

include_directories(${ProjDirPath}/../../../../../devices/MK64F12/utilities/str)

include_directories(${ProjDirPath}/../../../../../devices/MK64F12/utilities/debug_console)

include_directories(${ProjDirPath}/../../../../../devices/MK64F12)

include_directories(${ProjDirPath}/../../../../../components/uart)

include_directories(${ProjDirPath}/../../../../../components/serial_manager)

include_directories(${ProjDirPath}/../../../../../components/lists)

include_directories(${ProjDirPath}/../../../../../devices/MK64F12/utilities)

add_executable(mavlink_commander.elf 
"${ProjDirPath}/../mavlink_commander.c"
"${ProjDirPath}/../pin_mux.c"
"${ProjDirPath}/../pin_mux.h"
"${ProjDirPath}/../board.c"
"${ProjDirPath}/../board.h"
"${ProjDirPath}/../clock_config.c"
"${ProjDirPath}/../clock_config.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_port.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_sim.c"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_sim.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_clock.c"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_clock.h"
"${ProjDirPath}/../../../../../middleware/mavlink/checksum.h"
"${ProjDirPath}/../../../../../middleware/mavlink/mavlink_conversions.h"
"${ProjDirPath}/../../../../../middleware/mavlink/mavlink_get_info.h"
"${ProjDirPath}/../../../../../middleware/mavlink/mavlink_helpers.h"
"${ProjDirPath}/../../../../../middleware/mavlink/mavlink_sha256.h"
"${ProjDirPath}/../../../../../middleware/mavlink/mavlink_types.h"
"${ProjDirPath}/../../../../../middleware/mavlink/protocol.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/common.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_actuator_control_target.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_actuator_output_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_adsb_vehicle.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_altitude.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_att_pos_mocap.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_attitude.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_attitude_quaternion.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_attitude_quaternion_cov.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_attitude_target.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_auth_key.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_autopilot_version.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_battery_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_button_change.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_camera_capture_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_camera_image_captured.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_camera_information.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_camera_settings.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_camera_trigger.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_cellular_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_change_operator_control.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_change_operator_control_ack.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_collision.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_command_ack.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_command_int.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_command_long.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_control_system_state.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_data_stream.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_data_transmission_handshake.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_debug.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_debug_float_array.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_debug_vect.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_distance_sensor.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_encapsulated_data.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_estimator_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_extended_sys_state.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_fence_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_file_transfer_protocol.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_flight_information.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_follow_target.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_global_position_int.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_global_position_int_cov.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_global_vision_position_estimate.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_gps2_raw.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_gps2_rtk.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_gps_global_origin.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_gps_inject_data.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_gps_input.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_gps_raw_int.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_gps_rtcm_data.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_gps_rtk.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_gps_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_heartbeat.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_high_latency.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_high_latency2.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_highres_imu.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_hil_actuator_controls.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_hil_controls.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_hil_gps.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_hil_optical_flow.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_hil_rc_inputs_raw.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_hil_sensor.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_hil_state.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_hil_state_quaternion.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_home_position.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_isbd_link_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_landing_target.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_link_node_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_local_position_ned.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_local_position_ned_cov.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_local_position_ned_system_global_offset.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_log_data.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_log_entry.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_log_erase.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_log_request_data.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_log_request_end.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_log_request_list.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_logging_ack.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_logging_data.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_logging_data_acked.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_manual_control.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_manual_setpoint.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_memory_vect.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_message_interval.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_ack.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_changed.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_clear_all.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_count.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_current.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_item.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_item_int.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_item_reached.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_request.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_request_int.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_request_list.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_request_partial_list.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_set_current.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mission_write_partial_list.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_mount_orientation.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_named_value_float.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_named_value_int.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_nav_controller_output.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_obstacle_distance.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_odometry.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_optical_flow.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_optical_flow_rad.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_orbit_execution_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_param_ext_ack.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_param_ext_request_list.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_param_ext_request_read.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_param_ext_set.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_param_ext_value.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_param_map_rc.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_param_request_list.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_param_request_read.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_param_set.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_param_value.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_ping.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_play_tune.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_position_target_global_int.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_position_target_local_ned.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_power_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_protocol_version.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_radio_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_raw_imu.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_raw_pressure.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_rc_channels.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_rc_channels_override.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_rc_channels_raw.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_rc_channels_scaled.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_request_data_stream.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_resource_request.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_safety_allowed_area.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_safety_set_allowed_area.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_scaled_imu.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_scaled_imu2.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_scaled_imu3.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_scaled_pressure.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_scaled_pressure2.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_scaled_pressure3.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_serial_control.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_servo_output_raw.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_set_actuator_control_target.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_set_attitude_target.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_set_gps_global_origin.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_set_home_position.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_set_mode.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_set_position_target_global_int.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_set_position_target_local_ned.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_setup_signing.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_sim_state.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_smart_battery_info.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_smart_battery_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_statustext.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_statustext_long.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_storage_information.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_sys_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_system_time.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_terrain_check.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_terrain_data.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_terrain_report.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_terrain_request.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_time_estimate_to_target.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_timesync.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_trajectory_representation_bezier.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_trajectory_representation_waypoints.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_uavcan_node_info.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_uavcan_node_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_utm_global_position.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_v2_extension.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_vfr_hud.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_vibration.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_vicon_position_estimate.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_video_stream_information.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_video_stream_status.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_vision_position_estimate.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_vision_speed_estimate.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_wheel_distance.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_wifi_config_ap.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/mavlink_msg_wind_cov.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/testsuite.h"
"${ProjDirPath}/../../../../../middleware/mavlink/common/version.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_flash.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_ftfx_adapter.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_ftfx_cache.c"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_ftfx_cache.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_ftfx_controller.c"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_ftfx_controller.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_ftfx_features.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_ftfx_flash.c"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_ftfx_flash.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_ftfx_flexnvm.c"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_ftfx_flexnvm.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_ftfx_utilities.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_gpio.c"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_gpio.h"
"${ProjDirPath}/../../../../../devices/MK64F12/utilities/debug_console/fsl_debug_console.c"
"${ProjDirPath}/../../../../../devices/MK64F12/utilities/debug_console/fsl_debug_console.h"
"${ProjDirPath}/../../../../../devices/MK64F12/utilities/debug_console/fsl_debug_console_conf.h"
"${ProjDirPath}/../../../../../devices/MK64F12/utilities/str/fsl_str.c"
"${ProjDirPath}/../../../../../devices/MK64F12/utilities/str/fsl_str.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_uart.c"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_uart.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_smc.c"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_smc.h"
"${ProjDirPath}/../../../../../devices/MK64F12/MK64F12.h"
"${ProjDirPath}/../../../../../devices/MK64F12/MK64F12_features.h"
"${ProjDirPath}/../../../../../devices/MK64F12/fsl_device_registers.h"
"${ProjDirPath}/../../../../../components/uart/uart.h"
"${ProjDirPath}/../../../../../components/uart/uart_adapter.c"
"${ProjDirPath}/../../../../../components/serial_manager/serial_manager.c"
"${ProjDirPath}/../../../../../components/serial_manager/serial_manager.h"
"${ProjDirPath}/../../../../../components/serial_manager/serial_port_internal.h"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_common.c"
"${ProjDirPath}/../../../../../devices/MK64F12/drivers/fsl_common.h"
"${ProjDirPath}/../../../../../components/lists/generic_list.c"
"${ProjDirPath}/../../../../../components/lists/generic_list.h"
"${ProjDirPath}/../../../../../components/serial_manager/serial_port_uart.c"
"${ProjDirPath}/../../../../../components/serial_manager/serial_port_uart.h"
"${ProjDirPath}/../../../../../devices/MK64F12/gcc/startup_MK64F12.S"
"${ProjDirPath}/../../../../../devices/MK64F12/system_MK64F12.c"
"${ProjDirPath}/../../../../../devices/MK64F12/system_MK64F12.h"
"${ProjDirPath}/../../../../../devices/MK64F12/utilities/fsl_assert.c"
"${ProjDirPath}/../../../../../CMSIS/Include/core_cm4.h"
"${ProjDirPath}/../../../../../CMSIS/Include/cmsis_armcc.h"
"${ProjDirPath}/../../../../../CMSIS/Include/cmsis_armclang.h"
"${ProjDirPath}/../../../../../CMSIS/Include/cmsis_armclang_ltm.h"
"${ProjDirPath}/../../../../../CMSIS/Include/cmsis_compiler.h"
"${ProjDirPath}/../../../../../CMSIS/Include/cmsis_gcc.h"
"${ProjDirPath}/../../../../../CMSIS/Include/cmsis_iccarm.h"
"${ProjDirPath}/../../../../../CMSIS/Include/cmsis_version.h"
"${ProjDirPath}/../../../../../CMSIS/Include/core_armv81mml.h"
"${ProjDirPath}/../../../../../CMSIS/Include/core_armv8mbl.h"
"${ProjDirPath}/../../../../../CMSIS/Include/core_armv8mml.h"
"${ProjDirPath}/../../../../../CMSIS/Include/mpu_armv7.h"
"${ProjDirPath}/../../../../../CMSIS/Include/mpu_armv8.h"
"${ProjDirPath}/../../../../../CMSIS/Include/arm_common_tables.h"
"${ProjDirPath}/../../../../../CMSIS/Include/arm_const_structs.h"
"${ProjDirPath}/../../../../../CMSIS/Include/arm_math.h"
"${ProjDirPath}/../../../../../devices/MK64F12/utilities/fsl_sbrk.c"
)


set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -T${ProjDirPath}/MK64FN1M0xxx12_flash.ld -static")

set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${ProjDirPath}/MK64FN1M0xxx12_flash.ld -static")

TARGET_LINK_LIBRARIES(mavlink_commander.elf -Wl,--start-group)
target_link_libraries(mavlink_commander.elf debug m)

target_link_libraries(mavlink_commander.elf debug c)

target_link_libraries(mavlink_commander.elf debug gcc)

target_link_libraries(mavlink_commander.elf debug nosys)

target_link_libraries(mavlink_commander.elf optimized m)

target_link_libraries(mavlink_commander.elf optimized c)

target_link_libraries(mavlink_commander.elf optimized gcc)

target_link_libraries(mavlink_commander.elf optimized nosys)

TARGET_LINK_LIBRARIES(mavlink_commander.elf -Wl,--end-group)


