#include <stdio.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define XCOORD 0
#define ZCOORD 2

WbNodeRef robot;
WbFieldRef robot_translation_field;
const double *robot_translation;
static int time_step = 64;


int main() {
  wb_robot_init();
  //time_step  = wb_robot_get_basic_time_step();
  robot = wb_supervisor_node_get_from_def("PIONEER_3DX");
  robot_translation_field = wb_supervisor_node_get_field(robot,"translation");
  wb_robot_step(time_step);
  while(1){
    robot_translation = wb_supervisor_field_get_sf_vec3f(robot_translation_field);
    wb_robot_step(time_step);
    printf("Pioneer 3DX is at: (%f, %f)\n",robot_translation[XCOORD],robot_translation[ZCOORD]);
    wb_robot_step(time_step);
  }
  return 0;
}
