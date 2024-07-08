#!/usr/bin/python3

from typing import List
import rclpy
import time

from kant_dto import (
    PddlObjectDto,
    PddlConditionEffectDto,
)

from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at

from merlin2_action.merlin2_action import Merlin2Action

from waypoint_navigation_msgs.action import NavigateToWp
from merlin2_arch.msg import PlanAction

from geometry_msgs.msg import Twist
from pddl import wp_checked


class Merlin2NavigationAction(Merlin2Action):

    def __init__(self):

        #  PDDL parameters 
        self.__org = PddlObjectDto(wp_type, "o")
        self.__dst = PddlObjectDto(wp_type, "d")

        # super init
        super().__init__("navigation_prac")



        self.__wp_nav_client = self.create_action_client(
            NavigateToWp, "/waypoint_navigation/navigate_to_wp")

    # callback
    def run_action(self, goal: PlanAction) -> bool:

        print("nav action node : New plan action : " , goal)

        nav_goal = NavigateToWp.Goal()

        print("Nav goal ", nav_goal)

        dst = goal.objects[1]
        nav_goal.wp_id = dst

        
        self.__wp_nav_client.wait_for_server()
        self.__wp_nav_client.send_goal(nav_goal)
        self.__wp_nav_client.wait_for_result()

        #twist
        pub = self.create_publisher(Twist, '/cmd_vel', 10)

        if self.__wp_nav_client.is_succeeded():

        
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 8.0 # 12.56 rad = 2 * 360 deg


            pub.publish(twist) #12.56 rad = 2 * 360 deg

            time.sleep(2)

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0 # 12.56 rad = 2 * 360 deg
            pub.publish(twist)


            return True

        else:
            return False

    def cancel_action(self):
        self.__wp_nav_client.cancel_goal()

    #  PDDL parameters
    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__org, self.__dst]

    #  PDDL  conditions 
    def create_conditions(self) -> List[PddlConditionEffectDto]:
        condition_1 = PddlConditionEffectDto(robot_at,
                                             [self.__org],
                                             time=PddlConditionEffectDto.AT_START)
        return [condition_1]

    #  PDDL  effects 
    def create_efects(self) -> List[PddlConditionEffectDto]:
        effect_1 = PddlConditionEffectDto(robot_at,
                                          [self.__dst],
                                          time=PddlConditionEffectDto.AT_END)

        effect_2 = PddlConditionEffectDto(robot_at,
                                          [self.__org],
                                          is_negative=True,
                                          time=PddlConditionEffectDto.AT_START)

        effect_3 = PddlConditionEffectDto(wp_checked,
                                          [self.__dst],
                                          time=PddlConditionEffectDto.AT_START)

        return [effect_1, effect_2, effect_3]


def main(args=None):
    rclpy.init(args=args)
    node = Merlin2NavigationAction()
    node.join_spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()