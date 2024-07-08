#!/usr/bin/python3
import rclpy

from merlin2_mission import Merlin2MissionNode

from merlin2_basic_actions.merlin2_basic_types import (
    wp_type
)
from merlin2_basic_actions.merlin2_basic_predicates import (
    robot_at,
    person_at
)

from kant_dto import (
    PddlObjectDto,
    PddlPropositionDto
)

from pddl import wp_checked

class Merlin2DemoNode(Merlin2MissionNode):

    def __init__(self):
        super().__init__("mision_node")


    def create_objects(self):
   
        self.wp0 = PddlObjectDto(wp_type, "p0")  
        self.wp1 = PddlObjectDto(wp_type, "p1")
        self.wp2 = PddlObjectDto(wp_type, "p2")
        self.wp3 = PddlObjectDto(wp_type, "p3")
        self.wp4 = PddlObjectDto(wp_type, "p4")
        self.wp5 = PddlObjectDto(wp_type, "p5")


        objects = [self.wp0,self.wp1,self.wp2,self.wp3,self.wp4,self.wp5 ]
        return objects

    def create_propositions(self):
     
      
        robot_at_wp0 = PddlPropositionDto(robot_at, [self.wp0])

      
        return [robot_at_wp0]

    def execute_mission(self):
        self.get_logger().info("EXECUTING PATROL MISION")

        g1 = PddlPropositionDto(
            wp_checked, [self.wp1], is_goal = True)

        g2 = PddlPropositionDto(
        wp_checked, [self.wp2], is_goal = True)

        g3 = PddlPropositionDto(
        wp_checked, [self.wp3], is_goal = True)  

        g4 = PddlPropositionDto(
        wp_checked, [self.wp4], is_goal = True) 
        
        g5 = PddlPropositionDto(
        wp_checked, [self.wp5], is_goal = True)    

        l_aux = [g1,g2,g3,g4,g5]

       
        succeed = self.execute_goals(l_aux)
        self.get_logger().info(str(succeed))


def main(args=None):
    rclpy.init(args=args)

    node = Merlin2DemoNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
