#!/usr/bin/python
import sys
import rospy
from collections import OrderedDict, deque
from std_msgs.msg import String

class Controller:
    def __init__(self, size):
        rospy.init_node("Intersection_Manager_Node", anonymous=True)
        self.r = rospy.Rate(10)
        # self.robot_queue_north_south_road = OrderedDict()
        # self.robot_queue_east_west_road = OrderedDict()
        self.robot_queue_north_south_road = deque()
        self.robot_queue_east_west_road = deque()
        #TODO Once you stop being lazy, read from deque and update no_cars accordingly
        # This is important for right turn. Right turn, let opposite side clear first before 
        # turnign right. When robots are all queued on intersection, need to check timestamp to check
        # who goes first. I have not done this yet
        self.no_cars_on_ns_road = 0
        self.no_cars_on_ew_road = 0
        self.size = size
        self.occuppied_road = [0, 0] # First element is northsouth road, second element is eastwest road
        self.robot_listener = []
        self.robot_talker = []
        for i in range(self.size):
            self.robot_listener.append(rospy.Subscriber("tb" + str(i) + "_relative_position", String, self.robot_msg_callback))
        for i in range(self.size):
            self.robot_talker.append(rospy.Publisher("tb" + str(i) + "_talker", String, queue_size = 10))
        

    def robot_msg_callback(self, goal):
        # print(goal.data)
        robot_info = (goal.data.split(" "))
        robot_id = robot_info[0]
        time_stamp = robot_info[1]
        incoming_dir = robot_info[2]
        direction_travel = robot_info[3]
        relative_goal = robot_info[4]

        if (incoming_dir == "North" or incoming_dir == "South") and relative_goal == "Begin_Intersection":
            self.robot_queue_north_south_road.append(robot_info)
        elif (incoming_dir == "East" or incoming_dir == "West") and relative_goal == "Begin_Intersection":
            self.robot_queue_east_west_road.append(robot_info)
        
        # if (incoming_dir == "North" or incoming_dir == "South") and relative_goal == "In_Intersection":
        #     self.occuppied_road[0] = 1
        # elif (incoming_dir == "East" or incoming_dir == "West") and relative_goal == "In_Intersection":
        #     self.occuppied_road[1] = 1

        # This will hopefully reset the intersection to be free
        if relative_goal == "End_Intersection":
            if incoming_dir == "North" or incoming_dir == "South":
                self.no_cars_on_ns_road -= 1
                if self.no_cars_on_ns_road == 0:
                    print("NS road is clear now")
                    self.occuppied_road[0] = 0
            else:
                self.no_cars_on_ew_road -= 1
                if self.no_cars_on_ew_road == 0:
                    print("EW road is clear now")
                    self.occuppied_road[1] = 0
        else:
            pass

    def check_intersection_free(self):
        return (self.occuppied_road[0] == 0 and self.occuppied_road[1] == 0)

    def run(self):
        while True:
            ns_robot_info = None
            ew_robot_info = None
            # print("cars NS: %d and cars EW: %d" % (self.no_cars_on_ns_road, self.no_cars_on_ew_road))

            try:
                ns_robot_info = self.robot_queue_north_south_road[0]
            except Exception as e:
                pass
            try:
                print(self.robot_queue_east_west_road)
                ew_robot_info = self.robot_queue_east_west_road[0]
            except Exception as e:
                pass

            if ns_robot_info == None and ew_robot_info == None:
                print(self.occuppied_road)
                self.r.sleep()
                continue
            elif ns_robot_info is not None and ew_robot_info == None:
                if self.occuppied_road[0] == 1 or self.check_intersection_free():
                    self.occuppied_road[0] = 1
                    self.occuppied_road[1] = 0
                else:
                    print("Wait for EW rd")
                    print(self.occuppied_road)
                    continue
                try:
                    self.robot_queue_north_south_road.popleft()
                    self.no_cars_on_ns_road += 1
                except IndexError:
                    pass
            elif ns_robot_info == None and ew_robot_info is not None:
                if self.occuppied_road[1] == 1 or self.check_intersection_free():
                    self.occuppied_road[0] = 0
                    self.occuppied_road[1] = 1
                else:
                    print("Wait for NS rd")
                    print(self.occuppied_road)
                    continue
                try:
                    self.robot_queue_east_west_road.popleft()
                    self.no_cars_on_ew_road += 1
                    print(self.robot_queue_east_west_road)
                except IndexError:
                    pass
            else:   # There are robots queuing at the intersection, this entire section needs to be fixed
            #TODO, need to check timestamp when [0, 0]
                # if self.occuppied_road[0] == 0 and self.occuppied_road[1] == 0:
                #     try:
                #         north_robot = 
                #         south_robot =
                #         east_robot =
                #         west_robot = 
                #     except IndexError:
                #         pass

                if self.occuppied_road[0] == 1:
                    try:
                        self.robot_queue_north_south_road.popleft()
                        self.no_cars_on_ns_road += 1
                    except IndexError:
                        pass

                if self.occuppied_road[1] == 1:
                    try:
                        self.robot_queue_east_west_road.popleft()
                        self.no_cars_on_ew_road += 1
                    except IndexError:
                        pass

   
                # if ns_robot_info[1] <= ew_robot_info[1]:
                #     if self.occuppied_road[0] == 1 or self.check_intersection_free():
                #         self.occuppied_road[0] = 1
                #         self.occuppied_road[1] = 0
                #     else:
                #         print("Wait for EW rd")
                #         print(self.occuppied_road)
                #         continue
                #     try:
                #         self.robot_queue_north_south_road.popleft()
                #         self.no_cars_on_ns_road += 1
                #     except IndexError:
                #         pass
                # else:
                #     if self.occuppied_road[1] == 1 or self.check_intersection_free():
                #         self.occuppied_road[0] = 0
                #         self.occuppied_road[1] = 1
                #     else:
                #         print("Wait for NS rd")
                #         print(self.occuppied_road)
                #         continue
                #     try:
                #         self.robot_queue_east_west_road.popleft()
                #         self.no_cars_on_ew_road += 1
                #     except IndexError:
                #         pass
            
            print(self.occuppied_road)
            road_status = ' '.join(str(i) for i in self.occuppied_road)
            try:
                self.robot_talker[int(ns_robot_info[0])].publish(road_status)
            except Exception as e:
                pass
            try:
                self.robot_talker[int(ew_robot_info[0])].publish(road_status)
            except Exception as e:
                pass

            self.r.sleep()



if __name__ == '__main__':
    try:
        im = Controller(int(sys.argv[1]))
        im.run()
    except rospy.ROSInterruptException:
		pass