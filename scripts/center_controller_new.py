#!/usr/bin/python
import sys
import rospy
from collections import OrderedDict, deque
from std_msgs.msg import String


## TODO: Have a counter on each road segment, if counter reached. Switch over to alternate road
## this is to ensure fairness? How would right turn work?
class Controller:
    def __init__(self, size):
        rospy.init_node("Intersection_Manager_Node", anonymous=True)
        self.r = rospy.Rate(10)

        self.robot_queue_south_north_road = deque()
        self.robot_queue_north_south_road = deque()
        self.robot_queue_east_west_road = deque()
        self.robot_queue_west_east_road = deque()

        self.no_cars_on_sn_road = 0
        self.no_cars_on_ns_road = 0        
        self.no_cars_on_ew_road = 0
        self.no_cars_on_we_road = 0
        self.occuppied_road = [0, 0, 0, 0] # SNEW

        self.size = size
        self.robot_listener = []
        self.robot_talker = []
        for i in range(self.size):
            self.robot_listener.append(rospy.Subscriber("tb" + str(i) + "_relative_position", String, self.robot_msg_callback))
        for i in range(self.size):
            self.robot_talker.append(rospy.Publisher("tb" + str(i) + "_talker", String, queue_size = 10))
        

    def robot_msg_callback(self, goal):
        print(goal.data)
        robot_info = (goal.data.split(" "))
        robot_id = robot_info[0]
        time_stamp = robot_info[1]
        incoming_dir = robot_info[2]
        direction_travel = robot_info[3]
        relative_goal = robot_info[4]

        if incoming_dir == "North" and relative_goal == "Begin_Intersection":
            self.robot_queue_south_north_road.append(robot_info)
        elif incoming_dir == "South" and relative_goal == "Begin_Intersection":
            self.robot_queue_north_south_road.append(robot_info)
        elif incoming_dir == "East" and relative_goal == "Begin_Intersection":
            self.robot_queue_west_east_road.append(robot_info)
        elif incoming_dir == "West" and relative_goal == "Begin_Intersection":
            self.robot_queue_east_west_road.append(robot_info)
        
        # if (incoming_dir == "North" or incoming_dir == "South") and relative_goal == "In_Intersection":
        #     self.occuppied_road[0] = 1
        # elif (incoming_dir == "East" or incoming_dir == "West") and relative_goal == "In_Intersection":
        #     self.occuppied_road[1] = 1

        # This will hopefully reset the intersection to be free
        if relative_goal == "End_Intersection":
            if incoming_dir == "North":
                self.no_cars_on_sn_road -= 1
                if self.no_cars_on_sn_road == 0:
                    print("SN road clear now")
                    self.occuppied_road[0] = 0
            elif incoming_dir == "South":
                self.no_cars_on_ns_road -= 1
                if self.no_cars_on_ns_road == 0:
                    print("NS road clear now")
                    self.occuppied_road[1] = 0
            elif incoming_dir == "West":
                self.no_cars_on_ew_road -= 1
                if self.no_cars_on_ew_road == 0:
                    print("WE road is clear now")
                    self.occuppied_road[2] = 0
            elif incoming_dir == "East":
                self.no_cars_on_we_road -= 1
                if self.no_cars_on_we_road == 0:
                    print("EW road is clear now")
                    self.occuppied_road[3] = 0
        else:
            pass

    def check_intersection_free(self):
        return (self.occuppied_road[0] == 0 and self.occuppied_road[1] == 0 and \
                self.occuppied_road[2] == 0 and self.occuppied_road[3] == 0)

    def run(self):
        while True:
            ns_robot_info = None
            sn_robot_info = None
            ew_robot_info = None
            we_robot_info = None
            print("cars SN:%d, NS:%d, EW:%d, WE:%d" % (self.no_cars_on_sn_road, self.no_cars_on_ns_road, self.no_cars_on_ew_road,self.no_cars_on_we_road))

            try:
                ns_robot_info = self.robot_queue_north_south_road[0]
            except Exception as e:
                pass
            try:
                sn_robot_info = self.robot_queue_south_north_road[0]
            except Exception as e:
                pass
            try:
                ew_robot_info = self.robot_queue_east_west_road[0]
            except Exception as e:
                pass
            try:
                we_robot_info = self.robot_queue_west_east_road[0]
            except Exception as e:
                pass
            print(self.robot_queue_south_north_road)
            print(self.robot_queue_north_south_road)
            print(self.robot_queue_east_west_road)
            print(self.robot_queue_west_east_road)

            # All queues are empty, refresh while loop
            if ns_robot_info == None and sn_robot_info == None and\
                ew_robot_info == None and we_robot_info == None:
                self.r.sleep()
                print(self.occuppied_road)
                continue
            # if self.check_intersection_free():
            # 	print("AM I STUCK HERE?")
            # 	self.r.sleep()
            # 	print(self.occuppied_road)
            # 	continue

            # When robot first enters intersection and can just proceed straight away
            if sn_robot_info is not None and ns_robot_info == None\
                and ew_robot_info == None and we_robot_info == None:
                if self.occuppied_road[0] == 1 or self.check_intersection_free():
                    self.occuppied_road[0] = 1
                    # self.occuppied_road[1] = 1 # Keep state same dont change it 
                    self.occuppied_road[2] = 0
                    self.occuppied_road[3] = 0
                else:
                    print("Wait for EW and WE rd")
                    print(self.occuppied_road)
                    continue
                try:
                    self.robot_queue_south_north_road.popleft()
                    self.no_cars_on_sn_road += 1
                    print(self.robot_queue_south_north_road)
                except IndexError:
                    pass
            elif ns_robot_info is not None and sn_robot_info == None\
                and ew_robot_info == None and we_robot_info == None:
                if self.occuppied_road[1] == 1 or self.check_intersection_free():
                    # self.occuppied_road[0] = 1
                    self.occuppied_road[1] = 1
                    self.occuppied_road[2] = 0
                    self.occuppied_road[3] = 0
                else:
                    print("Wait for EW and WE rd")
                    print(self.occuppied_road)
                    continue
                try:
                    self.robot_queue_north_south_road.popleft()
                    self.no_cars_on_ns_road += 1
                except IndexError:
                    pass
            elif ew_robot_info is not None and ns_robot_info == None\
                and sn_robot_info == None and we_robot_info == None:
                if self.occuppied_road[2] == 1 or self.check_intersection_free():
                    self.occuppied_road[0] = 0
                    self.occuppied_road[1] = 0
                    self.occuppied_road[2] = 1
                    # self.occuppied_road[3] = 1
                else:
                    print("Wait for NS and SN rd")
                    print(self.occuppied_road)
                    continue
                try:
                    self.robot_queue_east_west_road.popleft()
                    self.no_cars_on_ew_road += 1
                    print(self.robot_queue_east_west_road)
                except IndexError:
                    pass
            elif we_robot_info is not None and ns_robot_info == None\
                and sn_robot_info == None and ew_robot_info == None:
                if self.occuppied_road[3] == 1 or self.check_intersection_free():
                    self.occuppied_road[0] = 0
                    self.occuppied_road[1] = 0
                    # self.occuppied_road[2] = 1
                    self.occuppied_road[3] = 1
                else:
                    print("Wait for NS and SN rd")
                    print(self.occuppied_road)
                    continue
                try:
                    self.robot_queue_west_east_road.popleft()
                    self.no_cars_on_we_road += 1
                    print(self.robot_queue_west_east_road)
                except Exception as e:
                    pass                                               
            else:   # There are robots queuing at the intersection, this entire section needs to be fixed
            #TODO, need to check timestamp when [0, 0]
                # if self.occuppied_road[0] == 0 and self.occuppied_road[1] == 0 and\
                #     self.occuppied_road[2] == 0 and self.occuppied_road[3] == 0:
                print("DID I GET HERE?")
                if self.check_intersection_free():
                    try:
                    	south_robot, north_robot, east_robot, west_robot = sys.maxint, sys.maxint, sys.maxint, sys.maxint
                		
                    	try:
                        	south_robot = float(sn_robot_info[1])
                        except Exception as e:
                        	# print(e.message, e.args)
                        	print("SOUTH none?")
                        	pass
                        try:
                        	north_robot = float(ns_robot_info[1])
                        except Exception as e:
                        	# print(e.message, e.args)
                        	print("NORTH none?")
                        	pass
                        try:
                        	east_robot = float(ew_robot_info[1])
                        except Exception as e:
                        	print(e.message, e.args)
                        	print("East none?")
                        	pass
                        try:
                        	west_robot = float(we_robot_info[1])
                        except Exception as e:
                        	print(e.message, e.args)
                        	print("West none?")
                        	pass
                        print("When do I get here")
                        robot_snew_ts = [sys.maxint, south_robot, north_robot, east_robot, west_robot]
                        print(robot_snew_ts)
                        first_ts = robot_snew_ts.index(min(robot_snew_ts))
                        print("min index is %d" % first_ts)
                        if first_ts == 0:
                        	print("This shouldnt happen, it means there is no robot queuing at the intersectopn\
                        		this should not occur because we should not be able to get to this block as \
                        		line 131 should've covere this")
                        elif first_ts == 1:
                            try:
                                self.robot_queue_south_north_road.popleft()
                                self.no_cars_on_sn_road += 1
                                self.occuppied_road[0] = 1
                            except IndexError:
                                pass
                        elif first_ts == 2:
                            try:
                                self.robot_queue_north_south_road.popleft()
                                self.no_cars_on_ns_road += 1
                                self.occuppied_road[1] = 1
                            except IndexError:
                                pass
                        elif first_ts == 3:
                            try:
                                self.robot_queue_east_west_road.popleft()
                                self.no_cars_on_ew_road += 1
                                self.occuppied_road[2] = 1
                            except IndexError:
                                pass
                        else:
                            try:
                                self.robot_queue_west_east_road.popleft()
                                self.no_cars_on_we_road += 1
                                self.occuppied_road[3] = 1
                            except IndexError:
                                pass
                    except Exception as e:
                        # print(e.message, e.args)
                        pass
                #TODO Can i combine [0] and [1] and [2] and [3] or are they required to be
                # separate once we introduce right turns?
                if self.occuppied_road[0] == 1:
                    try:
                        self.robot_queue_south_north_road.popleft()
                        self.no_cars_on_sn_road += 1
                    except IndexError:
                        pass
                    try:
                        self.robot_queue_north_south_road.popleft()
                        self.no_cars_on_ns_road += 1
                        self.occuppied_road[1] = 1
                    except IndexError:
                        pass
                elif self.occuppied_road[1] == 1:
                    try:
                        self.robot_queue_north_south_road.popleft()
                        self.no_cars_on_ns_road += 1
                    except IndexError:
                        pass
                    try:
                        self.robot_queue_south_north_road.popleft()
                        self.no_cars_on_sn_road += 1
                        self.occuppied_road[0] = 1
                    except IndexError:
                        pass
                elif self.occuppied_road[2] == 1:
                    try:
                        self.robot_queue_east_west_road.popleft()
                        self.no_cars_on_ew_road += 1
                    except IndexError:
                        pass
                    try:
                        self.robot_queue_west_east_road.popleft()
                        self.no_cars_on_we_road += 1
                        self.occuppied_road[3] = 1
                    except IndexError:
                        pass
                elif self.occuppied_road[3] == 1:
                    try:
                        self.robot_queue_west_east_road.popleft()
                        self.no_cars_on_we_road += 1
                    except IndexError:
                        pass
                    try:
                        self.robot_queue_east_west_road.popleft()
                        self.no_cars_on_ew_road += 1
                        self.occuppied_road[2] = 1
                    except IndexError:
                        pass

            print(self.occuppied_road)
            road_status = ' '.join(str(i) for i in self.occuppied_road)
            try:
                self.robot_talker[int(ns_robot_info[0])].publish(road_status)
            except Exception as e:
                pass
            try:
                self.robot_talker[int(sn_robot_info[0])].publish(road_status)
            except Exception as e:
                pass
            try:
                self.robot_talker[int(ew_robot_info[0])].publish(road_status)
            except Exception as e:
                pass
            try:
                self.robot_talker[int(we_robot_info[0])].publish(road_status)
            except Exception as e:
                pass
            print("Sent message")
            self.r.sleep()



if __name__ == '__main__':
    try:
        im = Controller(int(sys.argv[1]))
        im.run()
    except rospy.ROSInterruptException:
		pass