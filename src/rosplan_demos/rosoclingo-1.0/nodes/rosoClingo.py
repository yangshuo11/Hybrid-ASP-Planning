#!/usr/bin/env python
import rospy
import actionlib
from rosoclingo.msg import *
from rosoclingo.srv import *
import signal
from threading import Condition
from rosoClingoGringoControl import ROSoClingoGringoControl
from rosoClingoRequestHandler import ROSoClingoRequestHandler

class ROSoClingo:

    ACTIVE   = 1
    IDLE     = 2
    EXIT     = 3

    def __init__(self,arguments,checkers):
        self.condition = Condition()

        self.incoming = []
        self.message_send = []
        self.message_received = []
        self.message_aborted = []
        self.currentStep = 1
        self.state = ROSoClingo.IDLE

        self.arguments = arguments
        print(arguments)
        self.solver = ROSoClingoGringoControl(arguments,checkers,self)
        self.request_handler = ROSoClingoRequestHandler(self.solver.get_request_IDs())
        self.publisher = rospy.Publisher(rospy.get_name()+'/out',ROSoClingoOut,queue_size=10,latch=True) # TODO queue_size

    def run(self):
        self.solver.start()
        actionlib.ActionServer(rospy.get_name(),ROSoClingoAction,self.__requesting,self.__canceling,auto_start=False).start() # goal_cb, cancel_cb
        rospy.Subscriber(rospy.get_name()+"/in",ROSoClingoIn,self.__receiving)
        rospy.Service(rospy.get_name()+"/get_goal_information", GetGoalInformation, self.get_goal_information)
        while True:
            signal.signal(signal.SIGINT, self.__handle_signal)
            self.condition.acquire()
            if self.state == ROSoClingo.IDLE:
                self.condition.wait(float("inf"))
            elif   self.state == ROSoClingo.ACTIVE:
                self.state = ROSoClingo.IDLE
                for (type,content) in self.incoming:
                    if   type == "receive": self.__handle_receive(content)
                    elif type == "request": self.__handle_request(content)
                    elif type == "cancel":  self.__handle_cancel(content)
                    elif type == "status":  self.__handle_status(content)
                    # elif type == "update":
                        # self.solver.__init__(self.arguments)
                        # self.__handle_update(content)
                    else : 
                         rospy.logerr("Unknown content type! Content ignored " + str(content))
                self.incoming = []
            elif self.state == ROSoClingo.EXIT:
                self.solver.exit()
                rospy.signal_shutdown("Got an exit signal!")
                return
            self.condition.release()

    def __receiving(self, message):
        rospy.logdebug("message received: " + str(message))
        self.add("receive",message)

    def __requesting(self, request):
        rospy.logdebug("request received: " + str(request))
        rospy.loginfo("request received: " + str(request))
        self.add("request",request)

    def __canceling(self,request):
        rospy.logdebug("cancel received: " + str(request))
        self.add("cancel",request)

    def __handle_signal(self,signal,frame):
        self.condition.acquire()
        self.state = ROSoClingo.EXIT
        self.condition.notify()
        self.condition.release()

    def __handle_receive(self,message):
        if not (message in self.message_received):
            rospy.logdebug("handling message: " + str(message))
            self.message_received.append(message)
            self.solver.add([message.id,message.value,self.currentStep])
        else:
            rospy.logdebug("ignoring message: " + str(message))

    def __handle_request(self,request): #"request", bring(o2,o3),box
        id = self.request_handler.add(request)
        rospy.loginfo("id: " + str(id))
        if id is not None:
            # rospy.logdebug("handling request: " + str(request) + " as " + str(id))
            rospy.loginfo("handling request: " + str(request) + " as " + str(id))
            self.solver.add([id,str(request.get_goal().request),self.currentStep]) #[1,bring(o2,o3),1]
        elif self.currentStep == self.solver.horizon:
            id = self.request_handler.get()
            if id is not None:
                rospy.logdebug("handling request: " + str(request) + " as " + str(id))
                self.solver.add([id,str(request.get_goal().request),self.currentStep+1])

    def __handle_cancel(self,request):
        id = self.request_handler.cancel(request)
        if id is not None:
            rospy.logdebug("handling cancel: " + str(request) + " as " + str(id))
            self.solver.add([id,"cancel",self.currentStep])            

    def __handle_status(self,(key,status)):
        self.request_handler.status_update(key,status)
        id = self.request_handler.get()
        while(id is not None):
            self.solver.add  ([id,str(self.request_handler.get_request(id).get_goal().request),self.currentStep+1])
            id = self.request_handler.get()

    def __step_update(self):
        self.message_received = []
        self.message_send = []
        self.message_aborted = []
        self.request_handler.update()
        self.currentStep = self.currentStep + 1
        rospy.logdebug("new current timepoint: " + str(self.currentStep))

#######################

    def get_goal_information(self,args):
        return GetGoalInformationResponse(self.request_handler.get_information(str(args.key)))

    def publish_abort(self,atom):
        if not (str(atom.args()[0]) in self.message_aborted):
            message = ROSoClingoOut(str(atom.args()[0]),"abort")
            rospy.logdebug("publishing: " + str(message))
            self.message_aborted.append(str(atom.args()[0]))
            self.publisher.publish(message)
            rospy.sleep(0.1)

    def publish(self,atom):
        if not (str(atom.args()[0]) in self.message_send):
            message = ROSoClingoOut(str(atom.args()[0]),str(atom.args()[1]))
            rospy.logdebug("publishing: " + str(message))
            self.message_send.append(str(atom.args()[0]))
            self.publisher.publish(message)
            rospy.sleep(0.1)

    def get_step(self,horizon):
        if self.message_send != [] and all(s in [r.id for r in self.message_received] for s in self.message_send) and self.currentStep != horizon:
            self.__step_update()
        elif self.message_send == [] and self.currentStep < horizon:
            self.__step_update()
        return self.currentStep

    def add(self,type,message):
        self.condition.acquire()
        self.incoming.append((type,message))
        if self.state != ROSoClingo.EXIT:
            self.state = ROSoClingo.ACTIVE
        self.condition.notify()
        self.condition.release()
