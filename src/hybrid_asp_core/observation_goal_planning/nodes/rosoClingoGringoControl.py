#!/usr/bin/env python
import rospy
import gringo
import json
from threading import Thread, Condition, Event

from optparse import (OptionParser,BadOptionError,AmbiguousOptionError)


class ThreadWithReturnValue(Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs={}, Verbose=None):
        Thread.__init__(self, group, target, name, args, kwargs, Verbose)
        self._return = None
    def run(self):
        if self._Thread__target is not None:
            self._return = self._Thread__target(*self._Thread__args,
                                                **self._Thread__kwargs)
    def join(self):
        Thread.join(self)
        return self._return

class PassThroughOptionParser(OptionParser):
    """
    An unknown option pass-through implementation of OptionParser.

    When unknown arguments are encountered, bundle with largs and try again,
    until rargs is depleted.  

    sys.exit(status) will still be called if a known argument is passed
    incorrectly (e.g. missing arguments or bad argument types, etc.)        
    """
    def _process_args(self, largs, rargs, values):
        while rargs:
            try:
                OptionParser._process_args(self,largs,rargs,values)
            except (BadOptionError,AmbiguousOptionError), e:
                largs.append(e.opt_str)

class ROSoClingoGringoControl(Thread):

    IDLE = 1
    ACTIVE = 2
    EVENT = 3
    EXIT = 4

    def __init__(self,arguments,checkers,publisher):
        Thread.__init__(self)
        self.condition = Condition()
        self.incoming = []
        self.horizon = 0
        self.atoms = []

        self.checkers = checkers
        self.event = Event()
        self.solver = self.__init_solver(arguments)
        self.state = ROSoClingoGringoControl.ACTIVE
        self.publisher = publisher
        self.lastStep = 1

    def run(self):
        while True:
            self.condition.acquire()
            if self.state == ROSoClingoGringoControl.IDLE:
                rospy.logdebug("idle start")
                self.condition.wait(float("inf"))
                rospy.logdebug("idle end")
            elif   self.state == ROSoClingoGringoControl.ACTIVE:
                rospy.logdebug("active start")
                if self.state != ROSoClingoGringoControl.EXIT: 
                    self.state = ROSoClingoGringoControl.IDLE
                self.__find_plan()
                rospy.logdebug("active end")
            elif    self.state == ROSoClingoGringoControl.EVENT:
                rospy.logdebug("grounding start")
                if self.state != ROSoClingoGringoControl.EXIT:
                    self.state = ROSoClingoGringoControl.ACTIVE
                for event in self.incoming:
                    self.__handle_event(event)
                self.incoming = []
                self.event.clear()
                rospy.logdebug("grounding end")
            elif self.state == ROSoClingoGringoControl.EXIT:
                rospy.loginfo("Stats: " + json.dumps(self.solver.stats, sort_keys=True, indent=4, separators=(',', ': ')))
                rospy.logdebug("exit")
                return
            self.condition.release()

    def __handle_event(self,parameter):
        print("parameter before mapped: ")
        print(parameter)
        parameter = map(self.__string2Fun,parameter)
        print("parameter: ")
        print(parameter)
        while(parameter[-1]>self.horizon):
            self.__new_horizon()
        rospy.loginfo("Grounding: set_event " + str(parameter))
        self.solver.ground([("set_event",parameter)]) ###################

    def __init_solver(self,arguments):
        parser = PassThroughOptionParser()
        parser.add_option("-f", "--file", action="append", type="string", dest="files")
        (rosoclingoarguments,gringoarguments) = parser.parse_args()
        print("rosoclingoarguments: ")
        # {'files': ['/home/ys/20210817-RAL-Project/TiagoOffice/src/rosplan_demos/rosoclingo-1.0/mailbot/instances/graph_wg.lp',
        # '/home/ys/20210817-RAL-Project/TiagoOffice/src/rosplan_demos/rosoclingo-1.0/mailbot/mailbot_sensing_actuating_concurrent_complete_dynamic.lp']}
        print("gringoarguments: ")
        # []
        solver = gringo.Control(gringoarguments)
        for file in rosoclingoarguments.files:
            solver.load(file)
        rospy.logdebug("Grounding: " + "base" + " " + str([]))
        rospy.logdebug("Grounding: " + "state" + " " + str([self.horizon]))
        rospy.logdebug("Grounding: " + "query" + " " + str([self.horizon]))
        solver.ground([("base",[]),("state",[self.horizon]),("query",[self.horizon])])
        rospy.loginfo("plan length: " + str(self.horizon))
        solver.assign_external(gringo.Fun("query", [self.horizon]),True)
        return solver

    def __string2Fun(self,string):
        fun = None
        try: fun = int(string)
        except: fun = gringo.parse_term(string)
        return fun

    def __finalize(self,step):
        rospy.logdebug("Grounding: " + "finalize" + " " + str([step]))
        self.solver.ground([("finalize",[step])])
        for domain in self.solver.domains:
            if domain.is_external and domain.atom.name() != "query" and domain.atom.args()[-1] == step:
                self.solver.release_external(domain.atom)
        self.solver.cleanup_domains()

    def __parse(self,currentStep):
        rospy.logdebug("Parsing step: " + str(currentStep))
        for atom in self.atoms:
            if   str(atom.name()) == "do"       and atom.args()[-1] == currentStep and len(atom.args()) == 3:
                rospy.logdebug("Grounding: " + "commit" + " " + str(atom.args()))
                self.solver.ground([("commit",atom.args())]) ########################
                self.publisher.publish(atom)
                rospy.loginfo("Parsing: " + str(atom))
            elif str(atom.name()) == "abort"    and atom.args()[-1] == currentStep and len(atom.args()) == 2:
                rospy.logdebug("Keyword found: " + str(atom))
                self.publisher.publish_abort(atom)
            elif str(atom.name()) == "status"   and atom.args()[-1] == currentStep and len(atom.args()) == 3:
                rospy.logdebug("Keyword found: " + str(atom))
                self.publisher.add("status",(str(atom.args()[0]),str(atom.args()[1])))

    def __find_plan(self):
        constraints = True
        while(constraints):
            result = self.__solve() ########################
            while(result == gringo.SolveResult.UNSAT and not self.event.isSet()):
                self.__new_horizon()
                result = self.__solve()
            constraints = self.__run_checkers()
        if result == gringo.SolveResult.SAT:
            rospy.logdebug("Stats: " + json.dumps(self.solver.stats, sort_keys=True, indent=4, separators=(',', ': ')))
            self.__parse(self.lastStep)
            currentStep = self.publisher.get_step(self.horizon)
            while(self.lastStep != currentStep):
                self.__finalize(self.lastStep)
                self.__parse(currentStep)
                self.lastStep = currentStep
                currentStep = self.publisher.get_step(self.horizon)

    def __run_checkers(self):
        constraints = []
        for checker in self.checkers:
            if self.event.isSet(): break
            rospy.logdebug("Running: " + str(checker))
            thread = ThreadWithReturnValue(target=checker, args=(self.atoms,self.event,))
            thread.start()
            constraints = thread.join()            
            if constraints != []: break
        for constraint in constraints:
             rospy.logdebug("Grounding: " + "constraints" + " " + str(constraint))
             self.solver.ground([("constraints",[constraint])])
        return constraints != []
 
    def __on_model(self,model):
        self.atoms[:] = model.atoms(gringo.Model.ATOMS)

    def __on_finish(self,result,interrupted):
        if self.condition._is_owned.im_self._RLock__owner is not None:
            rospy.sleep(0.1)
        if self.condition._is_owned.im_self._RLock__owner is None:
            self.condition.acquire()
            self.condition.notify()
            self.condition.release()

    def __solve(self):
        self.atoms = []
        future = self.solver.solve_async(None,self.__on_model,self.__on_finish)
        rospy.loginfo("Clasp: solving")
        self.condition.wait(float("inf"))
        future.cancel()
        result = future.get()
        rospy.loginfo("Clasp Result: " + str(result))
        return result

    def __new_horizon(self):
        self.solver.assign_external(gringo.Fun("query",[self.horizon]),False)
        self.horizon += 1
        rospy.logdebug("Grounding: " + "state" + " " + str([self.horizon]))
        rospy.logdebug("Grounding: " + "transition" + " " + str([self.horizon]))
        rospy.logdebug("Grounding: " + "query" + " " + str([self.horizon]))
        self.solver.ground([("state",[self.horizon]),("transition", [self.horizon]),("query",[self.horizon])])
        rospy.loginfo("Plan Length: " + str(self.horizon))
        self.solver.assign_external(gringo.Fun("query", [self.horizon]), True)

#######################

    def exit(self):
        rospy.logdebug("Exit signal")
        self.event.set()
        self.condition.acquire()
        self.state = ROSoClingoGringoControl.EXIT
        self.condition.notify()
        self.condition.release()

    def get_request_IDs(self):
        ids = []
        for element in self.solver.domains.by_signature("rslot",1):
            ids.append(element.atom.args()[0])
        print("ids = ")
        print(ids)
        return ids

    def add(self,message):
        self.event.set()
        self.condition.acquire()
        self.incoming.append(message)
        if self.state != ROSoClingoGringoControl.EXIT:
            self.state = ROSoClingoGringoControl.EVENT
        self.condition.notify()
        self.condition.release()
