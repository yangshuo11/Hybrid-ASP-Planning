#const c = 2.
#const slots = 50.

rslot(1..slots).
actuator(mailbot).

action(go(W))                         :- office(W).
action(senseTransition(ID,L,V,W))     :- rslot(ID),landmark(L),viewpoint(V),office(W).
action(senseRotation(ID,L,A,W))       :- rslot(ID),landmark(L),angle(A),office(W).


holds(F,0) :- init(F).

#program state(t).

%%%#external event(ID,possibleOrientation(L,A),t)     : rslot(ID),landmark(L),angle(A).
#external event(ID,visit(L,W),t)                   : rslot(ID),landmark(L),office(W).
#external event(ID,cancel,t)                       : rslot(ID).
#external event(R,failure,t)                       : actuator(R).
#external event(R,success,t)                       : actuator(R).

%%% status(ID,accepted,t)           :- event(ID,objectAt(L,P),t).  update a knowledge into ASP while robot moving and detecting the target
%%% holds(objectAt(red,P),t)        :- event(ID,objectAt(red,P),t), status(ID,accepted,t). %%% update this knowledge as a dynamic state


status(ID,accepted,t)           :- event(ID,visit(L,W),t).  %%% when receive an event of visit red, first reason an angle value based on the dynamic knowledge
status(ID,succeeded,t)          :- do(R,senseTransition(ID,L,V,W),t),event(R,success,t),not holds(cancel(ID),t).
status(ID,succeeded,t)          :- do(R,senseRotation(ID,L,A,W),t),event(R,success,t),not holds(cancel(ID),t).
holds(request(ID,L,W),t)        :- event(ID,visit(L,W),t),status(ID,accepted,t).
%%%not observableRotation(L,A)     :- event(ID,failed(L,A),t).
%%%holds(request(ID,L,W),t)        :- event(ID,failed(L,A),t), not observableRotation(L,A), office(W).

%%%holds(observableRotation(L,A),t)     :- event(ID,possibleOrientation(L,A),t).

#program transition(t).
{do(R,A,t):action(A)}1 :- actuator(R).
:- do(R,A,t),not possible(R,A,t).

possible(R,go(W),t)         :- holds(at(R,W'),t-1),
                               connection(W',W).


possible(R,senseTransition(ID,L,V,W),t)   :-    holds(at(R,W),t-1),
                                                holds(request(ID,L,W),t-1),
                                                belong(L,W),
                                                within(V,W),
                                                observableTransition(L,V),
                                                holds(found(ID,L,W),t-1).

possible(R,senseRotation(ID,L,A,W),t)        :-   holds(at(R,W),t-1),
                                                  holds(request(ID,L,W),t-1),
                                                  belong(L,W),
                                                  observableRotation(L,A),
                                                  not holds(notFound(ID,L,A,W),t-1).

holds(at(R,W),t) :- do(R,go(W),t),not event(R,failure,t).
abnormal(at(R,W'),t) :- holds(at(R,W'),t-1),do(R,go(_),t),not event(R,failure,t).

holds(found(ID,L,W),t)                            :- do(R,senseRotation(ID,L,A,W),t),not event(R,failure,t).
holds(notFound(ID,L,A,W),t)                         :- do(R,senseRotation(ID,L,A,W),t),not holds(found(ID,L,A,W),t-1),event(R,failure,t).


abnormal(request(ID,L,W),t) :- holds(request(ID,L,W),t-1),do(R,senseTransition(ID,L,V,W),t),not event(R,failure,t).

%%%abnormal(observableRotation(L,A),t)     :- event(ID,possibleOrientation(L,A'),t), holds(observableRotation(L,A),t-1).


holds(F,t) :- holds(F,t-1), not abnormal(F,t).
:- rslot(ID), 2{holds(found(ID,L,V,W),t)}.

#program query(t).
goal(t) :- not holds(request(_,_,_),t).
:- do(_,go(_),t),goal(t).

firstgoal(M,t) :- M=#min{X:goal(X)},M<=t.
#minimize{X: firstgoal(X,t)}.

#external query(t).
:- not goal(t),query(t).


%%% ROSoClingo
#program commit(r,a,t). % maybe wirh scripting
:- not do(r,a,t).
non_idle(r,t).

#program finalize(t).
:- do(R,_,t), not non_idle(R,t).

#program set_event(id,e,t). % needed
event(id,e,t).

