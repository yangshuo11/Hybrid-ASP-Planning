#const c=2. %capacity
#const slots=3. %request slots

rslot(1..slots). %%% rslot(1),rslot(2),rslot(3)
actuator(mailbot).

action(go(W))            :- waypoint(W).
action(sense(ID,L,W))    :- rslot(ID),landmark(L), waypoint(W).

holds(F,0) :- init(F).

#program state(t).

#external event(ID,visit(L,W),t)        : rslot(ID), landmark(L), waypoint(W).
#external event(ID,cancel,t)            : rslot(ID).
#external event(R,failure,t)            : actuator(R).
#external event(R,success,t)            : actuator(R).

status(ID,accepted,t)   :- event(ID,visit(L,W),t).
status(ID,succeeded,t)  :- do(R,sense(ID,L,W),t), event(R,success,t), not holds(cancel(ID),t).

holds(request(ID,L,W),t) :- event(ID,visit(L,W),t), status(ID,accepted,t).

#program transition(t).

{do(R,A,t):action(A)}1 :- actuator(R).
:- do(R,A,t), not possible(R,A,t).

possible(R,go(W),t)             :- holds(at(R,W'),t-1),
                                   connection(W',W).
possible(R,sense(ID,L,W),t)     :- holds(at(R,W),t-1),
                                   holds(request(ID,L,W),t-1),
                                   belong(L,W).

holds(at(R,W),t)                                    :- do(R,go(W),t), not event(R,failure,t).
holds(found(ID,L,W),t)                              :- do(R,sense(ID,L,W),t), not event(R,failure,t).
%%%holds(known(L),t)                                :- do(R,sense(ID,L,W),t), not event(R,failure,t).
%%%holds(unknown(L),t)                              :- not holds(known(L),t).

abnormal(at(R,W'),t)    :- holds(at(R,W'),t-1),do(R,go(_),t),not event(R,failure,t).

abnormal(request(ID,L,W),t) :- holds(request(ID,L,W),t-1),do(R,sense(ID,L,W),t),not event(R,failure,t).

holds(F,t) :- holds(F,t-1), not abnormal(F,t).
:- rslot(ID), 2{holds(found(ID,L,W),t)}.

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




