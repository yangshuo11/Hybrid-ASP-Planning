#const c = 2.
#const slots = 3.

rslot(1..slots).
actuator(mailbot).

action(go(P))        :- waypoint(P).
action(sense(L,A))   :- landmark(L),angle(A).
action(sense(L,P))   :- landmark(L),waypoint(P).

holds(F,0) :- init(F).

#program state(t).

#external event(ID,visit(L),t)          : rslot(ID),landmark(L).
#external event(ID,cancel,t)            : rslot(ID).
#external event(R,failure,t)            : actuator(R).
#external event(R,success,t)            : actuator(R).
%%%#external event(info,unknown(L,A),t)    : landmark(L),angle(A).
%%%#external event(info,known(L,P),t)      : landmark(L),waypoint(P).
%%%#external event(info,at(R,P),t)         : actuator(R),waypoint(P).

status(ID,accepted,t)   :- event(ID,visit(L),t).
status(ID,succeeded,t)  :- rslot(ID),do(R,sense(L,A),t),event(R,success,t).
status(ID,failed,t)     :- rslot(ID),do(R,sense(L,A),t),event(R,failure,t).
status(ID,succeeded,t)  :- rslot(ID),do(R,sense(L,P),t),event(R,success,t).
status(ID,failed,t)     :- rslot(ID),do(R,sense(L,P),t),event(R,failure,t).

holds(request(ID,L),t)  :- event(ID,visit(L),t).

#program transition(t).

{do(R,A,t):action(A)}1 :- actuator(R).
:- do(R,A,t),not possible(R,A,t).

possible(R,go(P),t)         :- holds(at(R,P'),t-1),
                               connection(P',P).

possible(R,sense(L,A),t)    :- holds(request(ID,L),t-1),
                               holds(unknown(L),t-1),
                               angle(A),actuator(R).

%%%possible(R,sense(L,P),t)    :- holds(request(ID,L),t-1),
%%%                               possible(R,go(P),t-1).

holds(at(R,W),t)            :- do(R,go(W),t),not event(R,failure,t).
abnormal(at(R,W'),t)        :- holds(at(R,W'),t-1),do(R,go(_),t),not event(R,failure,t).

holds(unknown(L),t)         :- do(R,sense(ID,L,A),t), event(R,failure,t).
abnormal(unknown(L),t)      :- holds(unknown(L),t-1),holds(known(L,P),t).

holds(known(L,P),t)         :- do(R,sense(ID,L,A),t), event(R,success,t), waypoint(P).
abnormal(known(L,P),t)      :- holds(known(L,P),t-1),holds(unknown(L),t).

%%%holds(unknown(L),t)         :- do(R,sense(ID,L,P),t), event(R,failure,t).
%%%holds(known(L,P),t)         :- do(R,sense(ID,L,P),t), event(R,success,t), waypoint(P).
%%%holds(unknown(L),t)       :- event(info,unknown(L),t), not event(info,known(L),t).
%%%holds(known(L,P),t)       :- event(info,known(L,P),t), not event(info,unknwon(L),t).
%%%holds(unknown(L),t) :- not holds(known(L,P),t),landmark(L),waypoint(P).

holds(F,t) :- holds(F,t-1),not abnormal(F,t).
%%%:- rslot(ID),2{holds(known(L,P),t)}.

#program query(t).
goal(t) :- not holds(request(_,_),t).
:- do(_,go(_),t),goal(t).

firstgoal(M,t) :- M=#min{X:goal(X)},M<=t.
#minimize{X: firstgoal(X,t)}.

#external query(t).
:- not goal(t),query(t).

%%%ROSoClingo
#program commit(r,a,t).
:- not do(r,a,t).
non_idle(r,t).

#program finalize(t).
:- do(R,_,t),not non_idle(R,t).

#program set_event(id,e,t).
event(id,e,t).





