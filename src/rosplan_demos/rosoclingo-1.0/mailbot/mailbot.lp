#const c=2.   % capacity
#const slots=3.   % request slots

rslot(1..slots). % fact
actuator(mailbot). % fact

action(go(W))       :- waypoint(W).
action(pickup(ID))  :- rslot(ID).
action(deliver(ID)) :- rslot(ID).

holds(F,0) :- init(F).

#program state(t).

#external event(ID,bring(O,P),t)		: rslot(ID), office(O), office(P).
#external event(ID,cancel,t)			: rslot(ID).
#external event(info,blocked(W,W'),t)	: connection(W,W').
#external event(info,unblocked(W,W'),t) : connection(W,W').
#external event(R,failure,t)            : actuator(R).
#external event(R,success,t)            : actuator(R).

status(ID,accepted,t)                     :- event(ID,bring(O,P),t), not status(ID,rejected,t).
status(ID,rejected,t)                     :- event(ID,bring(O,O),t).
status(ID,succeeded,t)                    :- do(R,deliver(ID),t), event(R,success,t), not holds(cancel(ID),t).
status(ID,canceled,t)                     :- do(R,deliver(ID),t), event(R,success,t), holds(cancel(ID),t).

holds(request(ID,O,P),t)                :- event(ID,bring(O,P),t), status(ID,accepted,t).
holds(cancel(ID),t)                     :- event(ID,cancel,t), not status(ID,succeeded,t).

#program transition(t).

{ do(R,A,t):action(A) } 1   :-  actuator(R).
:- do(R,A,t), not possible(R,A,t).

possible(R,go(W),t)         :-  holds(at(R,W'),t-1),
                                connection(W',W),
                                not holds(blocked(W',W),t-1).

possible(R,pickup(ID),t)	:-  holds(at(R,O),t-1),
                                holds(request(ID,O,_),t-1),
                                not c {holds(carry(R,_),t-1)}.

possible(R,deliver(ID),t)	:-	holds(at(R,P),t-1),
                                holds(carry(R,ID),t-1),
            					holds(request(ID,_,P),t-1),
            					not holds(cancel(ID),t-1).

possible(R,deliver(ID),t)	:-	holds(at(R,O),t-1),
            					holds(carry(R,ID),t-1),
			            		holds(request(ID,O,_),t-1),
			            		holds(cancel(ID),t-1).


holds(at(R,W),t)		    :-	do(R,go(W),t), not event(R,failure,t).

holds(carry(R,ID),t)		:-	do(R,pickup(ID),t), not event(R,failure,t).

abnormal(at(R,W'),t)		:-	holds(at(R,W'),t-1), do(R,go(_),t), not event(R,failure,t).
abnormal(carry(R,ID),t)		:-	holds(carry(R,ID),t-1), do(R,deliver(ID),t), not event(R,failure,t).


abnormal(request(ID,O,P),t)	:-	holds(request(ID,O,P),t-1),
		            			do(R,deliver(ID),t),
					            not event(R,failure,t).

abnormal(cancel(ID),t)	    :-	holds(cancel(ID),t-1),
					            do(R,deliver(ID),t),
					            not event(R,failure,t).

holds(blocked(W',W),t) :- do(R,go(W),t), event(R,failure,t), holds(at(R,W'),t-1).
holds(blocked(W,W'),t) :- event(info,blocked(W,W'),t), not event(info,unblocked(W,W'),t), not event(info,unblocked(W',W),t).
holds(blocked(W',W),t) :- event(info,blocked(W,W'),t), not event(info,unblocked(W,W'),t), not event(info,unblocked(W',W),t).

abnormal(blocked(W,W'),t) :- holds(blocked(W,W'),t-1), event(info,unblocked(W,W'),t).
abnormal(blocked(W',W),t) :- holds(blocked(W',W),t-1), event(info,unblocked(W,W'),t).

holds(F,t) :- holds(F,t-1), not abnormal(F,t).

:- rslot(ID), 2 {holds(carry(R,ID),t)}. %%% 2{holds(carry(R,ID1),t),holds(carry(R,ID2),t)}

#program query(t).

goal(t) :- not holds(request(_,_,_),t).

:- do(_,go(_),t), goal(t).

firstgoal(M,t) :- M=#min{ X: goal(X) }, M<=t. 
#minimize{ X: firstgoal(X,t) }.

#external query(t).

:- not goal(t), query(t).

%%% ROSoClingo

#program commit(r,a,t). % maybe with scripting
:- not do(r,a,t).
non_idle(r,t).

#program finalize(t).
:- do(R,_,t), not non_idle(R,t).

#program set_event(id,e,t). % needed
event(id,e,t).
