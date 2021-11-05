
#const steps = 20.
start(0).
end(steps).
time_step(S..E) :- start(S), end(E).
next(X,X+1) :- time_step(X), time_step(X+1).

external(result(sat)).
external(result(unsat)).

save(meta(g,G)) :- time_step(G).
save(situation(grounded)).
save(situation(released_step)).
save(situation(released_query)).
save(situation(ready)).
save(situation(solved)).
save(situation(done)).

#external event(info,E,0) : external(E).
#external event(save,F,0) : save(F).

holds(val(F,true),0) :- event(info,F,0).
holds(val(F,true),0) :- event(save,F,0).

#show.
#show holds(val(F,true),1) : holds(val(F,true),1), save(F).
%#show event(save,F,0) : holds(val(F,true),1), save(F).
#show do(controller,initialize,1)                       : do(act(initialize),1).
#show do(controller,release_step(G),1)                  : do(act(release_step),1),          holds(val(meta(g,G),true),0).
#show do(controller,release_external(query(G)),1)       : do(act(release_external),1),      holds(val(meta(g,G),true),0).
#show do(controller,ground(G+1),1)                      : do(act(ground),1),                holds(val(meta(g,G),true),0).
#show do(controller,assign_external(query(G),true),1)   : do(act(assign_external(true)),1), holds(val(meta(g,G),true),0).
#show do(controller,solve,1)                            : do(act(solve),1).
#show do(controller,publish,1)                          : do(act(publish),1).
#show do(controller,exit,1)                             : do(act(exit),1).



step(0..1).
{holds(F,0)} :- initially(F).
{do(A,T) : action(A)} :- step(T), T > 0.

holds(A,T-1)        :-  do(A,T).
holds(F,T)          :-  static_law(ID), head(ID,F), step(T),
                        holds(C,T) : if(ID,C);
                        not not holds(C,T) : ifcons(ID,C).
holds(F,T)          :-  dynamic_law(ID), head(ID,F), step(T),
                        holds(A,T-1) : after(ID,A);
                        not not holds(C,T) : ifcons(ID,C).
{holds(val(F,V),T)} :-  inertial(F), step(T), holds(val(F,V),T-1).
{holds(F,T)}        :-  default(F), step(T).

:- fluent(F), step(T), not 1{holds(val(F,_),T)}1.
:- impossible(ID,_), step(T), holds(C,T) : impossible(ID,C).
:- nonexecutable(ID,_), step(T), holds(A,T-1) : nonexecutable(ID,A); holds(C,T) : nonexecutable_if(ID,C).

action(act(initialize)).
action(act(release_step)).
action(act(release_external)).
action(act(ground)).
action(act(assign_external(true))).
action(act(solve)).
action(act(publish)).
action(act(exit)).
fluent(F) :- external(F).
fluent(F) :- save(F).
fluent(possible(A)) :- action(act(A)).
fluent(situation(initialize)).
fluent(executing).
domain(executing,true). domain(executing,false). 
domain(situation(initialize),true). domain(situation(initialize),false). 
domain(possible(A),true) :- action(act(A)). domain(possible(A),false) :- action(act(A)). 
domain(F,true) :- external(F). domain(F,false) :- external(F). domain(F,true) :- save(F). domain(F,false) :- save(F). 

static_law(law(1,F)) :- save(situation(F)).
head(law(1,F),val(situation(initialize),false)) :- save(situation(F)).
if(law(1,F),val(situation(F),true)) :- save(situation(F)).
static_law(law(2,A)) :- action(act(A)).
head(law(2,A),val(executing,true)) :- action(act(A)).
if(law(2,A),act(A)) :- action(act(A)).
static_law(law(3)).
head(law(3),val(possible(initialize),true)).
if(law(3),val(situation(initialize),true)).
static_law(law(4)).
head(law(4),val(possible(release_step),true)).
if(law(4),val(situation(grounded),true)).
static_law(law(5,G)) :- time_step(G),end(E),G<E.
head(law(5,G),val(possible(release_external),true)) :- time_step(G),end(E),G<E.
if(law(5,G),val(situation(released_step),true)) :- time_step(G),end(E),G<E.
if(law(5,G),val(meta(g,G),true)) :- time_step(G),end(E),G<E.
static_law(law(6)).
head(law(6),val(possible(ground),true)).
if(law(6),val(situation(released_query),true)).
static_law(law(7,G)) :- time_step(G),end(E),G=E.
head(law(7,G),val(possible(assign_external(true)),true)) :- time_step(G),end(E),G=E.
if(law(7,G),val(situation(released_step),true)) :- time_step(G),end(E),G=E.
if(law(7,G),val(meta(g,G),true)) :- time_step(G),end(E),G=E.
static_law(law(8)).
head(law(8),val(possible(solve),true)).
if(law(8),val(situation(ready),true)).
static_law(law(9)).
head(law(9),val(possible(publish),true)).
if(law(9),val(situation(solved),true)).
if(law(9),val(result(sat),true)).
static_law(law(10)).
head(law(10),val(possible(exit),true)).
if(law(10),val(situation(solved),true)).
if(law(10),val(result(unsat),true)).

dynamic_law(law(11)).
head(law(11),val(situation(grounded),true)).
after(law(11),act(initialize)).
dynamic_law(law(12)).
head(law(12),val(situation(initialize),false)).
after(law(12),act(initialize)).
dynamic_law(law(13)).
head(law(13),val(meta(g,0),true)).
after(law(13),act(initialize)).
dynamic_law(law(14)).
head(law(14),val(situation(released_step),true)).
after(law(14),act(release_step)).
dynamic_law(law(15)).
head(law(15),val(situation(grounded),false)).
after(law(15),act(release_step)).
dynamic_law(law(16)).
head(law(16),val(situation(released_query),true)).
after(law(16),act(release_external)).
dynamic_law(law(17)).
head(law(17),val(situation(released_step),false)).
after(law(17),act(release_external)).
dynamic_law(law(18)).
head(law(18),val(situation(grounded),true)).
after(law(18),act(ground)).
dynamic_law(law(19)).
head(law(19),val(situation(released_query),false)).
after(law(19),act(ground)).
dynamic_law(law(20,G2,G1)) :- next(G1,G2).
head(law(20,G2,G1),val(meta(g,G2),true)) :- next(G1,G2).
after(law(20,G2,G1),act(ground)) :- next(G1,G2).
after(law(20,G2,G1),val(meta(g,G1),true)) :- next(G1,G2).
dynamic_law(law(21,G)) :- time_step(G).
head(law(21,G),val(meta(g,G),false)) :- time_step(G).
after(law(21,G),act(ground)) :- time_step(G).
after(law(21,G),val(meta(g,G),true)) :- time_step(G).
dynamic_law(law(22)).
head(law(22),val(situation(ready),true)).
after(law(22),act(assign_external(true))).
dynamic_law(law(23)).
head(law(23),val(situation(released_step),false)).
after(law(23),act(assign_external(true))).
dynamic_law(law(24)).
head(law(24),val(situation(solved),true)).
after(law(24),act(solve)).
dynamic_law(law(25)).
head(law(25),val(situation(ready),false)).
after(law(25),act(solve)).
dynamic_law(law(26)).
head(law(26),val(situation(done),true)).
after(law(26),act(exit)).
dynamic_law(law(27)).
head(law(27),val(situation(solved),false)).
after(law(27),act(exit)).

nonexecutable(law(28),val(executing,false)).
nonexecutable(law(29,A),act(A)) :- action(act(A)).
nonexecutable(law(29,A),val(possible(A),false)) :- action(act(A)).
default(val(possible(A),false)) :- action(act(A)).
default(val(state(S),false)) :- fluent(state(S)).
default(val(F,false)) :- external(F).
default(val(situation(initialize),true)).
default(val(executing,false)).
inertial(F) :- save(F).

initially(val(F,false)) :- save(F).

