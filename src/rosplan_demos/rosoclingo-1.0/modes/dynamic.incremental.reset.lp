
#const steps = 20.
#const r = 5.
start(0).
end(steps).
reset_horizon(r).
time_step(S..E) :- start(S), end(E).
next(X,X+1) :- time_step(X), time_step(X+1).
minus(X,X-R) :- time_step(X), reset_horizon(R), X-R >= 0.
minus(X,0) :- time_step(X), reset_horizon(R), X-R < 0.

external(result(sat)).
external(result(unsat)).
external(pending_actions).
external(new_messages).

save(meta(g,G)) :- time_step(G).
save(meta(c,C)) :- time_step(C).
save(meta(h,H)) :- time_step(H).
save(situation(grounded)).
save(situation(released_step)).
save(situation(no_query)).
save(situation(ready)).
save(situation(solved)).
save(situation(done)).
save(situation(updated)).
save(situation(committed)).
save(situation(reseted)).

#external event(info,E,0) : external(E).
#external event(save,F,0) : save(F).

holds(val(F,true),0) :- event(info,F,0).
holds(val(F,true),0) :- event(save,F,0).

#show.
#show holds(val(F,true),1) : holds(val(F,true),1), save(F).
%#show event(save,F,0) : holds(val(F,true),1), save(F).
#show do(controller,initialize,1)                       : do(act(initialize),1).
#show do(controller,release_step(G),1)                  : do(act(release_step),1),          holds(val(meta(g,G),true),0).
#show do(controller,ground(G+1),1)                      : do(act(ground),1),                holds(val(meta(g,G),true),0).
#show do(controller,assign_external(query(H),Value),1)  : do(act(assign_external(Value)),1), holds(val(meta(h,H),true),0).
#show do(controller,solve,1)                            : do(act(solve),1).
#show do(controller,exit,1)                             : do(act(exit),1).
#show do(controller,status_update(C),1)                 : do(act(status_update),1),         holds(val(meta(c,C),true),0), holds(val(situation(solved),true),0).
#show do(controller,status_update(C+1),1)               : do(act(status_update),1),         holds(val(meta(c,C),true),0), holds(val(situation(committed),true),0).
#show do(controller,commit_actions(C),1)                : do(act(commit_actions),1),        holds(val(meta(c,C),true),0).
#show do(controller,assign_messages(C),1)               : do(act(assign_messages),1),       holds(val(meta(c,C),true),0).
#show do(controller,reset(C-1),1)                       : do(act(reset),1),                 holds(val(meta(c,C),true),0).
#show do(controller,idle,1)                             : do(act(idle),1).


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
action(act(ground)).
action(act(assign_external(true))).
action(act(assign_external(false))).
action(act(solve)).
action(act(exit)).
action(act(status_update)).
action(act(commit_actions)).
action(act(assign_messages)).
action(act(reset)).
action(act(idle)).
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
static_law(law(4,G)) :- time_step(G),reset_horizon(R),G<R.
head(law(4,G),val(possible(assign_external(true)),true)) :- time_step(G),reset_horizon(R),G<R.
if(law(4,G),val(situation(grounded),true)) :- time_step(G),reset_horizon(R),G<R.
if(law(4,G),val(meta(g,G),true)) :- time_step(G),reset_horizon(R),G<R.
static_law(law(5)).
head(law(5),val(possible(assign_external(true)),true)).
if(law(5),val(situation(released_step),true)).
static_law(law(6,H,G)) :- time_step(H),time_step(G),H<G.
head(law(6,H,G),val(possible(assign_external(true)),true)) :- time_step(H),time_step(G),H<G.
if(law(6,H,G),val(situation(no_query),true)) :- time_step(H),time_step(G),H<G.
if(law(6,H,G),val(meta(h,H),true)) :- time_step(H),time_step(G),H<G.
if(law(6,H,G),val(meta(g,G),true)) :- time_step(H),time_step(G),H<G.
static_law(law(7,G)) :- time_step(G),reset_horizon(R),G>=R.
head(law(7,G),val(possible(release_step),true)) :- time_step(G),reset_horizon(R),G>=R.
if(law(7,G),val(situation(grounded),true)) :- time_step(G),reset_horizon(R),G>=R.
if(law(7,G),val(meta(g,G),true)) :- time_step(G),reset_horizon(R),G>=R.
static_law(law(8)).
head(law(8),val(possible(solve),true)).
if(law(8),val(situation(ready),true)).
static_law(law(9)).
head(law(9),val(possible(assign_messages),true)).
if(law(9),val(situation(solved),true)).
if(law(9),val(new_messages,true)).
static_law(law(10)).
head(law(10),val(possible(assign_messages),true)).
if(law(10),val(situation(committed),true)).
if(law(10),val(new_messages,true)).
static_law(law(11,H,C)) :- time_step(C),time_step(H),end(E),H=E,C=0.
head(law(11,H,C),val(possible(exit),true)) :- time_step(C),time_step(H),end(E),H=E,C=0.
if(law(11,H,C),val(situation(solved),true)) :- time_step(C),time_step(H),end(E),H=E,C=0.
if(law(11,H,C),val(result(unsat),true)) :- time_step(C),time_step(H),end(E),H=E,C=0.
if(law(11,H,C),val(new_messages,false)) :- time_step(C),time_step(H),end(E),H=E,C=0.
if(law(11,H,C),val(meta(c,C),true)) :- time_step(C),time_step(H),end(E),H=E,C=0.
if(law(11,H,C),val(meta(h,H),true)) :- time_step(C),time_step(H),end(E),H=E,C=0.
static_law(law(12,H,C)) :- time_step(C),time_step(H),end(E),H=E,C>0.
head(law(12,H,C),val(possible(reset),true)) :- time_step(C),time_step(H),end(E),H=E,C>0.
if(law(12,H,C),val(situation(solved),true)) :- time_step(C),time_step(H),end(E),H=E,C>0.
if(law(12,H,C),val(result(unsat),true)) :- time_step(C),time_step(H),end(E),H=E,C>0.
if(law(12,H,C),val(new_messages,false)) :- time_step(C),time_step(H),end(E),H=E,C>0.
if(law(12,H,C),val(meta(c,C),true)) :- time_step(C),time_step(H),end(E),H=E,C>0.
if(law(12,H,C),val(meta(h,H),true)) :- time_step(C),time_step(H),end(E),H=E,C>0.
static_law(law(13,C)) :- time_step(C),reset_horizon(R),C=R.
head(law(13,C),val(possible(reset),true)) :- time_step(C),reset_horizon(R),C=R.
if(law(13,C),val(situation(updated),true)) :- time_step(C),reset_horizon(R),C=R.
if(law(13,C),val(meta(c,C),true)) :- time_step(C),reset_horizon(R),C=R.
static_law(law(14,H)) :- time_step(H),end(E),H<E.
head(law(14,H),val(possible(assign_external(false)),true)) :- time_step(H),end(E),H<E.
if(law(14,H),val(situation(solved),true)) :- time_step(H),end(E),H<E.
if(law(14,H),val(result(unsat),true)) :- time_step(H),end(E),H<E.
if(law(14,H),val(new_messages,false)) :- time_step(H),end(E),H<E.
if(law(14,H),val(meta(h,H),true)) :- time_step(H),end(E),H<E.
static_law(law(15)).
head(law(15),val(possible(assign_external(false)),true)).
if(law(15),val(situation(reseted),true)).
static_law(law(16,H,G)) :- time_step(H),time_step(G),H>=G.
head(law(16,H,G),val(possible(ground),true)) :- time_step(H),time_step(G),H>=G.
if(law(16,H,G),val(situation(no_query),true)) :- time_step(H),time_step(G),H>=G.
if(law(16,H,G),val(meta(h,H),true)) :- time_step(H),time_step(G),H>=G.
if(law(16,H,G),val(meta(g,G),true)) :- time_step(H),time_step(G),H>=G.
static_law(law(17)).
head(law(17),val(possible(status_update),true)).
if(law(17),val(situation(solved),true)).
if(law(17),val(result(sat),true)).
if(law(17),val(new_messages,false)).
static_law(law(18,H,C)) :- time_step(C),time_step(H),C<H.
head(law(18,H,C),val(possible(status_update),true)) :- time_step(C),time_step(H),C<H.
if(law(18,H,C),val(situation(committed),true)) :- time_step(C),time_step(H),C<H.
if(law(18,H,C),val(pending_actions,false)) :- time_step(C),time_step(H),C<H.
if(law(18,H,C),val(meta(c,C),true)) :- time_step(C),time_step(H),C<H.
if(law(18,H,C),val(meta(h,H),true)) :- time_step(C),time_step(H),C<H.
static_law(law(19,C)) :- time_step(C),reset_horizon(R),C<R.
head(law(19,C),val(possible(commit_actions),true)) :- time_step(C),reset_horizon(R),C<R.
if(law(19,C),val(situation(updated),true)) :- time_step(C),reset_horizon(R),C<R.
if(law(19,C),val(meta(c,C),true)) :- time_step(C),reset_horizon(R),C<R.
static_law(law(20)).
head(law(20),val(possible(idle),true)).
if(law(20),val(situation(committed),true)).
if(law(20),val(pending_actions,true)).
if(law(20),val(new_messages,false)).
static_law(law(21,H,C)) :- time_step(C),time_step(H),C=H.
head(law(21,H,C),val(possible(idle),true)) :- time_step(C),time_step(H),C=H.
if(law(21,H,C),val(situation(committed),true)) :- time_step(C),time_step(H),C=H.
if(law(21,H,C),val(pending_actions,false)) :- time_step(C),time_step(H),C=H.
if(law(21,H,C),val(new_messages,false)) :- time_step(C),time_step(H),C=H.
if(law(21,H,C),val(meta(c,C),true)) :- time_step(C),time_step(H),C=H.
if(law(21,H,C),val(meta(h,H),true)) :- time_step(C),time_step(H),C=H.

dynamic_law(law(22)).
head(law(22),val(situation(grounded),true)).
after(law(22),act(initialize)).
dynamic_law(law(23)).
head(law(23),val(situation(initialize),false)).
after(law(23),act(initialize)).
dynamic_law(law(24)).
head(law(24),val(meta(g,0),true)).
after(law(24),act(initialize)).
dynamic_law(law(25)).
head(law(25),val(meta(c,0),true)).
after(law(25),act(initialize)).
dynamic_law(law(26)).
head(law(26),val(meta(h,0),true)).
after(law(26),act(initialize)).
dynamic_law(law(27)).
head(law(27),val(situation(ready),true)).
after(law(27),act(assign_external(true))).
dynamic_law(law(28)).
head(law(28),val(situation(grounded),false)).
after(law(28),act(assign_external(true))).
dynamic_law(law(29)).
head(law(29),val(situation(released_step),false)).
after(law(29),act(assign_external(true))).
dynamic_law(law(30)).
head(law(30),val(situation(no_query),false)).
after(law(30),act(assign_external(true))).
dynamic_law(law(31)).
head(law(31),val(situation(no_query),true)).
after(law(31),act(assign_external(false))).
dynamic_law(law(32)).
head(law(32),val(situation(solved),false)).
after(law(32),act(assign_external(false))).
dynamic_law(law(33)).
head(law(33),val(situation(reseted),false)).
after(law(33),act(assign_external(false))).
dynamic_law(law(34,H2,H1)) :- next(H1,H2).
head(law(34,H2,H1),val(meta(h,H2),true)) :- next(H1,H2).
after(law(34,H2,H1),act(assign_external(false))) :- next(H1,H2).
after(law(34,H2,H1),val(situation(solved),true)) :- next(H1,H2).
after(law(34,H2,H1),val(meta(h,H1),true)) :- next(H1,H2).
dynamic_law(law(35,H2,H1)) :- minus(H1,H2).
head(law(35,H2,H1),val(meta(h,H2),true)) :- minus(H1,H2).
after(law(35,H2,H1),act(assign_external(false))) :- minus(H1,H2).
after(law(35,H2,H1),val(situation(reseted),true)) :- minus(H1,H2).
after(law(35,H2,H1),val(meta(h,H1),true)) :- minus(H1,H2).
dynamic_law(law(36,H)) :- time_step(H).
head(law(36,H),val(meta(h,H),false)) :- time_step(H).
after(law(36,H),act(assign_external(false))) :- time_step(H).
after(law(36,H),val(meta(h,H),true)) :- time_step(H).
dynamic_law(law(37)).
head(law(37),val(situation(solved),true)).
after(law(37),act(solve)).
dynamic_law(law(38)).
head(law(38),val(situation(ready),false)).
after(law(38),act(solve)).
dynamic_law(law(39)).
head(law(39),val(situation(ready),true)).
after(law(39),act(assign_messages)).
dynamic_law(law(40)).
head(law(40),val(situation(solved),false)).
after(law(40),act(assign_messages)).
dynamic_law(law(41)).
head(law(41),val(situation(committed),false)).
after(law(41),act(assign_messages)).
dynamic_law(law(42)).
head(law(42),val(situation(done),true)).
after(law(42),act(exit)).
dynamic_law(law(43)).
head(law(43),val(situation(solved),false)).
after(law(43),act(exit)).
dynamic_law(law(44)).
head(law(44),val(situation(grounded),true)).
after(law(44),act(ground)).
dynamic_law(law(45)).
head(law(45),val(situation(no_query),false)).
after(law(45),act(ground)).
dynamic_law(law(46,G2,G1)) :- next(G1,G2).
head(law(46,G2,G1),val(meta(g,G2),true)) :- next(G1,G2).
after(law(46,G2,G1),act(ground)) :- next(G1,G2).
after(law(46,G2,G1),val(meta(g,G1),true)) :- next(G1,G2).
dynamic_law(law(47,G)) :- time_step(G).
head(law(47,G),val(meta(g,G),false)) :- time_step(G).
after(law(47,G),act(ground)) :- time_step(G).
after(law(47,G),val(meta(g,G),true)) :- time_step(G).
dynamic_law(law(48)).
head(law(48),val(situation(updated),true)).
after(law(48),act(status_update)).
dynamic_law(law(49)).
head(law(49),val(situation(solved),false)).
after(law(49),act(status_update)).
dynamic_law(law(50)).
head(law(50),val(situation(committed),false)).
after(law(50),act(status_update)).
dynamic_law(law(51,C2,C1)) :- next(C1,C2).
head(law(51,C2,C1),val(meta(c,C2),true)) :- next(C1,C2).
after(law(51,C2,C1),act(status_update)) :- next(C1,C2).
after(law(51,C2,C1),val(situation(committed),true)) :- next(C1,C2).
after(law(51,C2,C1),val(meta(c,C1),true)) :- next(C1,C2).
dynamic_law(law(52,C)) :- time_step(C).
head(law(52,C),val(meta(c,C),false)) :- time_step(C).
after(law(52,C),act(status_update)) :- time_step(C).
after(law(52,C),val(situation(committed),true)) :- time_step(C).
after(law(52,C),val(meta(c,C),true)) :- time_step(C).
dynamic_law(law(53)).
head(law(53),val(situation(committed),true)).
after(law(53),act(commit_actions)).
dynamic_law(law(54)).
head(law(54),val(situation(updated),false)).
after(law(54),act(commit_actions)).
dynamic_law(law(55)).
head(law(55),val(situation(released_step),true)).
after(law(55),act(release_step)).
dynamic_law(law(56)).
head(law(56),val(situation(grounded),false)).
after(law(56),act(release_step)).
dynamic_law(law(57)).
head(law(57),val(situation(reseted),true)).
after(law(57),act(reset)).
after(law(57),val(situation(updated),true)).
dynamic_law(law(58)).
head(law(58),val(situation(ready),true)).
after(law(58),act(reset)).
after(law(58),val(situation(solved),true)).
dynamic_law(law(59)).
head(law(59),val(meta(c,0),true)).
after(law(59),act(reset)).
dynamic_law(law(60,C)) :- time_step(C).
head(law(60,C),val(meta(c,C),false)) :- time_step(C).
after(law(60,C),act(reset)) :- time_step(C).
after(law(60,C),val(meta(c,C),true)) :- time_step(C).
dynamic_law(law(61)).
head(law(61),val(situation(updated),false)).
after(law(61),act(reset)).
after(law(61),val(situation(updated),true)).
dynamic_law(law(62)).
head(law(62),val(situation(solved),false)).
after(law(62),act(reset)).
after(law(62),val(situation(solved),true)).

nonexecutable(law(63),val(executing,false)).
nonexecutable(law(64,A),act(A)) :- action(act(A)).
nonexecutable(law(64,A),val(possible(A),false)) :- action(act(A)).
default(val(possible(A),false)) :- action(act(A)).
default(val(state(S),false)) :- fluent(state(S)).
default(val(F,false)) :- external(F).
default(val(situation(initialize),true)).
default(val(executing,false)).
inertial(F) :- save(F).

initially(val(F,false)) :- save(F).

