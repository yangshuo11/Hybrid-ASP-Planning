corridor(c0).
corridor(c1).
corridor(c2).
corridor(c3).

office(o0).
office(o1).
office(o2).
office(o3).
office(o4).
office(o5).


viewpoint(v0).
viewpoint(v1).
viewpoint(v2).
viewpoint(v3).

landmark(red).
landmark(blue).

angle(a1).
angle(a2).
angle(a3).
angle(a4).

waypoint(X) :- corridor(X).
waypoint(X) :- office(X).
waypoint(X) :- landmark(X).
waypoint(X) :- viewpoint(X).


within(v0,o0).
within(v1,o0).
within(v2,o0).
within(v3,o0).

belong(X,Y) :- landmark(X),office(Y).
observableTransition(L,V) :- landmark(L),viewpoint(V).
observableRotation(L,A) :- landmark(L),angle(A).
connection(X,Y) :- connection(Y,X).

observableTransition(red,v0).
%%%observableTransition(red,v1).
%%%observableTransition(red,v2).
%%%observableTransition(red,v3).

observableRotation(red,a1).
observableRotation(red,a2).


observableRotation(red,a2) :- not observableRotation(red,a1).

%%%observableRotation(red,a3).
%%%observableRotation(red,a4).

belong(red,o0).
connection(c0, o0).
connection(c0, o1).
connection(c1, o0).
connection(c1, o2).
connection(c2, o2).
connection(c2, o4).
connection(c3, o3).
connection(c3, o4).
connection(c4, o4).
connection(c4, o5).

init(at(mailbot,o0)).
init(observableRotation(red,a1)).
%%%init(observableRotation(red,a2)).
