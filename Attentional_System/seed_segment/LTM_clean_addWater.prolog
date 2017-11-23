:-op(500,xfy,.).
:-op(600,fx,-).
:-['Map2Memory.prolog'].

:- dynamic schema/2.
:- dynamic goal/2.

%Server
getSemantics(X):-
	yield([],_),
	schema(X,SemDef),
	yield(SemDef,_),!.

getGoal(X):-
	yield([],_),
	goal(X,Goal),
	yield(Goal,_),!.

getConstraint(X):-
	yield([],_),
	constraint(X,Constraint),
	yield(Constraint,_),!.

getEffect(X):-
	yield([],_),
	effect(X,Effect),
	yield(Effect,_),!.

getSchemaFromGoal(X):-
	yield([],_),
	goal(Schema,Goal),
	subset(X,Goal),
	yield(Schema,_),!.

getSchemaFromEffect(X):-
	yield([],_),
	effect(Schema,Effect),
	subset(X,Effect),
	yield(Schema,_),!.

%schema: nome,[[subschema1(Param),importanza,[releaserList]], [subschema2...]...]
schema(installBracket(Obj,Sup),[
	[get(Obj),1,["TRUE"]],
    [give(Obj,human),1,[hand.Obj]],
    [attachbracket(Obj,Sup),1,[hand.Obj]]
	]).
    
schema(get(Obj),[
	[take(Obj),1,["TRUE"]],
    [receive(human,Obj),1,["TRUE"]]
	]).

%schemi ASTRATTI
%schema(wanderUntil(X),[[wander,1,[X.asd]], [avoid,1,[t]]]).
schema(alive,[
	[inputStream,1,["TRUE"]],
	[rosStream,1,["TRUE"]],
        %[show(alive),1,["TRUE"]],
	[requestStream,1,["TRUE"]],
	%[simulationscene,1,["TRUE"]],
	%[scene,1,["TRUE"]],
	%[plan_listener,1,["TRUE"]],
	%[rodyman,1,["TRUE"]]]).
	[tumkuka,1,["TRUE"]],
	[preparecoffee,1,["TRUE"]],
	[preparetea,1,["TRUE"]],
	%[prepareboth,1,["TRUE"]],
	[show(preparecoffee),1,["TRUE"]],
	[saphari,1,["TRUE"]]]).

schema(prepareboth,[
	[preparecoffee,1,["TRUE"]],
	[preparetea,1,["TRUE"]]]).

schema(goto(X,Y),[
	[avoid,1,["TRUE"]],
	[gotoxy(X,Y),1,["TRUE"]]]).

schema( giveTo(Obj, Sup, human), [
	[give(Obj, human), 1, ["TRUE"]],
	[go(Sup), 1, ["TRUE"]],
	[place(Obj, Sup), 1, ["TRUE"]]]).


schema( getObj(Obj), [
	[take(Obj), 1, ["TRUE"]],
	[search(location1),1,["TRUE"]] 
	]).

schema( search(_), [
	[go(location1),1,["TRUE"]]
	]).

%Schemi CONCRETI (ie. behaviour)
%%Saphari domain

schema(take(_),[]).			%%prende dal tavolo
schema(give(_,_),[]).		%%da all'uomo
schema(go(_),[]).
schema(receive(_,_),[]).
schema(place(_,_), []).
schema(clean(_), []).
schema(glue(_,_),[]).
schema(attachbracket(_,_),[]).
schema(pointTo(_),[]).
schema(simulationscene,[]).
schema(scene,[]).
schema(giveTo(_,_,_),[]).

%%SAPHARI HRI DOMAIN START
schema(saphari,[
	[gestureRecognition,1,["TRUE"]],
	[objectRecognition,1,["TRUE"]],
	%[audioStream,1,["TRUE"]],
	[fusionEngine,1,["TRUE"]]]).

schema(vrepExec(_,_),[]).

schema(fusionEngine,[]).

schema(gestureRecognition,[]).
%schema(gestureRecognition,[
%	[gesture(point),0,["TRUE"]],
%	[gesture(take),0,["TRUE"]],
%	[gesture(come),0,["TRUE"]],
%	[gesture(leaves),0,["TRUE"]],
%	[gesture(no),0,["TRUE"]],
%	[gesture(give),0,["TRUE"]],
%	[gesture(search),0,["TRUE"]],
%	[gesture(stop),0,["TRUE"]]]).

schema(objectRecognition,[
	[object(null),1.5,["TRUE"]]]).

%%SAPHARI HRI DOMAIN END

%%Seed domain
schema(forget(_),[]).
schema(inputStream,[]).
schema(listing,[]).
schema(avoid,[]).
schema(fg(_),[]).
schema(show(_),[]).
schema(requestStream,[]).
schema(plan_listener,[]).
schema(rosStream,[]).
schema($(_),[]).

schema(set(_,_,_),[]).
schema(send(_,_,_,_),[]).
schema(postGoal(_),[]).

%%Player stage domain
schema(laserStream,[]).
schema(engineStream,[]).
schema(gotoxy(_,_),[]).


%elaborazione del linguaggio per SAPHARI
schema(en(Sentence),[
	[speech(shutdown),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,"shut","down"]).

schema(en(Sentence),[
	[speech(repeat),1,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,"repeat"]).

schema(en(Sentence),[
	[userTake(water),1,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,"water"]).

schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]],
	[object(TermObject),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action,Noun,Object]),
		term_string(TermAction,Action),
		term_string(me,Noun),
		term_string(TermObject,Object).

	%oggetti composti (eg. screwdriver)
schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]],
	[object(TermObject),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action,"me",Ob_pt1,Ob_pt2]),
		term_string(TermAction,Action),
		concat_string([Ob_pt1,Ob_pt2],Object),
		term_string(TermObject,Object).

schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]],
	[object(TermObject),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action,Ob_pt1,Ob_pt2]),
		term_string(TermAction,Action),
		concat_string([Ob_pt1,Ob_pt2],Object),
		term_string(TermObject,Object).

schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]],
	[object(TermObject),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action,Object]),
		term_string(TermAction,Action),
		term_string(TermObject,Object).

schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action]),
		term_string(TermAction,Action),
		schema(speech(TermAction),_).

% i prossimi 2 sono aggiunti prima di tolosa
schema(en(Sentence),[
	[object(TermObject),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Ob_pt1,Ob_pt2]),
		concat_string([Ob_pt1,Ob_pt2],Object),
		term_string(TermObject,Object).

schema(en(Sentence),[
	[object(TermObject),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Object]),
		term_string(TermObject,Object).

schema(gesture(take),[]).
schema(gesture(give),[]).
schema(gesture(stop),[]).
schema(gesture(no),[]).
schema(gesture(leaves),[]).
schema(gesture(pointRight),[]).
schema(gesture(pointLeft),[]).
schema(gesture(point),[]).
schema(gesture(come),[]).
schema(gesture(search),[]).
%schema(gesture(pointleft),[]).
%schema(gesture(pointfront),[]).

schema(speech(take),[]).
schema(speech(give),[]).
schema(speech(stop),[]).
schema(speech(done),[]).
schema(speech(no),[]).
schema(speech(place),[]).
schema(speech(teach),[]).
schema(speech(close),[]).
schema(speech(open),[]).
schema(speech(repeat),[]).
schema(repeattask,[]).
schema(userTake(_),[]).

schema(object(tea),[]).
schema(object(coffee),[]).
schema(object(sugar),[]).
schema(object(milk),[]).
schema(object(cup),[]).
schema(object(water),[]).
schema(object(null),[]).

schema(intention(_,_),[]).

%TUM KUKA TEA/COFFEE SCENARIO
schema(kukaStream,[]).
schema(kukaTake(_),[]).
schema(kukaGive(_),[]).
schema(kukaPlace(_),[]).
schema(kukaGripper(_),[]).
schema(kukaTeach,[]).
schema(kukaDone,[]).
schema(kukaStop,[]).
schema(kukaShutdown,[]).
schema(kuka(_),[]).
schema(kuka(_,_),[]).
schema(kukaExecute(_,_),[]).
schema(subtask(_,_),[]).

schema(tumkuka,[
	%[prepareCoffee,1,[-kuka.teach]],
	%[prepareTea,1,[-kuka.teach]],
	[kukaStream,1,["TRUE"]]
	]).

schema(preparetea,[
	%[add(milk),1,["TRUE"]],
	[add(water),1,["TRUE"]],
	%[add(coffee),1,["TRUE"]],
	[add(tea),1,[water.used]]
	]).

schema(preparecoffee,[
	%[add(milk),1,["TRUE"]],
	[add(water),1,["TRUE"]],
	[add(coffee),1,["TRUE"]],
	[use(spoon),1,[water.used,coffee.used]]
	]).

schema(add(OBJ),[
	[subtask(take,OBJ),1,[kukahand.free]],
	[subtask(poor,OBJ),1,[OBJ.taken]]
	]).

schema(use(OBJ),[
	[subtask(take,OBJ),1,[kukahand.free]],
	[subtask(mix,OBJ),1,[OBJ.taken]]
	]).



%schema(preparetea2,[
%	[giveto(milk),1,["TRUE"]],
%	[giveto(water),1,["TRUE"]],
%	[giveto(tea),1,["TRUE"]]
%	]).

%schema(preparetea,[
%	[placeto(milk),1,["TRUE"]],
%	[placeto(water),1,["TRUE"]],
%	[placeto(tea),1,["TRUE"]]
%	]).

schema(giveto(OBJ),[
	[kukaTake(OBJ),1,[kukahand.free]],
	[kukaGive(OBJ),1,[OBJ.taken]]
	]).

%schema(placeto(OBJ),[
%	[kukaTake(OBJ),1,[kukahand.free]],
%	[kukaPlace(OBJ),1,[OBJ.taken]]
%	]).
schema(placeto(OBJ),[
	[subtask(take,OBJ),1,[kukahand.free]],
	[subtask(place,OBJ),1,[OBJ.taken]]
	]).


%graspingPoint(null,null,0).
%graspingPoint(milk,1).

goal(gotoxy(X,Y),[gotoxy(X,Y).arrived]).
goal(goto(X,Y),[gotoxy(X,Y).arrived]).


%%goal SAPHARI
goal(take(Obj),[hand.Obj]).
goal(give(Agent,Obj),[Agent.Obj]).	%l'agente ha l'obj in mano
goal(receive(_,Obj),[hand.Obj]).
goal(place(Obj,Pos),[Pos.Obj]).
goal(go(Pos),[go(Pos).arrived]).	%robot in position
goal(glue(_,Pos),[Pos.pasted]).
goal(pointTo(Pos),[pointTo(Pos).done]).
goal(clean(Pos),[clean(Pos).clear]).
goal(attachbracket(Obj,Sup), [Sup.installed]).

goal(get(Obj),[hand.Obj]).
goal(installBracket(_,Sup),[Sup.installed]).

goal(kukaTake(X),[X.taken]).
goal(kukaGive(X),[human.X]).
goal(kukaGripper(open),[kukahand.free]).
goal(kukaGripper(close),[human.X]).
goal(kuka(gripper(open)),[kukahand.free]).
goal(kuka(gripper(close)),[-kukahand.free]).
goal(kukaDone,[-kuka.teach]).
goal(kukaPlace(X),[human.X]).
goal(giveto(X),[human.X]).
goal(placeto(X),[human.X]).

goal(subtask(take,Obj),[Obj.taken]).
goal(subtask(poor,Obj),[Obj.used]).
goal(subtask(mix,Obj),[Obj.used]).
goal(add(Obj),[Obj.used]).
goal(use(Obj),[Obj.used]).

%goal(preparecoffee,[spoon.used]).
goal(preparecoffee,[water.used]).
goal(preparetea,[tea.used]).


updateSchema(Instance,NewSon,Releaser,null):-
		retract(schema(Instance,[H|Rest])),!,
		append([H|Rest],[[NewSon,1,[Releaser]]],NewList),
		asserta(schema(Instance,NewList)).
		%listing(schema/2).

updateSchema(Instance,NewSon,Releaser,Goal):-
		retract(schema(Instance,[H|Rest])),!,
		append([H|Rest],[[NewSon,1,[Releaser]]],NewList),
		asserta(schema(Instance,NewList)),
		assertz(goal(NewSon,[Goal])).
		%listing(schema/2).

updateSchema(Instance,NewSon,Releaser,Goal):-!,
		asserta(schema(Instance,[[NewSon,1,[Releaser]]])),
		assertz(goal(NewSon,[Goal])).

%saveLTM:-tell('LTM_autosave.prolog'), listing(schema/2), told.
saveLTM:-
	open('LTM_autosave.prolog',write,S),
	set_stream(output,S),
	listing(schema/2),
	listing(goal/2),
	set_stream(output,stdout),
	%listing(schema/2),
	close(S).



