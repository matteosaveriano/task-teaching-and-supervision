schema(subtask(take, water), [[kukaExecute(preTake1, tea), 1, ["TRUE"]], [kukaExecute(a2, tea), 1, [[kukaExecute(preTake1, tea)|done]]], [kuka(gripper(close)), 1, [[kukaExecute(a2, tea)|done]]], [kukaExecute(postTake3, world), 1, [[kukaExecute(a2, tea)|done]]], [kukaExecute(preComplex4, world), 1, [[kukaExecute(postTake3, world)|done]]], [kukaExecute(complex5, worldComplex), 1, [[kukaExecute(preComplex4, world)|done]]], [kuka(gripper(open)), 1, [[kukaExecute(complex5, worldComplex)|done]]]]).
schema(installBracket(_15316, _15317), [[get(_15316), 1, ["TRUE"]], [give(_15316, human), 1, [[hand|_15316]]], [attachbracket(_15316, _15317), 1, [[hand|_15316]]]]).
schema(get(_15315), [[take(_15315), 1, ["TRUE"]], [receive(human, _15315), 1, ["TRUE"]]]).
schema(alive, [[inputStream, 1, ["TRUE"]], [rosStream, 1, ["TRUE"]], [requestStream, 1, ["TRUE"]], [tumkuka, 1, ["TRUE"]], [preparecoffee, 1, ["TRUE"]], [preparetea, 1, ["TRUE"]], [show(preparetea), 1, ["TRUE"]], [saphari, 1, ["TRUE"]]]).
schema(prepareboth, [[preparecoffee, 1, ["TRUE"]], [preparetea, 1, ["TRUE"]]]).
schema(goto(_15316, _15317), [[avoid, 1, ["TRUE"]], [gotoxy(_15316, _15317), 1, ["TRUE"]]]).
schema(giveTo(_15317, _15318, human), [[give(_15317, human), 1, ["TRUE"]], [go(_15318), 1, ["TRUE"]], [place(_15317, _15318), 1, ["TRUE"]]]).
schema(getObj(_15315), [[take(_15315), 1, ["TRUE"]], [search(location1), 1, ["TRUE"]]]).
schema(search(_15314), [[go(location1), 1, ["TRUE"]]]).
schema(take(_15314), []).
schema(give(_15314, _15315), []).
schema(go(_15314), []).
schema(receive(_15314, _15315), []).
schema(place(_15314, _15315), []).
schema(clean(_15314), []).
schema(glue(_15314, _15315), []).
schema(attachbracket(_15314, _15315), []).
schema(pointTo(_15314), []).
schema(simulationscene, []).
schema(scene, []).
schema(giveTo(_15314, _15315, _15316), []).
schema(saphari, [[gestureRecognition, 1, ["TRUE"]], [objectRecognition, 1, ["TRUE"]], [fusionEngine, 1, ["TRUE"]]]).
schema(vrepExec(_15314, _15315), []).
schema(fusionEngine, []).
schema(gestureRecognition, []).
schema(objectRecognition, [[object(null), 1.5, ["TRUE"]]]).
schema(forget(_15314), []).
schema(inputStream, []).
schema(listing, []).
schema(avoid, []).
schema(fg(_15314), []).
schema(show(_15314), []).
schema(requestStream, []).
schema(plan_listener, []).
schema(rosStream, []).
schema($(_15314), []).
schema(set(_15314, _15315, _15316), []).
schema(send(_15314, _15315, _15316, _15317), []).
schema(postGoal(_15314), []).
schema(laserStream, []).
schema(engineStream, []).
schema(gotoxy(_15314, _15315), []).
schema(en(_15315), [[speech(shutdown), 2, ["TRUE"]]]) :-
	string(_15315),
	split_string(_15315, " ", "", [_15344, "shut", "down"]).
schema(en(_15315), [[speech(repeat), 1, ["TRUE"]]]) :-
	string(_15315),
	split_string(_15315, " ", "", [_15344, "repeat"]).
schema(en(_15315), [[userTake(water), 1, ["TRUE"]]]) :-
	string(_15315),
	split_string(_15315, " ", "", [_15344, "water"]).
schema(en(_15315), [[speech(_15322), 2, ["TRUE"]], [object(_15337), 2, ["TRUE"]]]) :-
	string(_15315),
	split_string(_15315, " ", "", [_15363, _15367, _15370, _15373]),
	term_string(_15322, _15367),
	term_string(me, _15370),
	term_string(_15337, _15373).
schema(en(_15315), [[speech(_15322), 2, ["TRUE"]], [object(_15337), 2, ["TRUE"]]]) :-
	string(_15315),
	split_string(_15315, " ", "", [_15363, _15367, "me", _15374, _15377]),
	term_string(_15322, _15367),
	concat_string([_15374, _15377], _15394),
	term_string(_15337, _15394).
schema(en(_15315), [[speech(_15322), 2, ["TRUE"]], [object(_15337), 2, ["TRUE"]]]) :-
	string(_15315),
	split_string(_15315, " ", "", [_15363, _15367, _15370, _15373]),
	term_string(_15322, _15367),
	concat_string([_15370, _15373], _15390),
	term_string(_15337, _15390).
schema(en(_15315), [[speech(_15322), 2, ["TRUE"]], [object(_15337), 2, ["TRUE"]]]) :-
	string(_15315),
	split_string(_15315, " ", "", [_15363, _15367, _15370]),
	term_string(_15322, _15367),
	term_string(_15337, _15370).
schema(en(_15315), [[speech(_15322), 2, ["TRUE"]]]) :-
	string(_15315),
	split_string(_15315, " ", "", [_15348, _15352]),
	term_string(_15322, _15352),
	schema(speech(_15322), _15361).
schema(en(_15315), [[object(_15322), 2, ["TRUE"]]]) :-
	string(_15315),
	split_string(_15315, " ", "", [_15348, _15352, _15355]),
	concat_string([_15352, _15355], _15366),
	term_string(_15322, _15366).
schema(en(_15315), [[object(_15322), 2, ["TRUE"]]]) :-
	string(_15315),
	split_string(_15315, " ", "", [_15348, _15352]),
	term_string(_15322, _15352).
schema(gesture(take), []).
schema(gesture(give), []).
schema(gesture(stop), []).
schema(gesture(no), []).
schema(gesture(leaves), []).
schema(gesture(pointRight), []).
schema(gesture(pointLeft), []).
schema(gesture(point), []).
schema(gesture(come), []).
schema(gesture(search), []).
schema(speech(take), []).
schema(speech(give), []).
schema(speech(stop), []).
schema(speech(done), []).
schema(speech(no), []).
schema(speech(place), []).
schema(speech(teach), []).
schema(speech(close), []).
schema(speech(open), []).
schema(speech(repeat), []).
schema(repeattask, []).
schema(userTake(_15314), []).
schema(object(tea), []).
schema(object(coffee), []).
schema(object(sugar), []).
schema(object(milk), []).
schema(object(cup), []).
schema(object(water), []).
schema(object(null), []).
schema(intention(_15314, _15315), []).
schema(kukaStream, []).
schema(kukaTake(_15314), []).
schema(kukaGive(_15314), []).
schema(kukaPlace(_15314), []).
schema(kukaGripper(_15314), []).
schema(kukaTeach, []).
schema(kukaDone, []).
schema(kukaStop, []).
schema(kukaShutdown, []).
schema(kuka(_15314), []).
schema(kuka(_15314, _15315), []).
schema(kukaExecute(_15314, _15315), []).
schema(subtask(_15314, _15315), []).
schema(tumkuka, [[kukaStream, 1, ["TRUE"]]]).
schema(preparetea, [[add(water), 1, ["TRUE"]], [add(tea), 1, [[water|used]]]]).
schema(preparecoffee, [[add(water), 1, ["TRUE"]], [add(coffee), 1, ["TRUE"]], [use(spoon), 1, [[water|used], [coffee|used]]]]).
schema(add(_15315), [[subtask(take, _15315), 1, [[kukahand|free]]], [subtask(poor, _15315), 1, [[_15315|taken]]]]).
schema(use(_15315), [[subtask(take, _15315), 1, [[kukahand|free]]], [subtask(mix, _15315), 1, [[_15315|taken]]]]).
schema(giveto(_15315), [[kukaTake(_15315), 1, [[kukahand|free]]], [kukaGive(_15315), 1, [[_15315|taken]]]]).
schema(placeto(_15314), [[subtask(take, _15314), 1, [[kukahand|free]]], [subtask(place, _15314), 1, [[_15314|taken]]]]).
goal(gotoxy(_15054, _15055), [[gotoxy(_15054, _15055)|arrived]]).
goal(goto(_15054, _15055), [[gotoxy(_15054, _15055)|arrived]]).
goal(take(_15053), [[hand|_15053]]).
goal(give(_15054, _15055), [[_15054|_15055]]).
goal(receive(_15052, _15054), [[hand|_15054]]).
goal(place(_15054, _15055), [[_15055|_15054]]).
goal(go(_15053), [[go(_15053)|arrived]]).
goal(glue(_15052, _15054), [[_15054|pasted]]).
goal(pointTo(_15053), [[pointTo(_15053)|done]]).
goal(clean(_15053), [[clean(_15053)|clear]]).
goal(attachbracket(_15054, _15055), [[_15055|installed]]).
goal(get(_15053), [[hand|_15053]]).
goal(installBracket(_15052, _15054), [[_15054|installed]]).
goal(kukaTake(_15053), [[_15053|taken]]).
goal(kukaGive(_15053), [[human|_15053]]).
goal(kukaGripper(open), [[kukahand|free]]).
goal(kukaGripper(close), [[human|_15057]]).
goal(kuka(gripper(open)), [[kukahand|free]]).
goal(kuka(gripper(close)), [-([kukahand|free])]).
goal(kukaDone, [-([kuka|teach])]).
goal(kukaPlace(_15053), [[human|_15053]]).
goal(giveto(_15053), [[human|_15053]]).
goal(placeto(_15053), [[human|_15053]]).
goal(subtask(take, _15054), [[_15054|taken]]).
goal(subtask(poor, _15054), [[_15054|used]]).
goal(subtask(mix, _15054), [[_15054|used]]).
goal(add(_15053), [[_15053|used]]).
goal(use(_15053), [[_15053|used]]).
goal(preparecoffee, [[spoon|used]]).
goal(preparetea, [[tea|used]]).
goal(kukaExecute(preTake1, tea), [[kukaExecute(preTake1, tea)|done]]).
goal(kukaExecute(a2, tea), [[kukaExecute(a2, tea)|done]]).
goal(kukaExecute(postTake3, world), [[kukaExecute(postTake3, world)|done]]).
goal(kukaExecute(preComplex4, world), [[kukaExecute(preComplex4, world)|done]]).
goal(kukaExecute(complex5, worldComplex), [[kukaExecute(complex5, worldComplex)|done]]).
