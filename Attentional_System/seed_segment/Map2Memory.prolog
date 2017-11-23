:- dynamic horizon/2.

%definizione dei landmark della mappa come nodi di un grafo
landmark(red,-6,-6).
landmark(green,6,-3).
landmark(yellow,-7,5).
landmark(worker,6,6).
landmark(north,0,7).
landmark(center,-1,1).
landmark(west,-7,-2).
landmark(south,2,-5).

%definizione dei percorsi tra i landmark come archi
near(worker,center).
near(center,worker).
near(green,south).
near(south,green).
near(south,red).
near(red,south).
near(south,center).
near(center,south).
near(green,worker).
near(worker,green).
near(north,worker).
near(worker,north).
near(north,yellow).
near(yellow,north).
near(yellow,west).
near(west,yellow).
near(west,center).
near(center,west).

%definizione dell'orizzonte di defaut per ogni landmark
horizon(red,1).
horizon(green,1).
horizon(yellow,1).
horizon(worker,1).
horizon(center,1).
horizon(south,1).
horizon(north,1).
horizon(west,1).


%proprietà degli oggetti
color(objRed,red).
color(objYellow,yellow).
color(objGreen,green).
color(objOrange,orange).
color(objPink,pink).
color(objBrown,brown).
color(worker,blue).

%position(objYellow,yellow).
%position(worker,worker).
%position(objGreen,green).
%position(objRed,red).
%position(objOrange,south).

position(Obj,X,Y):-position(Obj,L),landmark(L,X,Y).

planNext(Xstart,Ystart,Goal,OldPath,[Xnext,Ynext,Lnext,Horizon],NewPath):-
	getLandmark(Xstart,Ystart,Lstart),
	(member(Lstart,OldPath)->
%APPRENDIMENTO:
		(backReinforcement(OldPath),
		listing(horizon/2),
		append([Lstart],[],NewPath))
		;			
		(append([Lstart],OldPath,NewPath))),
	position(Goal,Lgoal),
	horizon(Lstart,Horizon),
	lookahead(Lstart,Lgoal,Horizon,[Lnext|_],_),
	landmark(Lnext,Xnext,Ynext).



%old planner
planNext(Xstart,Ystart,Goal,Horizon):-
	yeld([],_),
	getLandmark(Xstart,Ystart,Lstart),
	lookahead(Lstart,Goal,Horizon,[Lnext|_],_),
	landmark(Lnext,Xnext,Ynext),
	yeld([Xnext,Ynext],_).

getLandmark(X,Y,L):-
	findall(Land,landmark(Land,_,_),List),
	nearest(X,Y,List,9999,"null",L).


nearest(_,_,[],_,UpL,UpL).

nearest(X,Y,[Head|Rest],UpDist,UpL,ReturnL):-
	landmark(Head,XH,YH),
	HeadDist is ((XH-X)^2+(YH-Y)^2)^(1/2),
	(HeadDist<UpDist->
		nearest(X,Y,Rest,HeadDist,Head,ReturnL)
		;
		nearest(X,Y,Rest,UpDist,UpL,ReturnL)).

	


%ho immaginato parte del piano solo se
%	mi trovo nello stato desiderato
lookahead(Stop,Stop,_,[],0).

%altrimenti ho immaginato parte del piano solo se
%	ho esaurito la profondità di ricerca
%	restituisco la distanza euclidea dal landmark a goal (euristica)
lookahead(X,Stop,0,[],DistEuclid):-
	distance(X,Stop,DistEuclid).

%altrimenti ho immaginato parte del piano solo se
%	calcolo gli adiacenti al landmark attuale
%	decremento la profondità
%	trovo il landmark a distanza minima dal goal (euristica)
%	calcolo la distanza tra il landmark attuale ed il landmark migliore
%	restituisco la somma tra le 2 distanze
lookahead(X,Stop,Depth,[Min|MinSubPath],Distance):-
	findall(Y,near(X,Y),List),
	NewDepth is Depth-1,
	findmin(X,List,Distance,Stop,NewDepth,[Min|MinSubPath]).


%ho trovato il minimo solo se
%	ho finito gli adiacenti e restituisco distanza "infinita"
findmin(_,[],100,_,_,[none]).

%altrimenti ho trovato il minimo solo se
%	immagino una parte del piano per l'adiacente attuale
%	trovo il minimo tra gli altri adiacenti
%	se l'adiacente attuale è migliore del minimo tra i restanti
%		restituisco il percorso relativo all'adiacente attuale
%	altrimenti
%		restituisco il percorso relativo al minimo dei rimanenti
findmin(X,[Head|Rest],NewDist,Stop,Depth,[Min|Path]):-
	lookahead(Head,Stop,Depth,HeadPath,HeadDist),
	findmin(X,Rest,DistR,Stop,Depth,[RestMin|RestMinPath]),
	distance(X,Head,DistX2H),	
	DistPathHead is DistX2H+HeadDist,	
	(DistPathHead<DistR -> 
		(functor(Min,Head,0),
		NewDist is DistPathHead,
		Path=HeadPath)
		;
		(functor(Min,RestMin,0),
		NewDist is DistR,
		Path=RestMinPath)).

%distanza tra 2 landmark solo se
%	ho le coordinate del primo landmark
%	ho le coordinate del secondo landmark
%	restituisco la distanza euclidea tra e due coordinate
distance(L1,L2,Dist):-
	landmark(L1,X1,Y1),
	landmark(L2,X2,Y2),
	Dist is ((X1-X2)^2+(Y1-Y2)^2)^(1/2).

%ho potenziato il lookahead sul percorso solo se
%	il percorso è terminato
backReinforcement([]).

%altrimenti
%	rimuovi il vecchio orizzonte dal landmark attuale
%	incremento il vecchio orizzonte
%	asserisco il nuovo orizzonte per il landmark attuale
%	ho potenziato il lookahead anche per i restanti landmark
backReinforcement([Land|Rest]):-
	retract(horizon(Land,H)),
	NH is H+1,
	assert(horizon(Land,NH)),
	backReinforcement(Rest).

