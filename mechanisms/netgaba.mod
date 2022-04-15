COMMENT
//****************************//
// Created by Alon Polsky 	//
//    apmega@yahoo.com		//
//		2010			//
//****************************//
based on Sun et al 2006
Modified 2015 by Robert Egger
to include facilitation variable
as modeled by Varela et al. 1997
ENDCOMMENT
TITLE GABAA synapse activated by the network
NEURON {
	POINT_PROCESS gaba_syn
	NONSPECIFIC_CURRENT i
	RANGE i,ggaba
	RANGE decaygaba,dgaba,taudgaba
	RANGE facilgaba,fgaba,taufgaba
	:RANGE R,D
	RANGE risetime,decaytime,e 
}
PARAMETER {
	e= -60.0	(mV)
	risetime=1	(ms)	:2
	decaytime=20(ms)	:40

	v		(mV)
	taudgaba=200	(ms)
	decaygaba=0.8
	taufgaba=200    (ms)
	facilgaba=0.0
}
ASSIGNED {
	i		(nA)  
	ggaba
    factor     : conductance normalization factor
}

STATE {
	dgaba
	fgaba
	R
	D
}

INITIAL {
	LOCAL tp
    dgaba=1 
    fgaba=1
	R=0
	D=0
	ggaba=0
	
    tp = (risetime*decaytime)/(decaytime - risetime) * log(decaytime/risetime)
    factor = -exp(-tp/risetime) + exp(-tp/decaytime)
    factor = 1/factor
}
BREAKPOINT {
	SOLVE state METHOD cnexp
	ggaba=D-R
	i=(1e-3)*ggaba*(v-e)
}
NET_RECEIVE(weight) {
    R = R + factor*weight*dgaba*fgaba
    D = D + factor*weight*dgaba*fgaba
    dgaba = dgaba* decaygaba
    fgaba = fgaba + facilgaba
}
DERIVATIVE state {
	R'=-R/risetime
	D'=-D/decaytime
	dgaba'=(1-dgaba)/taudgaba
	fgaba'=(1-fgaba)/taufgaba

}