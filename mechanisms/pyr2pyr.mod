:Pyramidal Cells to Pyramidal Cells AMPA+NMDA with local Ca2+ pool

NEURON {
	POINT_PROCESS pyr2pyr
	:USEION ca READ eca	
	NONSPECIFIC_CURRENT inmda, iampa
	RANGE initW
	RANGE Cdur_nmda, AlphaTmax_nmda, Beta_nmda, Erev_nmda, gbar_nmda, W_nmda, on_nmda, g_nmda
	RANGE Cdur_ampa, AlphaTmax_ampa, Beta_ampa, Erev_ampa, gbar_ampa, W, on_ampa, g_ampa
	:RANGE eca, ICa, P0, fCa, tauCa, iCatotal
	:RANGE Cainf, pooldiam, z
	:RANGE lambda1, lambda2, threshold1, threshold2
	:RANGE fmax, fmin, Wmax, Wmin, maxChange, normW, scaleW, limitW, srcid,destid,tempW 
	:RANGE pregid, postgid
	RANGE thr_rp
	RANGE F, f, tauF, D1, d1, tauD1, D2, d2, tauD2
	RANGE facfactor
	:RANGE type
	:RANGE neuroM

	:Release probability
	RANGE random, P, P_0

	THREADSAFE
	POINTER randObjPtr
}

UNITS { 
	(mV) = (millivolt)
        (nA) = (nanoamp)
	(uS) = (microsiemens)
	FARADAY = 96485 (coul)
	pi = 3.141592 (1)
}

PARAMETER {

	:srcid = -1 (1)
	:destid = -1 (1)
	:type = -1
	
	Cdur_nmda = 16.7650 (ms)
	AlphaTmax_nmda = .2659 (/ms)
	Beta_nmda = 0.008 (/ms)
	Erev_nmda = 0 (mV)
	gbar_nmda = .5e-3 (uS)

	Cdur_ampa = 1.4210 (ms)
	AlphaTmax_ampa = 3.8142 (/ms)
	Beta_ampa =  0.1429(/ms) :0.1429 as original 0.2858 as half,0.07145 as twice
	Erev_ampa = 0 (mV)
	gbar_ampa = 1e-3 (uS)

	:eca = 120

	:Cainf = 50e-6 (mM)
	:pooldiam =  1.8172 (micrometer)
	:z = 2
	:neuroM = 0
	:tauCa = 50 (ms)
	:P0 = .015
	:fCa = .024
	
	:lambda1 = 40 : 60 : 12 :80: 20 : 15 :8 :5: 2.5
	:lambda2 = .03
	:threshold1 = 0.4 :  0.45 : 0.35 :0.35:0.2 :0.50 (uM)
	:threshold2 = 0.55 : 0.50 : 0.40 :0.4 :0.3 :0.60 (uM)

	initW = 5.0 : 1.0 :  0.9 : 0.8 : 2 : 10 : 6 :1.5
	:fmax = 3 : 2.5 : 4 : 2 : 3 : 1.5 : 3
	:fmin = .8
	
	:DAstart1 = 39500
	:DAstop1 = 40000	
	:DAstart2 = 35900
	:DAstop2 = 36000	

	:DA_t1 = 1.2
	:DA_t2 = 0.8 : 0.9
    :DA_t3 = 0.9
	:DA_S = 1.3 : 0.95 : 0.6	
	:Beta1 = 0.001  (/ms) : 1/decay time for neuromodulators
	:Beta2 = 0.0001  (/ms)

	thr_rp = 1 : .7
	
	facfactor = 1
	: the (1) is needed for the range limits to be effective
        f = 0 (1) < 0, 1e9 >    : facilitation
        tauF = 20 (ms) < 1e-9, 1e9 >
        d1 = 0.95 (1) < 0, 1 >     : fast depression
        tauD1 = 40 (ms) < 1e-9, 1e9 >
        d2 = 0.9 (1) < 0, 1 >     : slow depression
        tauD2 = 70 (ms) < 1e-9, 1e9 >	

	P_0 = 1 (1) < 0, 1 >               : base release probability	
}

ASSIGNED {
	v (mV)

	inmda (nA)
	g_nmda (uS)
	on_nmda
	W_nmda

	iampa (nA)
	g_ampa (uS)
	on_ampa
	: W
	limitW

	t0 (ms)

	:ICa (mA)
	:Afactor	(mM/ms/nA)
	:iCatotal (mA)

	:dW_ampa
	:Wmax
	:Wmin
	:maxChange
	:normW
	:scaleW
	
    :tempW
	:pregid
	:postgid

	rp
	tsyn
	
	fa
	F
	D1
	D2

	:Release probability
	P				        : instantaneous release probability
    randObjPtr              : pointer to a hoc random number generator Random.uniform(0,1)
    random   
}

STATE { r_nmda r_ampa W}

INITIAL {
	on_nmda = 0
	r_nmda = 0
	W_nmda = initW

	on_ampa = 0
	r_ampa = 0
	W = initW
    :limitW = 1
    
	:tempW = initW
	t0 = -1

	:Wmax = fmax*initW
	:Wmin = fmin*initW
	:maxChange = (Wmax-Wmin)/10
	:dW_ampa = 0

	:capoolcon = Cainf
	:Afactor	= 1/(z*FARADAY*4/3*pi*(pooldiam/2)^3)*(1e6)

	fa =0
	F = 1
	D1 = 1
	D2 = 1

	P = P_0
	random = 1
}

BREAKPOINT {

:if ((eta(capoolcon)*(lambda1*omega(capoolcon, threshold1, threshold2)-lambda2*W))>0&&W>=Wmax) {
:        limitW=1e-12
:	} else if ((eta(capoolcon)*(lambda1*omega(capoolcon, threshold1, threshold2)-lambda2*W))<0&&W<=Wmin) {
:        limitW=1e-12
:	} else {
:	limitW=1 }
	
	SOLVE release METHOD cnexp
	if (t0>0) {
		if (rp < thr_rp) {
			if (t-t0 < Cdur_ampa) {
				on_ampa = 1
			} else {
				on_ampa = 0
			}
		} else {
			on_ampa = 0
		}
	}
          : if (W >= Wmax || W <= Wmin ) {     : for limiting the weight
	 : limitW=1e-12
	 : } else {
	  : limitW=1
	 : }
	 
	 :if (W > Wmax) { 
		:W = Wmax
	:} else if (W < Wmin) {
 		:W = Wmin
	:}
	 
	:if (neuroM==1) {
	:g_nmda = gbar_nmda*r_nmda*facfactor*DA1(DAstart1,DAstop1)*DA2(DAstart2,DAstop2)        : Dopamine effect on NMDA to reduce NMDA current amplitude
	:	} else {
	g_nmda = gbar_nmda*r_nmda*facfactor
:		}
	inmda = W_nmda*g_nmda*(v - Erev_nmda)*sfunc(v)

	g_ampa = gbar_ampa*r_ampa*facfactor
	iampa = W*g_ampa*(v - Erev_ampa)

	:ICa = P0*g_nmda*(v - eca)*sfunc(v)
	
}

DERIVATIVE release {
	: W' = eta(capoolcon)*(lambda1*omega(capoolcon, threshold1, threshold2)-lambda2*W)	  : Long-term plasticity was implemented. (Shouval et al. 2002a, 2002b)
	
	:W' = 1e-12*limitW*eta(capoolcon)*(lambda1*omega(capoolcon, threshold1, threshold2)-lambda2*W)	  : Long-term plasticity was implemented. (Shouval et al. 2002a, 2002b)
	r_nmda' = AlphaTmax_nmda*on_nmda*(1-r_nmda)-Beta_nmda*r_nmda
	r_ampa' = AlphaTmax_ampa*on_ampa*(1-r_ampa)-Beta_ampa*r_ampa
  	:capoolcon'= -fCa*Afactor*ICa + (Cainf-capoolcon)/tauCa
}

NET_RECEIVE(dummy_weight) {
	random = randGen()

	if (flag==0 && random < P_0) {           :a spike arrived, start onset state if not already on
        if ((!on_nmda))
		{       :this synpase joins the set of synapses in onset state
        	t0=t
			on_nmda=1		
			net_send(Cdur_nmda,1)  
        } 
		else if (on_nmda==1) {             :already in onset state, so move offset time
        	net_move(t+Cdur_nmda)
			t0=t
	    }
    }		

	if (flag == 1) { : turn off transmitter, i.e. this synapse enters the offset state	
		on_nmda=0
    }
	         
	if (flag == 0 && random < P_0) {   : Short term plasticity was implemented(Varela et. al 1997):
		F  = 1 + (F-1)* exp(-(t - tsyn)/tauF)
		D1 = 1 - (1-D1)*exp(-(t - tsyn)/tauD1)
		D2 = 1 - (1-D2)*exp(-(t - tsyn)/tauD2)
		tsyn = t
		
		facfactor = F * D1 * D2	
		F = F * f
		if (F > 30) { 
			F=30	
		}
		if (facfactor < 0.2) { 
			facfactor=0.2
		}	
		D1 = D1 * d1
		D2 = D2 * d2
	}
}

:::::::::::: FUNCTIONs and PROCEDUREs ::::::::::::

FUNCTION sfunc (v (mV)) {
	UNITSOFF
	sfunc = 1/(1+0.33*exp(-0.06*v))
	UNITSON
}

VERBATIM
double nrn_random_pick(void* r);
void* nrn_random_arg(int argpos);
ENDVERBATIM

FUNCTION randGen() {
VERBATIM
   if (_p_randObjPtr) {
      /*
      :Supports separate independent but reproducible streams for
      : each instance. However, the corresponding hoc Random
      : distribution MUST be set to Random.uniform(0,1)
      */
      _lrandGen = nrn_random_pick(_p_randObjPtr);
   }else{
      hoc_execerror("Random object ref not set correctly for randObjPtr"," only via hoc Random");
   }
ENDVERBATIM
}

PROCEDURE setRandObjRef() {
VERBATIM
   void** pv4 = (void**)(&_p_randObjPtr);
   if (ifarg(1)) {
      *pv4 = nrn_random_arg(1);
   }else{
      *pv4 = (void*)0;
   }
ENDVERBATIM
}
:FUNCTION unirand() {    : uniform random numbers between 0 and 1
:        unirand = scop_random()
:}