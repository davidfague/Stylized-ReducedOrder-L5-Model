:Comment : mtau deduced from text (said to be 6 times faster than for NaT)
:Comment : so I used the equations from NaT and multiplied by 6
:Reference : Modeled according to kinetics derived from Magistretti & Alonso 1999

NEURON	{
	SUFFIX Nap_E
	USEION na READ ena WRITE ina
	RANGE gNap_Ebar, gNap_E, ina
}

UNITS	{
	(S) = (siemens)
	(mV) = (millivolt)
	(mA) = (milliamp)
}

PARAMETER	{
	gNap_Ebar = 0.00001 (S/cm2)
}

ASSIGNED	{
	v	(mV)
	ena	(mV)
	ina	(mA/cm2)
	gNap_E	(S/cm2)
	mInf
	mTau
	mAlpha
	mBeta
	hInf
	hTau
	hAlpha
	hBeta
}

STATE	{
	m
	h
}

BREAKPOINT	{
	SOLVE states METHOD cnexp
	gNap_E = gNap_Ebar*m*m*m*h
	ina = gNap_E*(v-ena)
}

DERIVATIVE states	{
	rates()
	m' = (mInf-m)/mTau
	h' = (hInf-h)/hTau
}

INITIAL{
	rates()
	m = mInf
	h = hInf
}

PROCEDURE rates(){
	UNITSOFF
		mInf = 1.0/(1+exp((v- -52.6)/-4.6))
				if(v == -25){
      		v = v + 0.0001
    		}
		mAlpha = (0.182 * (v- -25))/(1-(exp(-(v- -25)/9)))
		mBeta  = (0.124 * (-v -25))/(1-(exp(-(-v -25)/9)))
		mTau = 6 * 1/(mAlpha + mBeta)
				if(v == -50){
      		v = v + 0.0001
    		}
		hAlpha = (0.00001 * (v- -50))/(1-(exp(-(v- -50)/13)))
        if(v == -75){
            v = v+0.0001
        }
		hBeta  = (0.00001 * (-v - 75))/(1-(exp(-(-v - 75)/13)))
		hInf = 1.0/(1+exp((v- -48.8)/10))
		hTau = (1/(hAlpha + hBeta))
	UNITSON
}