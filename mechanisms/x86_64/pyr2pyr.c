/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__pyr2pyr
#define _nrn_initial _nrn_initial__pyr2pyr
#define nrn_cur _nrn_cur__pyr2pyr
#define _nrn_current _nrn_current__pyr2pyr
#define nrn_jacob _nrn_jacob__pyr2pyr
#define nrn_state _nrn_state__pyr2pyr
#define _net_receive _net_receive__pyr2pyr 
#define release release__pyr2pyr 
#define setRandObjRef setRandObjRef__pyr2pyr 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define Cdur_nmda _p[0]
#define AlphaTmax_nmda _p[1]
#define Beta_nmda _p[2]
#define Erev_nmda _p[3]
#define gbar_nmda _p[4]
#define Cdur_ampa _p[5]
#define AlphaTmax_ampa _p[6]
#define Beta_ampa _p[7]
#define Erev_ampa _p[8]
#define gbar_ampa _p[9]
#define initW _p[10]
#define thr_rp _p[11]
#define facfactor _p[12]
#define f _p[13]
#define tauF _p[14]
#define d1 _p[15]
#define tauD1 _p[16]
#define d2 _p[17]
#define tauD2 _p[18]
#define P_0 _p[19]
#define inmda _p[20]
#define g_nmda _p[21]
#define on_nmda _p[22]
#define W_nmda _p[23]
#define iampa _p[24]
#define g_ampa _p[25]
#define on_ampa _p[26]
#define F _p[27]
#define D1 _p[28]
#define D2 _p[29]
#define P _p[30]
#define random _p[31]
#define r_nmda _p[32]
#define r_ampa _p[33]
#define W _p[34]
#define limitW _p[35]
#define t0 _p[36]
#define rp _p[37]
#define tsyn _p[38]
#define fa _p[39]
#define Dr_nmda _p[40]
#define Dr_ampa _p[41]
#define DW _p[42]
#define v _p[43]
#define _g _p[44]
#define _tsav _p[45]
#define _nd_area  *_ppvar[0]._pval
#define randObjPtr	*_ppvar[2]._pval
#define _p_randObjPtr	_ppvar[2]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  2;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_randGen(void*);
 static double _hoc_setRandObjRef(void*);
 static double _hoc_sfunc(void*);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(Object* _ho) { void* create_point_process(int, Object*);
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt(void*);
 static double _hoc_loc_pnt(void* _vptr) {double loc_point_process(int, void*);
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(void* _vptr) {double has_loc_point(void*);
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(void* _vptr) {
 double get_loc_point_process(void*); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "randGen", _hoc_randGen,
 "setRandObjRef", _hoc_setRandObjRef,
 "sfunc", _hoc_sfunc,
 0, 0
};
#define randGen randGen_pyr2pyr
#define sfunc sfunc_pyr2pyr
 extern double randGen( _threadargsproto_ );
 extern double sfunc( _threadargsprotocomma_ double );
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "P_0", 0, 1,
 "d2", 0, 1,
 "d1", 0, 1,
 "f", 0, 1e+09,
 "tauD2", 1e-09, 1e+09,
 "tauD1", 1e-09, 1e+09,
 "tauF", 1e-09, 1e+09,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "Cdur_nmda", "ms",
 "AlphaTmax_nmda", "/ms",
 "Beta_nmda", "/ms",
 "Erev_nmda", "mV",
 "gbar_nmda", "uS",
 "Cdur_ampa", "ms",
 "AlphaTmax_ampa", "/ms",
 "Beta_ampa", "/ms",
 "Erev_ampa", "mV",
 "gbar_ampa", "uS",
 "f", "1",
 "tauF", "ms",
 "d1", "1",
 "tauD1", "ms",
 "d2", "1",
 "tauD2", "ms",
 "P_0", "1",
 "inmda", "nA",
 "g_nmda", "uS",
 "iampa", "nA",
 "g_ampa", "uS",
 0,0
};
 static double W0 = 0;
 static double delta_t = 0.01;
 static double r_ampa0 = 0;
 static double r_nmda0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(NrnThread*, _Memb_list*, int);
static void nrn_state(NrnThread*, _Memb_list*, int);
 static void nrn_cur(NrnThread*, _Memb_list*, int);
static void  nrn_jacob(NrnThread*, _Memb_list*, int);
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(NrnThread*, _Memb_list*, int);
static void _ode_matsol(NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[4]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"pyr2pyr",
 "Cdur_nmda",
 "AlphaTmax_nmda",
 "Beta_nmda",
 "Erev_nmda",
 "gbar_nmda",
 "Cdur_ampa",
 "AlphaTmax_ampa",
 "Beta_ampa",
 "Erev_ampa",
 "gbar_ampa",
 "initW",
 "thr_rp",
 "facfactor",
 "f",
 "tauF",
 "d1",
 "tauD1",
 "d2",
 "tauD2",
 "P_0",
 0,
 "inmda",
 "g_nmda",
 "on_nmda",
 "W_nmda",
 "iampa",
 "g_ampa",
 "on_ampa",
 "F",
 "D1",
 "D2",
 "P",
 "random",
 0,
 "r_nmda",
 "r_ampa",
 "W",
 0,
 "randObjPtr",
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 46, _prop);
 	/*initialize range parameters*/
 	Cdur_nmda = 16.765;
 	AlphaTmax_nmda = 0.2659;
 	Beta_nmda = 0.008;
 	Erev_nmda = 0;
 	gbar_nmda = 0.0005;
 	Cdur_ampa = 1.421;
 	AlphaTmax_ampa = 3.8142;
 	Beta_ampa = 0.1429;
 	Erev_ampa = 0;
 	gbar_ampa = 0.001;
 	initW = 5;
 	thr_rp = 1;
 	facfactor = 1;
 	f = 0;
 	tauF = 20;
 	d1 = 0.95;
 	tauD1 = 40;
 	d2 = 0.9;
 	tauD2 = 70;
 	P_0 = 1;
  }
 	_prop->param = _p;
 	_prop->param_size = 46;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 5, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 
#define _tqitem &(_ppvar[3]._pvoid)
 static void _net_receive(Point_process*, double*, double);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _pyr2pyr_reg() {
	int _vectorized = 1;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 1,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 46, 5);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "pointer");
  hoc_register_dparam_semantics(_mechtype, 3, "netsend");
  hoc_register_dparam_semantics(_mechtype, 4, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 pyr2pyr /content/drive/MyDrive/Stylized-Cell-model/mechanisms/pyr2pyr.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 96485.0;
 static double pi = 3.141592;
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int setRandObjRef(_threadargsproto_);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[2], _dlist1[2];
 static int release(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {int _reset = 0; {
   Dr_nmda = AlphaTmax_nmda * on_nmda * ( 1.0 - r_nmda ) - Beta_nmda * r_nmda ;
   Dr_ampa = AlphaTmax_ampa * on_ampa * ( 1.0 - r_ampa ) - Beta_ampa * r_ampa ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
 Dr_nmda = Dr_nmda  / (1. - dt*( ( AlphaTmax_nmda * on_nmda )*( ( ( - 1.0 ) ) ) - ( Beta_nmda )*( 1.0 ) )) ;
 Dr_ampa = Dr_ampa  / (1. - dt*( ( AlphaTmax_ampa * on_ampa )*( ( ( - 1.0 ) ) ) - ( Beta_ampa )*( 1.0 ) )) ;
  return 0;
}
 /*END CVODE*/
 static int release (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) { {
    r_nmda = r_nmda + (1. - exp(dt*(( AlphaTmax_nmda * on_nmda )*( ( ( - 1.0 ) ) ) - ( Beta_nmda )*( 1.0 ))))*(- ( ( ( AlphaTmax_nmda )*( on_nmda ) )*( ( 1.0 ) ) ) / ( ( ( AlphaTmax_nmda )*( on_nmda ) )*( ( ( - 1.0 ) ) ) - ( Beta_nmda )*( 1.0 ) ) - r_nmda) ;
    r_ampa = r_ampa + (1. - exp(dt*(( AlphaTmax_ampa * on_ampa )*( ( ( - 1.0 ) ) ) - ( Beta_ampa )*( 1.0 ))))*(- ( ( ( AlphaTmax_ampa )*( on_ampa ) )*( ( 1.0 ) ) ) / ( ( ( AlphaTmax_ampa )*( on_ampa ) )*( ( ( - 1.0 ) ) ) - ( Beta_ampa )*( 1.0 ) ) - r_ampa) ;
   }
  return 0;
}
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{  double* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   _thread = (Datum*)0; _nt = (NrnThread*)_pnt->_vnt;   _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t;   if (_lflag == 1. ) {*(_tqitem) = 0;}
 {
   random = randGen ( _threadargs_ ) ;
   if ( _lflag  == 0.0  && random < P_0 ) {
     if ( (  ! on_nmda ) ) {
       t0 = t ;
       on_nmda = 1.0 ;
       net_send ( _tqitem, _args, _pnt, t +  Cdur_nmda , 1.0 ) ;
       }
     else if ( on_nmda  == 1.0 ) {
       net_move ( _tqitem, _pnt, t + Cdur_nmda ) ;
       t0 = t ;
       }
     }
   if ( _lflag  == 1.0 ) {
     on_nmda = 0.0 ;
     }
   if ( _lflag  == 0.0  && random < P_0 ) {
     F = 1.0 + ( F - 1.0 ) * exp ( - ( t - tsyn ) / tauF ) ;
     D1 = 1.0 - ( 1.0 - D1 ) * exp ( - ( t - tsyn ) / tauD1 ) ;
     D2 = 1.0 - ( 1.0 - D2 ) * exp ( - ( t - tsyn ) / tauD2 ) ;
     tsyn = t ;
     facfactor = F * D1 * D2 ;
     F = F * f ;
     if ( F > 30.0 ) {
       F = 30.0 ;
       }
     if ( facfactor < 0.2 ) {
       facfactor = 0.2 ;
       }
     D1 = D1 * d1 ;
     D2 = D2 * d2 ;
     }
   } }
 
double sfunc ( _threadargsprotocomma_ double _lv ) {
   double _lsfunc;
  _lsfunc = 1.0 / ( 1.0 + 0.33 * exp ( - 0.06 * _lv ) ) ;
    
return _lsfunc;
 }
 
static double _hoc_sfunc(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  sfunc ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
/*VERBATIM*/
double nrn_random_pick(void* r);
void* nrn_random_arg(int argpos);
 
double randGen ( _threadargsproto_ ) {
   double _lrandGen;
 
/*VERBATIM*/
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
 
return _lrandGen;
 }
 
static double _hoc_randGen(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  randGen ( _p, _ppvar, _thread, _nt );
 return(_r);
}
 
static int  setRandObjRef ( _threadargsproto_ ) {
   
/*VERBATIM*/
   void** pv4 = (void**)(&_p_randObjPtr);
   if (ifarg(1)) {
      *pv4 = nrn_random_arg(1);
   }else{
      *pv4 = (void*)0;
   }
  return 0; }
 
static double _hoc_setRandObjRef(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (NrnThread*)((Point_process*)_vptr)->_vnt;
 _r = 1.;
 setRandObjRef ( _p, _ppvar, _thread, _nt );
 return(_r);
}
 
static int _ode_count(int _type){ return 2;}
 
static void _ode_spec(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 2; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
  int _i; double _save;{
  W = W0;
  r_ampa = r_ampa0;
  r_nmda = r_nmda0;
 {
   on_nmda = 0.0 ;
   r_nmda = 0.0 ;
   W_nmda = initW ;
   on_ampa = 0.0 ;
   r_ampa = 0.0 ;
   W = initW ;
   t0 = - 1.0 ;
   fa = 0.0 ;
   F = 1.0 ;
   D1 = 1.0 ;
   D2 = 1.0 ;
   P = P_0 ;
   random = 1.0 ;
   }
 
}
}

static void nrn_init(NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _tsav = -1e20;
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   if ( t0 > 0.0 ) {
     if ( rp < thr_rp ) {
       if ( t - t0 < Cdur_ampa ) {
         on_ampa = 1.0 ;
         }
       else {
         on_ampa = 0.0 ;
         }
       }
     else {
       on_ampa = 0.0 ;
       }
     }
   g_nmda = gbar_nmda * r_nmda * facfactor ;
   inmda = W_nmda * g_nmda * ( v - Erev_nmda ) * sfunc ( _threadargscomma_ v ) ;
   g_ampa = gbar_ampa * r_ampa * facfactor ;
   iampa = W * g_ampa * ( v - Erev_ampa ) ;
   }
 _current += inmda;
 _current += iampa;

} return _current;
}

static void nrn_cur(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {   release(_p, _ppvar, _thread, _nt);
  }}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(r_nmda) - _p;  _dlist1[0] = &(Dr_nmda) - _p;
 _slist1[1] = &(r_ampa) - _p;  _dlist1[1] = &(Dr_ampa) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "/content/drive/MyDrive/Stylized-Cell-model/mechanisms/pyr2pyr.mod";
static const char* nmodl_file_text = 
  ":Pyramidal Cells to Pyramidal Cells AMPA+NMDA with local Ca2+ pool\n"
  "\n"
  "NEURON {\n"
  "	POINT_PROCESS pyr2pyr\n"
  "	:USEION ca READ eca	\n"
  "	NONSPECIFIC_CURRENT inmda, iampa\n"
  "	RANGE initW\n"
  "	RANGE Cdur_nmda, AlphaTmax_nmda, Beta_nmda, Erev_nmda, gbar_nmda, W_nmda, on_nmda, g_nmda\n"
  "	RANGE Cdur_ampa, AlphaTmax_ampa, Beta_ampa, Erev_ampa, gbar_ampa, W, on_ampa, g_ampa\n"
  "	:RANGE eca, ICa, P0, fCa, tauCa, iCatotal\n"
  "	:RANGE Cainf, pooldiam, z\n"
  "	:RANGE lambda1, lambda2, threshold1, threshold2\n"
  "	:RANGE fmax, fmin, Wmax, Wmin, maxChange, normW, scaleW, limitW, srcid,destid,tempW \n"
  "	:RANGE pregid, postgid\n"
  "	RANGE thr_rp\n"
  "	RANGE F, f, tauF, D1, d1, tauD1, D2, d2, tauD2\n"
  "	RANGE facfactor\n"
  "	:RANGE type\n"
  "	:RANGE neuroM\n"
  "\n"
  "	:Release probability\n"
  "	RANGE random, P, P_0\n"
  "\n"
  "	THREADSAFE\n"
  "	POINTER randObjPtr\n"
  "}\n"
  "\n"
  "UNITS { \n"
  "	(mV) = (millivolt)\n"
  "        (nA) = (nanoamp)\n"
  "	(uS) = (microsiemens)\n"
  "	FARADAY = 96485 (coul)\n"
  "	pi = 3.141592 (1)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "\n"
  "	:srcid = -1 (1)\n"
  "	:destid = -1 (1)\n"
  "	:type = -1\n"
  "	\n"
  "	Cdur_nmda = 16.7650 (ms)\n"
  "	AlphaTmax_nmda = .2659 (/ms)\n"
  "	Beta_nmda = 0.008 (/ms)\n"
  "	Erev_nmda = 0 (mV)\n"
  "	gbar_nmda = .5e-3 (uS)\n"
  "\n"
  "	Cdur_ampa = 1.4210 (ms)\n"
  "	AlphaTmax_ampa = 3.8142 (/ms)\n"
  "	Beta_ampa =  0.1429(/ms) :0.1429 as original 0.2858 as half,0.07145 as twice\n"
  "	Erev_ampa = 0 (mV)\n"
  "	gbar_ampa = 1e-3 (uS)\n"
  "\n"
  "	:eca = 120\n"
  "\n"
  "	:Cainf = 50e-6 (mM)\n"
  "	:pooldiam =  1.8172 (micrometer)\n"
  "	:z = 2\n"
  "	:neuroM = 0\n"
  "	:tauCa = 50 (ms)\n"
  "	:P0 = .015\n"
  "	:fCa = .024\n"
  "	\n"
  "	:lambda1 = 40 : 60 : 12 :80: 20 : 15 :8 :5: 2.5\n"
  "	:lambda2 = .03\n"
  "	:threshold1 = 0.4 :  0.45 : 0.35 :0.35:0.2 :0.50 (uM)\n"
  "	:threshold2 = 0.55 : 0.50 : 0.40 :0.4 :0.3 :0.60 (uM)\n"
  "\n"
  "	initW = 5.0 : 1.0 :  0.9 : 0.8 : 2 : 10 : 6 :1.5\n"
  "	:fmax = 3 : 2.5 : 4 : 2 : 3 : 1.5 : 3\n"
  "	:fmin = .8\n"
  "	\n"
  "	:DAstart1 = 39500\n"
  "	:DAstop1 = 40000	\n"
  "	:DAstart2 = 35900\n"
  "	:DAstop2 = 36000	\n"
  "\n"
  "	:DA_t1 = 1.2\n"
  "	:DA_t2 = 0.8 : 0.9\n"
  "    :DA_t3 = 0.9\n"
  "	:DA_S = 1.3 : 0.95 : 0.6	\n"
  "	:Beta1 = 0.001  (/ms) : 1/decay time for neuromodulators\n"
  "	:Beta2 = 0.0001  (/ms)\n"
  "\n"
  "	thr_rp = 1 : .7\n"
  "	\n"
  "	facfactor = 1\n"
  "	: the (1) is needed for the range limits to be effective\n"
  "        f = 0 (1) < 0, 1e9 >    : facilitation\n"
  "        tauF = 20 (ms) < 1e-9, 1e9 >\n"
  "        d1 = 0.95 (1) < 0, 1 >     : fast depression\n"
  "        tauD1 = 40 (ms) < 1e-9, 1e9 >\n"
  "        d2 = 0.9 (1) < 0, 1 >     : slow depression\n"
  "        tauD2 = 70 (ms) < 1e-9, 1e9 >	\n"
  "\n"
  "	P_0 = 1 (1) < 0, 1 >               : base release probability	\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	v (mV)\n"
  "\n"
  "	inmda (nA)\n"
  "	g_nmda (uS)\n"
  "	on_nmda\n"
  "	W_nmda\n"
  "\n"
  "	iampa (nA)\n"
  "	g_ampa (uS)\n"
  "	on_ampa\n"
  "	: W\n"
  "	limitW\n"
  "\n"
  "	t0 (ms)\n"
  "\n"
  "	:ICa (mA)\n"
  "	:Afactor	(mM/ms/nA)\n"
  "	:iCatotal (mA)\n"
  "\n"
  "	:dW_ampa\n"
  "	:Wmax\n"
  "	:Wmin\n"
  "	:maxChange\n"
  "	:normW\n"
  "	:scaleW\n"
  "	\n"
  "    :tempW\n"
  "	:pregid\n"
  "	:postgid\n"
  "\n"
  "	rp\n"
  "	tsyn\n"
  "	\n"
  "	fa\n"
  "	F\n"
  "	D1\n"
  "	D2\n"
  "\n"
  "	:Release probability\n"
  "	P				        : instantaneous release probability\n"
  "    randObjPtr              : pointer to a hoc random number generator Random.uniform(0,1)\n"
  "    random   \n"
  "}\n"
  "\n"
  "STATE { r_nmda r_ampa W}\n"
  "\n"
  "INITIAL {\n"
  "	on_nmda = 0\n"
  "	r_nmda = 0\n"
  "	W_nmda = initW\n"
  "\n"
  "	on_ampa = 0\n"
  "	r_ampa = 0\n"
  "	W = initW\n"
  "    :limitW = 1\n"
  "    \n"
  "	:tempW = initW\n"
  "	t0 = -1\n"
  "\n"
  "	:Wmax = fmax*initW\n"
  "	:Wmin = fmin*initW\n"
  "	:maxChange = (Wmax-Wmin)/10\n"
  "	:dW_ampa = 0\n"
  "\n"
  "	:capoolcon = Cainf\n"
  "	:Afactor	= 1/(z*FARADAY*4/3*pi*(pooldiam/2)^3)*(1e6)\n"
  "\n"
  "	fa =0\n"
  "	F = 1\n"
  "	D1 = 1\n"
  "	D2 = 1\n"
  "\n"
  "	P = P_0\n"
  "	random = 1\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "\n"
  ":if ((eta(capoolcon)*(lambda1*omega(capoolcon, threshold1, threshold2)-lambda2*W))>0&&W>=Wmax) {\n"
  ":        limitW=1e-12\n"
  ":	} else if ((eta(capoolcon)*(lambda1*omega(capoolcon, threshold1, threshold2)-lambda2*W))<0&&W<=Wmin) {\n"
  ":        limitW=1e-12\n"
  ":	} else {\n"
  ":	limitW=1 }\n"
  "	\n"
  "	SOLVE release METHOD cnexp\n"
  "	if (t0>0) {\n"
  "		if (rp < thr_rp) {\n"
  "			if (t-t0 < Cdur_ampa) {\n"
  "				on_ampa = 1\n"
  "			} else {\n"
  "				on_ampa = 0\n"
  "			}\n"
  "		} else {\n"
  "			on_ampa = 0\n"
  "		}\n"
  "	}\n"
  "          : if (W >= Wmax || W <= Wmin ) {     : for limiting the weight\n"
  "	 : limitW=1e-12\n"
  "	 : } else {\n"
  "	  : limitW=1\n"
  "	 : }\n"
  "	 \n"
  "	 :if (W > Wmax) { \n"
  "		:W = Wmax\n"
  "	:} else if (W < Wmin) {\n"
  " 		:W = Wmin\n"
  "	:}\n"
  "	 \n"
  "	:if (neuroM==1) {\n"
  "	:g_nmda = gbar_nmda*r_nmda*facfactor*DA1(DAstart1,DAstop1)*DA2(DAstart2,DAstop2)        : Dopamine effect on NMDA to reduce NMDA current amplitude\n"
  "	:	} else {\n"
  "	g_nmda = gbar_nmda*r_nmda*facfactor\n"
  ":		}\n"
  "	inmda = W_nmda*g_nmda*(v - Erev_nmda)*sfunc(v)\n"
  "\n"
  "	g_ampa = gbar_ampa*r_ampa*facfactor\n"
  "	iampa = W*g_ampa*(v - Erev_ampa)\n"
  "\n"
  "	:ICa = P0*g_nmda*(v - eca)*sfunc(v)\n"
  "	\n"
  "}\n"
  "\n"
  "DERIVATIVE release {\n"
  "	: W' = eta(capoolcon)*(lambda1*omega(capoolcon, threshold1, threshold2)-lambda2*W)	  : Long-term plasticity was implemented. (Shouval et al. 2002a, 2002b)\n"
  "	\n"
  "	:W' = 1e-12*limitW*eta(capoolcon)*(lambda1*omega(capoolcon, threshold1, threshold2)-lambda2*W)	  : Long-term plasticity was implemented. (Shouval et al. 2002a, 2002b)\n"
  "	r_nmda' = AlphaTmax_nmda*on_nmda*(1-r_nmda)-Beta_nmda*r_nmda\n"
  "	r_ampa' = AlphaTmax_ampa*on_ampa*(1-r_ampa)-Beta_ampa*r_ampa\n"
  "  	:capoolcon'= -fCa*Afactor*ICa + (Cainf-capoolcon)/tauCa\n"
  "}\n"
  "\n"
  "NET_RECEIVE(dummy_weight) {\n"
  "	random = randGen()\n"
  "\n"
  "	if (flag==0 && random < P_0) {           :a spike arrived, start onset state if not already on\n"
  "        if ((!on_nmda))\n"
  "		{       :this synpase joins the set of synapses in onset state\n"
  "        	t0=t\n"
  "			on_nmda=1		\n"
  "			net_send(Cdur_nmda,1)  \n"
  "        } \n"
  "		else if (on_nmda==1) {             :already in onset state, so move offset time\n"
  "        	net_move(t+Cdur_nmda)\n"
  "			t0=t\n"
  "	    }\n"
  "    }		\n"
  "\n"
  "	if (flag == 1) { : turn off transmitter, i.e. this synapse enters the offset state	\n"
  "		on_nmda=0\n"
  "    }\n"
  "	         \n"
  "	if (flag == 0 && random < P_0) {   : Short term plasticity was implemented(Varela et. al 1997):\n"
  "		F  = 1 + (F-1)* exp(-(t - tsyn)/tauF)\n"
  "		D1 = 1 - (1-D1)*exp(-(t - tsyn)/tauD1)\n"
  "		D2 = 1 - (1-D2)*exp(-(t - tsyn)/tauD2)\n"
  "		tsyn = t\n"
  "		\n"
  "		facfactor = F * D1 * D2	\n"
  "		F = F * f\n"
  "		if (F > 30) { \n"
  "			F=30	\n"
  "		}\n"
  "		if (facfactor < 0.2) { \n"
  "			facfactor=0.2\n"
  "		}	\n"
  "		D1 = D1 * d1\n"
  "		D2 = D2 * d2\n"
  "	}\n"
  "}\n"
  "\n"
  ":::::::::::: FUNCTIONs and PROCEDUREs ::::::::::::\n"
  "\n"
  "FUNCTION sfunc (v (mV)) {\n"
  "	UNITSOFF\n"
  "	sfunc = 1/(1+0.33*exp(-0.06*v))\n"
  "	UNITSON\n"
  "}\n"
  "\n"
  "VERBATIM\n"
  "double nrn_random_pick(void* r);\n"
  "void* nrn_random_arg(int argpos);\n"
  "ENDVERBATIM\n"
  "\n"
  "FUNCTION randGen() {\n"
  "VERBATIM\n"
  "   if (_p_randObjPtr) {\n"
  "      /*\n"
  "      :Supports separate independent but reproducible streams for\n"
  "      : each instance. However, the corresponding hoc Random\n"
  "      : distribution MUST be set to Random.uniform(0,1)\n"
  "      */\n"
  "      _lrandGen = nrn_random_pick(_p_randObjPtr);\n"
  "   }else{\n"
  "      hoc_execerror(\"Random object ref not set correctly for randObjPtr\",\" only via hoc Random\");\n"
  "   }\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "PROCEDURE setRandObjRef() {\n"
  "VERBATIM\n"
  "   void** pv4 = (void**)(&_p_randObjPtr);\n"
  "   if (ifarg(1)) {\n"
  "      *pv4 = nrn_random_arg(1);\n"
  "   }else{\n"
  "      *pv4 = (void*)0;\n"
  "   }\n"
  "ENDVERBATIM\n"
  "}\n"
  ":FUNCTION unirand() {    : uniform random numbers between 0 and 1\n"
  ":        unirand = scop_random()\n"
  ":}\n"
  ;
#endif
