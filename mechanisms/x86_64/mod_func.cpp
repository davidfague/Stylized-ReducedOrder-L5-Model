#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;
#if defined(__cplusplus)
extern "C" {
#endif

extern void _AlphaSynapse1_reg(void);
extern void _CaDynamics_E2_reg(void);
extern void _CaDynamics_reg(void);
extern void _Ca_HVA_reg(void);
extern void _Ca_LVA_reg(void);
extern void _Ca_LVAst_reg(void);
extern void _epsp_reg(void);
extern void _Ih_reg(void);
extern void _Im_reg(void);
extern void _Im_v2_reg(void);
extern void _int2pyr_reg(void);
extern void _Kd_reg(void);
extern void _K_P_reg(void);
extern void _K_Pst_reg(void);
extern void _K_T_reg(void);
extern void _K_Tst_reg(void);
extern void _Kv2like_reg(void);
extern void _Kv3_1_reg(void);
extern void _Nap_Et2_reg(void);
extern void _Nap_reg(void);
extern void _NaTa_reg(void);
extern void _NaTa_t_reg(void);
extern void _NaTg_reg(void);
extern void _NaTs2_t_reg(void);
extern void _NaTs_reg(void);
extern void _NaV_reg(void);
extern void _netgaba_reg(void);
extern void _netglutamate_reg(void);
extern void _ProbAMPANMDA2_reg(void);
extern void _pyr2pyr_reg(void);
extern void _SK_E2_reg(void);
extern void _SK_reg(void);
extern void _SKv3_1_reg(void);
extern void _vecevent_reg(void);

void modl_reg(){
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");

    fprintf(stderr," \"AlphaSynapse1.mod\"");
    fprintf(stderr," \"CaDynamics_E2.mod\"");
    fprintf(stderr," \"CaDynamics.mod\"");
    fprintf(stderr," \"Ca_HVA.mod\"");
    fprintf(stderr," \"Ca_LVA.mod\"");
    fprintf(stderr," \"Ca_LVAst.mod\"");
    fprintf(stderr," \"epsp.mod\"");
    fprintf(stderr," \"Ih.mod\"");
    fprintf(stderr," \"Im.mod\"");
    fprintf(stderr," \"Im_v2.mod\"");
    fprintf(stderr," \"int2pyr.mod\"");
    fprintf(stderr," \"Kd.mod\"");
    fprintf(stderr," \"K_P.mod\"");
    fprintf(stderr," \"K_Pst.mod\"");
    fprintf(stderr," \"K_T.mod\"");
    fprintf(stderr," \"K_Tst.mod\"");
    fprintf(stderr," \"Kv2like.mod\"");
    fprintf(stderr," \"Kv3_1.mod\"");
    fprintf(stderr," \"Nap_Et2.mod\"");
    fprintf(stderr," \"Nap.mod\"");
    fprintf(stderr," \"NaTa.mod\"");
    fprintf(stderr," \"NaTa_t.mod\"");
    fprintf(stderr," \"NaTg.mod\"");
    fprintf(stderr," \"NaTs2_t.mod\"");
    fprintf(stderr," \"NaTs.mod\"");
    fprintf(stderr," \"NaV.mod\"");
    fprintf(stderr," \"netgaba.mod\"");
    fprintf(stderr," \"netglutamate.mod\"");
    fprintf(stderr," \"ProbAMPANMDA2.mod\"");
    fprintf(stderr," \"pyr2pyr.mod\"");
    fprintf(stderr," \"SK_E2.mod\"");
    fprintf(stderr," \"SK.mod\"");
    fprintf(stderr," \"SKv3_1.mod\"");
    fprintf(stderr," \"vecevent.mod\"");
    fprintf(stderr, "\n");
  }
  _AlphaSynapse1_reg();
  _CaDynamics_E2_reg();
  _CaDynamics_reg();
  _Ca_HVA_reg();
  _Ca_LVA_reg();
  _Ca_LVAst_reg();
  _epsp_reg();
  _Ih_reg();
  _Im_reg();
  _Im_v2_reg();
  _int2pyr_reg();
  _Kd_reg();
  _K_P_reg();
  _K_Pst_reg();
  _K_T_reg();
  _K_Tst_reg();
  _Kv2like_reg();
  _Kv3_1_reg();
  _Nap_Et2_reg();
  _Nap_reg();
  _NaTa_reg();
  _NaTa_t_reg();
  _NaTg_reg();
  _NaTs2_t_reg();
  _NaTs_reg();
  _NaV_reg();
  _netgaba_reg();
  _netglutamate_reg();
  _ProbAMPANMDA2_reg();
  _pyr2pyr_reg();
  _SK_E2_reg();
  _SK_reg();
  _SKv3_1_reg();
  _vecevent_reg();
}

#if defined(__cplusplus)
}
#endif
