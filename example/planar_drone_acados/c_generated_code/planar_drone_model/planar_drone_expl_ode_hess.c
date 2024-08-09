/* This file was automatically generated by CasADi 3.6.3.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) planar_drone_expl_ode_hess_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s1[45] = {6, 6, 0, 6, 12, 18, 24, 30, 36, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s2[17] = {6, 2, 0, 6, 12, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s3[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s4[3] = {0, 0, 0};
static const casadi_int casadi_s5[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s6[40] = {36, 1, 0, 36, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35};

/* planar_drone_expl_ode_hess:(i0[6],i1[6x6],i2[6x2],i3[6],i4[2],i5[])->(o0[8],o1[36]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a3, a4, a5, a6, a7, a8, a9;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[0]? arg[0][2] : 0;
  a1=sin(a0);
  a2=arg[4]? arg[4][0] : 0;
  a3=arg[4]? arg[4][1] : 0;
  a4=(a2+a3);
  a5=6.6666666666666663e-01;
  a6=arg[3]? arg[3][4] : 0;
  a6=(a5*a6);
  a4=(a4*a6);
  a7=(a1*a4);
  a8=cos(a0);
  a2=(a2+a3);
  a3=arg[3]? arg[3][3] : 0;
  a5=(a5*a3);
  a2=(a2*a5);
  a3=(a8*a2);
  a7=(a7+a3);
  a7=(-a7);
  if (res[0]!=0) res[0][2]=a7;
  a7=arg[3]? arg[3][0] : 0;
  if (res[0]!=0) res[0][3]=a7;
  a7=arg[3]? arg[3][1] : 0;
  if (res[0]!=0) res[0][4]=a7;
  a7=arg[3]? arg[3][2] : 0;
  if (res[0]!=0) res[0][5]=a7;
  a7=cos(a0);
  a7=(a7*a6);
  a3=2.;
  a9=arg[3]? arg[3][5] : 0;
  a3=(a3*a9);
  a9=(a7-a3);
  a10=sin(a0);
  a10=(a10*a5);
  a9=(a9-a10);
  if (res[0]!=0) res[0][6]=a9;
  a3=(a3+a7);
  a3=(a3-a10);
  if (res[0]!=0) res[0][7]=a3;
  a3=arg[1]? arg[1][2] : 0;
  a10=cos(a0);
  a7=(a10*a3);
  a7=(a4*a7);
  a9=sin(a0);
  a11=(a9*a3);
  a11=(a2*a11);
  a7=(a7-a11);
  a11=(a3*a7);
  a11=(-a11);
  if (res[1]!=0) res[1][0]=a11;
  a11=arg[1]? arg[1][8] : 0;
  a12=(a11*a7);
  a12=(-a12);
  if (res[1]!=0) res[1][1]=a12;
  a12=arg[1]? arg[1][14] : 0;
  a13=(a12*a7);
  a13=(-a13);
  if (res[1]!=0) res[1][2]=a13;
  a13=arg[1]? arg[1][20] : 0;
  a14=(a13*a7);
  a14=(-a14);
  if (res[1]!=0) res[1][3]=a14;
  a14=arg[1]? arg[1][26] : 0;
  a15=(a14*a7);
  a15=(-a15);
  if (res[1]!=0) res[1][4]=a15;
  a15=arg[1]? arg[1][32] : 0;
  a16=(a15*a7);
  a16=(-a16);
  if (res[1]!=0) res[1][5]=a16;
  a16=arg[2]? arg[2][2] : 0;
  a17=(a16*a7);
  a18=sin(a0);
  a19=(a18*a3);
  a19=(a6*a19);
  a0=cos(a0);
  a3=(a0*a3);
  a3=(a5*a3);
  a20=(a19+a3);
  a17=(a17+a20);
  a17=(-a17);
  if (res[1]!=0) res[1][6]=a17;
  a17=arg[2]? arg[2][8] : 0;
  a7=(a17*a7);
  a19=(a19+a3);
  a7=(a7+a19);
  a7=(-a7);
  if (res[1]!=0) res[1][7]=a7;
  a7=(a10*a11);
  a7=(a4*a7);
  a19=(a9*a11);
  a19=(a2*a19);
  a7=(a7-a19);
  a19=(a11*a7);
  a19=(-a19);
  if (res[1]!=0) res[1][8]=a19;
  a19=(a12*a7);
  a19=(-a19);
  if (res[1]!=0) res[1][9]=a19;
  a19=(a13*a7);
  a19=(-a19);
  if (res[1]!=0) res[1][10]=a19;
  a19=(a14*a7);
  a19=(-a19);
  if (res[1]!=0) res[1][11]=a19;
  a19=(a15*a7);
  a19=(-a19);
  if (res[1]!=0) res[1][12]=a19;
  a19=(a16*a7);
  a3=(a18*a11);
  a3=(a6*a3);
  a11=(a0*a11);
  a11=(a5*a11);
  a20=(a3+a11);
  a19=(a19+a20);
  a19=(-a19);
  if (res[1]!=0) res[1][13]=a19;
  a7=(a17*a7);
  a3=(a3+a11);
  a7=(a7+a3);
  a7=(-a7);
  if (res[1]!=0) res[1][14]=a7;
  a7=(a10*a12);
  a7=(a4*a7);
  a3=(a9*a12);
  a3=(a2*a3);
  a7=(a7-a3);
  a3=(a12*a7);
  a3=(-a3);
  if (res[1]!=0) res[1][15]=a3;
  a3=(a13*a7);
  a3=(-a3);
  if (res[1]!=0) res[1][16]=a3;
  a3=(a14*a7);
  a3=(-a3);
  if (res[1]!=0) res[1][17]=a3;
  a3=(a15*a7);
  a3=(-a3);
  if (res[1]!=0) res[1][18]=a3;
  a3=(a16*a7);
  a11=(a18*a12);
  a11=(a6*a11);
  a12=(a0*a12);
  a12=(a5*a12);
  a19=(a11+a12);
  a3=(a3+a19);
  a3=(-a3);
  if (res[1]!=0) res[1][19]=a3;
  a7=(a17*a7);
  a11=(a11+a12);
  a7=(a7+a11);
  a7=(-a7);
  if (res[1]!=0) res[1][20]=a7;
  a7=(a10*a13);
  a7=(a4*a7);
  a11=(a9*a13);
  a11=(a2*a11);
  a7=(a7-a11);
  a11=(a13*a7);
  a11=(-a11);
  if (res[1]!=0) res[1][21]=a11;
  a11=(a14*a7);
  a11=(-a11);
  if (res[1]!=0) res[1][22]=a11;
  a11=(a15*a7);
  a11=(-a11);
  if (res[1]!=0) res[1][23]=a11;
  a11=(a16*a7);
  a12=(a18*a13);
  a12=(a6*a12);
  a13=(a0*a13);
  a13=(a5*a13);
  a3=(a12+a13);
  a11=(a11+a3);
  a11=(-a11);
  if (res[1]!=0) res[1][24]=a11;
  a7=(a17*a7);
  a12=(a12+a13);
  a7=(a7+a12);
  a7=(-a7);
  if (res[1]!=0) res[1][25]=a7;
  a7=(a10*a14);
  a7=(a4*a7);
  a12=(a9*a14);
  a12=(a2*a12);
  a7=(a7-a12);
  a12=(a14*a7);
  a12=(-a12);
  if (res[1]!=0) res[1][26]=a12;
  a12=(a15*a7);
  a12=(-a12);
  if (res[1]!=0) res[1][27]=a12;
  a12=(a16*a7);
  a13=(a18*a14);
  a13=(a6*a13);
  a14=(a0*a14);
  a14=(a5*a14);
  a11=(a13+a14);
  a12=(a12+a11);
  a12=(-a12);
  if (res[1]!=0) res[1][28]=a12;
  a7=(a17*a7);
  a13=(a13+a14);
  a7=(a7+a13);
  a7=(-a7);
  if (res[1]!=0) res[1][29]=a7;
  a7=(a10*a15);
  a7=(a4*a7);
  a13=(a9*a15);
  a13=(a2*a13);
  a7=(a7-a13);
  a13=(a15*a7);
  a13=(-a13);
  if (res[1]!=0) res[1][30]=a13;
  a13=(a16*a7);
  a14=(a18*a15);
  a14=(a6*a14);
  a15=(a0*a15);
  a15=(a5*a15);
  a12=(a14+a15);
  a13=(a13+a12);
  a13=(-a13);
  if (res[1]!=0) res[1][31]=a13;
  a7=(a17*a7);
  a14=(a14+a15);
  a7=(a7+a14);
  a7=(-a7);
  if (res[1]!=0) res[1][32]=a7;
  a7=(a10*a16);
  a7=(a4*a7);
  a14=(a1*a6);
  a7=(a7+a14);
  a14=(a8*a5);
  a15=(a9*a16);
  a15=(a2*a15);
  a14=(a14-a15);
  a7=(a7+a14);
  a14=(a16*a7);
  a15=(a18*a16);
  a15=(a6*a15);
  a16=(a0*a16);
  a16=(a5*a16);
  a13=(a15+a16);
  a14=(a14+a13);
  a14=(-a14);
  if (res[1]!=0) res[1][33]=a14;
  a7=(a17*a7);
  a15=(a15+a16);
  a7=(a7+a15);
  a7=(-a7);
  if (res[1]!=0) res[1][34]=a7;
  a10=(a10*a17);
  a4=(a4*a10);
  a1=(a1*a6);
  a4=(a4+a1);
  a8=(a8*a5);
  a9=(a9*a17);
  a2=(a2*a9);
  a8=(a8-a2);
  a4=(a4+a8);
  a4=(a17*a4);
  a18=(a18*a17);
  a6=(a6*a18);
  a0=(a0*a17);
  a5=(a5*a0);
  a6=(a6+a5);
  a4=(a4+a6);
  a4=(-a4);
  if (res[1]!=0) res[1][35]=a4;
  return 0;
}

CASADI_SYMBOL_EXPORT int planar_drone_expl_ode_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int planar_drone_expl_ode_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int planar_drone_expl_ode_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void planar_drone_expl_ode_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int planar_drone_expl_ode_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void planar_drone_expl_ode_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void planar_drone_expl_ode_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void planar_drone_expl_ode_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int planar_drone_expl_ode_hess_n_in(void) { return 6;}

CASADI_SYMBOL_EXPORT casadi_int planar_drone_expl_ode_hess_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real planar_drone_expl_ode_hess_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* planar_drone_expl_ode_hess_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    case 5: return "i5";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* planar_drone_expl_ode_hess_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* planar_drone_expl_ode_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s0;
    case 4: return casadi_s3;
    case 5: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* planar_drone_expl_ode_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s5;
    case 1: return casadi_s6;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int planar_drone_expl_ode_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif