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
  #define CASADI_PREFIX(ID) planar_drone_impl_dae_fun_jac_x_xdot_z_ ## ID
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
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[14] = {6, 6, 0, 0, 0, 2, 3, 4, 5, 3, 4, 0, 1, 2};
static const casadi_int casadi_s4[15] = {6, 6, 0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s5[3] = {6, 0, 0};

/* planar_drone_impl_dae_fun_jac_x_xdot_z:(i0[6],i1[6],i2[2],i3[],i4[],i5[])->(o0[6],o1[6x6,5nz],o2[6x6,6nz],o3[6x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5, a6, a7;
  a0=arg[1]? arg[1][0] : 0;
  a1=arg[0]? arg[0][3] : 0;
  a0=(a0-a1);
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[1]? arg[1][1] : 0;
  a1=arg[0]? arg[0][4] : 0;
  a0=(a0-a1);
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[1]? arg[1][2] : 0;
  a1=arg[0]? arg[0][5] : 0;
  a0=(a0-a1);
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[1]? arg[1][3] : 0;
  a1=arg[2]? arg[2][0] : 0;
  a2=arg[2]? arg[2][1] : 0;
  a3=(a1+a2);
  a4=arg[0]? arg[0][2] : 0;
  a5=sin(a4);
  a5=(a3*a5);
  a6=1.5000000000000000e+00;
  a5=(a5/a6);
  a0=(a0+a5);
  if (res[0]!=0) res[0][3]=a0;
  a0=arg[1]? arg[1][4] : 0;
  a5=(a1+a2);
  a7=cos(a4);
  a7=(a5*a7);
  a7=(a7/a6);
  a6=9.8100000000000005e+00;
  a7=(a7-a6);
  a0=(a0-a7);
  if (res[0]!=0) res[0][4]=a0;
  a0=arg[1]? arg[1][5] : 0;
  a2=(a2-a1);
  a1=5.0000000000000000e-01;
  a2=(a2/a1);
  a0=(a0-a2);
  if (res[0]!=0) res[0][5]=a0;
  a0=6.6666666666666663e-01;
  a2=cos(a4);
  a3=(a3*a2);
  a3=(a0*a3);
  if (res[1]!=0) res[1][0]=a3;
  a4=sin(a4);
  a5=(a5*a4);
  a0=(a0*a5);
  if (res[1]!=0) res[1][1]=a0;
  a0=-1.;
  if (res[1]!=0) res[1][2]=a0;
  if (res[1]!=0) res[1][3]=a0;
  if (res[1]!=0) res[1][4]=a0;
  a0=1.;
  if (res[2]!=0) res[2][0]=a0;
  if (res[2]!=0) res[2][1]=a0;
  if (res[2]!=0) res[2][2]=a0;
  if (res[2]!=0) res[2][3]=a0;
  if (res[2]!=0) res[2][4]=a0;
  if (res[2]!=0) res[2][5]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int planar_drone_impl_dae_fun_jac_x_xdot_z(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int planar_drone_impl_dae_fun_jac_x_xdot_z_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int planar_drone_impl_dae_fun_jac_x_xdot_z_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void planar_drone_impl_dae_fun_jac_x_xdot_z_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int planar_drone_impl_dae_fun_jac_x_xdot_z_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void planar_drone_impl_dae_fun_jac_x_xdot_z_release(int mem) {
}

CASADI_SYMBOL_EXPORT void planar_drone_impl_dae_fun_jac_x_xdot_z_incref(void) {
}

CASADI_SYMBOL_EXPORT void planar_drone_impl_dae_fun_jac_x_xdot_z_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int planar_drone_impl_dae_fun_jac_x_xdot_z_n_in(void) { return 6;}

CASADI_SYMBOL_EXPORT casadi_int planar_drone_impl_dae_fun_jac_x_xdot_z_n_out(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_real planar_drone_impl_dae_fun_jac_x_xdot_z_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* planar_drone_impl_dae_fun_jac_x_xdot_z_name_in(casadi_int i) {
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

CASADI_SYMBOL_EXPORT const char* planar_drone_impl_dae_fun_jac_x_xdot_z_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* planar_drone_impl_dae_fun_jac_x_xdot_z_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    case 4: return casadi_s2;
    case 5: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* planar_drone_impl_dae_fun_jac_x_xdot_z_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s3;
    case 2: return casadi_s4;
    case 3: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int planar_drone_impl_dae_fun_jac_x_xdot_z_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 4;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
