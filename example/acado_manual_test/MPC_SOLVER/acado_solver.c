/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


/** Row vector of size: 59 */
real_t state[ 59 ];

int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
#pragma omp parallel for private(lRun1, state) shared(acadoWorkspace, acadoVariables)
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
state[0] = acadoVariables.x[lRun1 * 6];
state[1] = acadoVariables.x[lRun1 * 6 + 1];
state[2] = acadoVariables.x[lRun1 * 6 + 2];
state[3] = acadoVariables.x[lRun1 * 6 + 3];
state[4] = acadoVariables.x[lRun1 * 6 + 4];
state[5] = acadoVariables.x[lRun1 * 6 + 5];

state[54] = acadoVariables.u[lRun1 * 2];
state[55] = acadoVariables.u[lRun1 * 2 + 1];
state[56] = acadoVariables.od[lRun1 * 3];
state[57] = acadoVariables.od[lRun1 * 3 + 1];
state[58] = acadoVariables.od[lRun1 * 3 + 2];

ret = acado_integrate(state, 1);

acadoWorkspace.d[lRun1 * 6] = state[0] - acadoVariables.x[lRun1 * 6 + 6];
acadoWorkspace.d[lRun1 * 6 + 1] = state[1] - acadoVariables.x[lRun1 * 6 + 7];
acadoWorkspace.d[lRun1 * 6 + 2] = state[2] - acadoVariables.x[lRun1 * 6 + 8];
acadoWorkspace.d[lRun1 * 6 + 3] = state[3] - acadoVariables.x[lRun1 * 6 + 9];
acadoWorkspace.d[lRun1 * 6 + 4] = state[4] - acadoVariables.x[lRun1 * 6 + 10];
acadoWorkspace.d[lRun1 * 6 + 5] = state[5] - acadoVariables.x[lRun1 * 6 + 11];

acadoWorkspace.evGx[lRun1 * 36] = state[6];
acadoWorkspace.evGx[lRun1 * 36 + 1] = state[7];
acadoWorkspace.evGx[lRun1 * 36 + 2] = state[8];
acadoWorkspace.evGx[lRun1 * 36 + 3] = state[9];
acadoWorkspace.evGx[lRun1 * 36 + 4] = state[10];
acadoWorkspace.evGx[lRun1 * 36 + 5] = state[11];
acadoWorkspace.evGx[lRun1 * 36 + 6] = state[12];
acadoWorkspace.evGx[lRun1 * 36 + 7] = state[13];
acadoWorkspace.evGx[lRun1 * 36 + 8] = state[14];
acadoWorkspace.evGx[lRun1 * 36 + 9] = state[15];
acadoWorkspace.evGx[lRun1 * 36 + 10] = state[16];
acadoWorkspace.evGx[lRun1 * 36 + 11] = state[17];
acadoWorkspace.evGx[lRun1 * 36 + 12] = state[18];
acadoWorkspace.evGx[lRun1 * 36 + 13] = state[19];
acadoWorkspace.evGx[lRun1 * 36 + 14] = state[20];
acadoWorkspace.evGx[lRun1 * 36 + 15] = state[21];
acadoWorkspace.evGx[lRun1 * 36 + 16] = state[22];
acadoWorkspace.evGx[lRun1 * 36 + 17] = state[23];
acadoWorkspace.evGx[lRun1 * 36 + 18] = state[24];
acadoWorkspace.evGx[lRun1 * 36 + 19] = state[25];
acadoWorkspace.evGx[lRun1 * 36 + 20] = state[26];
acadoWorkspace.evGx[lRun1 * 36 + 21] = state[27];
acadoWorkspace.evGx[lRun1 * 36 + 22] = state[28];
acadoWorkspace.evGx[lRun1 * 36 + 23] = state[29];
acadoWorkspace.evGx[lRun1 * 36 + 24] = state[30];
acadoWorkspace.evGx[lRun1 * 36 + 25] = state[31];
acadoWorkspace.evGx[lRun1 * 36 + 26] = state[32];
acadoWorkspace.evGx[lRun1 * 36 + 27] = state[33];
acadoWorkspace.evGx[lRun1 * 36 + 28] = state[34];
acadoWorkspace.evGx[lRun1 * 36 + 29] = state[35];
acadoWorkspace.evGx[lRun1 * 36 + 30] = state[36];
acadoWorkspace.evGx[lRun1 * 36 + 31] = state[37];
acadoWorkspace.evGx[lRun1 * 36 + 32] = state[38];
acadoWorkspace.evGx[lRun1 * 36 + 33] = state[39];
acadoWorkspace.evGx[lRun1 * 36 + 34] = state[40];
acadoWorkspace.evGx[lRun1 * 36 + 35] = state[41];

acadoWorkspace.evGu[lRun1 * 12] = state[42];
acadoWorkspace.evGu[lRun1 * 12 + 1] = state[43];
acadoWorkspace.evGu[lRun1 * 12 + 2] = state[44];
acadoWorkspace.evGu[lRun1 * 12 + 3] = state[45];
acadoWorkspace.evGu[lRun1 * 12 + 4] = state[46];
acadoWorkspace.evGu[lRun1 * 12 + 5] = state[47];
acadoWorkspace.evGu[lRun1 * 12 + 6] = state[48];
acadoWorkspace.evGu[lRun1 * 12 + 7] = state[49];
acadoWorkspace.evGu[lRun1 * 12 + 8] = state[50];
acadoWorkspace.evGu[lRun1 * 12 + 9] = state[51];
acadoWorkspace.evGu[lRun1 * 12 + 10] = state[52];
acadoWorkspace.evGu[lRun1 * 12 + 11] = state[53];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 6;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = u[0];
out[7] = u[1];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ2[35] = +tmpObjS[35];
tmpQ2[36] = +tmpObjS[36];
tmpQ2[37] = +tmpObjS[37];
tmpQ2[38] = +tmpObjS[38];
tmpQ2[39] = +tmpObjS[39];
tmpQ2[40] = +tmpObjS[40];
tmpQ2[41] = +tmpObjS[41];
tmpQ2[42] = +tmpObjS[42];
tmpQ2[43] = +tmpObjS[43];
tmpQ2[44] = +tmpObjS[44];
tmpQ2[45] = +tmpObjS[45];
tmpQ2[46] = +tmpObjS[46];
tmpQ2[47] = +tmpObjS[47];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[8];
tmpQ1[7] = + tmpQ2[9];
tmpQ1[8] = + tmpQ2[10];
tmpQ1[9] = + tmpQ2[11];
tmpQ1[10] = + tmpQ2[12];
tmpQ1[11] = + tmpQ2[13];
tmpQ1[12] = + tmpQ2[16];
tmpQ1[13] = + tmpQ2[17];
tmpQ1[14] = + tmpQ2[18];
tmpQ1[15] = + tmpQ2[19];
tmpQ1[16] = + tmpQ2[20];
tmpQ1[17] = + tmpQ2[21];
tmpQ1[18] = + tmpQ2[24];
tmpQ1[19] = + tmpQ2[25];
tmpQ1[20] = + tmpQ2[26];
tmpQ1[21] = + tmpQ2[27];
tmpQ1[22] = + tmpQ2[28];
tmpQ1[23] = + tmpQ2[29];
tmpQ1[24] = + tmpQ2[32];
tmpQ1[25] = + tmpQ2[33];
tmpQ1[26] = + tmpQ2[34];
tmpQ1[27] = + tmpQ2[35];
tmpQ1[28] = + tmpQ2[36];
tmpQ1[29] = + tmpQ2[37];
tmpQ1[30] = + tmpQ2[40];
tmpQ1[31] = + tmpQ2[41];
tmpQ1[32] = + tmpQ2[42];
tmpQ1[33] = + tmpQ2[43];
tmpQ1[34] = + tmpQ2[44];
tmpQ1[35] = + tmpQ2[45];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[48];
tmpR2[1] = +tmpObjS[49];
tmpR2[2] = +tmpObjS[50];
tmpR2[3] = +tmpObjS[51];
tmpR2[4] = +tmpObjS[52];
tmpR2[5] = +tmpObjS[53];
tmpR2[6] = +tmpObjS[54];
tmpR2[7] = +tmpObjS[55];
tmpR2[8] = +tmpObjS[56];
tmpR2[9] = +tmpObjS[57];
tmpR2[10] = +tmpObjS[58];
tmpR2[11] = +tmpObjS[59];
tmpR2[12] = +tmpObjS[60];
tmpR2[13] = +tmpObjS[61];
tmpR2[14] = +tmpObjS[62];
tmpR2[15] = +tmpObjS[63];
tmpR1[0] = + tmpR2[6];
tmpR1[1] = + tmpR2[7];
tmpR1[2] = + tmpR2[14];
tmpR1[3] = + tmpR2[15];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
tmpQN1[16] = + tmpQN2[16];
tmpQN1[17] = + tmpQN2[17];
tmpQN1[18] = + tmpQN2[18];
tmpQN1[19] = + tmpQN2[19];
tmpQN1[20] = + tmpQN2[20];
tmpQN1[21] = + tmpQN2[21];
tmpQN1[22] = + tmpQN2[22];
tmpQN1[23] = + tmpQN2[23];
tmpQN1[24] = + tmpQN2[24];
tmpQN1[25] = + tmpQN2[25];
tmpQN1[26] = + tmpQN2[26];
tmpQN1[27] = + tmpQN2[27];
tmpQN1[28] = + tmpQN2[28];
tmpQN1[29] = + tmpQN2[29];
tmpQN1[30] = + tmpQN2[30];
tmpQN1[31] = + tmpQN2[31];
tmpQN1[32] = + tmpQN2[32];
tmpQN1[33] = + tmpQN2[33];
tmpQN1[34] = + tmpQN2[34];
tmpQN1[35] = + tmpQN2[35];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 6];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 6 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 6 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 6 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 6 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 6 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[7] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 3];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 3 + 1];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 8] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 8 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 8 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 8 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 8 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 8 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 8 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 8 + 7] = acadoWorkspace.objValueOut[7];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 64 ]), &(acadoWorkspace.Q1[ runObj * 36 ]), &(acadoWorkspace.Q2[ runObj * 48 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 64 ]), &(acadoWorkspace.R1[ runObj * 4 ]), &(acadoWorkspace.R2[ runObj * 16 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.x[124];
acadoWorkspace.objValueIn[5] = acadoVariables.x[125];
acadoWorkspace.objValueIn[6] = acadoVariables.od[60];
acadoWorkspace.objValueIn[7] = acadoVariables.od[61];
acadoWorkspace.objValueIn[8] = acadoVariables.od[62];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8] + Gx1[5]*Gu1[10];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9] + Gx1[5]*Gu1[11];
Gu2[2] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[2] + Gx1[8]*Gu1[4] + Gx1[9]*Gu1[6] + Gx1[10]*Gu1[8] + Gx1[11]*Gu1[10];
Gu2[3] = + Gx1[6]*Gu1[1] + Gx1[7]*Gu1[3] + Gx1[8]*Gu1[5] + Gx1[9]*Gu1[7] + Gx1[10]*Gu1[9] + Gx1[11]*Gu1[11];
Gu2[4] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[2] + Gx1[14]*Gu1[4] + Gx1[15]*Gu1[6] + Gx1[16]*Gu1[8] + Gx1[17]*Gu1[10];
Gu2[5] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[5] + Gx1[15]*Gu1[7] + Gx1[16]*Gu1[9] + Gx1[17]*Gu1[11];
Gu2[6] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[2] + Gx1[20]*Gu1[4] + Gx1[21]*Gu1[6] + Gx1[22]*Gu1[8] + Gx1[23]*Gu1[10];
Gu2[7] = + Gx1[18]*Gu1[1] + Gx1[19]*Gu1[3] + Gx1[20]*Gu1[5] + Gx1[21]*Gu1[7] + Gx1[22]*Gu1[9] + Gx1[23]*Gu1[11];
Gu2[8] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[2] + Gx1[26]*Gu1[4] + Gx1[27]*Gu1[6] + Gx1[28]*Gu1[8] + Gx1[29]*Gu1[10];
Gu2[9] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[3] + Gx1[26]*Gu1[5] + Gx1[27]*Gu1[7] + Gx1[28]*Gu1[9] + Gx1[29]*Gu1[11];
Gu2[10] = + Gx1[30]*Gu1[0] + Gx1[31]*Gu1[2] + Gx1[32]*Gu1[4] + Gx1[33]*Gu1[6] + Gx1[34]*Gu1[8] + Gx1[35]*Gu1[10];
Gu2[11] = + Gx1[30]*Gu1[1] + Gx1[31]*Gu1[3] + Gx1[32]*Gu1[5] + Gx1[33]*Gu1[7] + Gx1[34]*Gu1[9] + Gx1[35]*Gu1[11];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 80) + (iCol * 2)] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + Gu1[10]*Gu2[10];
acadoWorkspace.H[(iRow * 80) + (iCol * 2 + 1)] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + Gu1[10]*Gu2[11];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2)] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + Gu1[11]*Gu2[10];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2 + 1)] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + Gu1[11]*Gu2[11];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 82] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + Gu1[10]*Gu2[10] + R11[0];
acadoWorkspace.H[iRow * 82 + 1] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + Gu1[10]*Gu2[11] + R11[1];
acadoWorkspace.H[iRow * 82 + 40] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + Gu1[11]*Gu2[10] + R11[2];
acadoWorkspace.H[iRow * 82 + 41] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + Gu1[11]*Gu2[11] + R11[3];
acadoWorkspace.H[iRow * 82] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 82 + 41] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[6]*Gu1[2] + Gx1[12]*Gu1[4] + Gx1[18]*Gu1[6] + Gx1[24]*Gu1[8] + Gx1[30]*Gu1[10];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[6]*Gu1[3] + Gx1[12]*Gu1[5] + Gx1[18]*Gu1[7] + Gx1[24]*Gu1[9] + Gx1[30]*Gu1[11];
Gu2[2] = + Gx1[1]*Gu1[0] + Gx1[7]*Gu1[2] + Gx1[13]*Gu1[4] + Gx1[19]*Gu1[6] + Gx1[25]*Gu1[8] + Gx1[31]*Gu1[10];
Gu2[3] = + Gx1[1]*Gu1[1] + Gx1[7]*Gu1[3] + Gx1[13]*Gu1[5] + Gx1[19]*Gu1[7] + Gx1[25]*Gu1[9] + Gx1[31]*Gu1[11];
Gu2[4] = + Gx1[2]*Gu1[0] + Gx1[8]*Gu1[2] + Gx1[14]*Gu1[4] + Gx1[20]*Gu1[6] + Gx1[26]*Gu1[8] + Gx1[32]*Gu1[10];
Gu2[5] = + Gx1[2]*Gu1[1] + Gx1[8]*Gu1[3] + Gx1[14]*Gu1[5] + Gx1[20]*Gu1[7] + Gx1[26]*Gu1[9] + Gx1[32]*Gu1[11];
Gu2[6] = + Gx1[3]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[15]*Gu1[4] + Gx1[21]*Gu1[6] + Gx1[27]*Gu1[8] + Gx1[33]*Gu1[10];
Gu2[7] = + Gx1[3]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[15]*Gu1[5] + Gx1[21]*Gu1[7] + Gx1[27]*Gu1[9] + Gx1[33]*Gu1[11];
Gu2[8] = + Gx1[4]*Gu1[0] + Gx1[10]*Gu1[2] + Gx1[16]*Gu1[4] + Gx1[22]*Gu1[6] + Gx1[28]*Gu1[8] + Gx1[34]*Gu1[10];
Gu2[9] = + Gx1[4]*Gu1[1] + Gx1[10]*Gu1[3] + Gx1[16]*Gu1[5] + Gx1[22]*Gu1[7] + Gx1[28]*Gu1[9] + Gx1[34]*Gu1[11];
Gu2[10] = + Gx1[5]*Gu1[0] + Gx1[11]*Gu1[2] + Gx1[17]*Gu1[4] + Gx1[23]*Gu1[6] + Gx1[29]*Gu1[8] + Gx1[35]*Gu1[10];
Gu2[11] = + Gx1[5]*Gu1[1] + Gx1[11]*Gu1[3] + Gx1[17]*Gu1[5] + Gx1[23]*Gu1[7] + Gx1[29]*Gu1[9] + Gx1[35]*Gu1[11];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[2] + Q11[2]*Gu1[4] + Q11[3]*Gu1[6] + Q11[4]*Gu1[8] + Q11[5]*Gu1[10] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[3] + Q11[2]*Gu1[5] + Q11[3]*Gu1[7] + Q11[4]*Gu1[9] + Q11[5]*Gu1[11] + Gu2[1];
Gu3[2] = + Q11[6]*Gu1[0] + Q11[7]*Gu1[2] + Q11[8]*Gu1[4] + Q11[9]*Gu1[6] + Q11[10]*Gu1[8] + Q11[11]*Gu1[10] + Gu2[2];
Gu3[3] = + Q11[6]*Gu1[1] + Q11[7]*Gu1[3] + Q11[8]*Gu1[5] + Q11[9]*Gu1[7] + Q11[10]*Gu1[9] + Q11[11]*Gu1[11] + Gu2[3];
Gu3[4] = + Q11[12]*Gu1[0] + Q11[13]*Gu1[2] + Q11[14]*Gu1[4] + Q11[15]*Gu1[6] + Q11[16]*Gu1[8] + Q11[17]*Gu1[10] + Gu2[4];
Gu3[5] = + Q11[12]*Gu1[1] + Q11[13]*Gu1[3] + Q11[14]*Gu1[5] + Q11[15]*Gu1[7] + Q11[16]*Gu1[9] + Q11[17]*Gu1[11] + Gu2[5];
Gu3[6] = + Q11[18]*Gu1[0] + Q11[19]*Gu1[2] + Q11[20]*Gu1[4] + Q11[21]*Gu1[6] + Q11[22]*Gu1[8] + Q11[23]*Gu1[10] + Gu2[6];
Gu3[7] = + Q11[18]*Gu1[1] + Q11[19]*Gu1[3] + Q11[20]*Gu1[5] + Q11[21]*Gu1[7] + Q11[22]*Gu1[9] + Q11[23]*Gu1[11] + Gu2[7];
Gu3[8] = + Q11[24]*Gu1[0] + Q11[25]*Gu1[2] + Q11[26]*Gu1[4] + Q11[27]*Gu1[6] + Q11[28]*Gu1[8] + Q11[29]*Gu1[10] + Gu2[8];
Gu3[9] = + Q11[24]*Gu1[1] + Q11[25]*Gu1[3] + Q11[26]*Gu1[5] + Q11[27]*Gu1[7] + Q11[28]*Gu1[9] + Q11[29]*Gu1[11] + Gu2[9];
Gu3[10] = + Q11[30]*Gu1[0] + Q11[31]*Gu1[2] + Q11[32]*Gu1[4] + Q11[33]*Gu1[6] + Q11[34]*Gu1[8] + Q11[35]*Gu1[10] + Gu2[10];
Gu3[11] = + Q11[30]*Gu1[1] + Q11[31]*Gu1[3] + Q11[32]*Gu1[5] + Q11[33]*Gu1[7] + Q11[34]*Gu1[9] + Q11[35]*Gu1[11] + Gu2[11];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[6]*w11[1] + Gx1[12]*w11[2] + Gx1[18]*w11[3] + Gx1[24]*w11[4] + Gx1[30]*w11[5] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[7]*w11[1] + Gx1[13]*w11[2] + Gx1[19]*w11[3] + Gx1[25]*w11[4] + Gx1[31]*w11[5] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[8]*w11[1] + Gx1[14]*w11[2] + Gx1[20]*w11[3] + Gx1[26]*w11[4] + Gx1[32]*w11[5] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[9]*w11[1] + Gx1[15]*w11[2] + Gx1[21]*w11[3] + Gx1[27]*w11[4] + Gx1[33]*w11[5] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[10]*w11[1] + Gx1[16]*w11[2] + Gx1[22]*w11[3] + Gx1[28]*w11[4] + Gx1[34]*w11[5] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[11]*w11[1] + Gx1[17]*w11[2] + Gx1[23]*w11[3] + Gx1[29]*w11[4] + Gx1[35]*w11[5] + w12[5];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[2]*w11[1] + Gu1[4]*w11[2] + Gu1[6]*w11[3] + Gu1[8]*w11[4] + Gu1[10]*w11[5];
U1[1] += + Gu1[1]*w11[0] + Gu1[3]*w11[1] + Gu1[5]*w11[2] + Gu1[7]*w11[3] + Gu1[9]*w11[4] + Gu1[11]*w11[5];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + w12[0];
w13[1] = + Q11[6]*w11[0] + Q11[7]*w11[1] + Q11[8]*w11[2] + Q11[9]*w11[3] + Q11[10]*w11[4] + Q11[11]*w11[5] + w12[1];
w13[2] = + Q11[12]*w11[0] + Q11[13]*w11[1] + Q11[14]*w11[2] + Q11[15]*w11[3] + Q11[16]*w11[4] + Q11[17]*w11[5] + w12[2];
w13[3] = + Q11[18]*w11[0] + Q11[19]*w11[1] + Q11[20]*w11[2] + Q11[21]*w11[3] + Q11[22]*w11[4] + Q11[23]*w11[5] + w12[3];
w13[4] = + Q11[24]*w11[0] + Q11[25]*w11[1] + Q11[26]*w11[2] + Q11[27]*w11[3] + Q11[28]*w11[4] + Q11[29]*w11[5] + w12[4];
w13[5] = + Q11[30]*w11[0] + Q11[31]*w11[1] + Q11[32]*w11[2] + Q11[33]*w11[3] + Q11[34]*w11[4] + Q11[35]*w11[5] + w12[5];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5];
w12[1] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2] + Gx1[9]*w11[3] + Gx1[10]*w11[4] + Gx1[11]*w11[5];
w12[2] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3] + Gx1[16]*w11[4] + Gx1[17]*w11[5];
w12[3] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5];
w12[4] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5];
w12[5] += + Gx1[30]*w11[0] + Gx1[31]*w11[1] + Gx1[32]*w11[2] + Gx1[33]*w11[3] + Gx1[34]*w11[4] + Gx1[35]*w11[5];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5];
w12[1] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2] + Gx1[9]*w11[3] + Gx1[10]*w11[4] + Gx1[11]*w11[5];
w12[2] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3] + Gx1[16]*w11[4] + Gx1[17]*w11[5];
w12[3] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5];
w12[4] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5];
w12[5] += + Gx1[30]*w11[0] + Gx1[31]*w11[1] + Gx1[32]*w11[2] + Gx1[33]*w11[3] + Gx1[34]*w11[4] + Gx1[35]*w11[5];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1];
w12[1] += + Gu1[2]*U1[0] + Gu1[3]*U1[1];
w12[2] += + Gu1[4]*U1[0] + Gu1[5]*U1[1];
w12[3] += + Gu1[6]*U1[0] + Gu1[7]*U1[1];
w12[4] += + Gu1[8]*U1[0] + Gu1[9]*U1[1];
w12[5] += + Gu1[10]*U1[0] + Gu1[11]*U1[1];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 80) + (iCol * 2)] = acadoWorkspace.H[(iCol * 80) + (iRow * 2)];
acadoWorkspace.H[(iRow * 80) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 80 + 40) + (iRow * 2)];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2)] = acadoWorkspace.H[(iCol * 80) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 80 + 40) + (iRow * 2 + 1)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7];
RDy1[1] = + R2[8]*Dy1[0] + R2[9]*Dy1[1] + R2[10]*Dy1[2] + R2[11]*Dy1[3] + R2[12]*Dy1[4] + R2[13]*Dy1[5] + R2[14]*Dy1[6] + R2[15]*Dy1[7];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7];
QDy1[1] = + Q2[8]*Dy1[0] + Q2[9]*Dy1[1] + Q2[10]*Dy1[2] + Q2[11]*Dy1[3] + Q2[12]*Dy1[4] + Q2[13]*Dy1[5] + Q2[14]*Dy1[6] + Q2[15]*Dy1[7];
QDy1[2] = + Q2[16]*Dy1[0] + Q2[17]*Dy1[1] + Q2[18]*Dy1[2] + Q2[19]*Dy1[3] + Q2[20]*Dy1[4] + Q2[21]*Dy1[5] + Q2[22]*Dy1[6] + Q2[23]*Dy1[7];
QDy1[3] = + Q2[24]*Dy1[0] + Q2[25]*Dy1[1] + Q2[26]*Dy1[2] + Q2[27]*Dy1[3] + Q2[28]*Dy1[4] + Q2[29]*Dy1[5] + Q2[30]*Dy1[6] + Q2[31]*Dy1[7];
QDy1[4] = + Q2[32]*Dy1[0] + Q2[33]*Dy1[1] + Q2[34]*Dy1[2] + Q2[35]*Dy1[3] + Q2[36]*Dy1[4] + Q2[37]*Dy1[5] + Q2[38]*Dy1[6] + Q2[39]*Dy1[7];
QDy1[5] = + Q2[40]*Dy1[0] + Q2[41]*Dy1[1] + Q2[42]*Dy1[2] + Q2[43]*Dy1[3] + Q2[44]*Dy1[4] + Q2[45]*Dy1[5] + Q2[46]*Dy1[6] + Q2[47]*Dy1[7];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 120 */
static const int xBoundIndices[ 120 ] = 
{ 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125 };
for (lRun2 = 0; lRun2 < 20; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 41)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 12 ]), &(acadoWorkspace.E[ lRun3 * 12 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 20; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (6)) * (6)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (6)) * (2)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (6)) * (2)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (20)) - (1)) * (6)) * (2)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 19; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 12 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 36 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 36 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (6)) * (2)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 4 ]), &(acadoWorkspace.evGu[ lRun2 * 12 ]), acadoWorkspace.W1, lRun2 );
}

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );
acado_copyHTH( 0, 10 );
acado_copyHTH( 1, 10 );
acado_copyHTH( 2, 10 );
acado_copyHTH( 3, 10 );
acado_copyHTH( 4, 10 );
acado_copyHTH( 5, 10 );
acado_copyHTH( 6, 10 );
acado_copyHTH( 7, 10 );
acado_copyHTH( 8, 10 );
acado_copyHTH( 9, 10 );
acado_copyHTH( 0, 11 );
acado_copyHTH( 1, 11 );
acado_copyHTH( 2, 11 );
acado_copyHTH( 3, 11 );
acado_copyHTH( 4, 11 );
acado_copyHTH( 5, 11 );
acado_copyHTH( 6, 11 );
acado_copyHTH( 7, 11 );
acado_copyHTH( 8, 11 );
acado_copyHTH( 9, 11 );
acado_copyHTH( 10, 11 );
acado_copyHTH( 0, 12 );
acado_copyHTH( 1, 12 );
acado_copyHTH( 2, 12 );
acado_copyHTH( 3, 12 );
acado_copyHTH( 4, 12 );
acado_copyHTH( 5, 12 );
acado_copyHTH( 6, 12 );
acado_copyHTH( 7, 12 );
acado_copyHTH( 8, 12 );
acado_copyHTH( 9, 12 );
acado_copyHTH( 10, 12 );
acado_copyHTH( 11, 12 );
acado_copyHTH( 0, 13 );
acado_copyHTH( 1, 13 );
acado_copyHTH( 2, 13 );
acado_copyHTH( 3, 13 );
acado_copyHTH( 4, 13 );
acado_copyHTH( 5, 13 );
acado_copyHTH( 6, 13 );
acado_copyHTH( 7, 13 );
acado_copyHTH( 8, 13 );
acado_copyHTH( 9, 13 );
acado_copyHTH( 10, 13 );
acado_copyHTH( 11, 13 );
acado_copyHTH( 12, 13 );
acado_copyHTH( 0, 14 );
acado_copyHTH( 1, 14 );
acado_copyHTH( 2, 14 );
acado_copyHTH( 3, 14 );
acado_copyHTH( 4, 14 );
acado_copyHTH( 5, 14 );
acado_copyHTH( 6, 14 );
acado_copyHTH( 7, 14 );
acado_copyHTH( 8, 14 );
acado_copyHTH( 9, 14 );
acado_copyHTH( 10, 14 );
acado_copyHTH( 11, 14 );
acado_copyHTH( 12, 14 );
acado_copyHTH( 13, 14 );
acado_copyHTH( 0, 15 );
acado_copyHTH( 1, 15 );
acado_copyHTH( 2, 15 );
acado_copyHTH( 3, 15 );
acado_copyHTH( 4, 15 );
acado_copyHTH( 5, 15 );
acado_copyHTH( 6, 15 );
acado_copyHTH( 7, 15 );
acado_copyHTH( 8, 15 );
acado_copyHTH( 9, 15 );
acado_copyHTH( 10, 15 );
acado_copyHTH( 11, 15 );
acado_copyHTH( 12, 15 );
acado_copyHTH( 13, 15 );
acado_copyHTH( 14, 15 );
acado_copyHTH( 0, 16 );
acado_copyHTH( 1, 16 );
acado_copyHTH( 2, 16 );
acado_copyHTH( 3, 16 );
acado_copyHTH( 4, 16 );
acado_copyHTH( 5, 16 );
acado_copyHTH( 6, 16 );
acado_copyHTH( 7, 16 );
acado_copyHTH( 8, 16 );
acado_copyHTH( 9, 16 );
acado_copyHTH( 10, 16 );
acado_copyHTH( 11, 16 );
acado_copyHTH( 12, 16 );
acado_copyHTH( 13, 16 );
acado_copyHTH( 14, 16 );
acado_copyHTH( 15, 16 );
acado_copyHTH( 0, 17 );
acado_copyHTH( 1, 17 );
acado_copyHTH( 2, 17 );
acado_copyHTH( 3, 17 );
acado_copyHTH( 4, 17 );
acado_copyHTH( 5, 17 );
acado_copyHTH( 6, 17 );
acado_copyHTH( 7, 17 );
acado_copyHTH( 8, 17 );
acado_copyHTH( 9, 17 );
acado_copyHTH( 10, 17 );
acado_copyHTH( 11, 17 );
acado_copyHTH( 12, 17 );
acado_copyHTH( 13, 17 );
acado_copyHTH( 14, 17 );
acado_copyHTH( 15, 17 );
acado_copyHTH( 16, 17 );
acado_copyHTH( 0, 18 );
acado_copyHTH( 1, 18 );
acado_copyHTH( 2, 18 );
acado_copyHTH( 3, 18 );
acado_copyHTH( 4, 18 );
acado_copyHTH( 5, 18 );
acado_copyHTH( 6, 18 );
acado_copyHTH( 7, 18 );
acado_copyHTH( 8, 18 );
acado_copyHTH( 9, 18 );
acado_copyHTH( 10, 18 );
acado_copyHTH( 11, 18 );
acado_copyHTH( 12, 18 );
acado_copyHTH( 13, 18 );
acado_copyHTH( 14, 18 );
acado_copyHTH( 15, 18 );
acado_copyHTH( 16, 18 );
acado_copyHTH( 17, 18 );
acado_copyHTH( 0, 19 );
acado_copyHTH( 1, 19 );
acado_copyHTH( 2, 19 );
acado_copyHTH( 3, 19 );
acado_copyHTH( 4, 19 );
acado_copyHTH( 5, 19 );
acado_copyHTH( 6, 19 );
acado_copyHTH( 7, 19 );
acado_copyHTH( 8, 19 );
acado_copyHTH( 9, 19 );
acado_copyHTH( 10, 19 );
acado_copyHTH( 11, 19 );
acado_copyHTH( 12, 19 );
acado_copyHTH( 13, 19 );
acado_copyHTH( 14, 19 );
acado_copyHTH( 15, 19 );
acado_copyHTH( 16, 19 );
acado_copyHTH( 17, 19 );
acado_copyHTH( 18, 19 );

acadoWorkspace.sbar[6] = acadoWorkspace.d[0];
acadoWorkspace.sbar[7] = acadoWorkspace.d[1];
acadoWorkspace.sbar[8] = acadoWorkspace.d[2];
acadoWorkspace.sbar[9] = acadoWorkspace.d[3];
acadoWorkspace.sbar[10] = acadoWorkspace.d[4];
acadoWorkspace.sbar[11] = acadoWorkspace.d[5];
acadoWorkspace.sbar[12] = acadoWorkspace.d[6];
acadoWorkspace.sbar[13] = acadoWorkspace.d[7];
acadoWorkspace.sbar[14] = acadoWorkspace.d[8];
acadoWorkspace.sbar[15] = acadoWorkspace.d[9];
acadoWorkspace.sbar[16] = acadoWorkspace.d[10];
acadoWorkspace.sbar[17] = acadoWorkspace.d[11];
acadoWorkspace.sbar[18] = acadoWorkspace.d[12];
acadoWorkspace.sbar[19] = acadoWorkspace.d[13];
acadoWorkspace.sbar[20] = acadoWorkspace.d[14];
acadoWorkspace.sbar[21] = acadoWorkspace.d[15];
acadoWorkspace.sbar[22] = acadoWorkspace.d[16];
acadoWorkspace.sbar[23] = acadoWorkspace.d[17];
acadoWorkspace.sbar[24] = acadoWorkspace.d[18];
acadoWorkspace.sbar[25] = acadoWorkspace.d[19];
acadoWorkspace.sbar[26] = acadoWorkspace.d[20];
acadoWorkspace.sbar[27] = acadoWorkspace.d[21];
acadoWorkspace.sbar[28] = acadoWorkspace.d[22];
acadoWorkspace.sbar[29] = acadoWorkspace.d[23];
acadoWorkspace.sbar[30] = acadoWorkspace.d[24];
acadoWorkspace.sbar[31] = acadoWorkspace.d[25];
acadoWorkspace.sbar[32] = acadoWorkspace.d[26];
acadoWorkspace.sbar[33] = acadoWorkspace.d[27];
acadoWorkspace.sbar[34] = acadoWorkspace.d[28];
acadoWorkspace.sbar[35] = acadoWorkspace.d[29];
acadoWorkspace.sbar[36] = acadoWorkspace.d[30];
acadoWorkspace.sbar[37] = acadoWorkspace.d[31];
acadoWorkspace.sbar[38] = acadoWorkspace.d[32];
acadoWorkspace.sbar[39] = acadoWorkspace.d[33];
acadoWorkspace.sbar[40] = acadoWorkspace.d[34];
acadoWorkspace.sbar[41] = acadoWorkspace.d[35];
acadoWorkspace.sbar[42] = acadoWorkspace.d[36];
acadoWorkspace.sbar[43] = acadoWorkspace.d[37];
acadoWorkspace.sbar[44] = acadoWorkspace.d[38];
acadoWorkspace.sbar[45] = acadoWorkspace.d[39];
acadoWorkspace.sbar[46] = acadoWorkspace.d[40];
acadoWorkspace.sbar[47] = acadoWorkspace.d[41];
acadoWorkspace.sbar[48] = acadoWorkspace.d[42];
acadoWorkspace.sbar[49] = acadoWorkspace.d[43];
acadoWorkspace.sbar[50] = acadoWorkspace.d[44];
acadoWorkspace.sbar[51] = acadoWorkspace.d[45];
acadoWorkspace.sbar[52] = acadoWorkspace.d[46];
acadoWorkspace.sbar[53] = acadoWorkspace.d[47];
acadoWorkspace.sbar[54] = acadoWorkspace.d[48];
acadoWorkspace.sbar[55] = acadoWorkspace.d[49];
acadoWorkspace.sbar[56] = acadoWorkspace.d[50];
acadoWorkspace.sbar[57] = acadoWorkspace.d[51];
acadoWorkspace.sbar[58] = acadoWorkspace.d[52];
acadoWorkspace.sbar[59] = acadoWorkspace.d[53];
acadoWorkspace.sbar[60] = acadoWorkspace.d[54];
acadoWorkspace.sbar[61] = acadoWorkspace.d[55];
acadoWorkspace.sbar[62] = acadoWorkspace.d[56];
acadoWorkspace.sbar[63] = acadoWorkspace.d[57];
acadoWorkspace.sbar[64] = acadoWorkspace.d[58];
acadoWorkspace.sbar[65] = acadoWorkspace.d[59];
acadoWorkspace.sbar[66] = acadoWorkspace.d[60];
acadoWorkspace.sbar[67] = acadoWorkspace.d[61];
acadoWorkspace.sbar[68] = acadoWorkspace.d[62];
acadoWorkspace.sbar[69] = acadoWorkspace.d[63];
acadoWorkspace.sbar[70] = acadoWorkspace.d[64];
acadoWorkspace.sbar[71] = acadoWorkspace.d[65];
acadoWorkspace.sbar[72] = acadoWorkspace.d[66];
acadoWorkspace.sbar[73] = acadoWorkspace.d[67];
acadoWorkspace.sbar[74] = acadoWorkspace.d[68];
acadoWorkspace.sbar[75] = acadoWorkspace.d[69];
acadoWorkspace.sbar[76] = acadoWorkspace.d[70];
acadoWorkspace.sbar[77] = acadoWorkspace.d[71];
acadoWorkspace.sbar[78] = acadoWorkspace.d[72];
acadoWorkspace.sbar[79] = acadoWorkspace.d[73];
acadoWorkspace.sbar[80] = acadoWorkspace.d[74];
acadoWorkspace.sbar[81] = acadoWorkspace.d[75];
acadoWorkspace.sbar[82] = acadoWorkspace.d[76];
acadoWorkspace.sbar[83] = acadoWorkspace.d[77];
acadoWorkspace.sbar[84] = acadoWorkspace.d[78];
acadoWorkspace.sbar[85] = acadoWorkspace.d[79];
acadoWorkspace.sbar[86] = acadoWorkspace.d[80];
acadoWorkspace.sbar[87] = acadoWorkspace.d[81];
acadoWorkspace.sbar[88] = acadoWorkspace.d[82];
acadoWorkspace.sbar[89] = acadoWorkspace.d[83];
acadoWorkspace.sbar[90] = acadoWorkspace.d[84];
acadoWorkspace.sbar[91] = acadoWorkspace.d[85];
acadoWorkspace.sbar[92] = acadoWorkspace.d[86];
acadoWorkspace.sbar[93] = acadoWorkspace.d[87];
acadoWorkspace.sbar[94] = acadoWorkspace.d[88];
acadoWorkspace.sbar[95] = acadoWorkspace.d[89];
acadoWorkspace.sbar[96] = acadoWorkspace.d[90];
acadoWorkspace.sbar[97] = acadoWorkspace.d[91];
acadoWorkspace.sbar[98] = acadoWorkspace.d[92];
acadoWorkspace.sbar[99] = acadoWorkspace.d[93];
acadoWorkspace.sbar[100] = acadoWorkspace.d[94];
acadoWorkspace.sbar[101] = acadoWorkspace.d[95];
acadoWorkspace.sbar[102] = acadoWorkspace.d[96];
acadoWorkspace.sbar[103] = acadoWorkspace.d[97];
acadoWorkspace.sbar[104] = acadoWorkspace.d[98];
acadoWorkspace.sbar[105] = acadoWorkspace.d[99];
acadoWorkspace.sbar[106] = acadoWorkspace.d[100];
acadoWorkspace.sbar[107] = acadoWorkspace.d[101];
acadoWorkspace.sbar[108] = acadoWorkspace.d[102];
acadoWorkspace.sbar[109] = acadoWorkspace.d[103];
acadoWorkspace.sbar[110] = acadoWorkspace.d[104];
acadoWorkspace.sbar[111] = acadoWorkspace.d[105];
acadoWorkspace.sbar[112] = acadoWorkspace.d[106];
acadoWorkspace.sbar[113] = acadoWorkspace.d[107];
acadoWorkspace.sbar[114] = acadoWorkspace.d[108];
acadoWorkspace.sbar[115] = acadoWorkspace.d[109];
acadoWorkspace.sbar[116] = acadoWorkspace.d[110];
acadoWorkspace.sbar[117] = acadoWorkspace.d[111];
acadoWorkspace.sbar[118] = acadoWorkspace.d[112];
acadoWorkspace.sbar[119] = acadoWorkspace.d[113];
acadoWorkspace.sbar[120] = acadoWorkspace.d[114];
acadoWorkspace.sbar[121] = acadoWorkspace.d[115];
acadoWorkspace.sbar[122] = acadoWorkspace.d[116];
acadoWorkspace.sbar[123] = acadoWorkspace.d[117];
acadoWorkspace.sbar[124] = acadoWorkspace.d[118];
acadoWorkspace.sbar[125] = acadoWorkspace.d[119];

for (lRun1 = 0; lRun1 < 120; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 6;
lRun4 = ((lRun3) / (6)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 39)) / (2)) + (lRun4)) - (1)) * (6)) + ((lRun3) % (6));
acadoWorkspace.A[(lRun1 * 40) + (lRun2 * 2)] = acadoWorkspace.E[lRun5 * 2];
acadoWorkspace.A[(lRun1 * 40) + (lRun2 * 2 + 1)] = acadoWorkspace.E[lRun5 * 2 + 1];
}
}

}

void acado_condenseFdb(  )
{
int lRun1;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
for (lRun1 = 0; lRun1 < 160; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 16 ]), &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 32 ]), &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 48 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 64 ]), &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 80 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 96 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 112 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 128 ]), &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 144 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 160 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 176 ]), &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 192 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 208 ]), &(acadoWorkspace.Dy[ 104 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 224 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 256 ]), &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 272 ]), &(acadoWorkspace.Dy[ 136 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 288 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 304 ]), &(acadoWorkspace.Dy[ 152 ]), &(acadoWorkspace.g[ 38 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 48 ]), &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 96 ]), &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 144 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 192 ]), &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 240 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 288 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 336 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 384 ]), &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 432 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 480 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 528 ]), &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.QDy[ 66 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 576 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 624 ]), &(acadoWorkspace.Dy[ 104 ]), &(acadoWorkspace.QDy[ 78 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 672 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 720 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 768 ]), &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 816 ]), &(acadoWorkspace.Dy[ 136 ]), &(acadoWorkspace.QDy[ 102 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 864 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 912 ]), &(acadoWorkspace.Dy[ 152 ]), &(acadoWorkspace.QDy[ 114 ]) );

acadoWorkspace.QDy[120] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[121] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[122] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[123] = + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[124] = + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[125] = + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[5];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 6 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 120 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[125] + acadoWorkspace.QDy[120];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[125] + acadoWorkspace.QDy[121];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[125] + acadoWorkspace.QDy[122];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[125] + acadoWorkspace.QDy[123];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[125] + acadoWorkspace.QDy[124];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[122] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[123] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[124] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[125] + acadoWorkspace.QDy[125];
acado_macBTw1( &(acadoWorkspace.evGu[ 228 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 38 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 684 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 114 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 684 ]), &(acadoWorkspace.sbar[ 114 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 204 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 34 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 612 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 102 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 612 ]), &(acadoWorkspace.sbar[ 102 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 96 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 540 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 540 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 168 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 504 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 504 ]), &(acadoWorkspace.sbar[ 84 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 156 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 26 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 468 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 78 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 468 ]), &(acadoWorkspace.sbar[ 78 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 132 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 22 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 396 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 66 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 396 ]), &(acadoWorkspace.sbar[ 66 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 360 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 360 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 84 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 252 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 42 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 252 ]), &(acadoWorkspace.sbar[ 42 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 216 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 180 ]), &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.sbar[ 6 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];

tmp = acadoWorkspace.sbar[6] + acadoVariables.x[6];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - tmp;
acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - tmp;
tmp = acadoWorkspace.sbar[7] + acadoVariables.x[7];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - tmp;
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - tmp;
tmp = acadoWorkspace.sbar[8] + acadoVariables.x[8];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - tmp;
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - tmp;
tmp = acadoWorkspace.sbar[9] + acadoVariables.x[9];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - tmp;
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - tmp;
tmp = acadoWorkspace.sbar[10] + acadoVariables.x[10];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - tmp;
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - tmp;
tmp = acadoWorkspace.sbar[11] + acadoVariables.x[11];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - tmp;
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - tmp;
tmp = acadoWorkspace.sbar[12] + acadoVariables.x[12];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - tmp;
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - tmp;
tmp = acadoWorkspace.sbar[13] + acadoVariables.x[13];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - tmp;
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - tmp;
tmp = acadoWorkspace.sbar[14] + acadoVariables.x[14];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - tmp;
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - tmp;
tmp = acadoWorkspace.sbar[15] + acadoVariables.x[15];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - tmp;
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - tmp;
tmp = acadoWorkspace.sbar[16] + acadoVariables.x[16];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - tmp;
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - tmp;
tmp = acadoWorkspace.sbar[17] + acadoVariables.x[17];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - tmp;
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - tmp;
tmp = acadoWorkspace.sbar[18] + acadoVariables.x[18];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - tmp;
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - tmp;
tmp = acadoWorkspace.sbar[19] + acadoVariables.x[19];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - tmp;
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - tmp;
tmp = acadoWorkspace.sbar[20] + acadoVariables.x[20];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - tmp;
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - tmp;
tmp = acadoWorkspace.sbar[21] + acadoVariables.x[21];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - tmp;
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - tmp;
tmp = acadoWorkspace.sbar[22] + acadoVariables.x[22];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - tmp;
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - tmp;
tmp = acadoWorkspace.sbar[23] + acadoVariables.x[23];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - tmp;
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - tmp;
tmp = acadoWorkspace.sbar[24] + acadoVariables.x[24];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - tmp;
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - tmp;
tmp = acadoWorkspace.sbar[25] + acadoVariables.x[25];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - tmp;
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - tmp;
tmp = acadoWorkspace.sbar[26] + acadoVariables.x[26];
acadoWorkspace.lbA[20] = acadoVariables.lbAValues[20] - tmp;
acadoWorkspace.ubA[20] = acadoVariables.ubAValues[20] - tmp;
tmp = acadoWorkspace.sbar[27] + acadoVariables.x[27];
acadoWorkspace.lbA[21] = acadoVariables.lbAValues[21] - tmp;
acadoWorkspace.ubA[21] = acadoVariables.ubAValues[21] - tmp;
tmp = acadoWorkspace.sbar[28] + acadoVariables.x[28];
acadoWorkspace.lbA[22] = acadoVariables.lbAValues[22] - tmp;
acadoWorkspace.ubA[22] = acadoVariables.ubAValues[22] - tmp;
tmp = acadoWorkspace.sbar[29] + acadoVariables.x[29];
acadoWorkspace.lbA[23] = acadoVariables.lbAValues[23] - tmp;
acadoWorkspace.ubA[23] = acadoVariables.ubAValues[23] - tmp;
tmp = acadoWorkspace.sbar[30] + acadoVariables.x[30];
acadoWorkspace.lbA[24] = acadoVariables.lbAValues[24] - tmp;
acadoWorkspace.ubA[24] = acadoVariables.ubAValues[24] - tmp;
tmp = acadoWorkspace.sbar[31] + acadoVariables.x[31];
acadoWorkspace.lbA[25] = acadoVariables.lbAValues[25] - tmp;
acadoWorkspace.ubA[25] = acadoVariables.ubAValues[25] - tmp;
tmp = acadoWorkspace.sbar[32] + acadoVariables.x[32];
acadoWorkspace.lbA[26] = acadoVariables.lbAValues[26] - tmp;
acadoWorkspace.ubA[26] = acadoVariables.ubAValues[26] - tmp;
tmp = acadoWorkspace.sbar[33] + acadoVariables.x[33];
acadoWorkspace.lbA[27] = acadoVariables.lbAValues[27] - tmp;
acadoWorkspace.ubA[27] = acadoVariables.ubAValues[27] - tmp;
tmp = acadoWorkspace.sbar[34] + acadoVariables.x[34];
acadoWorkspace.lbA[28] = acadoVariables.lbAValues[28] - tmp;
acadoWorkspace.ubA[28] = acadoVariables.ubAValues[28] - tmp;
tmp = acadoWorkspace.sbar[35] + acadoVariables.x[35];
acadoWorkspace.lbA[29] = acadoVariables.lbAValues[29] - tmp;
acadoWorkspace.ubA[29] = acadoVariables.ubAValues[29] - tmp;
tmp = acadoWorkspace.sbar[36] + acadoVariables.x[36];
acadoWorkspace.lbA[30] = acadoVariables.lbAValues[30] - tmp;
acadoWorkspace.ubA[30] = acadoVariables.ubAValues[30] - tmp;
tmp = acadoWorkspace.sbar[37] + acadoVariables.x[37];
acadoWorkspace.lbA[31] = acadoVariables.lbAValues[31] - tmp;
acadoWorkspace.ubA[31] = acadoVariables.ubAValues[31] - tmp;
tmp = acadoWorkspace.sbar[38] + acadoVariables.x[38];
acadoWorkspace.lbA[32] = acadoVariables.lbAValues[32] - tmp;
acadoWorkspace.ubA[32] = acadoVariables.ubAValues[32] - tmp;
tmp = acadoWorkspace.sbar[39] + acadoVariables.x[39];
acadoWorkspace.lbA[33] = acadoVariables.lbAValues[33] - tmp;
acadoWorkspace.ubA[33] = acadoVariables.ubAValues[33] - tmp;
tmp = acadoWorkspace.sbar[40] + acadoVariables.x[40];
acadoWorkspace.lbA[34] = acadoVariables.lbAValues[34] - tmp;
acadoWorkspace.ubA[34] = acadoVariables.ubAValues[34] - tmp;
tmp = acadoWorkspace.sbar[41] + acadoVariables.x[41];
acadoWorkspace.lbA[35] = acadoVariables.lbAValues[35] - tmp;
acadoWorkspace.ubA[35] = acadoVariables.ubAValues[35] - tmp;
tmp = acadoWorkspace.sbar[42] + acadoVariables.x[42];
acadoWorkspace.lbA[36] = acadoVariables.lbAValues[36] - tmp;
acadoWorkspace.ubA[36] = acadoVariables.ubAValues[36] - tmp;
tmp = acadoWorkspace.sbar[43] + acadoVariables.x[43];
acadoWorkspace.lbA[37] = acadoVariables.lbAValues[37] - tmp;
acadoWorkspace.ubA[37] = acadoVariables.ubAValues[37] - tmp;
tmp = acadoWorkspace.sbar[44] + acadoVariables.x[44];
acadoWorkspace.lbA[38] = acadoVariables.lbAValues[38] - tmp;
acadoWorkspace.ubA[38] = acadoVariables.ubAValues[38] - tmp;
tmp = acadoWorkspace.sbar[45] + acadoVariables.x[45];
acadoWorkspace.lbA[39] = acadoVariables.lbAValues[39] - tmp;
acadoWorkspace.ubA[39] = acadoVariables.ubAValues[39] - tmp;
tmp = acadoWorkspace.sbar[46] + acadoVariables.x[46];
acadoWorkspace.lbA[40] = acadoVariables.lbAValues[40] - tmp;
acadoWorkspace.ubA[40] = acadoVariables.ubAValues[40] - tmp;
tmp = acadoWorkspace.sbar[47] + acadoVariables.x[47];
acadoWorkspace.lbA[41] = acadoVariables.lbAValues[41] - tmp;
acadoWorkspace.ubA[41] = acadoVariables.ubAValues[41] - tmp;
tmp = acadoWorkspace.sbar[48] + acadoVariables.x[48];
acadoWorkspace.lbA[42] = acadoVariables.lbAValues[42] - tmp;
acadoWorkspace.ubA[42] = acadoVariables.ubAValues[42] - tmp;
tmp = acadoWorkspace.sbar[49] + acadoVariables.x[49];
acadoWorkspace.lbA[43] = acadoVariables.lbAValues[43] - tmp;
acadoWorkspace.ubA[43] = acadoVariables.ubAValues[43] - tmp;
tmp = acadoWorkspace.sbar[50] + acadoVariables.x[50];
acadoWorkspace.lbA[44] = acadoVariables.lbAValues[44] - tmp;
acadoWorkspace.ubA[44] = acadoVariables.ubAValues[44] - tmp;
tmp = acadoWorkspace.sbar[51] + acadoVariables.x[51];
acadoWorkspace.lbA[45] = acadoVariables.lbAValues[45] - tmp;
acadoWorkspace.ubA[45] = acadoVariables.ubAValues[45] - tmp;
tmp = acadoWorkspace.sbar[52] + acadoVariables.x[52];
acadoWorkspace.lbA[46] = acadoVariables.lbAValues[46] - tmp;
acadoWorkspace.ubA[46] = acadoVariables.ubAValues[46] - tmp;
tmp = acadoWorkspace.sbar[53] + acadoVariables.x[53];
acadoWorkspace.lbA[47] = acadoVariables.lbAValues[47] - tmp;
acadoWorkspace.ubA[47] = acadoVariables.ubAValues[47] - tmp;
tmp = acadoWorkspace.sbar[54] + acadoVariables.x[54];
acadoWorkspace.lbA[48] = acadoVariables.lbAValues[48] - tmp;
acadoWorkspace.ubA[48] = acadoVariables.ubAValues[48] - tmp;
tmp = acadoWorkspace.sbar[55] + acadoVariables.x[55];
acadoWorkspace.lbA[49] = acadoVariables.lbAValues[49] - tmp;
acadoWorkspace.ubA[49] = acadoVariables.ubAValues[49] - tmp;
tmp = acadoWorkspace.sbar[56] + acadoVariables.x[56];
acadoWorkspace.lbA[50] = acadoVariables.lbAValues[50] - tmp;
acadoWorkspace.ubA[50] = acadoVariables.ubAValues[50] - tmp;
tmp = acadoWorkspace.sbar[57] + acadoVariables.x[57];
acadoWorkspace.lbA[51] = acadoVariables.lbAValues[51] - tmp;
acadoWorkspace.ubA[51] = acadoVariables.ubAValues[51] - tmp;
tmp = acadoWorkspace.sbar[58] + acadoVariables.x[58];
acadoWorkspace.lbA[52] = acadoVariables.lbAValues[52] - tmp;
acadoWorkspace.ubA[52] = acadoVariables.ubAValues[52] - tmp;
tmp = acadoWorkspace.sbar[59] + acadoVariables.x[59];
acadoWorkspace.lbA[53] = acadoVariables.lbAValues[53] - tmp;
acadoWorkspace.ubA[53] = acadoVariables.ubAValues[53] - tmp;
tmp = acadoWorkspace.sbar[60] + acadoVariables.x[60];
acadoWorkspace.lbA[54] = acadoVariables.lbAValues[54] - tmp;
acadoWorkspace.ubA[54] = acadoVariables.ubAValues[54] - tmp;
tmp = acadoWorkspace.sbar[61] + acadoVariables.x[61];
acadoWorkspace.lbA[55] = acadoVariables.lbAValues[55] - tmp;
acadoWorkspace.ubA[55] = acadoVariables.ubAValues[55] - tmp;
tmp = acadoWorkspace.sbar[62] + acadoVariables.x[62];
acadoWorkspace.lbA[56] = acadoVariables.lbAValues[56] - tmp;
acadoWorkspace.ubA[56] = acadoVariables.ubAValues[56] - tmp;
tmp = acadoWorkspace.sbar[63] + acadoVariables.x[63];
acadoWorkspace.lbA[57] = acadoVariables.lbAValues[57] - tmp;
acadoWorkspace.ubA[57] = acadoVariables.ubAValues[57] - tmp;
tmp = acadoWorkspace.sbar[64] + acadoVariables.x[64];
acadoWorkspace.lbA[58] = acadoVariables.lbAValues[58] - tmp;
acadoWorkspace.ubA[58] = acadoVariables.ubAValues[58] - tmp;
tmp = acadoWorkspace.sbar[65] + acadoVariables.x[65];
acadoWorkspace.lbA[59] = acadoVariables.lbAValues[59] - tmp;
acadoWorkspace.ubA[59] = acadoVariables.ubAValues[59] - tmp;
tmp = acadoWorkspace.sbar[66] + acadoVariables.x[66];
acadoWorkspace.lbA[60] = acadoVariables.lbAValues[60] - tmp;
acadoWorkspace.ubA[60] = acadoVariables.ubAValues[60] - tmp;
tmp = acadoWorkspace.sbar[67] + acadoVariables.x[67];
acadoWorkspace.lbA[61] = acadoVariables.lbAValues[61] - tmp;
acadoWorkspace.ubA[61] = acadoVariables.ubAValues[61] - tmp;
tmp = acadoWorkspace.sbar[68] + acadoVariables.x[68];
acadoWorkspace.lbA[62] = acadoVariables.lbAValues[62] - tmp;
acadoWorkspace.ubA[62] = acadoVariables.ubAValues[62] - tmp;
tmp = acadoWorkspace.sbar[69] + acadoVariables.x[69];
acadoWorkspace.lbA[63] = acadoVariables.lbAValues[63] - tmp;
acadoWorkspace.ubA[63] = acadoVariables.ubAValues[63] - tmp;
tmp = acadoWorkspace.sbar[70] + acadoVariables.x[70];
acadoWorkspace.lbA[64] = acadoVariables.lbAValues[64] - tmp;
acadoWorkspace.ubA[64] = acadoVariables.ubAValues[64] - tmp;
tmp = acadoWorkspace.sbar[71] + acadoVariables.x[71];
acadoWorkspace.lbA[65] = acadoVariables.lbAValues[65] - tmp;
acadoWorkspace.ubA[65] = acadoVariables.ubAValues[65] - tmp;
tmp = acadoWorkspace.sbar[72] + acadoVariables.x[72];
acadoWorkspace.lbA[66] = acadoVariables.lbAValues[66] - tmp;
acadoWorkspace.ubA[66] = acadoVariables.ubAValues[66] - tmp;
tmp = acadoWorkspace.sbar[73] + acadoVariables.x[73];
acadoWorkspace.lbA[67] = acadoVariables.lbAValues[67] - tmp;
acadoWorkspace.ubA[67] = acadoVariables.ubAValues[67] - tmp;
tmp = acadoWorkspace.sbar[74] + acadoVariables.x[74];
acadoWorkspace.lbA[68] = acadoVariables.lbAValues[68] - tmp;
acadoWorkspace.ubA[68] = acadoVariables.ubAValues[68] - tmp;
tmp = acadoWorkspace.sbar[75] + acadoVariables.x[75];
acadoWorkspace.lbA[69] = acadoVariables.lbAValues[69] - tmp;
acadoWorkspace.ubA[69] = acadoVariables.ubAValues[69] - tmp;
tmp = acadoWorkspace.sbar[76] + acadoVariables.x[76];
acadoWorkspace.lbA[70] = acadoVariables.lbAValues[70] - tmp;
acadoWorkspace.ubA[70] = acadoVariables.ubAValues[70] - tmp;
tmp = acadoWorkspace.sbar[77] + acadoVariables.x[77];
acadoWorkspace.lbA[71] = acadoVariables.lbAValues[71] - tmp;
acadoWorkspace.ubA[71] = acadoVariables.ubAValues[71] - tmp;
tmp = acadoWorkspace.sbar[78] + acadoVariables.x[78];
acadoWorkspace.lbA[72] = acadoVariables.lbAValues[72] - tmp;
acadoWorkspace.ubA[72] = acadoVariables.ubAValues[72] - tmp;
tmp = acadoWorkspace.sbar[79] + acadoVariables.x[79];
acadoWorkspace.lbA[73] = acadoVariables.lbAValues[73] - tmp;
acadoWorkspace.ubA[73] = acadoVariables.ubAValues[73] - tmp;
tmp = acadoWorkspace.sbar[80] + acadoVariables.x[80];
acadoWorkspace.lbA[74] = acadoVariables.lbAValues[74] - tmp;
acadoWorkspace.ubA[74] = acadoVariables.ubAValues[74] - tmp;
tmp = acadoWorkspace.sbar[81] + acadoVariables.x[81];
acadoWorkspace.lbA[75] = acadoVariables.lbAValues[75] - tmp;
acadoWorkspace.ubA[75] = acadoVariables.ubAValues[75] - tmp;
tmp = acadoWorkspace.sbar[82] + acadoVariables.x[82];
acadoWorkspace.lbA[76] = acadoVariables.lbAValues[76] - tmp;
acadoWorkspace.ubA[76] = acadoVariables.ubAValues[76] - tmp;
tmp = acadoWorkspace.sbar[83] + acadoVariables.x[83];
acadoWorkspace.lbA[77] = acadoVariables.lbAValues[77] - tmp;
acadoWorkspace.ubA[77] = acadoVariables.ubAValues[77] - tmp;
tmp = acadoWorkspace.sbar[84] + acadoVariables.x[84];
acadoWorkspace.lbA[78] = acadoVariables.lbAValues[78] - tmp;
acadoWorkspace.ubA[78] = acadoVariables.ubAValues[78] - tmp;
tmp = acadoWorkspace.sbar[85] + acadoVariables.x[85];
acadoWorkspace.lbA[79] = acadoVariables.lbAValues[79] - tmp;
acadoWorkspace.ubA[79] = acadoVariables.ubAValues[79] - tmp;
tmp = acadoWorkspace.sbar[86] + acadoVariables.x[86];
acadoWorkspace.lbA[80] = acadoVariables.lbAValues[80] - tmp;
acadoWorkspace.ubA[80] = acadoVariables.ubAValues[80] - tmp;
tmp = acadoWorkspace.sbar[87] + acadoVariables.x[87];
acadoWorkspace.lbA[81] = acadoVariables.lbAValues[81] - tmp;
acadoWorkspace.ubA[81] = acadoVariables.ubAValues[81] - tmp;
tmp = acadoWorkspace.sbar[88] + acadoVariables.x[88];
acadoWorkspace.lbA[82] = acadoVariables.lbAValues[82] - tmp;
acadoWorkspace.ubA[82] = acadoVariables.ubAValues[82] - tmp;
tmp = acadoWorkspace.sbar[89] + acadoVariables.x[89];
acadoWorkspace.lbA[83] = acadoVariables.lbAValues[83] - tmp;
acadoWorkspace.ubA[83] = acadoVariables.ubAValues[83] - tmp;
tmp = acadoWorkspace.sbar[90] + acadoVariables.x[90];
acadoWorkspace.lbA[84] = acadoVariables.lbAValues[84] - tmp;
acadoWorkspace.ubA[84] = acadoVariables.ubAValues[84] - tmp;
tmp = acadoWorkspace.sbar[91] + acadoVariables.x[91];
acadoWorkspace.lbA[85] = acadoVariables.lbAValues[85] - tmp;
acadoWorkspace.ubA[85] = acadoVariables.ubAValues[85] - tmp;
tmp = acadoWorkspace.sbar[92] + acadoVariables.x[92];
acadoWorkspace.lbA[86] = acadoVariables.lbAValues[86] - tmp;
acadoWorkspace.ubA[86] = acadoVariables.ubAValues[86] - tmp;
tmp = acadoWorkspace.sbar[93] + acadoVariables.x[93];
acadoWorkspace.lbA[87] = acadoVariables.lbAValues[87] - tmp;
acadoWorkspace.ubA[87] = acadoVariables.ubAValues[87] - tmp;
tmp = acadoWorkspace.sbar[94] + acadoVariables.x[94];
acadoWorkspace.lbA[88] = acadoVariables.lbAValues[88] - tmp;
acadoWorkspace.ubA[88] = acadoVariables.ubAValues[88] - tmp;
tmp = acadoWorkspace.sbar[95] + acadoVariables.x[95];
acadoWorkspace.lbA[89] = acadoVariables.lbAValues[89] - tmp;
acadoWorkspace.ubA[89] = acadoVariables.ubAValues[89] - tmp;
tmp = acadoWorkspace.sbar[96] + acadoVariables.x[96];
acadoWorkspace.lbA[90] = acadoVariables.lbAValues[90] - tmp;
acadoWorkspace.ubA[90] = acadoVariables.ubAValues[90] - tmp;
tmp = acadoWorkspace.sbar[97] + acadoVariables.x[97];
acadoWorkspace.lbA[91] = acadoVariables.lbAValues[91] - tmp;
acadoWorkspace.ubA[91] = acadoVariables.ubAValues[91] - tmp;
tmp = acadoWorkspace.sbar[98] + acadoVariables.x[98];
acadoWorkspace.lbA[92] = acadoVariables.lbAValues[92] - tmp;
acadoWorkspace.ubA[92] = acadoVariables.ubAValues[92] - tmp;
tmp = acadoWorkspace.sbar[99] + acadoVariables.x[99];
acadoWorkspace.lbA[93] = acadoVariables.lbAValues[93] - tmp;
acadoWorkspace.ubA[93] = acadoVariables.ubAValues[93] - tmp;
tmp = acadoWorkspace.sbar[100] + acadoVariables.x[100];
acadoWorkspace.lbA[94] = acadoVariables.lbAValues[94] - tmp;
acadoWorkspace.ubA[94] = acadoVariables.ubAValues[94] - tmp;
tmp = acadoWorkspace.sbar[101] + acadoVariables.x[101];
acadoWorkspace.lbA[95] = acadoVariables.lbAValues[95] - tmp;
acadoWorkspace.ubA[95] = acadoVariables.ubAValues[95] - tmp;
tmp = acadoWorkspace.sbar[102] + acadoVariables.x[102];
acadoWorkspace.lbA[96] = acadoVariables.lbAValues[96] - tmp;
acadoWorkspace.ubA[96] = acadoVariables.ubAValues[96] - tmp;
tmp = acadoWorkspace.sbar[103] + acadoVariables.x[103];
acadoWorkspace.lbA[97] = acadoVariables.lbAValues[97] - tmp;
acadoWorkspace.ubA[97] = acadoVariables.ubAValues[97] - tmp;
tmp = acadoWorkspace.sbar[104] + acadoVariables.x[104];
acadoWorkspace.lbA[98] = acadoVariables.lbAValues[98] - tmp;
acadoWorkspace.ubA[98] = acadoVariables.ubAValues[98] - tmp;
tmp = acadoWorkspace.sbar[105] + acadoVariables.x[105];
acadoWorkspace.lbA[99] = acadoVariables.lbAValues[99] - tmp;
acadoWorkspace.ubA[99] = acadoVariables.ubAValues[99] - tmp;
tmp = acadoWorkspace.sbar[106] + acadoVariables.x[106];
acadoWorkspace.lbA[100] = acadoVariables.lbAValues[100] - tmp;
acadoWorkspace.ubA[100] = acadoVariables.ubAValues[100] - tmp;
tmp = acadoWorkspace.sbar[107] + acadoVariables.x[107];
acadoWorkspace.lbA[101] = acadoVariables.lbAValues[101] - tmp;
acadoWorkspace.ubA[101] = acadoVariables.ubAValues[101] - tmp;
tmp = acadoWorkspace.sbar[108] + acadoVariables.x[108];
acadoWorkspace.lbA[102] = acadoVariables.lbAValues[102] - tmp;
acadoWorkspace.ubA[102] = acadoVariables.ubAValues[102] - tmp;
tmp = acadoWorkspace.sbar[109] + acadoVariables.x[109];
acadoWorkspace.lbA[103] = acadoVariables.lbAValues[103] - tmp;
acadoWorkspace.ubA[103] = acadoVariables.ubAValues[103] - tmp;
tmp = acadoWorkspace.sbar[110] + acadoVariables.x[110];
acadoWorkspace.lbA[104] = acadoVariables.lbAValues[104] - tmp;
acadoWorkspace.ubA[104] = acadoVariables.ubAValues[104] - tmp;
tmp = acadoWorkspace.sbar[111] + acadoVariables.x[111];
acadoWorkspace.lbA[105] = acadoVariables.lbAValues[105] - tmp;
acadoWorkspace.ubA[105] = acadoVariables.ubAValues[105] - tmp;
tmp = acadoWorkspace.sbar[112] + acadoVariables.x[112];
acadoWorkspace.lbA[106] = acadoVariables.lbAValues[106] - tmp;
acadoWorkspace.ubA[106] = acadoVariables.ubAValues[106] - tmp;
tmp = acadoWorkspace.sbar[113] + acadoVariables.x[113];
acadoWorkspace.lbA[107] = acadoVariables.lbAValues[107] - tmp;
acadoWorkspace.ubA[107] = acadoVariables.ubAValues[107] - tmp;
tmp = acadoWorkspace.sbar[114] + acadoVariables.x[114];
acadoWorkspace.lbA[108] = acadoVariables.lbAValues[108] - tmp;
acadoWorkspace.ubA[108] = acadoVariables.ubAValues[108] - tmp;
tmp = acadoWorkspace.sbar[115] + acadoVariables.x[115];
acadoWorkspace.lbA[109] = acadoVariables.lbAValues[109] - tmp;
acadoWorkspace.ubA[109] = acadoVariables.ubAValues[109] - tmp;
tmp = acadoWorkspace.sbar[116] + acadoVariables.x[116];
acadoWorkspace.lbA[110] = acadoVariables.lbAValues[110] - tmp;
acadoWorkspace.ubA[110] = acadoVariables.ubAValues[110] - tmp;
tmp = acadoWorkspace.sbar[117] + acadoVariables.x[117];
acadoWorkspace.lbA[111] = acadoVariables.lbAValues[111] - tmp;
acadoWorkspace.ubA[111] = acadoVariables.ubAValues[111] - tmp;
tmp = acadoWorkspace.sbar[118] + acadoVariables.x[118];
acadoWorkspace.lbA[112] = acadoVariables.lbAValues[112] - tmp;
acadoWorkspace.ubA[112] = acadoVariables.ubAValues[112] - tmp;
tmp = acadoWorkspace.sbar[119] + acadoVariables.x[119];
acadoWorkspace.lbA[113] = acadoVariables.lbAValues[113] - tmp;
acadoWorkspace.ubA[113] = acadoVariables.ubAValues[113] - tmp;
tmp = acadoWorkspace.sbar[120] + acadoVariables.x[120];
acadoWorkspace.lbA[114] = acadoVariables.lbAValues[114] - tmp;
acadoWorkspace.ubA[114] = acadoVariables.ubAValues[114] - tmp;
tmp = acadoWorkspace.sbar[121] + acadoVariables.x[121];
acadoWorkspace.lbA[115] = acadoVariables.lbAValues[115] - tmp;
acadoWorkspace.ubA[115] = acadoVariables.ubAValues[115] - tmp;
tmp = acadoWorkspace.sbar[122] + acadoVariables.x[122];
acadoWorkspace.lbA[116] = acadoVariables.lbAValues[116] - tmp;
acadoWorkspace.ubA[116] = acadoVariables.ubAValues[116] - tmp;
tmp = acadoWorkspace.sbar[123] + acadoVariables.x[123];
acadoWorkspace.lbA[117] = acadoVariables.lbAValues[117] - tmp;
acadoWorkspace.ubA[117] = acadoVariables.ubAValues[117] - tmp;
tmp = acadoWorkspace.sbar[124] + acadoVariables.x[124];
acadoWorkspace.lbA[118] = acadoVariables.lbAValues[118] - tmp;
acadoWorkspace.ubA[118] = acadoVariables.ubAValues[118] - tmp;
tmp = acadoWorkspace.sbar[125] + acadoVariables.x[125];
acadoWorkspace.lbA[119] = acadoVariables.lbAValues[119] - tmp;
acadoWorkspace.ubA[119] = acadoVariables.ubAValues[119] - tmp;

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.d[0];
acadoWorkspace.sbar[7] = acadoWorkspace.d[1];
acadoWorkspace.sbar[8] = acadoWorkspace.d[2];
acadoWorkspace.sbar[9] = acadoWorkspace.d[3];
acadoWorkspace.sbar[10] = acadoWorkspace.d[4];
acadoWorkspace.sbar[11] = acadoWorkspace.d[5];
acadoWorkspace.sbar[12] = acadoWorkspace.d[6];
acadoWorkspace.sbar[13] = acadoWorkspace.d[7];
acadoWorkspace.sbar[14] = acadoWorkspace.d[8];
acadoWorkspace.sbar[15] = acadoWorkspace.d[9];
acadoWorkspace.sbar[16] = acadoWorkspace.d[10];
acadoWorkspace.sbar[17] = acadoWorkspace.d[11];
acadoWorkspace.sbar[18] = acadoWorkspace.d[12];
acadoWorkspace.sbar[19] = acadoWorkspace.d[13];
acadoWorkspace.sbar[20] = acadoWorkspace.d[14];
acadoWorkspace.sbar[21] = acadoWorkspace.d[15];
acadoWorkspace.sbar[22] = acadoWorkspace.d[16];
acadoWorkspace.sbar[23] = acadoWorkspace.d[17];
acadoWorkspace.sbar[24] = acadoWorkspace.d[18];
acadoWorkspace.sbar[25] = acadoWorkspace.d[19];
acadoWorkspace.sbar[26] = acadoWorkspace.d[20];
acadoWorkspace.sbar[27] = acadoWorkspace.d[21];
acadoWorkspace.sbar[28] = acadoWorkspace.d[22];
acadoWorkspace.sbar[29] = acadoWorkspace.d[23];
acadoWorkspace.sbar[30] = acadoWorkspace.d[24];
acadoWorkspace.sbar[31] = acadoWorkspace.d[25];
acadoWorkspace.sbar[32] = acadoWorkspace.d[26];
acadoWorkspace.sbar[33] = acadoWorkspace.d[27];
acadoWorkspace.sbar[34] = acadoWorkspace.d[28];
acadoWorkspace.sbar[35] = acadoWorkspace.d[29];
acadoWorkspace.sbar[36] = acadoWorkspace.d[30];
acadoWorkspace.sbar[37] = acadoWorkspace.d[31];
acadoWorkspace.sbar[38] = acadoWorkspace.d[32];
acadoWorkspace.sbar[39] = acadoWorkspace.d[33];
acadoWorkspace.sbar[40] = acadoWorkspace.d[34];
acadoWorkspace.sbar[41] = acadoWorkspace.d[35];
acadoWorkspace.sbar[42] = acadoWorkspace.d[36];
acadoWorkspace.sbar[43] = acadoWorkspace.d[37];
acadoWorkspace.sbar[44] = acadoWorkspace.d[38];
acadoWorkspace.sbar[45] = acadoWorkspace.d[39];
acadoWorkspace.sbar[46] = acadoWorkspace.d[40];
acadoWorkspace.sbar[47] = acadoWorkspace.d[41];
acadoWorkspace.sbar[48] = acadoWorkspace.d[42];
acadoWorkspace.sbar[49] = acadoWorkspace.d[43];
acadoWorkspace.sbar[50] = acadoWorkspace.d[44];
acadoWorkspace.sbar[51] = acadoWorkspace.d[45];
acadoWorkspace.sbar[52] = acadoWorkspace.d[46];
acadoWorkspace.sbar[53] = acadoWorkspace.d[47];
acadoWorkspace.sbar[54] = acadoWorkspace.d[48];
acadoWorkspace.sbar[55] = acadoWorkspace.d[49];
acadoWorkspace.sbar[56] = acadoWorkspace.d[50];
acadoWorkspace.sbar[57] = acadoWorkspace.d[51];
acadoWorkspace.sbar[58] = acadoWorkspace.d[52];
acadoWorkspace.sbar[59] = acadoWorkspace.d[53];
acadoWorkspace.sbar[60] = acadoWorkspace.d[54];
acadoWorkspace.sbar[61] = acadoWorkspace.d[55];
acadoWorkspace.sbar[62] = acadoWorkspace.d[56];
acadoWorkspace.sbar[63] = acadoWorkspace.d[57];
acadoWorkspace.sbar[64] = acadoWorkspace.d[58];
acadoWorkspace.sbar[65] = acadoWorkspace.d[59];
acadoWorkspace.sbar[66] = acadoWorkspace.d[60];
acadoWorkspace.sbar[67] = acadoWorkspace.d[61];
acadoWorkspace.sbar[68] = acadoWorkspace.d[62];
acadoWorkspace.sbar[69] = acadoWorkspace.d[63];
acadoWorkspace.sbar[70] = acadoWorkspace.d[64];
acadoWorkspace.sbar[71] = acadoWorkspace.d[65];
acadoWorkspace.sbar[72] = acadoWorkspace.d[66];
acadoWorkspace.sbar[73] = acadoWorkspace.d[67];
acadoWorkspace.sbar[74] = acadoWorkspace.d[68];
acadoWorkspace.sbar[75] = acadoWorkspace.d[69];
acadoWorkspace.sbar[76] = acadoWorkspace.d[70];
acadoWorkspace.sbar[77] = acadoWorkspace.d[71];
acadoWorkspace.sbar[78] = acadoWorkspace.d[72];
acadoWorkspace.sbar[79] = acadoWorkspace.d[73];
acadoWorkspace.sbar[80] = acadoWorkspace.d[74];
acadoWorkspace.sbar[81] = acadoWorkspace.d[75];
acadoWorkspace.sbar[82] = acadoWorkspace.d[76];
acadoWorkspace.sbar[83] = acadoWorkspace.d[77];
acadoWorkspace.sbar[84] = acadoWorkspace.d[78];
acadoWorkspace.sbar[85] = acadoWorkspace.d[79];
acadoWorkspace.sbar[86] = acadoWorkspace.d[80];
acadoWorkspace.sbar[87] = acadoWorkspace.d[81];
acadoWorkspace.sbar[88] = acadoWorkspace.d[82];
acadoWorkspace.sbar[89] = acadoWorkspace.d[83];
acadoWorkspace.sbar[90] = acadoWorkspace.d[84];
acadoWorkspace.sbar[91] = acadoWorkspace.d[85];
acadoWorkspace.sbar[92] = acadoWorkspace.d[86];
acadoWorkspace.sbar[93] = acadoWorkspace.d[87];
acadoWorkspace.sbar[94] = acadoWorkspace.d[88];
acadoWorkspace.sbar[95] = acadoWorkspace.d[89];
acadoWorkspace.sbar[96] = acadoWorkspace.d[90];
acadoWorkspace.sbar[97] = acadoWorkspace.d[91];
acadoWorkspace.sbar[98] = acadoWorkspace.d[92];
acadoWorkspace.sbar[99] = acadoWorkspace.d[93];
acadoWorkspace.sbar[100] = acadoWorkspace.d[94];
acadoWorkspace.sbar[101] = acadoWorkspace.d[95];
acadoWorkspace.sbar[102] = acadoWorkspace.d[96];
acadoWorkspace.sbar[103] = acadoWorkspace.d[97];
acadoWorkspace.sbar[104] = acadoWorkspace.d[98];
acadoWorkspace.sbar[105] = acadoWorkspace.d[99];
acadoWorkspace.sbar[106] = acadoWorkspace.d[100];
acadoWorkspace.sbar[107] = acadoWorkspace.d[101];
acadoWorkspace.sbar[108] = acadoWorkspace.d[102];
acadoWorkspace.sbar[109] = acadoWorkspace.d[103];
acadoWorkspace.sbar[110] = acadoWorkspace.d[104];
acadoWorkspace.sbar[111] = acadoWorkspace.d[105];
acadoWorkspace.sbar[112] = acadoWorkspace.d[106];
acadoWorkspace.sbar[113] = acadoWorkspace.d[107];
acadoWorkspace.sbar[114] = acadoWorkspace.d[108];
acadoWorkspace.sbar[115] = acadoWorkspace.d[109];
acadoWorkspace.sbar[116] = acadoWorkspace.d[110];
acadoWorkspace.sbar[117] = acadoWorkspace.d[111];
acadoWorkspace.sbar[118] = acadoWorkspace.d[112];
acadoWorkspace.sbar[119] = acadoWorkspace.d[113];
acadoWorkspace.sbar[120] = acadoWorkspace.d[114];
acadoWorkspace.sbar[121] = acadoWorkspace.d[115];
acadoWorkspace.sbar[122] = acadoWorkspace.d[116];
acadoWorkspace.sbar[123] = acadoWorkspace.d[117];
acadoWorkspace.sbar[124] = acadoWorkspace.d[118];
acadoWorkspace.sbar[125] = acadoWorkspace.d[119];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 6 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.evGu[ 84 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.evGu[ 132 ]), &(acadoWorkspace.x[ 22 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.evGu[ 156 ]), &(acadoWorkspace.x[ 26 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.evGu[ 168 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.evGu[ 180 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.evGu[ 204 ]), &(acadoWorkspace.x[ 34 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.evGu[ 228 ]), &(acadoWorkspace.x[ 38 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 120 ]) );
acadoVariables.x[0] += acadoWorkspace.sbar[0];
acadoVariables.x[1] += acadoWorkspace.sbar[1];
acadoVariables.x[2] += acadoWorkspace.sbar[2];
acadoVariables.x[3] += acadoWorkspace.sbar[3];
acadoVariables.x[4] += acadoWorkspace.sbar[4];
acadoVariables.x[5] += acadoWorkspace.sbar[5];
acadoVariables.x[6] += acadoWorkspace.sbar[6];
acadoVariables.x[7] += acadoWorkspace.sbar[7];
acadoVariables.x[8] += acadoWorkspace.sbar[8];
acadoVariables.x[9] += acadoWorkspace.sbar[9];
acadoVariables.x[10] += acadoWorkspace.sbar[10];
acadoVariables.x[11] += acadoWorkspace.sbar[11];
acadoVariables.x[12] += acadoWorkspace.sbar[12];
acadoVariables.x[13] += acadoWorkspace.sbar[13];
acadoVariables.x[14] += acadoWorkspace.sbar[14];
acadoVariables.x[15] += acadoWorkspace.sbar[15];
acadoVariables.x[16] += acadoWorkspace.sbar[16];
acadoVariables.x[17] += acadoWorkspace.sbar[17];
acadoVariables.x[18] += acadoWorkspace.sbar[18];
acadoVariables.x[19] += acadoWorkspace.sbar[19];
acadoVariables.x[20] += acadoWorkspace.sbar[20];
acadoVariables.x[21] += acadoWorkspace.sbar[21];
acadoVariables.x[22] += acadoWorkspace.sbar[22];
acadoVariables.x[23] += acadoWorkspace.sbar[23];
acadoVariables.x[24] += acadoWorkspace.sbar[24];
acadoVariables.x[25] += acadoWorkspace.sbar[25];
acadoVariables.x[26] += acadoWorkspace.sbar[26];
acadoVariables.x[27] += acadoWorkspace.sbar[27];
acadoVariables.x[28] += acadoWorkspace.sbar[28];
acadoVariables.x[29] += acadoWorkspace.sbar[29];
acadoVariables.x[30] += acadoWorkspace.sbar[30];
acadoVariables.x[31] += acadoWorkspace.sbar[31];
acadoVariables.x[32] += acadoWorkspace.sbar[32];
acadoVariables.x[33] += acadoWorkspace.sbar[33];
acadoVariables.x[34] += acadoWorkspace.sbar[34];
acadoVariables.x[35] += acadoWorkspace.sbar[35];
acadoVariables.x[36] += acadoWorkspace.sbar[36];
acadoVariables.x[37] += acadoWorkspace.sbar[37];
acadoVariables.x[38] += acadoWorkspace.sbar[38];
acadoVariables.x[39] += acadoWorkspace.sbar[39];
acadoVariables.x[40] += acadoWorkspace.sbar[40];
acadoVariables.x[41] += acadoWorkspace.sbar[41];
acadoVariables.x[42] += acadoWorkspace.sbar[42];
acadoVariables.x[43] += acadoWorkspace.sbar[43];
acadoVariables.x[44] += acadoWorkspace.sbar[44];
acadoVariables.x[45] += acadoWorkspace.sbar[45];
acadoVariables.x[46] += acadoWorkspace.sbar[46];
acadoVariables.x[47] += acadoWorkspace.sbar[47];
acadoVariables.x[48] += acadoWorkspace.sbar[48];
acadoVariables.x[49] += acadoWorkspace.sbar[49];
acadoVariables.x[50] += acadoWorkspace.sbar[50];
acadoVariables.x[51] += acadoWorkspace.sbar[51];
acadoVariables.x[52] += acadoWorkspace.sbar[52];
acadoVariables.x[53] += acadoWorkspace.sbar[53];
acadoVariables.x[54] += acadoWorkspace.sbar[54];
acadoVariables.x[55] += acadoWorkspace.sbar[55];
acadoVariables.x[56] += acadoWorkspace.sbar[56];
acadoVariables.x[57] += acadoWorkspace.sbar[57];
acadoVariables.x[58] += acadoWorkspace.sbar[58];
acadoVariables.x[59] += acadoWorkspace.sbar[59];
acadoVariables.x[60] += acadoWorkspace.sbar[60];
acadoVariables.x[61] += acadoWorkspace.sbar[61];
acadoVariables.x[62] += acadoWorkspace.sbar[62];
acadoVariables.x[63] += acadoWorkspace.sbar[63];
acadoVariables.x[64] += acadoWorkspace.sbar[64];
acadoVariables.x[65] += acadoWorkspace.sbar[65];
acadoVariables.x[66] += acadoWorkspace.sbar[66];
acadoVariables.x[67] += acadoWorkspace.sbar[67];
acadoVariables.x[68] += acadoWorkspace.sbar[68];
acadoVariables.x[69] += acadoWorkspace.sbar[69];
acadoVariables.x[70] += acadoWorkspace.sbar[70];
acadoVariables.x[71] += acadoWorkspace.sbar[71];
acadoVariables.x[72] += acadoWorkspace.sbar[72];
acadoVariables.x[73] += acadoWorkspace.sbar[73];
acadoVariables.x[74] += acadoWorkspace.sbar[74];
acadoVariables.x[75] += acadoWorkspace.sbar[75];
acadoVariables.x[76] += acadoWorkspace.sbar[76];
acadoVariables.x[77] += acadoWorkspace.sbar[77];
acadoVariables.x[78] += acadoWorkspace.sbar[78];
acadoVariables.x[79] += acadoWorkspace.sbar[79];
acadoVariables.x[80] += acadoWorkspace.sbar[80];
acadoVariables.x[81] += acadoWorkspace.sbar[81];
acadoVariables.x[82] += acadoWorkspace.sbar[82];
acadoVariables.x[83] += acadoWorkspace.sbar[83];
acadoVariables.x[84] += acadoWorkspace.sbar[84];
acadoVariables.x[85] += acadoWorkspace.sbar[85];
acadoVariables.x[86] += acadoWorkspace.sbar[86];
acadoVariables.x[87] += acadoWorkspace.sbar[87];
acadoVariables.x[88] += acadoWorkspace.sbar[88];
acadoVariables.x[89] += acadoWorkspace.sbar[89];
acadoVariables.x[90] += acadoWorkspace.sbar[90];
acadoVariables.x[91] += acadoWorkspace.sbar[91];
acadoVariables.x[92] += acadoWorkspace.sbar[92];
acadoVariables.x[93] += acadoWorkspace.sbar[93];
acadoVariables.x[94] += acadoWorkspace.sbar[94];
acadoVariables.x[95] += acadoWorkspace.sbar[95];
acadoVariables.x[96] += acadoWorkspace.sbar[96];
acadoVariables.x[97] += acadoWorkspace.sbar[97];
acadoVariables.x[98] += acadoWorkspace.sbar[98];
acadoVariables.x[99] += acadoWorkspace.sbar[99];
acadoVariables.x[100] += acadoWorkspace.sbar[100];
acadoVariables.x[101] += acadoWorkspace.sbar[101];
acadoVariables.x[102] += acadoWorkspace.sbar[102];
acadoVariables.x[103] += acadoWorkspace.sbar[103];
acadoVariables.x[104] += acadoWorkspace.sbar[104];
acadoVariables.x[105] += acadoWorkspace.sbar[105];
acadoVariables.x[106] += acadoWorkspace.sbar[106];
acadoVariables.x[107] += acadoWorkspace.sbar[107];
acadoVariables.x[108] += acadoWorkspace.sbar[108];
acadoVariables.x[109] += acadoWorkspace.sbar[109];
acadoVariables.x[110] += acadoWorkspace.sbar[110];
acadoVariables.x[111] += acadoWorkspace.sbar[111];
acadoVariables.x[112] += acadoWorkspace.sbar[112];
acadoVariables.x[113] += acadoWorkspace.sbar[113];
acadoVariables.x[114] += acadoWorkspace.sbar[114];
acadoVariables.x[115] += acadoWorkspace.sbar[115];
acadoVariables.x[116] += acadoWorkspace.sbar[116];
acadoVariables.x[117] += acadoWorkspace.sbar[117];
acadoVariables.x[118] += acadoWorkspace.sbar[118];
acadoVariables.x[119] += acadoWorkspace.sbar[119];
acadoVariables.x[120] += acadoWorkspace.sbar[120];
acadoVariables.x[121] += acadoWorkspace.sbar[121];
acadoVariables.x[122] += acadoWorkspace.sbar[122];
acadoVariables.x[123] += acadoWorkspace.sbar[123];
acadoVariables.x[124] += acadoWorkspace.sbar[124];
acadoVariables.x[125] += acadoWorkspace.sbar[125];
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -1.0000000000000000e+01;
acadoVariables.lbValues[1] = -1.0000000000000000e+01;
acadoVariables.lbValues[2] = -1.0000000000000000e+01;
acadoVariables.lbValues[3] = -1.0000000000000000e+01;
acadoVariables.lbValues[4] = -1.0000000000000000e+01;
acadoVariables.lbValues[5] = -1.0000000000000000e+01;
acadoVariables.lbValues[6] = -1.0000000000000000e+01;
acadoVariables.lbValues[7] = -1.0000000000000000e+01;
acadoVariables.lbValues[8] = -1.0000000000000000e+01;
acadoVariables.lbValues[9] = -1.0000000000000000e+01;
acadoVariables.lbValues[10] = -1.0000000000000000e+01;
acadoVariables.lbValues[11] = -1.0000000000000000e+01;
acadoVariables.lbValues[12] = -1.0000000000000000e+01;
acadoVariables.lbValues[13] = -1.0000000000000000e+01;
acadoVariables.lbValues[14] = -1.0000000000000000e+01;
acadoVariables.lbValues[15] = -1.0000000000000000e+01;
acadoVariables.lbValues[16] = -1.0000000000000000e+01;
acadoVariables.lbValues[17] = -1.0000000000000000e+01;
acadoVariables.lbValues[18] = -1.0000000000000000e+01;
acadoVariables.lbValues[19] = -1.0000000000000000e+01;
acadoVariables.lbValues[20] = -1.0000000000000000e+01;
acadoVariables.lbValues[21] = -1.0000000000000000e+01;
acadoVariables.lbValues[22] = -1.0000000000000000e+01;
acadoVariables.lbValues[23] = -1.0000000000000000e+01;
acadoVariables.lbValues[24] = -1.0000000000000000e+01;
acadoVariables.lbValues[25] = -1.0000000000000000e+01;
acadoVariables.lbValues[26] = -1.0000000000000000e+01;
acadoVariables.lbValues[27] = -1.0000000000000000e+01;
acadoVariables.lbValues[28] = -1.0000000000000000e+01;
acadoVariables.lbValues[29] = -1.0000000000000000e+01;
acadoVariables.lbValues[30] = -1.0000000000000000e+01;
acadoVariables.lbValues[31] = -1.0000000000000000e+01;
acadoVariables.lbValues[32] = -1.0000000000000000e+01;
acadoVariables.lbValues[33] = -1.0000000000000000e+01;
acadoVariables.lbValues[34] = -1.0000000000000000e+01;
acadoVariables.lbValues[35] = -1.0000000000000000e+01;
acadoVariables.lbValues[36] = -1.0000000000000000e+01;
acadoVariables.lbValues[37] = -1.0000000000000000e+01;
acadoVariables.lbValues[38] = -1.0000000000000000e+01;
acadoVariables.lbValues[39] = -1.0000000000000000e+01;
acadoVariables.ubValues[0] = 1.0000000000000000e+01;
acadoVariables.ubValues[1] = 1.0000000000000000e+01;
acadoVariables.ubValues[2] = 1.0000000000000000e+01;
acadoVariables.ubValues[3] = 1.0000000000000000e+01;
acadoVariables.ubValues[4] = 1.0000000000000000e+01;
acadoVariables.ubValues[5] = 1.0000000000000000e+01;
acadoVariables.ubValues[6] = 1.0000000000000000e+01;
acadoVariables.ubValues[7] = 1.0000000000000000e+01;
acadoVariables.ubValues[8] = 1.0000000000000000e+01;
acadoVariables.ubValues[9] = 1.0000000000000000e+01;
acadoVariables.ubValues[10] = 1.0000000000000000e+01;
acadoVariables.ubValues[11] = 1.0000000000000000e+01;
acadoVariables.ubValues[12] = 1.0000000000000000e+01;
acadoVariables.ubValues[13] = 1.0000000000000000e+01;
acadoVariables.ubValues[14] = 1.0000000000000000e+01;
acadoVariables.ubValues[15] = 1.0000000000000000e+01;
acadoVariables.ubValues[16] = 1.0000000000000000e+01;
acadoVariables.ubValues[17] = 1.0000000000000000e+01;
acadoVariables.ubValues[18] = 1.0000000000000000e+01;
acadoVariables.ubValues[19] = 1.0000000000000000e+01;
acadoVariables.ubValues[20] = 1.0000000000000000e+01;
acadoVariables.ubValues[21] = 1.0000000000000000e+01;
acadoVariables.ubValues[22] = 1.0000000000000000e+01;
acadoVariables.ubValues[23] = 1.0000000000000000e+01;
acadoVariables.ubValues[24] = 1.0000000000000000e+01;
acadoVariables.ubValues[25] = 1.0000000000000000e+01;
acadoVariables.ubValues[26] = 1.0000000000000000e+01;
acadoVariables.ubValues[27] = 1.0000000000000000e+01;
acadoVariables.ubValues[28] = 1.0000000000000000e+01;
acadoVariables.ubValues[29] = 1.0000000000000000e+01;
acadoVariables.ubValues[30] = 1.0000000000000000e+01;
acadoVariables.ubValues[31] = 1.0000000000000000e+01;
acadoVariables.ubValues[32] = 1.0000000000000000e+01;
acadoVariables.ubValues[33] = 1.0000000000000000e+01;
acadoVariables.ubValues[34] = 1.0000000000000000e+01;
acadoVariables.ubValues[35] = 1.0000000000000000e+01;
acadoVariables.ubValues[36] = 1.0000000000000000e+01;
acadoVariables.ubValues[37] = 1.0000000000000000e+01;
acadoVariables.ubValues[38] = 1.0000000000000000e+01;
acadoVariables.ubValues[39] = 1.0000000000000000e+01;
acadoVariables.lbAValues[0] = -1.0000000000000000e+01;
acadoVariables.lbAValues[1] = -1.0000000000000000e+01;
acadoVariables.lbAValues[2] = -1.5707963267948966e+00;
acadoVariables.lbAValues[3] = -1.0000000000000000e+01;
acadoVariables.lbAValues[4] = -1.0000000000000000e+01;
acadoVariables.lbAValues[5] = -1.0000000000000000e+01;
acadoVariables.lbAValues[6] = -1.0000000000000000e+01;
acadoVariables.lbAValues[7] = -1.0000000000000000e+01;
acadoVariables.lbAValues[8] = -1.5707963267948966e+00;
acadoVariables.lbAValues[9] = -1.0000000000000000e+01;
acadoVariables.lbAValues[10] = -1.0000000000000000e+01;
acadoVariables.lbAValues[11] = -1.0000000000000000e+01;
acadoVariables.lbAValues[12] = -1.0000000000000000e+01;
acadoVariables.lbAValues[13] = -1.0000000000000000e+01;
acadoVariables.lbAValues[14] = -1.5707963267948966e+00;
acadoVariables.lbAValues[15] = -1.0000000000000000e+01;
acadoVariables.lbAValues[16] = -1.0000000000000000e+01;
acadoVariables.lbAValues[17] = -1.0000000000000000e+01;
acadoVariables.lbAValues[18] = -1.0000000000000000e+01;
acadoVariables.lbAValues[19] = -1.0000000000000000e+01;
acadoVariables.lbAValues[20] = -1.5707963267948966e+00;
acadoVariables.lbAValues[21] = -1.0000000000000000e+01;
acadoVariables.lbAValues[22] = -1.0000000000000000e+01;
acadoVariables.lbAValues[23] = -1.0000000000000000e+01;
acadoVariables.lbAValues[24] = -1.0000000000000000e+01;
acadoVariables.lbAValues[25] = -1.0000000000000000e+01;
acadoVariables.lbAValues[26] = -1.5707963267948966e+00;
acadoVariables.lbAValues[27] = -1.0000000000000000e+01;
acadoVariables.lbAValues[28] = -1.0000000000000000e+01;
acadoVariables.lbAValues[29] = -1.0000000000000000e+01;
acadoVariables.lbAValues[30] = -1.0000000000000000e+01;
acadoVariables.lbAValues[31] = -1.0000000000000000e+01;
acadoVariables.lbAValues[32] = -1.5707963267948966e+00;
acadoVariables.lbAValues[33] = -1.0000000000000000e+01;
acadoVariables.lbAValues[34] = -1.0000000000000000e+01;
acadoVariables.lbAValues[35] = -1.0000000000000000e+01;
acadoVariables.lbAValues[36] = -1.0000000000000000e+01;
acadoVariables.lbAValues[37] = -1.0000000000000000e+01;
acadoVariables.lbAValues[38] = -1.5707963267948966e+00;
acadoVariables.lbAValues[39] = -1.0000000000000000e+01;
acadoVariables.lbAValues[40] = -1.0000000000000000e+01;
acadoVariables.lbAValues[41] = -1.0000000000000000e+01;
acadoVariables.lbAValues[42] = -1.0000000000000000e+01;
acadoVariables.lbAValues[43] = -1.0000000000000000e+01;
acadoVariables.lbAValues[44] = -1.5707963267948966e+00;
acadoVariables.lbAValues[45] = -1.0000000000000000e+01;
acadoVariables.lbAValues[46] = -1.0000000000000000e+01;
acadoVariables.lbAValues[47] = -1.0000000000000000e+01;
acadoVariables.lbAValues[48] = -1.0000000000000000e+01;
acadoVariables.lbAValues[49] = -1.0000000000000000e+01;
acadoVariables.lbAValues[50] = -1.5707963267948966e+00;
acadoVariables.lbAValues[51] = -1.0000000000000000e+01;
acadoVariables.lbAValues[52] = -1.0000000000000000e+01;
acadoVariables.lbAValues[53] = -1.0000000000000000e+01;
acadoVariables.lbAValues[54] = -1.0000000000000000e+01;
acadoVariables.lbAValues[55] = -1.0000000000000000e+01;
acadoVariables.lbAValues[56] = -1.5707963267948966e+00;
acadoVariables.lbAValues[57] = -1.0000000000000000e+01;
acadoVariables.lbAValues[58] = -1.0000000000000000e+01;
acadoVariables.lbAValues[59] = -1.0000000000000000e+01;
acadoVariables.lbAValues[60] = -1.0000000000000000e+01;
acadoVariables.lbAValues[61] = -1.0000000000000000e+01;
acadoVariables.lbAValues[62] = -1.5707963267948966e+00;
acadoVariables.lbAValues[63] = -1.0000000000000000e+01;
acadoVariables.lbAValues[64] = -1.0000000000000000e+01;
acadoVariables.lbAValues[65] = -1.0000000000000000e+01;
acadoVariables.lbAValues[66] = -1.0000000000000000e+01;
acadoVariables.lbAValues[67] = -1.0000000000000000e+01;
acadoVariables.lbAValues[68] = -1.5707963267948966e+00;
acadoVariables.lbAValues[69] = -1.0000000000000000e+01;
acadoVariables.lbAValues[70] = -1.0000000000000000e+01;
acadoVariables.lbAValues[71] = -1.0000000000000000e+01;
acadoVariables.lbAValues[72] = -1.0000000000000000e+01;
acadoVariables.lbAValues[73] = -1.0000000000000000e+01;
acadoVariables.lbAValues[74] = -1.5707963267948966e+00;
acadoVariables.lbAValues[75] = -1.0000000000000000e+01;
acadoVariables.lbAValues[76] = -1.0000000000000000e+01;
acadoVariables.lbAValues[77] = -1.0000000000000000e+01;
acadoVariables.lbAValues[78] = -1.0000000000000000e+01;
acadoVariables.lbAValues[79] = -1.0000000000000000e+01;
acadoVariables.lbAValues[80] = -1.5707963267948966e+00;
acadoVariables.lbAValues[81] = -1.0000000000000000e+01;
acadoVariables.lbAValues[82] = -1.0000000000000000e+01;
acadoVariables.lbAValues[83] = -1.0000000000000000e+01;
acadoVariables.lbAValues[84] = -1.0000000000000000e+01;
acadoVariables.lbAValues[85] = -1.0000000000000000e+01;
acadoVariables.lbAValues[86] = -1.5707963267948966e+00;
acadoVariables.lbAValues[87] = -1.0000000000000000e+01;
acadoVariables.lbAValues[88] = -1.0000000000000000e+01;
acadoVariables.lbAValues[89] = -1.0000000000000000e+01;
acadoVariables.lbAValues[90] = -1.0000000000000000e+01;
acadoVariables.lbAValues[91] = -1.0000000000000000e+01;
acadoVariables.lbAValues[92] = -1.5707963267948966e+00;
acadoVariables.lbAValues[93] = -1.0000000000000000e+01;
acadoVariables.lbAValues[94] = -1.0000000000000000e+01;
acadoVariables.lbAValues[95] = -1.0000000000000000e+01;
acadoVariables.lbAValues[96] = -1.0000000000000000e+01;
acadoVariables.lbAValues[97] = -1.0000000000000000e+01;
acadoVariables.lbAValues[98] = -1.5707963267948966e+00;
acadoVariables.lbAValues[99] = -1.0000000000000000e+01;
acadoVariables.lbAValues[100] = -1.0000000000000000e+01;
acadoVariables.lbAValues[101] = -1.0000000000000000e+01;
acadoVariables.lbAValues[102] = -1.0000000000000000e+01;
acadoVariables.lbAValues[103] = -1.0000000000000000e+01;
acadoVariables.lbAValues[104] = -1.5707963267948966e+00;
acadoVariables.lbAValues[105] = -1.0000000000000000e+01;
acadoVariables.lbAValues[106] = -1.0000000000000000e+01;
acadoVariables.lbAValues[107] = -1.0000000000000000e+01;
acadoVariables.lbAValues[108] = -1.0000000000000000e+01;
acadoVariables.lbAValues[109] = -1.0000000000000000e+01;
acadoVariables.lbAValues[110] = -1.5707963267948966e+00;
acadoVariables.lbAValues[111] = -1.0000000000000000e+01;
acadoVariables.lbAValues[112] = -1.0000000000000000e+01;
acadoVariables.lbAValues[113] = -1.0000000000000000e+01;
acadoVariables.lbAValues[114] = -1.0000000000000000e+01;
acadoVariables.lbAValues[115] = -1.0000000000000000e+01;
acadoVariables.lbAValues[116] = -1.5707963267948966e+00;
acadoVariables.lbAValues[117] = -1.0000000000000000e+01;
acadoVariables.lbAValues[118] = -1.0000000000000000e+01;
acadoVariables.lbAValues[119] = -1.0000000000000000e+01;
acadoVariables.ubAValues[0] = 1.0000000000000000e+01;
acadoVariables.ubAValues[1] = 1.0000000000000000e+01;
acadoVariables.ubAValues[2] = 1.5707963267948966e+00;
acadoVariables.ubAValues[3] = 1.0000000000000000e+01;
acadoVariables.ubAValues[4] = 1.0000000000000000e+01;
acadoVariables.ubAValues[5] = 1.0000000000000000e+01;
acadoVariables.ubAValues[6] = 1.0000000000000000e+01;
acadoVariables.ubAValues[7] = 1.0000000000000000e+01;
acadoVariables.ubAValues[8] = 1.5707963267948966e+00;
acadoVariables.ubAValues[9] = 1.0000000000000000e+01;
acadoVariables.ubAValues[10] = 1.0000000000000000e+01;
acadoVariables.ubAValues[11] = 1.0000000000000000e+01;
acadoVariables.ubAValues[12] = 1.0000000000000000e+01;
acadoVariables.ubAValues[13] = 1.0000000000000000e+01;
acadoVariables.ubAValues[14] = 1.5707963267948966e+00;
acadoVariables.ubAValues[15] = 1.0000000000000000e+01;
acadoVariables.ubAValues[16] = 1.0000000000000000e+01;
acadoVariables.ubAValues[17] = 1.0000000000000000e+01;
acadoVariables.ubAValues[18] = 1.0000000000000000e+01;
acadoVariables.ubAValues[19] = 1.0000000000000000e+01;
acadoVariables.ubAValues[20] = 1.5707963267948966e+00;
acadoVariables.ubAValues[21] = 1.0000000000000000e+01;
acadoVariables.ubAValues[22] = 1.0000000000000000e+01;
acadoVariables.ubAValues[23] = 1.0000000000000000e+01;
acadoVariables.ubAValues[24] = 1.0000000000000000e+01;
acadoVariables.ubAValues[25] = 1.0000000000000000e+01;
acadoVariables.ubAValues[26] = 1.5707963267948966e+00;
acadoVariables.ubAValues[27] = 1.0000000000000000e+01;
acadoVariables.ubAValues[28] = 1.0000000000000000e+01;
acadoVariables.ubAValues[29] = 1.0000000000000000e+01;
acadoVariables.ubAValues[30] = 1.0000000000000000e+01;
acadoVariables.ubAValues[31] = 1.0000000000000000e+01;
acadoVariables.ubAValues[32] = 1.5707963267948966e+00;
acadoVariables.ubAValues[33] = 1.0000000000000000e+01;
acadoVariables.ubAValues[34] = 1.0000000000000000e+01;
acadoVariables.ubAValues[35] = 1.0000000000000000e+01;
acadoVariables.ubAValues[36] = 1.0000000000000000e+01;
acadoVariables.ubAValues[37] = 1.0000000000000000e+01;
acadoVariables.ubAValues[38] = 1.5707963267948966e+00;
acadoVariables.ubAValues[39] = 1.0000000000000000e+01;
acadoVariables.ubAValues[40] = 1.0000000000000000e+01;
acadoVariables.ubAValues[41] = 1.0000000000000000e+01;
acadoVariables.ubAValues[42] = 1.0000000000000000e+01;
acadoVariables.ubAValues[43] = 1.0000000000000000e+01;
acadoVariables.ubAValues[44] = 1.5707963267948966e+00;
acadoVariables.ubAValues[45] = 1.0000000000000000e+01;
acadoVariables.ubAValues[46] = 1.0000000000000000e+01;
acadoVariables.ubAValues[47] = 1.0000000000000000e+01;
acadoVariables.ubAValues[48] = 1.0000000000000000e+01;
acadoVariables.ubAValues[49] = 1.0000000000000000e+01;
acadoVariables.ubAValues[50] = 1.5707963267948966e+00;
acadoVariables.ubAValues[51] = 1.0000000000000000e+01;
acadoVariables.ubAValues[52] = 1.0000000000000000e+01;
acadoVariables.ubAValues[53] = 1.0000000000000000e+01;
acadoVariables.ubAValues[54] = 1.0000000000000000e+01;
acadoVariables.ubAValues[55] = 1.0000000000000000e+01;
acadoVariables.ubAValues[56] = 1.5707963267948966e+00;
acadoVariables.ubAValues[57] = 1.0000000000000000e+01;
acadoVariables.ubAValues[58] = 1.0000000000000000e+01;
acadoVariables.ubAValues[59] = 1.0000000000000000e+01;
acadoVariables.ubAValues[60] = 1.0000000000000000e+01;
acadoVariables.ubAValues[61] = 1.0000000000000000e+01;
acadoVariables.ubAValues[62] = 1.5707963267948966e+00;
acadoVariables.ubAValues[63] = 1.0000000000000000e+01;
acadoVariables.ubAValues[64] = 1.0000000000000000e+01;
acadoVariables.ubAValues[65] = 1.0000000000000000e+01;
acadoVariables.ubAValues[66] = 1.0000000000000000e+01;
acadoVariables.ubAValues[67] = 1.0000000000000000e+01;
acadoVariables.ubAValues[68] = 1.5707963267948966e+00;
acadoVariables.ubAValues[69] = 1.0000000000000000e+01;
acadoVariables.ubAValues[70] = 1.0000000000000000e+01;
acadoVariables.ubAValues[71] = 1.0000000000000000e+01;
acadoVariables.ubAValues[72] = 1.0000000000000000e+01;
acadoVariables.ubAValues[73] = 1.0000000000000000e+01;
acadoVariables.ubAValues[74] = 1.5707963267948966e+00;
acadoVariables.ubAValues[75] = 1.0000000000000000e+01;
acadoVariables.ubAValues[76] = 1.0000000000000000e+01;
acadoVariables.ubAValues[77] = 1.0000000000000000e+01;
acadoVariables.ubAValues[78] = 1.0000000000000000e+01;
acadoVariables.ubAValues[79] = 1.0000000000000000e+01;
acadoVariables.ubAValues[80] = 1.5707963267948966e+00;
acadoVariables.ubAValues[81] = 1.0000000000000000e+01;
acadoVariables.ubAValues[82] = 1.0000000000000000e+01;
acadoVariables.ubAValues[83] = 1.0000000000000000e+01;
acadoVariables.ubAValues[84] = 1.0000000000000000e+01;
acadoVariables.ubAValues[85] = 1.0000000000000000e+01;
acadoVariables.ubAValues[86] = 1.5707963267948966e+00;
acadoVariables.ubAValues[87] = 1.0000000000000000e+01;
acadoVariables.ubAValues[88] = 1.0000000000000000e+01;
acadoVariables.ubAValues[89] = 1.0000000000000000e+01;
acadoVariables.ubAValues[90] = 1.0000000000000000e+01;
acadoVariables.ubAValues[91] = 1.0000000000000000e+01;
acadoVariables.ubAValues[92] = 1.5707963267948966e+00;
acadoVariables.ubAValues[93] = 1.0000000000000000e+01;
acadoVariables.ubAValues[94] = 1.0000000000000000e+01;
acadoVariables.ubAValues[95] = 1.0000000000000000e+01;
acadoVariables.ubAValues[96] = 1.0000000000000000e+01;
acadoVariables.ubAValues[97] = 1.0000000000000000e+01;
acadoVariables.ubAValues[98] = 1.5707963267948966e+00;
acadoVariables.ubAValues[99] = 1.0000000000000000e+01;
acadoVariables.ubAValues[100] = 1.0000000000000000e+01;
acadoVariables.ubAValues[101] = 1.0000000000000000e+01;
acadoVariables.ubAValues[102] = 1.0000000000000000e+01;
acadoVariables.ubAValues[103] = 1.0000000000000000e+01;
acadoVariables.ubAValues[104] = 1.5707963267948966e+00;
acadoVariables.ubAValues[105] = 1.0000000000000000e+01;
acadoVariables.ubAValues[106] = 1.0000000000000000e+01;
acadoVariables.ubAValues[107] = 1.0000000000000000e+01;
acadoVariables.ubAValues[108] = 1.0000000000000000e+01;
acadoVariables.ubAValues[109] = 1.0000000000000000e+01;
acadoVariables.ubAValues[110] = 1.5707963267948966e+00;
acadoVariables.ubAValues[111] = 1.0000000000000000e+01;
acadoVariables.ubAValues[112] = 1.0000000000000000e+01;
acadoVariables.ubAValues[113] = 1.0000000000000000e+01;
acadoVariables.ubAValues[114] = 1.0000000000000000e+01;
acadoVariables.ubAValues[115] = 1.0000000000000000e+01;
acadoVariables.ubAValues[116] = 1.5707963267948966e+00;
acadoVariables.ubAValues[117] = 1.0000000000000000e+01;
acadoVariables.ubAValues[118] = 1.0000000000000000e+01;
acadoVariables.ubAValues[119] = 1.0000000000000000e+01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
state[0] = acadoVariables.x[index * 6];
state[1] = acadoVariables.x[index * 6 + 1];
state[2] = acadoVariables.x[index * 6 + 2];
state[3] = acadoVariables.x[index * 6 + 3];
state[4] = acadoVariables.x[index * 6 + 4];
state[5] = acadoVariables.x[index * 6 + 5];
state[54] = acadoVariables.u[index * 2];
state[55] = acadoVariables.u[index * 2 + 1];
state[56] = acadoVariables.od[index * 3];
state[57] = acadoVariables.od[index * 3 + 1];
state[58] = acadoVariables.od[index * 3 + 2];

acado_integrate(state, index == 0);

acadoVariables.x[index * 6 + 6] = state[0];
acadoVariables.x[index * 6 + 7] = state[1];
acadoVariables.x[index * 6 + 8] = state[2];
acadoVariables.x[index * 6 + 9] = state[3];
acadoVariables.x[index * 6 + 10] = state[4];
acadoVariables.x[index * 6 + 11] = state[5];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 6] = acadoVariables.x[index * 6 + 6];
acadoVariables.x[index * 6 + 1] = acadoVariables.x[index * 6 + 7];
acadoVariables.x[index * 6 + 2] = acadoVariables.x[index * 6 + 8];
acadoVariables.x[index * 6 + 3] = acadoVariables.x[index * 6 + 9];
acadoVariables.x[index * 6 + 4] = acadoVariables.x[index * 6 + 10];
acadoVariables.x[index * 6 + 5] = acadoVariables.x[index * 6 + 11];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[120] = xEnd[0];
acadoVariables.x[121] = xEnd[1];
acadoVariables.x[122] = xEnd[2];
acadoVariables.x[123] = xEnd[3];
acadoVariables.x[124] = xEnd[4];
acadoVariables.x[125] = xEnd[5];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[120];
state[1] = acadoVariables.x[121];
state[2] = acadoVariables.x[122];
state[3] = acadoVariables.x[123];
state[4] = acadoVariables.x[124];
state[5] = acadoVariables.x[125];
if (uEnd != 0)
{
state[54] = uEnd[0];
state[55] = uEnd[1];
}
else
{
state[54] = acadoVariables.u[38];
state[55] = acadoVariables.u[39];
}
state[56] = acadoVariables.od[60];
state[57] = acadoVariables.od[61];
state[58] = acadoVariables.od[62];

acado_integrate(state, 1);

acadoVariables.x[120] = state[0];
acadoVariables.x[121] = state[1];
acadoVariables.x[122] = state[2];
acadoVariables.x[123] = state[3];
acadoVariables.x[124] = state[4];
acadoVariables.x[125] = state[5];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[38] = uEnd[0];
acadoVariables.u[39] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39];
kkt = fabs( kkt );
for (index = 0; index < 40; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 120; ++index)
{
prd = acadoWorkspace.y[index + 40];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 8 */
real_t tmpDy[ 8 ];

/** Row vector of size: 6 */
real_t tmpDyN[ 6 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 6];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 6 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 6 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 6 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 6 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 6 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[7] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 8] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 8];
acadoWorkspace.Dy[lRun1 * 8 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 8 + 1];
acadoWorkspace.Dy[lRun1 * 8 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 8 + 2];
acadoWorkspace.Dy[lRun1 * 8 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 8 + 3];
acadoWorkspace.Dy[lRun1 * 8 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 8 + 4];
acadoWorkspace.Dy[lRun1 * 8 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 8 + 5];
acadoWorkspace.Dy[lRun1 * 8 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 8 + 6];
acadoWorkspace.Dy[lRun1 * 8 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 8 + 7];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.x[124];
acadoWorkspace.objValueIn[5] = acadoVariables.x[125];
acadoWorkspace.objValueIn[6] = acadoVariables.od[60];
acadoWorkspace.objValueIn[7] = acadoVariables.od[61];
acadoWorkspace.objValueIn[8] = acadoVariables.od[62];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 8]*acadoVariables.W[lRun1 * 64] + acadoWorkspace.Dy[lRun1 * 8 + 1]*acadoVariables.W[lRun1 * 64 + 8] + acadoWorkspace.Dy[lRun1 * 8 + 2]*acadoVariables.W[lRun1 * 64 + 16] + acadoWorkspace.Dy[lRun1 * 8 + 3]*acadoVariables.W[lRun1 * 64 + 24] + acadoWorkspace.Dy[lRun1 * 8 + 4]*acadoVariables.W[lRun1 * 64 + 32] + acadoWorkspace.Dy[lRun1 * 8 + 5]*acadoVariables.W[lRun1 * 64 + 40] + acadoWorkspace.Dy[lRun1 * 8 + 6]*acadoVariables.W[lRun1 * 64 + 48] + acadoWorkspace.Dy[lRun1 * 8 + 7]*acadoVariables.W[lRun1 * 64 + 56];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 8]*acadoVariables.W[lRun1 * 64 + 1] + acadoWorkspace.Dy[lRun1 * 8 + 1]*acadoVariables.W[lRun1 * 64 + 9] + acadoWorkspace.Dy[lRun1 * 8 + 2]*acadoVariables.W[lRun1 * 64 + 17] + acadoWorkspace.Dy[lRun1 * 8 + 3]*acadoVariables.W[lRun1 * 64 + 25] + acadoWorkspace.Dy[lRun1 * 8 + 4]*acadoVariables.W[lRun1 * 64 + 33] + acadoWorkspace.Dy[lRun1 * 8 + 5]*acadoVariables.W[lRun1 * 64 + 41] + acadoWorkspace.Dy[lRun1 * 8 + 6]*acadoVariables.W[lRun1 * 64 + 49] + acadoWorkspace.Dy[lRun1 * 8 + 7]*acadoVariables.W[lRun1 * 64 + 57];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 8]*acadoVariables.W[lRun1 * 64 + 2] + acadoWorkspace.Dy[lRun1 * 8 + 1]*acadoVariables.W[lRun1 * 64 + 10] + acadoWorkspace.Dy[lRun1 * 8 + 2]*acadoVariables.W[lRun1 * 64 + 18] + acadoWorkspace.Dy[lRun1 * 8 + 3]*acadoVariables.W[lRun1 * 64 + 26] + acadoWorkspace.Dy[lRun1 * 8 + 4]*acadoVariables.W[lRun1 * 64 + 34] + acadoWorkspace.Dy[lRun1 * 8 + 5]*acadoVariables.W[lRun1 * 64 + 42] + acadoWorkspace.Dy[lRun1 * 8 + 6]*acadoVariables.W[lRun1 * 64 + 50] + acadoWorkspace.Dy[lRun1 * 8 + 7]*acadoVariables.W[lRun1 * 64 + 58];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 8]*acadoVariables.W[lRun1 * 64 + 3] + acadoWorkspace.Dy[lRun1 * 8 + 1]*acadoVariables.W[lRun1 * 64 + 11] + acadoWorkspace.Dy[lRun1 * 8 + 2]*acadoVariables.W[lRun1 * 64 + 19] + acadoWorkspace.Dy[lRun1 * 8 + 3]*acadoVariables.W[lRun1 * 64 + 27] + acadoWorkspace.Dy[lRun1 * 8 + 4]*acadoVariables.W[lRun1 * 64 + 35] + acadoWorkspace.Dy[lRun1 * 8 + 5]*acadoVariables.W[lRun1 * 64 + 43] + acadoWorkspace.Dy[lRun1 * 8 + 6]*acadoVariables.W[lRun1 * 64 + 51] + acadoWorkspace.Dy[lRun1 * 8 + 7]*acadoVariables.W[lRun1 * 64 + 59];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 8]*acadoVariables.W[lRun1 * 64 + 4] + acadoWorkspace.Dy[lRun1 * 8 + 1]*acadoVariables.W[lRun1 * 64 + 12] + acadoWorkspace.Dy[lRun1 * 8 + 2]*acadoVariables.W[lRun1 * 64 + 20] + acadoWorkspace.Dy[lRun1 * 8 + 3]*acadoVariables.W[lRun1 * 64 + 28] + acadoWorkspace.Dy[lRun1 * 8 + 4]*acadoVariables.W[lRun1 * 64 + 36] + acadoWorkspace.Dy[lRun1 * 8 + 5]*acadoVariables.W[lRun1 * 64 + 44] + acadoWorkspace.Dy[lRun1 * 8 + 6]*acadoVariables.W[lRun1 * 64 + 52] + acadoWorkspace.Dy[lRun1 * 8 + 7]*acadoVariables.W[lRun1 * 64 + 60];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 8]*acadoVariables.W[lRun1 * 64 + 5] + acadoWorkspace.Dy[lRun1 * 8 + 1]*acadoVariables.W[lRun1 * 64 + 13] + acadoWorkspace.Dy[lRun1 * 8 + 2]*acadoVariables.W[lRun1 * 64 + 21] + acadoWorkspace.Dy[lRun1 * 8 + 3]*acadoVariables.W[lRun1 * 64 + 29] + acadoWorkspace.Dy[lRun1 * 8 + 4]*acadoVariables.W[lRun1 * 64 + 37] + acadoWorkspace.Dy[lRun1 * 8 + 5]*acadoVariables.W[lRun1 * 64 + 45] + acadoWorkspace.Dy[lRun1 * 8 + 6]*acadoVariables.W[lRun1 * 64 + 53] + acadoWorkspace.Dy[lRun1 * 8 + 7]*acadoVariables.W[lRun1 * 64 + 61];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 8]*acadoVariables.W[lRun1 * 64 + 6] + acadoWorkspace.Dy[lRun1 * 8 + 1]*acadoVariables.W[lRun1 * 64 + 14] + acadoWorkspace.Dy[lRun1 * 8 + 2]*acadoVariables.W[lRun1 * 64 + 22] + acadoWorkspace.Dy[lRun1 * 8 + 3]*acadoVariables.W[lRun1 * 64 + 30] + acadoWorkspace.Dy[lRun1 * 8 + 4]*acadoVariables.W[lRun1 * 64 + 38] + acadoWorkspace.Dy[lRun1 * 8 + 5]*acadoVariables.W[lRun1 * 64 + 46] + acadoWorkspace.Dy[lRun1 * 8 + 6]*acadoVariables.W[lRun1 * 64 + 54] + acadoWorkspace.Dy[lRun1 * 8 + 7]*acadoVariables.W[lRun1 * 64 + 62];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 8]*acadoVariables.W[lRun1 * 64 + 7] + acadoWorkspace.Dy[lRun1 * 8 + 1]*acadoVariables.W[lRun1 * 64 + 15] + acadoWorkspace.Dy[lRun1 * 8 + 2]*acadoVariables.W[lRun1 * 64 + 23] + acadoWorkspace.Dy[lRun1 * 8 + 3]*acadoVariables.W[lRun1 * 64 + 31] + acadoWorkspace.Dy[lRun1 * 8 + 4]*acadoVariables.W[lRun1 * 64 + 39] + acadoWorkspace.Dy[lRun1 * 8 + 5]*acadoVariables.W[lRun1 * 64 + 47] + acadoWorkspace.Dy[lRun1 * 8 + 6]*acadoVariables.W[lRun1 * 64 + 55] + acadoWorkspace.Dy[lRun1 * 8 + 7]*acadoVariables.W[lRun1 * 64 + 63];
objVal += + acadoWorkspace.Dy[lRun1 * 8]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 8 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 8 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 8 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 8 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 8 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 8 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 8 + 7]*tmpDy[7];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[7];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[14];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[21];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[28];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[35];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5];

objVal *= 0.5;
return objVal;
}

