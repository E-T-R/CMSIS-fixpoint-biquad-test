/*
 * BiquadLP.cpp
 *
 *  Created on: 13.12.2018
 *      Author: hene
 */

#include "stdlib.h"
#include "BiquadLP.h"
#include "lib/Include/arm_math.h"

extern "C" {
    void arm_biquad_cascade_df2T_f32(
    const arm_biquad_cascade_df2T_instance_f32 * S, /* points to an instance of the filter data structure. */
    float32_t * pSrc, /* points to the block of input data. */
    float32_t * pDst, /* points to the block of output data */
    uint32_t blockSize); /* number of samples to process. */

    // arm_biquad_cascade_df2T_f32(&IIR, &fInData, &fOutData, 1);

    void arm_biquad_cascade_df2T_init_f32(
      arm_biquad_cascade_df2T_instance_f32 * S, /*  points to an instance of the filter data structure. */
      uint8_t numStages,   /* number of 2nd order stages in the filter.  */
      float32_t * pCoeffs, /* points to the array of coefficients. The array is of length 5*numStages. */
      float32_t * pState); /* points to the array of state coefficients. The array is of length 2*numStages. */

    // arm_biquad_cascade_df2T_init_f32(&IIR, STAGES, fCoeff, fIIRstate);

    void arm_biquad_cascade_df1_init_q15(
      arm_biquad_casd_df1_inst_q15 * S,
      uint8_t numStages,
      q15_t * pCoeffs,
      q15_t * pState,
      int8_t postShift);

    void arm_biquad_cascade_df1_q15(
      const arm_biquad_casd_df1_inst_q15 * S,
      q15_t * pSrc,
      q15_t * pDst,
      uint32_t blockSize);
}

BiquadLP::BiquadLP(){}

BiquadLP::BiquadLP(uint8_t numStages_, float32_t * pCoeffs_, float32_t * pState_)
{
    //IIR_void = new BiquadF32();
    //arm_biquad_cascade_df2T_init_f32(static_cast<BiquadF32 *>(IIR_void->IIR), numStages_, pCoeffs_, pState_);

    // arm_biquad_cascade_df2T_instance_f32* IIR_r32 = dynamic_cast<arm_biquad_cascade_df2T_instance_f32>(IIR_void);
    // if(IIR_r32 == NULL)
    //   error
   pfIIR = new arm_biquad_cascade_df2T_instance_f32();
   arm_biquad_cascade_df2T_init_f32(pfIIR, numStages_, pCoeffs_, pState_);
}

BiquadLP::BiquadLP(uint8_t numStages_, q15_t * pCoeffs_, q15_t * pState_)
{
//    IIR_void = new arm_biquad_casd_df1_inst_q15();
//    arm_biquad_cascade_df1_init_q15(static_cast<arm_biquad_casd_df1_inst_q15*>(IIR_void), numStages_, pCoeffs_, pState_, 1);
    //piqIIR = (arm_biquad_casd_df1_inst_q15 *)malloc(sizeof(arm_biquad_casd_df1_inst_q15));

    piqIIR = new arm_biquad_casd_df1_inst_q15();
    arm_biquad_cascade_df1_init_q15(piqIIR, numStages_, pCoeffs_, pState_, 1);

    //arm_biquad_cascade_df1_init_q15(&IIR_q15, numStages_, pCoeffs_, pState_, 1);

}

BiquadLP::~BiquadLP()
{
    // TODO Auto-generated destructor stub
}

float BiquadLP::filter(float fInData) {
    //arm_biquad_cascade_df2T_f32(pfIIR, &fInData, &fOutData, 1); // mit Funktionsaufruf + 50ns

       float32_t *pState = pfIIR->pState;             //  State pointer
       float32_t *pCoeffs = pfIIR->pCoeffs;           //  coefficient pointer
       float32_t acc1;                                //  accumulator
       float32_t b0, b1, b2, a1, a2;                  //  Filter coefficients
       float32_t Xn1 = fInData;                       //  input data
       float32_t d1, d2;                              //  state variables

       float32_t p0, p1, p2, p3, p4, A1;

         // Run the below code for Cortex-M4 and Cortex-M3
        // Reading the coefficients
        b0 = pCoeffs[0];
        b1 = pCoeffs[1];
        b2 = pCoeffs[2];
        a1 = pCoeffs[3];
        a2 = pCoeffs[4];

        //Reading the state values
        d1 = pState[0];
        d2 = pState[1];

        // First part of the processing with loop unrolling.  Compute 4 outputs at a time.
        // a second loop below computes the remaining 1 to 3 samples.

        p0 = b0 * Xn1;
        p1 = b1 * Xn1;
        acc1 = p0 + d1;
        p3 = a1 * acc1;
        p2 = b2 * Xn1;
        A1 = p1 + p3;
        p4 = a2 * acc1;
        d1 = A1 + d2;
        d2 = p2 + p4;

        // Store the updated state variables back into the state array
        pState[0] = d1;
        pState[1] = d2;

        return acc1;

//    return fOutData;
}

q15_t BiquadLP::filter(int16_t * pn16InData) {
    arm_biquad_cascade_df1_q15(piqIIR, pn16InData, &n16OutData, 1);
    return n16OutData;
}
