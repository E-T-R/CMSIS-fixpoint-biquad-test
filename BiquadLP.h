/*
 * BiquadLP.h
 *
 *  Created on: 13.12.2018
 *      Author: hene
 */

#ifndef COMMON_OPERATIONMODES_BIQUADLP_H_
#define COMMON_OPERATIONMODES_BIQUADLP_H_

/************ FILTER *******************/
/*
* ACHTUNG:
* MATLAB DF2 verwendet minus für die a Koeffizienten (denominator)
* CMSIS verwendet plus für diese => Vorzeichenwechsel!
*
* http://ch.mathworks.com/help/dsp/ref/biquadfilter.html#brttl5a-1_1
* https://ch.mathworks.com/help/signal/ref/tf2sos.html
*
* http://www.keil.com/pack/doc/CMSIS/DSP/html/group__BiquadCascadeDF2T.html
* CMSIS siehe arm_biquad_cascade_df2T_f32.c
*         y[n] = b0 * x[n] + d1
*         d1 = b1 * x[n] + a1 * y[n] + d2
*         d2 = b2 * x[n] + a2 * y[n]
*
* The coefficients are stored in the array pCoeffs in the following order:
* {b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...}
*/


// number of 2nd order stages in the filter. Overall order is 2*numStages.


#include "arm_math.h"
#include <stdint.h>

class BiquadQ15
{
    public:
        arm_biquad_casd_df1_inst_q15 IIR;
};

class BiquadF32
{
    public:
        arm_biquad_casd_df1_inst_q15 IIR;
};

class BiquadLP
{
    public:
        BiquadLP();
        BiquadLP(uint8_t numStages_, float32_t * pCoeffs_, float32_t * pState_);
        BiquadLP(uint8_t numStages_, q15_t * pCoeffs_, q15_t * pState_);
        virtual ~BiquadLP();

        float filter(float fInData);
        q15_t filter(q15_t * n16InData);

    private:

        float fOutData;
        q15_t n16OutData;

        // gibt es hier Laufzeitunterschiede?
        arm_biquad_cascade_df2T_instance_f32 * pfIIR;
        arm_biquad_casd_df1_inst_q15 * piqIIR;

//        arm_biquad_cascade_df2T_instance_f32 IIR_f;
//        arm_biquad_casd_df1_inst_q15 IIR_q15;
};

#endif /* COMMON_OPERATIONMODES_BIQUADLP_H_ */
