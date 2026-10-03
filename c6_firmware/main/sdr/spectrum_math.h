/* Integer power-to-wire conversion, shared by scalar spectrum backends.
 * Piecewise-linear log2 approximation has <0.06 dB error before 0.5 dB
 * quantization. It avoids soft-float logarithms in the capture deadline. */
#pragma once
#include <stdint.h>
static inline uint8_t spectrum_power_code(uint32_t power) {
    static const uint16_t log2_q12[17]={0,358,696,1016,1319,1607,1882,2144,2396,2637,2869,3092,3307,3515,3715,3908,4096};
    if(!power)return 0;
    unsigned exponent=31u-__builtin_clz(power);
    uint32_t norm=(power<<(31u-exponent))>>23; /* 256..511 */
    unsigned slot=(norm-256u)>>4,frac=norm&15u;
    uint32_t logarithm=(exponent<<12)+log2_q12[slot]+((log2_q12[slot+1]-log2_q12[slot])*frac+8)/16;
    unsigned code=(logarithm*1541u+(1u<<19))>>20; /* 20 log10(power), rounded */
    return code>255?255:code;
}
