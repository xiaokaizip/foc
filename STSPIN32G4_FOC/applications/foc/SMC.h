//
// Created by SXF-Admin on 26-2-2.
//

#ifndef SMC_H
#define SMC_H


#define R (0.17f/2.0f)
#define L ((60.0f/2.0f)*0.000001f)

#define Ts (1.0f/16000.0f)
#define F (1.0f-Ts*R/L)
#define G (Ts/L)
#endif //SMC_H
