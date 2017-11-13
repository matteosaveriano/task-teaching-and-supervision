#ifndef LWRCONTROLLERDEFINITIONS_H
#define LWRCONTROLLERDEFINITIONS_H

// Cartesian stiffness
#define MAX_CART_LIN_STIF 5000.0
#define MAX_CART_ANG_STIF 300.0

#define DEF_CART_LIN_STIF 2000.0
#define DEF_CART_ANG_STIF 200.0

#define MIN_CART_LIN_STIF 0.01
#define MIN_CART_ANG_STIF 0.01

// Cartesian damping
#define MAX_CART_LIN_DAMP 1.0
#define MAX_CART_ANG_DAMP 1.0

#define DEF_CART_LIN_DAMP 0.7
#define DEF_CART_ANG_DAMP 0.7

#define MIN_CART_LIN_DAMP 0.1
#define MIN_CART_ANG_DAMP 0.1


// Joint stiffness
#define MAX_JOINT_STIF 2000.0

#define DEF_JOINT_STIF 1000.0

#define MIN_JOINT_STIF 0.01

// Joint damping
#define MAX_JOINT_DAMP 1.0

#define DEF_JOINT_DAMP 0.7

#define MIN_JOINT_DAMP 0.1

#endif // objectSLIST_H
