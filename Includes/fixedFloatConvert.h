#ifndef __FIXED_FLOAT_CONVERT_H__
#define __FIXED_FLOAT_CONVERT_H__

#include <assert.h>

short float_to_fixed(double x);
double fixed_to_float(short x);
short check_fixed(long x);
short add_fixed(short x, short y);
short sub_fixed(short x, short y);
short mult_fixed(short x, short y);
short div_fixed(short x, short y);

const double MIN_REAL = (short)0x8000 / 16.0;
const double MAX_REAL = (short)0x7fff / 16.0;

const double MIN_FP = (short)0x8000;
const double MAX_FP = (short)0x7fff;

short float_to_fixed(double x) {
	if( x < MIN_REAL ) {
		assert( 0 /* Underflow */ );
	}
	else if( x > MAX_REAL ) {
		assert( 0 /* Overflow */ );
	}
	else {
		return (short)(x * 16.0);
	}
}

double fixed_to_float(short x) {
	return x / 16.0;
}


short check_fixed(long x) {
	if( x < MIN_FP ) {
		assert( 0 /* underflow */ );
	}
	else if( x > MAX_FP ) {
		assert( 0 /* overflow */ );
	}
	else {
		return (short)x;
	}
}

short add_fixed(short x, short y) {
	return check_fixed( (long)x + (long)y );
}

short sub_fixed(short x, short y) {
	return check_fixed( (long)x - (long)y );
}

short mult_fixed(short x, short y) {
	long r = (long)x * (long)y;
	return check_fixed( r >> 4 );
}

short div_fixed(short x, short y) {
	long r = ((long)x << 4) / (long)y;
	return check_fixed( r );
}

#endif //__FIXED_FLOAT_CONVERT_H__