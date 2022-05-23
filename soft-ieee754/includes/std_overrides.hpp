#ifndef __IEEE754_STD_OVERRIDES_H__
#define __IEEE754_STD_OVERRIDES_H__

template<unsigned M, unsigned E, unsigned TF, int B >
class IEEE754;

namespace std {
	/**
	 * The IEEE754 class with any template parameter is a floating point type.
	 */
	template<unsigned M, unsigned E, unsigned TF, int B >
	struct is_floating_point<IEEE754<M, E, TF, B > > : public std::true_type { };

	/**
	 * Specialization of std::numeric_limits for any bit format provided.
	 */
	template <unsigned M, unsigned E, unsigned TF, int B >
	struct numeric_limits<IEEE754<M, E, TF, B > > {
		public:
			static constexpr bool is_specialized = true;

			static IEEE754<M, E, TF, B > min()			{ return from_components(+1,  0); }
			static IEEE754<M, E, TF, B > max()			{ return from_components(-2, -1); }

			static constexpr int digits = M + 1;
			static constexpr int digits10 = M * M_LOG10E / M_LOG2E;
			static constexpr bool is_signed = true;
			static constexpr bool is_integer = false;
			static constexpr bool is_exact = false;
			static constexpr int radix = 2;

			static IEEE754<M, E, TF, B > epsilon()		{ return from_components(B - M, 0); }
			static IEEE754<M, E, TF, B > round_error()	{ return 0.5f; }

			static constexpr int min_exponent = -B + 2;
			static constexpr int max_exponent =  B + 1;
			static constexpr int min_exponent10 = min_exponent * M_LOG10E / M_LOG2E;
			static constexpr int max_exponent10 = max_exponent * M_LOG10E / M_LOG2E;

			static constexpr bool has_infinity = true;
			static constexpr bool has_quiet_NaN = true;
			static constexpr bool has_signaling_NaN = false;
			static constexpr float_denorm_style has_denorm = denorm_present;
			static constexpr bool has_denorm_loss = false;

			static IEEE754<M, E, TF, B > infinity()		{ return from_components(-1,  0); }
			static IEEE754<M, E, TF, B > quiet_NaN()	{ return from_components(-1, +1); }
			static IEEE754<M, E, TF, B > signaling_NaN(){ return from_components(-1, +1); }
			static IEEE754<M, E, TF, B > denorm_min()	{ return from_components( 0, +1); }
			static IEEE754<M, E, TF, B > zero()	{ return from_components( 0, 0); }

			static constexpr bool is_iec559 = has_infinity && has_quiet_NaN && has_denorm == denorm_present;
			static constexpr bool is_bounded = true;
			static constexpr bool is_modulo = false;

			static constexpr bool traps = false;
			static constexpr bool tinyness_before = false;
			static constexpr float_round_style round_style = round_to_nearest;

		private:
			typedef typename IEEE754<M, E, TF, B >::primitive primitive;

			inline static IEEE754<M, E, TF, B > from_components(primitive exponent, primitive mantissa) {
				IEEE754<M, E, TF, B > result;
				result.comp.mantissa	= mantissa;
				result.comp.exponent	= exponent;
				result.comp.sign = 0;
				return result;
			}
	};

	// --------------------------- Classification --------------------------- //

	/*
	 * Categorizes the given floating point value
	 */
	template <unsigned M, unsigned E, unsigned TF, int B >
	int fpclassify(const IEEE754<M, E, TF, B > &arg) {
		// Zero exponent can be either zero or a subnormal (denormal)
		if(arg.comp.exponent == 0) {
			if(arg.comp.mantissa == 0)
				return FP_ZERO;

			return FP_SUBNORMAL;
		}

		// Exponent of mask can be NaN or Infinite
		if(arg.comp.exponent == IEEE754<M, E, TF, B >::EXPONENT_MASK) {
			if(arg.comp.mantissa == 0)
				return FP_INFINITE;

			return FP_NAN;
		}

		// Any other value is considered a normal value
		return FP_NORMAL;
	}

	/*
	 * Checks if the given number is zero
	 */
	template <unsigned M, unsigned E, unsigned TF, int B >
	bool iszero(const IEEE754<M, E, TF, B > &arg) {
		return (arg.comp.exponent == 0) && (arg.comp.mantissa == 0);
	}

	/*
	 * Checks if the given number has finite value
	 */
	template <unsigned M, unsigned E, unsigned TF, int B >
	bool isfinite(const IEEE754<M, E, TF, B > &arg) {
		return (arg.comp.exponent != IEEE754<M, E, TF, B >::EXPONENT_MASK);
	}

	/*
	 * Checks if the given number is infinite
	 */
	template <unsigned M, unsigned E, unsigned TF, int B >
	bool isinf(const IEEE754<M, E, TF, B > &arg) {
		return (arg.comp.exponent == IEEE754<M, E, TF, B >::EXPONENT_MASK)
			&& arg.comp.mantissa == 0;
	}

	/**
	 * Checks if the given number is NaN
	 */
	template <unsigned M, unsigned E, unsigned TF, int B >
	bool isnan(const IEEE754<M, E, TF, B > &arg) {
		return (arg.comp.exponent == IEEE754<M, E, TF, B >::EXPONENT_MASK)
			&& arg.comp.mantissa != 0;
	}

	/*
	 * Checks if the given number is normal
	 */
	template <unsigned M, unsigned E, unsigned TF, int B >
	bool isnormal(const IEEE754<M, E, TF, B > &arg) {
		return (arg.comp.exponent != 0)
			&& (arg.comp.exponent != IEEE754<M, E, TF, B >::EXPONENT_MASK);
	}

	/**
	 * Checks if two floating-point values are unordered
	 */
	template <unsigned M, unsigned E, unsigned TF, int B >
	bool isunordered(const IEEE754<M, E, TF, B > &arg1, const IEEE754<M, E, TF, B > &arg2) {
		return isnan(arg1)
			|| isnan(arg2);
	}

	// ------------------------- Sign manipulation -------------------------- //

	/*
	 * Checks if the given number is negative
	 */
	template <unsigned M, unsigned E, unsigned TF, int B >
	bool signbit(const IEEE754<M, E, TF, B > &arg) {
		return arg.comp.sign != 0;
	}

	/**
	 * Computes the absolute value of a floating point value
	 */
	template <unsigned M, unsigned E, unsigned TF, int B >
	IEEE754<M, E, TF, B > abs(const IEEE754<M, E, TF, B > &arg) {
		IEEE754<M, E, TF, B > result = arg;
		result.comp.sign = 0;
		return result;
	}

	/**
	 * Composes a floating point value with the magnitude of x and the sign of y
	 */
	template <unsigned M, unsigned E, unsigned TF, int B >
	IEEE754<M, E, TF, B > copysign(const IEEE754<M, E, TF, B > &x, const IEEE754<M, E, TF, B > &y) {
		IEEE754<M, E, TF, B > result = x;
		result.comp.sign = y.comp.sign;
		return result;
	}

	// ----------------------------- Components ----------------------------- //

	/**
	 *  Returns the resulting floating point value from multiplying
	 *  the significand by 2 raised to the power of the exponent.
	 */
	template <unsigned M, unsigned E, unsigned TF, int B >
	IEEE754<M, E, TF, B > ldexp(const IEEE754<M, E, TF, B > &x, int exp) {
		// XXX: Conversion of double to IEEE754 instance
		return x * std::pow(2.0, exp);
	}

	/**
	 * Breaks the floating point number x into its binary significand
	 * and an integral exponent for 2
	 */
	template <unsigned M, unsigned E, unsigned TF, int B >
	IEEE754<M, E, TF, B > frexp(const IEEE754<M, E, TF, B > &x, int* exp) {
		IEEE754<M, E, TF, B > result = x;

		switch(std::fpclassify(x)) {
			case FP_NAN:
			case FP_ZERO:
			case FP_INFINITE:
					*exp = 0;
				break;
			case FP_SUBNORMAL: {
					int log2 = std::log2(x.comp.mantissa);

					*exp = log2 - B - 2;
					result.comp.exponent = B - 1;
					result.comp.mantissa = x.comp.mantissa << (M - log2);
				} break;
			case FP_NORMAL:
					*exp = x.comp.exponent - B + 1;
					result.comp.exponent = B - 1;
				break;
		}

		return result;
	}
}

#endif
