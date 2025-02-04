#ifndef __IEEE754_H__
#define __IEEE754_H__

#include <cmath>
#include <limits>
#include <algorithm>
#include <type_traits>
#include <iostream>
#include "std_overrides.hpp"
#include "smallest_unsigned.hpp"

template<unsigned M, unsigned E, unsigned TF=0, int B = (1 << (E - 1)) - 1 >
class IEEE754 {
    // XXX: This actually needs to be private, however I do not know how to make
    // The functions at std:: overrides able to access it without anything else does
    // (friending everything is very tedious and you cannot friend a namespace)
    public:
        enum {
            MANTISSA_MASK = (1UL << M) - 1,
            EXPONENT_MASK = (1UL << E) - 1,

            MIN_EXPONENT = -B + 2,
            MAX_EXPONENT =  B + 1,

            BITS = 1 + E + M,
        };

        typedef typename smallest_unsigned<BITS >::type primitive;
        typedef typename std::make_signed<primitive >::type signed_primitive;

        union {
            struct {
                primitive mantissa : M;
                primitive exponent : E;
                primitive sign : 1;
            } comp;
            primitive data;
        };

        struct fixed {
            int32_t int_;
            uint32_t frac_;
        };


        /**
         * Build a float from components
         */
        inline static IEEE754 from_components(primitive sign, primitive exponent, primitive mantissa) {
            IEEE754 result;
            result.comp.sign = sign;
            result.comp.exponent    = exponent;
            result.comp.mantissa    = mantissa;
            return result;
        }

        inline static IEEE754 from_mant_to_round(primitive sign, primitive exponent, uint64_t mant, uint32_t leadingone_bit = 53) {
            uint32_t lsb = leadingone_bit - M + 1;
            IEEE754 result;
            result.comp.sign = sign;
            result.comp.exponent    = exponent;
            result.comp.mantissa    = bit_range(mant, leadingone_bit, lsb);

            uint32_t rounding = (mant >> (lsb - 1)) & 0x1;
            result.roundup_float(rounding);
            return result;
        }

        inline static IEEE754 from_data(primitive data) {
            IEEE754 result;
            result.data = data;
            return result;
        }

        /**
         * extract range of bits
         */
        static uint64_t bit_range(uint64_t data, int32_t msb, int32_t lsb) {
            assert(msb >= lsb);
            // msb, lsb range [63,0]
            if (lsb > 63) return 0;
            msb = msb > 63 ? 63 : msb;
            lsb = lsb < 0 ?  0 : lsb;
            lsb = lsb > msb ? msb : lsb;

            data = data >> lsb;
            data &= (1 << (msb - lsb + 1)) -1 ;
            return data;
        }

    private:
        /**
         * Renormalizes a signed fixed point into a float
         */
        template<typename T >
        inline static IEEE754 renormalize(T unnormalized, int radix_point) {
            IEEE754 result;
            result.from_signed(unnormalized, radix_point);
            return result;
        }

        /**
         * Renormalizes an unsigned fixed point into a float
         */
        template<typename T >
        inline static IEEE754 renormalize(T unnormalized, int radix_point, int sign) {
            IEEE754 result;
            result.comp.sign = sign;
            result.from_unsigned(unnormalized, radix_point);
            return result;
        }

        /**
         * Returns nan with optional sign and mantissa
         */
        inline static IEEE754 nan(primitive sign = 0, primitive mantissa = 1) {
            return from_components(sign, EXPONENT_MASK, mantissa);
        }

        /**
         * Returns Infinity
         */
        inline static IEEE754 inf(primitive sign = 0) {
            return from_components(sign, EXPONENT_MASK, 0);
        }

        /**
         * Shift that allows negative values
         */
        template <
            typename T,
            typename RT = typename std::conditional<(sizeof(T) > sizeof(primitive)), T, primitive >::type
        >
        static RT shift(T value, int shift_amount, int shift_left_max = 0, int shift_right_max = 0) {
            if(shift_amount < 0) {
                if (shift_right_max != 0 && (-shift_amount >= shift_right_max)) return 0;
                return value >> -shift_amount;
            }
            if(shift_amount > 0) {
                if (shift_left_max != 0 && (shift_amount >= shift_left_max)) return 0;
                int len = sizeof(T) * 8;
                while (shift_amount-- > 0) {
                    if ((value & (1 << (len  - 1))) != 0) {
                        value = -1;
                        break;
                    }
                    value = value << 1;
                }
                return value;
                //return value << shift_amount;
            }

            return value;
        }

        /**
         * Computes the real value of the mantissa.
         * This adds the implicit 1.xxxx to the mantissa when needed
         */
        primitive real_mantissa() const {
            if (comp.exponent) {
                auto tf_real = (comp.mantissa >> TF) << TF;
                auto tmp = tf_real | (1 << M);
                return tmp;
            } else {
                auto tf_real = (comp.mantissa >> TF) << TF;
                auto tmp = (tf_real << 1);
                return tmp;
            }
            //return comp.exponent ? comp.mantissa | (1 << M) : (comp.mantissa << 1);
        }

        primitive real_exp() const {
            return comp.exponent - B;
        }

        /**
         * Fills up exponent and mantissa from an unsigned value.
         * Sign is left unchanged.
         */
        template <typename T >
        void from_unsigned(T unsigned_value, int radix_point = 0) {
            if(unsigned_value > shift<T >((1 << (M + 1)) - 1, E - radix_point)) {
                comp.exponent = EXPONENT_MASK;
                comp.mantissa = 0;
            } else {
                int log2 = std::log2(unsigned_value);

                if(radix_point + log2 + 1 < MIN_EXPONENT) {
                    comp.exponent = 0;
                    comp.mantissa = shift(unsigned_value, M - (MIN_EXPONENT - radix_point - 1));
                } else {
                    comp.exponent = unsigned_value ? log2 + radix_point + B : 0;
                    comp.mantissa = shift(unsigned_value, M - log2);
                }
            }
        }

        /**
         * Fills up sign, exponent and mantissa from an unsigned value.
         */
        template <typename T >
        void from_signed(T signed_value, int radix_point = 0) {
            typedef typename std::make_signed<T >::type signed_T;
            typedef typename std::make_unsigned<T >::type unsigned_T;

            signed_T forced_signed = signed_value;

            comp.sign = (forced_signed < 0);
            from_unsigned<unsigned_T >(std::abs(forced_signed), radix_point);
        }


        /**
         * extract range of bits
         */
        primitive roundup_fix(primitive mant, int32_t width, uint32_t round, bool &carry_in) {
            assert((mant & (~((1 << (width + 1)) - 1))) == 0);

            uint64_t mant_real = mant | 1UL << width;
            carry_in = false;
            if (round == 1) {
                mant_real = mant_real + 1;
                if (mant_real & 1UL << (width + 1)) {
                    carry_in = true;
                    mant_real = mant_real >> 1;
                }
            }
            return bit_range(mant_real, width - 1, 0);
        }
        /**
         * extract range of bits
         */
        void roundup_float(uint32_t round) {
            bool carry_in = false;
            comp.mantissa = roundup_fix(comp.mantissa, M, round, carry_in);
            comp.exponent = carry_in? comp.exponent + 1 : comp.exponent;
        }

        /**
         * Retrieve the value of this float as an unsigned value
         */
        uint32_t to_fixed_int(int radix_point = 0) const {
            uint64_t mant = real_mantissa();
            int exp = comp.exponent - radix_point - M;

            uint32_t f_int = 0;
            if (exp >= 0 && exp < 64) {
                exp -= B;
                mant = shift<uint64_t>(mant, exp);
                f_int = bit_range(mant, 31, 0);
            }
            return f_int;
        }

        fixed to_fixed(bool inverse, int radix_point = 0) const {
            uint64_t mant = real_mantissa();
            int exp = real_exp();
            uint64_t frac_bits = 0;
            fixed result;
            return result;

            int32_t f_int = to_fixed_int(radix_point);

            if (exp >= 0 ) {
                frac_bits = exp >= 64 ? 0 : mant << exp;
                if (inverse) frac_bits = ~frac_bits + 1;
            } else{
                uint32_t rem = bit_range(mant, (-exp) - 1, 0);
                uint32_t rounding = bit_range(rem, (-exp) - 1, (-exp) - 1);
                mant = (-exp) >= 64 ? 0 : mant >> (-exp);

                if (inverse) {
                    rounding = ~rounding & 0x1;
                    mant = bit_range(~mant, M - 1, 0);
                }

                bool carry_in = false;
                // do rounding half up
                frac_bits = roundup_fix(mant, M, rounding, carry_in);

                if (carry_in) {
                    if (inverse) {
                        if (comp.mantissa != 0) f_int--;
                    } else {
                        f_int++;
                    }
                }
            }
            result.int_ = f_int;
            result.frac_ = bit_range(frac_bits, M - 1, 0);

            return result;
        }

        /**
         * Retrieve the value of this float as an unsigned value
         */
        template <typename T >
        T to_unsigned(int radix_point = 0) const {
            int exp = comp.exponent - radix_point - M;
            exp -= B;
            //return shift<T >(real_mantissa(), comp.exponent - radix_point - B - M);
            return shift<T >(real_mantissa(), exp);
        }

        /**
         * Retrieve the value of this float as a signed value
         */
        template <typename T >
        T to_signed(int radix_point = 0) const {
            auto tmp = to_unsigned<T >(radix_point);
            auto tmp1 = tmp ^ - comp.sign;
            auto tmp2 = tmp1 + comp.sign;
            return tmp2;
            //return (to_unsigned<T >(radix_point) ^ - comp.sign) + comp.sign;
        }

    public:
        // -------------------------- Constructors -------------------------- //

        /**
         * Default constructor, undefined value.
         */
        IEEE754() = default;

        /**
         * Default copy constructor
         */
        IEEE754(const IEEE754 &) = default;

        /**
         * Conversion from another IEEE floating point object
         */
        template <unsigned OM, unsigned OE, unsigned OTF, int OB >
        IEEE754(const IEEE754<OM, OE, OB > &other_ieee754) {
            comp.sign = other_ieee754.sign;
            comp.exponent = other_ieee754.comp.exponent ? other_ieee754.comp.exponent - OB + B : 0;
            comp.mantissa = shift(other_ieee754.comp.mantissa, M - OM);
        }

        /**
         * Conversion from another IEEE floating point object
         */
        template <unsigned OM, unsigned OE, unsigned OTF, int OB >
        IEEE754(IEEE754<OM, OE, OB > &&other_ieee754) {
            comp.sign = other_ieee754.sign;
            comp.exponent = other_ieee754.comp.exponent ? other_ieee754.comp.exponent - OB + B : 0;
            comp.mantissa = shift(other_ieee754.comp.mantissa, M - OM);
        }

        /**
         * Conversion from a floating point value
         */
        template <
            typename T,
            typename = typename std::enable_if<std::is_floating_point<T >::value, T >::type
        >
        IEEE754(T floating_point) {
            comp.sign = std::signbit(floating_point);

            if(std::isnormal(floating_point)) {
                int exp = 0;
                primitive man = std::ldexp(std::frexp(floating_point, &exp), M + 1);

                if(exp > MAX_EXPONENT) {
                    comp.exponent = EXPONENT_MASK;
                    comp.mantissa = 0;
                } else if(exp < MIN_EXPONENT) {
                    comp.exponent = 0;
                    comp.mantissa = shift(man, exp - MIN_EXPONENT + 1);
                } else {
                    comp.exponent = exp + B - 1;
                    comp.mantissa = man;
                }
            } else {
                comp.exponent = floating_point == T() ? 0 : EXPONENT_MASK;
                comp.mantissa = std::isnan(floating_point);
            }
        }

        /**
         * Conversion from a signed integral type
         */
        template <
            typename T,
            typename = typename std::enable_if<!std::is_floating_point<T >::value, T >::type,
            typename = typename std::enable_if< std::is_signed<T >::value, T >::type
        >
        IEEE754(T signed_integral) {
            from_signed(signed_integral);
        }

        /**
         * Conversion from an unsigned integral type
         */
        template <
            typename T,
            typename = typename std::enable_if<!std::is_floating_point<T >::value, T >::type,
            typename = typename std::enable_if<!std::is_signed<T >::value, T >::type,
            typename = typename std::enable_if< std::is_unsigned<T >::value, T >::type
        >
        IEEE754(T unsigned_integral) {
            comp.sign = 0;
            from_unsigned(unsigned_integral);
        }

        // ------------------------- Cast operators ------------------------- //

        /**
         * Convert to another floating point value
         */
        template <
            typename T,
            typename = typename std::enable_if<std::is_floating_point<T >::value, T >::type
        >
        explicit operator T() const {
            T result;
            if(comp.exponent != EXPONENT_MASK) {
                result = std::ldexp(real_mantissa() / T(1 << M), comp.exponent - B);
            } else {
                if(comp.mantissa)
                    result = std::numeric_limits<T >::quiet_NaN();
                else
                    result = std::numeric_limits<T >::infinity();
            }

            return std::copysign(result, -comp.sign);
        }

        /**
         * Convert to a signed integer
         */
        template <
            typename T,
            typename = typename std::enable_if<!std::is_floating_point<T >::value, T >::type,
            typename = typename std::enable_if< std::is_signed<T >::value, T >::type
        >
        explicit operator T() const {
            return to_signed<T >();
        }

        /**
         * Convert to an unsigned integer
         */
        template <
            typename T,
            typename = typename std::enable_if<!std::is_floating_point<T >::value, T >::type,
            typename = typename std::enable_if<!std::is_signed<T >::value, T >::type,
            typename = typename std::enable_if< std::is_unsigned<T >::value, T >::type
        >
        explicit operator T() const {
            return to_unsigned<T >();
        }

        // --------------------------- Arithmetic --------------------------- //

        // Unary

        friend IEEE754 operator + (const IEEE754 &value) {
            return value;
        }

        friend IEEE754 operator - (const IEEE754 &value) {
            return from_components(
                value.comp.sign ^ 1,
                value.comp.exponent, value.comp.mantissa);
        }

        // Binary

        friend std::ostream& operator<<(std::ostream& os, const IEEE754& obj)
        {
            // write obj to stream
            float result = std::ldexp(obj.real_mantissa() / float(1 << 4), obj.comp.exponent - 3);
            std::cout << result;
            return os;
        }

        friend IEEE754 operator + (const IEEE754 &lhs, const IEEE754 &rhs) {
            if(std::isunordered(lhs, rhs))
                return std::numeric_limits<IEEE754>::quiet_NaN();
            if(std::isinf(lhs) || std::isinf(rhs)) {
                if(std::isinf(lhs) && !std::isinf(rhs))
                    return lhs;
                if(std::isinf(rhs) && !std::isinf(lhs))
                    return rhs;
                if(rhs == lhs)
                    return rhs;

                return std::numeric_limits<IEEE754>::quiet_NaN();
            }
            if(std::iszero(lhs) || std::iszero(rhs)) {
                if(!(std::iszero(rhs)))
                    return rhs;
                if(!(std::iszero(lhs)))
                    return lhs;
                return std::numeric_limits<IEEE754>::zero();
            }

            int exp = std::min(lhs.comp.exponent - B, rhs.comp.exponent - B) - M;

            auto lhs_signed = lhs.to_signed<signed_primitive >(exp);
            auto rhs_signed = rhs.to_signed<signed_primitive >(exp);
            auto sum = lhs_signed + rhs_signed;
            return renormalize(sum, exp);
            // return renormalize(
            //    lhs.to_signed<signed_primitive >(exp) +
            //    rhs.to_signed<signed_primitive >(exp), exp);
        }

        friend IEEE754 operator - (const IEEE754 &lhs, const IEEE754 &rhs) {
            if(std::isunordered(lhs, rhs))
                return std::numeric_limits<IEEE754>::quiet_NaN();
            if(std::isinf(lhs) || std::isinf(rhs)) {
                if(std::isinf(lhs) && !std::isinf(rhs))
                    return lhs;
                if(std::isinf(rhs) && !std::isinf(lhs))
                    return -rhs;
                if(rhs != lhs)
                    return lhs;

                return std::numeric_limits<IEEE754>::quiet_NaN();
            }

            int exp = std::min(lhs.comp.exponent - B, rhs.comp.exponent - B) - M;
            return renormalize(
                lhs.to_signed<signed_primitive >(exp) -
                rhs.to_signed<signed_primitive >(exp), exp);
        }

        friend IEEE754 operator * (const IEEE754 &lhs, const IEEE754 &rhs) {
            if(std::isunordered(lhs, rhs))
                return std::numeric_limits<IEEE754>::quiet_NaN();

            // FIXME on sign
            if(std::iszero(lhs) || std::iszero(rhs))
                return std::numeric_limits<IEEE754>::zero();

            // FIXME on sign
            if(std::isinf(lhs) || std::isinf(rhs))
                return std::numeric_limits<IEEE754>::infinity();

            int exponent = (lhs.comp.exponent - B) + (rhs.comp.exponent - B);
            primitive lhs_value = lhs.real_mantissa();
            primitive rhs_value = rhs.real_mantissa();
            uint64_t mul = (uint64_t)lhs_value * (uint64_t)rhs_value;
            mul = mul >> M;
            exponent -= M;

            return renormalize( mul,
                 exponent, lhs.comp.sign ^ rhs.comp.sign);
            /*
            return renormalize(
                (lhs.real_mantissa() *
                 rhs.real_mantissa()) >> M, exponent - M, lhs.comp.sign ^ rhs.comp.sign);
                 */
        }

        friend IEEE754 operator / (const IEEE754 &lhs, const IEEE754 &rhs) {
            if(std::isunordered(lhs, rhs))
                return std::numeric_limits<IEEE754>::quiet_NaN();

            if(rhs == 0)
                return inf(lhs.comp.sign ^ rhs.comp.sign);

            int exponent = (lhs.comp.exponent - B) - (rhs.comp.exponent - B);
            primitive lhs_value = lhs.real_mantissa();
            primitive rhs_value = rhs.real_mantissa();
            uint64_t div_result = ((uint64_t)lhs_value << M) / (uint64_t)rhs_value;
            return renormalize(
                 div_result, exponent - M, lhs.comp.sign ^ rhs.comp.sign);
        }

        // Placement

        IEEE754& operator += (const IEEE754 &value) {
            return *this = *this + value;
        }

        IEEE754& operator -= (const IEEE754 &value) {
            return *this = *this - value;
        }

        IEEE754& operator *= (const IEEE754 &value) {
            return *this = *this * value;
        }

        IEEE754& operator /= (const IEEE754 &value) {
            return *this = *this / value;
        }

        // --------------------------- Comparison --------------------------- //

        // Equality

        friend bool operator == (const IEEE754 &lhs, const IEEE754 &rhs) {
            if(std::isunordered(lhs, rhs))
                return false;

            if(lhs.comp.exponent == 0 && lhs.comp.mantissa == 0
            && rhs.comp.exponent == 0 && rhs.comp.mantissa == 0)
                return true;

            return (primitive&)lhs == (primitive&)rhs;
        }

        friend bool operator != (const IEEE754 &lhs, const IEEE754 &rhs) {
            if(std::isunordered(lhs, rhs))
                return true;

            if(lhs.comp.exponent == 0 && lhs.comp.mantissa == 0
            && rhs.comp.exponent == 0 && rhs.comp.mantissa == 0)
                return false;

            return (primitive&)lhs != (primitive&)rhs;
        }

        // Relative comparison

        friend bool operator < (const IEEE754 &lhs, const IEEE754 &rhs) {
            if(std::isunordered(lhs, rhs))
                return false;

            if(lhs.comp.exponent - B < rhs.comp.exponent - B)
                return !rhs.comp.sign;

            if(lhs.comp.mantissa < rhs.comp.mantissa)
                return !rhs.comp.sign;

            return lhs.comp.sign && !rhs.comp.sign;
        }

        friend bool operator > (const IEEE754 &lhs, const IEEE754 &rhs) {
            if(std::isunordered(lhs, rhs))
                return false;

            if(lhs.comp.exponent - B > rhs.comp.exponent - B)
                return !lhs.sign;

            if(lhs.comp.mantissa > rhs.comp.mantissa)
                return !lhs.comp.sign;

            return !lhs.comp.sign && rhs.comp.sign;
        }

        friend bool operator <= (const IEEE754 &lhs, const IEEE754 &rhs) {
            return (lhs < rhs)
                || (rhs == lhs);
        }

        friend bool operator >= (const IEEE754 &lhs, const IEEE754 &rhs) {
            return (lhs > rhs)
                || (rhs == lhs);
        }
};

#endif
