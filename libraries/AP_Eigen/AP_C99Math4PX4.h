/*
 * A tribute to the PX4 users
 * @authors: Daniel Frenzel <dgdanielf@gmail.com>
 */

#ifndef PX4FIX
#define PX4FIX

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 && CONFIG_HAL_BOARD != HAL_BOARD_LINUX
    #pragma GCC "-std=gnu++98"                         // the none-eabi is not working with cxx-11 (important for PX4 only)
    //#pragma GCC diagnostic ignored "-Wunknown-pragmas" // No OpenMP on PX4, no need to complain about ignored #pragmas
    
    #define RAND_MAX __RAND_MAX                        // Not defined on none-eabi
    #define _GLIBCXX_USE_C99_FP_MACROS_DYNAMIC 1       // Not defined on none-eabi
    
    #pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wfloat-equal"
	#undef isnanf
	static inline bool isnanf(const float &val) {
	    volatile float tmp = val;
	    return tmp != tmp ? true : false;
	}
    #pragma GCC diagnostic pop

    #include <cmath>
    namespace std {
        using ::lgamma;

        template<typename A, typename B>
        static inline auto max(const A &one, const B &two) -> decltype(one > two ? one : two) {              
            return one > two ? one : two;
        }

        template<typename T>
        static inline T abs(const T &val) {
            return val > 0 ? val : (-1*val);
        }
    };
#endif 

#endif //PX4FIX