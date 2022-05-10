/*
    Copyright: Batrak Denys (baden.i.ua@gmail.com)
*/

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_M48X_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_M48X_H_

//#include <clk.h>

// UART

#define UART0_MODULE     ((1UL<<30)|(1UL<<28)         |(0x3UL<<25)       |(24UL<<20)        |(0UL<<18)         |(0xFUL<<10)       |(8UL<<5)         |(16UL<<0)) /*!< UART0 Module \hideinitializer */
#define UART1_MODULE     ((1UL<<30)|(1UL<<28)         |(0x3UL<<25)       |(26UL<<20)        |(0UL<<18)         |(0xFUL<<10)       |(12UL<<5)        |(17UL<<0)) /*!< UART1 Module \hideinitializer */
#define UART2_MODULE     ((1UL<<30)|(3UL<<28)         |(0x3UL<<25)       |(24UL<<20)        |(3UL<<18)         |(0xFUL<<10)       |(0UL<<5)         |(18UL<<0)) /*!< UART2 Module \hideinitializer */
#define UART3_MODULE     ((1UL<<30)|(3UL<<28)         |(0x3UL<<25)       |(26UL<<20)        |(3UL<<18)         |(0xFUL<<10)       |(4UL<<5)         |(19UL<<0)) /*!< UART3 Module \hideinitializer */
#define UART4_MODULE     ((1UL<<30)|(3UL<<28)         |(0x3UL<<25)       |(28UL<<20)        |(3UL<<18)         |(0xFUL<<10)       |(8UL<<5)         |(20UL<<0)) /*!< UART4 Module \hideinitializer */
#define UART5_MODULE     ((1UL<<30)|(3UL<<28)         |(0x3UL<<25)       |(30UL<<20)        |(3UL<<18)         |(0xFUL<<10)       |(12UL<<5)        |(21UL<<0)) /*!< UART5 Module \hideinitializer */
#define UART6_MODULE     ((1UL<<30)|(3UL<<28)         |(0x3UL<<25)       |(20UL<<20)        |(3UL<<18)         |(0xFUL<<10)       |(16UL<<5)        |(22UL<<0)) /*!< UART6 Module \hideinitializer */
#define UART7_MODULE     ((1UL<<30)|(3UL<<28)         |(0x3UL<<25)       |(22UL<<20)        |(3UL<<18)         |(0xFUL<<10)       |(20UL<<5)        |(23UL<<0)) /*!< UART7 Module \hideinitializer */


#define CLK_CLKSEL1_UART0SEL_Pos         (24)                                              /*!< CLK_T::CLKSEL1: UART0SEL Position      */
#define CLK_CLKSEL1_UART1SEL_Pos         (26)                                              /*!< CLK_T::CLKSEL1: UART1SEL Position      */
#define CLK_CLKSEL3_UART2SEL_Pos         (24)                                              /*!< CLK_T::CLKSEL3: UART2SEL Position      */
#define CLK_CLKSEL3_UART3SEL_Pos         (26)                                              /*!< CLK_T::CLKSEL3: UART3SEL Position      */
#define CLK_CLKSEL3_UART4SEL_Pos         (28)                                              /*!< CLK_T::CLKSEL3: UART4SEL Position      */
#define CLK_CLKSEL3_UART5SEL_Pos         (30)                                              /*!< CLK_T::CLKSEL3: UART5SEL Position      */
#define CLK_CLKSEL3_UART6SEL_Pos         (20)                                              /*!< CLK_T::CLKSEL3: UART6SEL Position      */
#define CLK_CLKSEL3_UART7SEL_Pos         (22)                                              /*!< CLK_T::CLKSEL3: UART7SEL Position      */


#define CLK_CLKSEL1_UART0SEL_HXT         (0x0UL << CLK_CLKSEL1_UART0SEL_Pos)        /*!< Select UART0 clock source from high speed crystal \hideinitializer */
#define CLK_CLKSEL1_UART0SEL_LXT         (0x2UL << CLK_CLKSEL1_UART0SEL_Pos)        /*!< Select UART0 clock source from low speed crystal \hideinitializer */
#define CLK_CLKSEL1_UART0SEL_PLL         (0x1UL << CLK_CLKSEL1_UART0SEL_Pos)        /*!< Select UART0 clock source from PLL \hideinitializer */
#define CLK_CLKSEL1_UART0SEL_HIRC        (0x3UL << CLK_CLKSEL1_UART0SEL_Pos)        /*!< Select UART0 clock source from high speed oscillator \hideinitializer */

#define CLK_CLKSEL1_UART1SEL_HXT         (0x0UL << CLK_CLKSEL1_UART1SEL_Pos)        /*!< Select UART1 clock source from high speed crystal \hideinitializer */
#define CLK_CLKSEL1_UART1SEL_LXT         (0x2UL << CLK_CLKSEL1_UART1SEL_Pos)        /*!< Select UART1 clock source from low speed crystal \hideinitializer */
#define CLK_CLKSEL1_UART1SEL_PLL         (0x1UL << CLK_CLKSEL1_UART1SEL_Pos)        /*!< Select UART1 clock source from PLL \hideinitializer */
#define CLK_CLKSEL1_UART1SEL_HIRC        (0x3UL << CLK_CLKSEL1_UART1SEL_Pos)        /*!< Select UART1 clock source from high speed oscillator \hideinitializer */

#define CLK_CLKSEL3_UART2SEL_HXT         (0x0UL << CLK_CLKSEL3_UART2SEL_Pos)        /*!< Select UART2 clock source from high speed crystal \hideinitializer */
#define CLK_CLKSEL3_UART2SEL_LXT         (0x2UL << CLK_CLKSEL3_UART2SEL_Pos)        /*!< Select UART2 clock source from low speed crystal \hideinitializer */
#define CLK_CLKSEL3_UART2SEL_PLL         (0x1UL << CLK_CLKSEL3_UART2SEL_Pos)        /*!< Select UART2 clock source from PLL \hideinitializer */
#define CLK_CLKSEL3_UART2SEL_HIRC        (0x3UL << CLK_CLKSEL3_UART2SEL_Pos)        /*!< Select UART2 clock source from high speed oscillator \hideinitializer */

#define CLK_CLKSEL3_UART3SEL_HXT         (0x0UL << CLK_CLKSEL3_UART3SEL_Pos)        /*!< Select UART3 clock source from high speed crystal \hideinitializer */
#define CLK_CLKSEL3_UART3SEL_LXT         (0x2UL << CLK_CLKSEL3_UART3SEL_Pos)        /*!< Select UART3 clock source from low speed crystal \hideinitializer */
#define CLK_CLKSEL3_UART3SEL_PLL         (0x1UL << CLK_CLKSEL3_UART3SEL_Pos)        /*!< Select UART3 clock source from PLL \hideinitializer */
#define CLK_CLKSEL3_UART3SEL_HIRC        (0x3UL << CLK_CLKSEL3_UART3SEL_Pos)        /*!< Select UART3 clock source from high speed oscillator \hideinitializer */

#define CLK_CLKSEL3_UART4SEL_HXT         (0x0UL << CLK_CLKSEL3_UART4SEL_Pos)        /*!< Select UART4 clock source from high speed crystal \hideinitializer */
#define CLK_CLKSEL3_UART4SEL_LXT         (0x2UL << CLK_CLKSEL3_UART4SEL_Pos)        /*!< Select UART4 clock source from low speed crystal \hideinitializer */
#define CLK_CLKSEL3_UART4SEL_PLL         (0x1UL << CLK_CLKSEL3_UART4SEL_Pos)        /*!< Select UART4 clock source from PLL \hideinitializer */
#define CLK_CLKSEL3_UART4SEL_HIRC        (0x3UL << CLK_CLKSEL3_UART4SEL_Pos)        /*!< Select UART4 clock source from high speed oscillator \hideinitializer */

#define CLK_CLKSEL3_UART5SEL_HXT         (0x0UL << CLK_CLKSEL3_UART5SEL_Pos)        /*!< Select UART5 clock source from high speed crystal \hideinitializer */
#define CLK_CLKSEL3_UART5SEL_LXT         (0x2UL << CLK_CLKSEL3_UART5SEL_Pos)        /*!< Select UART5 clock source from low speed crystal \hideinitializer */
#define CLK_CLKSEL3_UART5SEL_PLL         (0x1UL << CLK_CLKSEL3_UART5SEL_Pos)        /*!< Select UART5 clock source from PLL \hideinitializer */
#define CLK_CLKSEL3_UART5SEL_HIRC        (0x3UL << CLK_CLKSEL3_UART5SEL_Pos)        /*!< Select UART5 clock source from high speed oscillator \hideinitializer */

#define CLK_CLKSEL3_UART6SEL_HXT         (0x0UL << CLK_CLKSEL3_UART6SEL_Pos)        /*!< Select UART6 clock source from high speed crystal \hideinitializer */
#define CLK_CLKSEL3_UART6SEL_LXT         (0x2UL << CLK_CLKSEL3_UART6SEL_Pos)        /*!< Select UART6 clock source from low speed crystal \hideinitializer */
#define CLK_CLKSEL3_UART6SEL_PLL         (0x1UL << CLK_CLKSEL3_UART6SEL_Pos)        /*!< Select UART6 clock source from PLL \hideinitializer */
#define CLK_CLKSEL3_UART6SEL_HIRC        (0x3UL << CLK_CLKSEL3_UART6SEL_Pos)        /*!< Select UART6 clock source from high speed oscillator \hideinitializer */

#define CLK_CLKSEL3_UART7SEL_HXT         (0x0UL << CLK_CLKSEL3_UART7SEL_Pos)        /*!< Select UART7 clock source from high speed crystal \hideinitializer */
#define CLK_CLKSEL3_UART7SEL_LXT         (0x2UL << CLK_CLKSEL3_UART7SEL_Pos)        /*!< Select UART7 clock source from low speed crystal \hideinitializer */
#define CLK_CLKSEL3_UART7SEL_PLL         (0x1UL << CLK_CLKSEL3_UART7SEL_Pos)        /*!< Select UART7 clock source from PLL \hideinitializer */
#define CLK_CLKSEL3_UART7SEL_HIRC        (0x3UL << CLK_CLKSEL3_UART7SEL_Pos)        /*!< Select UART7 clock source from high speed oscillator \hideinitializer */


#define CLK_CLKDIV0_UART0DIV_Pos         (8)                                               /*!< CLK_T::CLKDIV0: UART0DIV Position      */
#define CLK_CLKDIV0_UART1DIV_Pos         (12)                                              /*!< CLK_T::CLKDIV0: UART1DIV Position      */
#define CLK_CLKDIV4_UART2DIV_Pos         (0)                                               /*!< CLK_T::CLKDIV4: UART2DIV Position      */
#define CLK_CLKDIV4_UART3DIV_Pos         (4)                                               /*!< CLK_T::CLKDIV4: UART3DIV Position      */
#define CLK_CLKDIV4_UART4DIV_Pos         (8)                                               /*!< CLK_T::CLKDIV4: UART4DIV Position      */
#define CLK_CLKDIV4_UART5DIV_Pos         (12)                                              /*!< CLK_T::CLKDIV4: UART5DIV Position      */
#define CLK_CLKDIV4_UART6DIV_Pos         (16)                                              /*!< CLK_T::CLKDIV4: UART6DIV Position      */
#define CLK_CLKDIV4_UART7DIV_Pos         (20)                                              /*!< CLK_T::CLKDIV4: UART7DIV Position      */

#define CLK_CLKDIV0_UART0(x)             (((x) - 1UL) << CLK_CLKDIV0_UART0DIV_Pos)  /*!< CLKDIV0 Setting for UART0 clock divider. It could be 1~16 \hideinitializer */
#define CLK_CLKDIV0_UART1(x)             (((x) - 1UL) << CLK_CLKDIV0_UART1DIV_Pos)  /*!< CLKDIV0 Setting for UART1 clock divider. It could be 1~16 \hideinitializer */
#define CLK_CLKDIV4_UART2(x)             (((x) - 1UL) << CLK_CLKDIV4_UART2DIV_Pos)  /*!< CLKDIV4 Setting for UART2 clock divider. It could be 1~16 \hideinitializer */
#define CLK_CLKDIV4_UART3(x)             (((x) - 1UL) << CLK_CLKDIV4_UART3DIV_Pos)  /*!< CLKDIV4 Setting for UART3 clock divider. It could be 1~16 \hideinitializer */
#define CLK_CLKDIV4_UART4(x)             (((x) - 1UL) << CLK_CLKDIV4_UART4DIV_Pos)  /*!< CLKDIV4 Setting for UART4 clock divider. It could be 1~16 \hideinitializer */
#define CLK_CLKDIV4_UART5(x)             (((x) - 1UL) << CLK_CLKDIV4_UART5DIV_Pos)  /*!< CLKDIV4 Setting for UART5 clock divider. It could be 1~16 \hideinitializer */
#define CLK_CLKDIV4_UART6(x)             (((x) - 1UL) << CLK_CLKDIV4_UART6DIV_Pos)  /*!< CLKDIV4 Setting for UART6 clock divider. It could be 1~16 */
#define CLK_CLKDIV4_UART7(x)             (((x) - 1UL) << CLK_CLKDIV4_UART7DIV_Pos)  /*!< CLKDIV4 Setting for UART7 clock divider. It could be 1~16 */

#endif
