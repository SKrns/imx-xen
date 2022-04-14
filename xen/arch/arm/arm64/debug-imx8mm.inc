#include <asm/imx8-uart.h>

.macro early_uart_init wb wc wd
/* Already initialized in bootloader */
.endm
/* i.MX8QM wait LPUART to be ready to transmit
 * rb: register which contains the UART base address
 * rc: scratch register
 */
.macro early_uart_ready xb, c
1:
        ldr   w\c, [\xb, #UTS]   /* <- Flag register */
        tst   w\c, #UTS_TXFULL     /* Check FIFO EMPTY bit */
        beq   1b                      /* Wait for the UART to be ready */
.endm

/* i.MX8QM LPUART transmit character
 * rb: register which contains the UART base address
 * rt: register which contains the character to transmit */
.macro early_uart_transmit xb, wt
        str   \wt, [\xb, #UTXD]  /* -> Data Register */
.endm

