#include <xen/config.h>
#include <xen/console.h>
#include <xen/errno.h>
#include <xen/serial.h>
#include <xen/init.h>
#include <xen/irq.h>
#include <xen/mm.h>
#include <asm/device.h>
#include <asm/imx8-uart.h>
#include <asm/io.h>

// #define UART2_BASE_ADDR		0x30890000
// #define CONFIG_MXC_UART_BASE		UART2_BASE_ADDR

#define imx8_uart_read(uart, off)       readl((uart)->regs + off)
#define imx8_uart_write(uart, off, val) writel((val), (uart)->regs + off)

static struct imx8_uart {
    unsigned int baud, clock_hz, data_bits, parity, stop_bits, fifo_size;
    unsigned int irq;
    char __iomem *regs;
    struct irqaction irqaction;
    struct vuart_info vuart;
} imx8_com = {0};

static void imx8_uart_interrupt(int irq, void *data,
                                  struct cpu_user_regs *regs)
{
    struct serial_port *port = data;
    struct imx8_uart *uart = port->uart;
    unsigned int uts;

    uts = imx8_uart_read(uart, UTS);
    if ((uts & UTS_RXEMPTY))
        serial_rx_interrupt(port, regs);

    if ((uts & UTS_TXEMPTY))
        serial_tx_interrupt(port, regs);
}


static void __init imx8_uart_init_preirq(struct serial_port *port)
{
    struct imx8_uart *uart = port->uart;

    imx8_uart_write(uart, UCR1, 0);
    imx8_uart_write(uart, UCR2, 0);

	while (!(imx8_uart_write(uart, UCR2) & UCR2_SRST));

    imx8_uart_write(uart, 0x704 | UCR3_ADNIMP, UCR3);
	imx8_uart_write(uart, 0x8000, UCR4);
	imx8_uart_write(uart, 0x2b, UESC);
	imx8_uart_write(uart, 0, UTIM);

	imx8_uart_write(uart, 0, UTS);

    // Setbrg
    baudrate = 115200;
    clk = 100000000;

    unsigned int tmp; /* u32 */

	tmp = RFDIV << UFCR_RFDIV_SHF;

	tmp |= (TXTL << UFCR_TXTL_SHF) | (RXTL << UFCR_RXTL_SHF);
	writel(tmp, UFCR);

	writel(0xf, UBIR);
	writel(clk / (2 * baudrate), UBMR);
    // 3124 = clk / 230400
    // clk = 719769600

	writel(UCR2_WS | UCR2_IRTS | UCR2_RXEN | UCR2_TXEN | UCR2_SRST,
	       UCR2);
	writel(UCR1_UARTEN, UCR1);
}

static void __init imx8_uart_init_postirq(struct serial_port *port)
{
    struct imx8_uart *uart = port->uart;
    unsigned int temp;

    uart->irqaction.handler = imx8_uart_interrupt;
    uart->irqaction.name = "imx8_uart";
    uart->irqaction.dev_id = port;

    if ( setup_irq(uart->irq, 0, &uart->irqaction) != 0 )
    {
        dprintk(XENLOG_ERR, "%s\n", __func__, uart->irq);
        return;
    }
    
    /* Enable interrupt */
    dprintk(XENLOG_ERR, "%s\n", __func__);
    /* Need to add */
}

static void imx8_uart_suspend(struct serial_port *port)
{
    BUG();
}

static void imx8_uart_resume(struct serial_port *port)
{
    BUG();
}

static int imx8_lpuart_tx_ready(struct serial_port *port)
{
    struct imx8_uart *uart = port->uart;

    return (imx8_uart_read(uart, USR2) & USR2_TXDC) ? 1 : 0;
}

static void imx8_lpuart_putc(struct serial_port *port, char c)
{
    struct imx8_uart *uart = port->uart;

    /* If \n, also do \r */
	if (c == '\n')
		serial_putc('\r');

    imx8_uart_write(uart, TXD, c);

	/* wait for transmitter to be ready */
	while (!(imx8_uart_read(uart, UTS & UTS_TXEMPTY))
		barrier();

}

static int imx8_lpuart_getc(struct serial_port *port, char *pc)
{
    struct imx8_uart *uart = port->uart;
    int ch;

    while ( !(imx8_uart_read(uart, UTS) & UTS_RXEMPTY))
        barrier();

    ch = imx8_uart_read(uart, RXD);
    *pc = ch & URXD_RX_DATA;

    return 1;
}

static int __init imx8_lpuart_irq(struct serial_port *port)
{
    struct imx8_lpuart *uart = port->uart;

    return ((uart->irq >0) ? uart->irq : -1);
}

static const struct vuart_info *imx8_lpuart_vuart_info(struct serial_port *port)
{
    struct imx8_lpuart *uart = port->uart;

    return &uart->vuart;
}

static void imx8_lpuart_start_tx(struct serial_port *port)
{
    struct imx8_lpuart *uart = port->uart;
    unsigned int temp;

    temp = imx8_uart_read(uart, UTS);
    /* Wait until empty */
    while (!(temp & UTS_TXEMPTY))
	    barrier();

    temp = imx8_uart_read(uart, UCR1);
    imx8_uart_write(uart, UCR1, (temp | UCR1_TRDYEN));

    return;
}

static void imx8_lpuart_stop_tx(struct serial_port *port)
{
    struct imx8_uart *uart = port->uart;
    unsigned int temp;

    temp = imx8_uart_read(uart, UCR1);
    temp &= ~(UCR1_TRDYEN | UCR1_TXMPTYEN);
    imx8_uart_write(uart, UCR1, temp);

    return;
}

static struct uart_driver __read_mostly imx8_uart_driver = {
    .init_preirq = imx8_uart_init_preirq,
    .init_postirq = imx8_uart_init_postirq,
    .endboot = NULL,
    .suspend = imx8_uart_suspend,
    .resume = imx8_uart_resume,
    .tx_ready = imx8_lpuart_tx_ready,
    .putc = imx8_lpuart_putc,
    .getc = imx8_lpuart_getc,
    .irq = imx8_lpuart_irq,
    .start_tx = imx8_lpuart_start_tx,
    .stop_tx = imx8_lpuart_stop_tx,
    .vuart_info = imx8_lpuart_vuart_info,
};

static int __init imx8_uart_init(struct dt_device_node *dev,
                                     const void *data)
{
    const char *config = data;
    struct imx8_lpuart *uart;
    u32 clkspec;
    int res;
    u64 addr, size;

#ifdef CONFIG_EARLY_PRINTK
    dprintk(XENLOG_ERR, "xx %x\n", EARLY_UART_BASE_ADDRESS);
#endif
    if ( strcmp(config, "") )
        printk("WARNING: UART configuration is not supported\n");

    uart = &imx8_com;

    clkspec = 100000000;
// baudrate : 115200
#define PARITY_NONE  (0)
    uart->clock_hz = clkspec;
    uart->baud = 115200; //?
    uart->data_bits = 8;
    uart->parity = PARITY_NONE;
    uart->stop_bits = 1;

    res = dt_device_get_address(dev, 0, &addr, &size);
    if ( res )
    {
        printk("imx8-uart: Unable to retrieve the base"
               " address of the UART\n");
        return res;
    }

    res = platform_get_irq(dev, 0);
    if ( res < 0 )
    {
        printk("imx8-uart: Unable to retrieve the IRQ\n");
        return -EINVAL;
    }
    uart->irq = res;

    uart->regs = ioremap_nocache(addr, size);
    if ( !uart->regs )
    {
        printk("imx8-uart: Unable to map the UART memory\n");
        return -ENOMEM;
    }

    // uart->vuart.base_addr = addr;
    // uart->vuart.size = size;
    // uart->vuart.data_off = UARTDATA;
    // /* tmp from uboot */
    // uart->vuart.status_off = USR1;
    // uart->vuart.status = USR1_TRDY;

    dprintk(XENLOG_ERR, "11\n");
    /* Register with generic serial driver */
    serial_register_uart(SERHND_DTUART, &imx8_uart_driver, uart);

    dt_device_set_used_by(dev, DOMID_XEN);

    return 0;
}

static const struct dt_device_match imx8_uart_dt_compat[] __initconst =
{
    DT_MATCH_COMPATIBLE("fsl,imx8mm-uart"),
    DT_MATCH_COMPATIBLE("fsl,imx8mn-uart"),
    DT_MATCH_COMPATIBLE("fsl,imx8mp-uart"),
    DT_MATCH_COMPATIBLE("fsl,imx8mq-uart"),
    DT_MATCH_COMPATIBLE("fsl,imx6q-uart"),
    {},
};

DT_DEVICE_START(imx8_uart, "i.MX8 UART", DEVICE_SERIAL)
    .dt_match = imx8_uart_dt_compat,
    .init = imx8_uart_init,
DT_DEVICE_END
/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
