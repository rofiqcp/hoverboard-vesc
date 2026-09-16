.syntax unified
.thumb

/* Trampoline IRQ kuat untuk build LTO STM32F103.
 * Startup CMSIS mendefinisikan handler sebagai weak alias ke Default_Handler.
 * Simbol kuat di file assembly ini tidak dapat dieliminasi LTO dan meneruskan
 * eksekusi ke implementasi C yang tetap dapat dioptimalkan lintas-modul. */

.macro ISR_TRAMPOLINE public_name, impl_name
    .global \public_name
    .type \public_name, %function
    .thumb_func
\public_name:
    b \impl_name
    .size \public_name, .-\public_name
.endm

ISR_TRAMPOLINE NMI_Handler, f103_NMI_Handler_impl
ISR_TRAMPOLINE HardFault_Handler, f103_HardFault_Handler_impl
ISR_TRAMPOLINE MemManage_Handler, f103_MemManage_Handler_impl
ISR_TRAMPOLINE BusFault_Handler, f103_BusFault_Handler_impl
ISR_TRAMPOLINE UsageFault_Handler, f103_UsageFault_Handler_impl
ISR_TRAMPOLINE SVC_Handler, f103_SVC_Handler_impl
ISR_TRAMPOLINE DebugMon_Handler, f103_DebugMon_Handler_impl
ISR_TRAMPOLINE PendSV_Handler, f103_PendSV_Handler_impl
ISR_TRAMPOLINE SysTick_Handler, f103_SysTick_Handler_impl
ISR_TRAMPOLINE DMA1_Channel1_IRQHandler, f103_DMA1_Channel1_IRQHandler_impl
#if defined(F103_CONTROL_USART2)
ISR_TRAMPOLINE DMA1_Channel7_IRQHandler, f103_ControlUartTxDma_IRQHandler_impl
ISR_TRAMPOLINE DMA1_Channel6_IRQHandler, f103_ControlUartRxDma_IRQHandler_impl
ISR_TRAMPOLINE USART2_IRQHandler, f103_ControlUart_IRQHandler_impl
#elif defined(F103_CONTROL_USART3)
ISR_TRAMPOLINE DMA1_Channel2_IRQHandler, f103_ControlUartTxDma_IRQHandler_impl
ISR_TRAMPOLINE DMA1_Channel3_IRQHandler, f103_ControlUartRxDma_IRQHandler_impl
ISR_TRAMPOLINE USART3_IRQHandler, f103_ControlUart_IRQHandler_impl
#else
#error "Control UART mode missing"
#endif
