#ifndef usci_isr_handler_h
#define usci_isr_handler_h

typedef void CHardwareSerial;
#ifdef __cplusplus
extern "C" {
#endif
void uart_tx_isr(uint8_t offset);
void uart_rx_isr(uint8_t offset);
void usci_isr_install(void);
boolean i2c_txrx_isr(uint8_t module);
boolean i2c_state_isr(uint8_t module);
#ifdef __cplusplus
}
#endif
#endif /* usci_isr_handler_h */
