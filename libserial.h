#ifndef LIBSERIAL_H_A9JN230Q
#define LIBSERIAL_H_A9JN230Q

#ifdef __cplusplus
extern "C" {
#endif

#define SPEED_ERR   (~0U)

struct port_attributes {
    unsigned int baud;

/* Flow control */
#define FC_NONE     0x0
#define FC_SOFTWARE 0x1
#define FC_HARDWARE 0x2
    unsigned int flow_control;

/* Parity */
#define P_NONE      0x0
#define P_EVEN      0x1
#define P_ODD       0x2
    unsigned int parity;

/* Stop bits */
#define SB_ONE      0x0
#define SB_TWO      0x1
    unsigned char stop_bits;
    unsigned char character_size; /* 5 - 8 */
};

#define PORT_ATTR_INIT_BAUD(baud) \
    { baud, FC_NONE, P_NONE, SB_ONE, 8 }

#define PORT_ATTR_INIT \
    PORT_ATTR_INIT_BAUD(115200)

/* On success, zero is returned. On error, -1 is returned. */
int serial_set_attributes(int fd, struct port_attributes *attr);
int serial_get_attributes(int fd, struct port_attributes *attr);
int serial_open(const char *dev, int flags, struct port_attributes *attr);
int serial_set_rts(int fd, int level);

#ifdef __cplusplus
}
#endif

#endif /* end of include guard: LIBSERIAL_H_A9JN230Q */
