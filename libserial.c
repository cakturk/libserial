/* open syscall */
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <unistd.h> /* close syscall */
#include <termios.h>

#include "libserial.h"

static int set_speed(struct termios *storage, unsigned int value)
{
	int retval = -1;
	speed_t baud;

	switch (value) {
		/* Do POSIX-specified rates first. */
	case 0: baud = B0; break;
	case 50: baud = B50; break;
	case 75: baud = B75; break;
	case 110: baud = B110; break;
	case 134: baud = B134; break;
	case 150: baud = B150; break;
	case 200: baud = B200; break;
	case 300: baud = B300; break;
	case 600: baud = B600; break;
	case 1200: baud = B1200; break;
	case 1800: baud = B1800; break;
	case 2400: baud = B2400; break;
	case 4800: baud = B4800; break;
	case 9600: baud = B9600; break;
	case 19200: baud = B19200; break;
	case 38400: baud = B38400; break;
# ifdef B7200
	case 7200: baud = B7200; break;
# endif
# ifdef B14400
	case 14400: baud = B14400; break;
# endif
# ifdef B57600
	case 57600: baud = B57600; break;
# endif
# ifdef B115200
	case 115200: baud = B115200; break;
# endif
# ifdef B230400
	case 230400: baud = B230400; break;
# endif
# ifdef B460800
	case 460800: baud = B460800; break;
# endif
# ifdef B500000
	case 500000: baud = B500000; break;
# endif
# ifdef B576000
	case 576000: baud = B576000; break;
# endif
# ifdef B921600
	case 921600: baud = B921600; break;
# endif
# ifdef B1000000
	case 1000000: baud = B1000000; break;
# endif
# ifdef B1152000
	case 1152000: baud = B1152000; break;
# endif
# ifdef B2000000
	case 2000000: baud = B2000000; break;
# endif
# ifdef B3000000
	case 3000000: baud = B3000000; break;
# endif
# ifdef B3500000
	case 3500000: baud = B3500000; break;
# endif
# ifdef B4000000
	case 4000000: baud = B4000000; break;
# endif
	default:
		      baud = B0;
		      return retval;
	}

	do {
# if defined(_BSD_SOURCE)
		retval = cfsetspeed(storage, baud);
		if (retval)
			break;
# else
		retval = cfsetispeed(storage, baud);
		if (retval)
			break;

		retval = cfsetospeed(storage, baud);
		if (retval)
			break;

#endif
	} while (0);

	return retval;
}

static unsigned int get_speed(struct termios *storage)
{
	unsigned int value;
	speed_t baud = cfgetospeed(storage);

	switch (baud) {
	case B0: value = 0; break;
	case B50: value = 50; break;
	case B75: value = 75; break;
	case B110: value = 110; break;
	case B134: value = 134; break;
	case B150: value = 150; break;
	case B200: value = 200; break;
	case B300: value = 300; break;
	case B600: value = 600; break;
	case B1200: value = 1200; break;
	case B1800: value = 1800; break;
	case B2400: value = 2400; break;
	case B4800: value = 4800; break;
	case B9600: value = 9600; break;
	case B19200: value = 19200; break;
	case B38400: value = 38400; break;
# ifdef B7200
	case B7200: value = 7200; break;
# endif
# ifdef B14400
	case B14400: value = 14400; break;
# endif
# ifdef B57600
	case B57600: value = 57600; break;
# endif
# ifdef B115200
	case B115200: value = 115200; break;
# endif
# ifdef B230400
	case B230400: value = 230400; break;
# endif
# ifdef B460800
	case B460800: value = 460800; break;
# endif
# ifdef B500000
	case B500000: value = 500000; break;
# endif
# ifdef B576000
	case B576000: value = 576000; break;
# endif
# ifdef B921600
	case B921600: value = 921600; break;
# endif
# ifdef B1000000
	case B1000000: value = 1000000; break;
# endif
# ifdef B1152000
	case B1152000: value = 1152000; break;
# endif
# ifdef B2000000
	case B2000000: value = 2000000; break;
# endif
# ifdef B3000000
	case B3000000: value = 3000000; break;
# endif
# ifdef B3500000
	case B3500000: value = 3500000; break;
# endif
# ifdef B4000000
	case B4000000: value = 4000000; break;
# endif
	default:
		       value = SPEED_ERR;
	}

	return value;
}

static inline int set_flow_control(struct termios *storage, unsigned int value)
{
	switch (value) {
	case FC_NONE:
		storage->c_iflag &= ~(IXOFF | IXON);
# if defined(_BSD_SOURCE)
		storage->c_cflag &= ~CRTSCTS;
# elif defined(__QNXNTO__)
		storage->c_cflag &= ~(IHFLOW | OHFLOW);
# endif
		break;
	case FC_SOFTWARE:
		storage->c_iflag |= IXOFF | IXON;
# if defined(_BSD_SOURCE)
		storage->c_cflag &= ~CRTSCTS;
# elif defined(__QNXNTO__)
		storage->c_cflag &= ~(IHFLOW | OHFLOW);
# endif
		break;
	case FC_HARDWARE:
# if defined(_BSD_SOURCE)
		storage->c_iflag &= ~(IXOFF | IXON);
		storage->c_cflag |= CRTSCTS;
		break;
# elif defined(__QNXNTO__)
		storage->c_iflag &= ~(IXOFF | IXON);
		storage->c_cflag |= (IHFLOW | OHFLOW);
		break;
# else
		return -1;
# endif
	default:
		return -1;
	}

	return 0;
}

static inline unsigned int get_flow_control(struct termios *storage)
{
	unsigned int value;
	if (storage->c_iflag & (IXOFF | IXON))
		value = FC_SOFTWARE;
# if defined(_BSD_SOURCE)
	else if (storage->c_cflag & CRTSCTS)
		value = FC_HARDWARE;
# elif defined(__QNXNTO__)
	else if (storage->c_cflag & IHFLOW && storage->c_cflag & OHFLOW)
		value = FC_HARDWARE;
# endif
	else
		value = FC_NONE;

	return value;
}

static inline int set_parity(struct termios *storage, unsigned int value)
{
	switch (value) {
	case P_NONE:
		storage->c_iflag |= IGNPAR;
		storage->c_cflag &= ~(PARENB | PARODD);
		break;
	case P_EVEN:
		storage->c_iflag &= ~(IGNPAR | PARMRK);
		storage->c_iflag |= INPCK;
		storage->c_cflag |= PARENB;
		storage->c_cflag &= ~PARODD;
		break;
	case P_ODD:
		storage->c_iflag &= ~(IGNPAR | PARMRK);
		storage->c_iflag |= INPCK;
		storage->c_cflag |= (PARENB | PARODD);
		break;
	default:
		return -1;
	}
	return 0;
}

static inline unsigned int get_parity(struct termios *storage)
{
	unsigned int value;
	if (storage->c_cflag & PARENB) {
		if (storage->c_cflag & PARODD)
			value = P_ODD;
		else
			value = P_EVEN;
	} else {
		value = P_NONE;
	}
	return value;
}

static inline int set_character_size(struct termios *storage,
				     unsigned char value)
{
	storage->c_cflag &= ~CSIZE;
	switch (value) {
	case 5: storage->c_cflag |= CS5; break;
	case 6: storage->c_cflag |= CS6; break;
	case 7: storage->c_cflag |= CS7; break;
	case 8: storage->c_cflag |= CS8; break;
	default:
		return -1;
	}
	return 0;
}

static inline unsigned char get_character_size(struct termios *storage)
{
	unsigned int value;
	unsigned int flags = storage->c_cflag & CSIZE;

	if (flags == CS5)
		value = 5;
	else if (flags == CS6)
		value = 6;
	else if (flags == CS7)
		value = 7;
	else if (flags == CS8)
		value = 8;
	else
		value = 8;

	return value;
}

static inline int set_stop_bits(struct termios *storage, unsigned char value)
{
	switch (value) {
	case SB_ONE:
		storage->c_cflag &= ~CSTOPB;
		break;
	case SB_TWO:
		storage->c_cflag |= CSTOPB;
		break;
	default:
		return -1;
	}
	return 0;
}

static inline unsigned char get_stop_bits(struct termios *storage)
{
	return (storage->c_cflag & CSTOPB) ? SB_TWO : SB_ONE;
}

static int do_set_attributes(int fd, struct termios *ios,
			     struct port_attributes *attr)
{
	int retval = -1;
	do {
		if (set_speed(ios, attr->baud))
			break;
		if (set_flow_control(ios, attr->flow_control))
			break;
		if (set_parity(ios, attr->parity))
			break;
		if (set_stop_bits(ios, attr->stop_bits))
			break;
		if (set_character_size(ios, attr->character_size))
			break;
		if (tcsetattr(fd, TCSANOW, ios))
			break;
		retval = 0;
	} while (0);

	return retval;
}

int serial_set_rts(int fd, int level)
{
	int flags;
	if (ioctl(fd, TIOCMGET, &flags))
		return -1;
	if (level)
		flags |= TIOCM_RTS;
	else
		flags &= ~TIOCM_RTS;
	return ioctl(fd, TIOCMSET, &flags);
}

/* On success, zero is returned. On error, -1 is returned. */
int serial_set_attributes(int fd, struct port_attributes *attr)
{
	struct termios ios;
	if (tcgetattr(fd, &ios))
		return -1;

	return do_set_attributes(fd, &ios, attr);
}

/* On success, zero is returned. On error, -1 is returned. */
int serial_get_attributes(int fd, struct port_attributes *attr)
{
	struct termios ios;

	if (tcgetattr(fd, &ios))
		return -1;

	attr->baud = get_speed(&ios);
	attr->flow_control = get_flow_control(&ios);
	attr->parity = get_parity(&ios);
	attr->stop_bits = get_stop_bits(&ios);
	attr->character_size = get_character_size(&ios);

	return 0;
}

int serial_open(const char *dev, int flags, struct port_attributes *attr)
{
	struct termios ios;
	int fd = open(dev, O_RDWR  | O_NOCTTY | flags);
	if (fd == -1)
		goto err_out;

	if (tcgetattr(fd, &ios))
		goto err_out;

	ios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK
			 | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	ios.c_oflag &= ~OPOST;
	ios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	ios.c_cflag &= ~(CSIZE | PARENB);
	ios.c_cflag |= CS8;

	ios.c_iflag |= IGNPAR;
	ios.c_cflag |= CREAD | CLOCAL;

	if (do_set_attributes(fd, &ios, attr) != -1)
		return fd;

err_out:
	close(fd);
	return -1;
}
