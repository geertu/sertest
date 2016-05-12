/*
 *  Serial Link Test Program
 *
 *  (C) Copyright 2015-2016 Geert Uytterhoeven
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License.
 */

#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <bsd/stdlib.h>

#include <libbrahe/prng.h>

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <linux/serial.h>


#define DEFAULT_MAX_MSG_LEN	256
#define MAX_MAX_MSG_LEN		4096

#define MAX_LIST_SIZE		64

#define TX_TIMEOUT		5
#define RX_TIMEOUT		5
#define RX_TIMEOUT_INIT		60

#define max(x, y) ({ \
	typeof(x) _x = (x);     \
	typeof(y) _y = (y);     \
	(void) (&_x == &_y);	\
	_x > _y ? _x : _y; })

#define min(x, y) ({ \
	typeof(x) _x = (x);     \
	typeof(y) _y = (y);     \
	(void) (&_x == &_y);	\
	_x < _y ? _x : _y; })

static int opt_master;
static int opt_chain;
static int opt_icount;
static const char *opt_device1, *opt_device2;
static uint32_t opt_seed = 42;
static uint32_t opt_msglen = DEFAULT_MAX_MSG_LEN;
static uint32_t opt_speed;
static int opt_verbose;

#define ESC_BLACK	"\e[30m"
#define ESC_RED		"\e[31m"
#define ESC_GREEN	"\e[32m"
#define ESC_YELLOW	"\e[33m"
#define ESC_BLUE	"\e[34m"
#define ESC_PURPLE	"\e[35m"
#define ESC_CYAN	"\e[36m"
#define ESC_WHITE	"\e[37m"
#define ESC_RM		"\e[0m"

#define TAG_TX		ESC_BLUE "[tx]"
#define TAG_RX		ESC_PURPLE "[rx]"

static brahe_prng_state_t prng;

struct msg {
	unsigned long long stamp;
	struct msg *next;
	unsigned int len;
	unsigned char buf[0];
};

static struct msg *list_head, *list_tail;
static unsigned int list_size;

static pthread_mutex_t list_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t list_nonempty = PTHREAD_COND_INITIALIZER;
static pthread_cond_t list_full = PTHREAD_COND_INITIALIZER;

static pthread_t rx_thread, tx_thread;
static unsigned long long rx_bytes, tx_bytes;

static unsigned long long start_stamp;

static unsigned char numslaves;

static const struct speed {
	speed_t sym;
	unsigned int val;
} speeds[] = {
	{ B0,		0 },
	{ B50,		50 },
	{ B75,		75 },
	{ B110,		110 },
	{ B134,		134 },
	{ B150,		150 },
	{ B200,		200 },
	{ B300,		300 },
	{ B600,		600 },
	{ B1200,	1200 },
	{ B1800,	1800 },
	{ B2400,	2400 },
	{ B4800,	4800 },
	{ B9600,	9600 },
	{ B19200,	19200 },
	{ B38400,	38400 },
#ifdef B57600
	{ B57600,	57600 },
#endif
#ifdef B115200
	{ B115200,	115200 },
#endif
#ifdef B230400
	{ B230400,	230400 },
#endif
#ifdef B460800
	{ B460800,	460800 },
#endif
#ifdef B500000
	{ B500000,	500000 },
#endif
#ifdef B576000
	{ B576000,	576000 },
#endif
#ifdef B921600
	{ B921600,	921600 },
#endif
#ifdef B1000000
	{ B1000000,	1000000 },
#endif
#ifdef B1152000
	{ B1152000,	1152000 },
#endif
#ifdef B1500000
	{ B1500000,	1500000 },
#endif
#ifdef B2000000
	{ B2000000,	2000000 },
#endif
#ifdef B2500000
	{ B2500000,	2500000 },
#endif
#ifdef B3000000
	{ B3000000,	3000000 },
#endif
#ifdef B3500000
	{ B3500000,	3500000 },
#endif
#ifdef B4000000
	{ B4000000,	4000000 },
#endif
};

static int get_speed_val(speed_t speed)
{
	unsigned int i;

	for (i = 0; i < sizeof(speeds)/sizeof(*speeds); i++)
		if (speeds[i].sym == speed)
			return speeds[i].val;
	return -1;
}

static speed_t get_speed_sym(unsigned speed)
{
	unsigned int i;

	for (i = 0; i < sizeof(speeds)/sizeof(*speeds); i++)
		if (speeds[i].val == speed)
			return speeds[i].sym;
	return -1;
}

static const char *thread_prefix(void)
{
	pthread_t self = pthread_self();

	if (self == rx_thread)
		return ESC_PURPLE "[rx] ";
	if (self == tx_thread)
		return ESC_BLUE "[tx] ";

	return "";
}

#define pr_debug(fmt, ...) \
{ \
	if (opt_verbose) \
		printf("%s" fmt ESC_RM, thread_prefix(), ##__VA_ARGS__); \
}

#define pr_info(fmt, ...) \
	printf("%s" fmt ESC_RM, thread_prefix(), ##__VA_ARGS__)

#define pr_warn(fmt, ...) \
	printf("%s" ESC_YELLOW fmt ESC_RM, thread_prefix(), ##__VA_ARGS__)

#define pr_error(fmt, ...) \
	fprintf(stderr, "%s" ESC_RED fmt ESC_RM, thread_prefix(), ##__VA_ARGS__)

static unsigned long long get_timestamp(void)
{
	struct timeval tv;

	if (gettimeofday(&tv, NULL)) {
		pr_error("Failed to get time: %s\n", strerror(errno));
		exit(-1);
	}

	return tv.tv_sec * 1000000ULL + tv.tv_usec;
}

static struct msg *msg_gen(int len)
{
	struct msg *msg;
	unsigned int i;

	if (len < 0)
		len = brahe_prng_range(&prng, 1, -len);

	msg = malloc(sizeof(*msg) + len);
	memset(msg, 0, sizeof(*msg));

	msg->len = len;
	for (i = 0; i < len; i++)
		msg->buf[i] = brahe_prng_next(&prng);
	msg->stamp = get_timestamp();

	return msg;
}

static void print_stats(void)
{
	unsigned long long delay = get_timestamp() - start_stamp;

	if (!delay)
		delay = 1;

	pr_warn("TX: %llu bytes (%llu bps), RX: %llu bytes (%llu bps)\n",
		tx_bytes, tx_bytes * 8000000 / delay, rx_bytes,
		rx_bytes * 8000000 / delay);
}

static void msg_add(struct msg *msg)
{
	unsigned int timeout = TX_TIMEOUT;
	struct timespec abstime;
	int error;

	assert(!msg->next);

	pthread_mutex_lock(&list_mutex);

	if (list_size > MAX_LIST_SIZE) {
		pr_debug("Too many bytes in flight, pausing...\n");

		if (clock_gettime(CLOCK_REALTIME, &abstime)) {
			pr_error("Failed to get clock: %s\n", strerror(errno));
			exit(-1);
		}
		abstime.tv_sec += timeout;

		do {
			error = pthread_cond_timedwait(&list_full, &list_mutex,
						       &abstime);
			if (error == ETIMEDOUT) {
				pr_error("Timeout, aborting\n");
				print_stats();
				exit(-1);
			}
		} while (list_size > MAX_LIST_SIZE);
	}

	if (list_tail)
		list_tail->next = msg;
	else
		list_head = msg;
	list_tail = msg;
	list_size++;

	pthread_mutex_unlock(&list_mutex);

	pthread_cond_signal(&list_nonempty);
}

static struct msg *msg_get(void)
{
	static unsigned int timeout = RX_TIMEOUT_INIT;
	struct timespec abstime;
	struct msg *msg;
	int error;

	pthread_mutex_lock(&list_mutex);

	msg = list_head;
	if (!msg) {
		pr_debug("No more messages, waiting...\n");

		if (clock_gettime(CLOCK_REALTIME, &abstime)) {
			pr_error("Failed to get clock: %s\n", strerror(errno));
			exit(-1);
		}
		abstime.tv_sec += timeout;

		do {
			error = pthread_cond_timedwait(&list_nonempty,
						       &list_mutex, &abstime);
			if (error == ETIMEDOUT) {
				pr_error("Timeout, aborting\n");
				print_stats();
				exit(-1);
			}

			msg = list_head;
		} while (!msg);
	}

	list_head = msg->next;
	msg->next = NULL;
	if (msg == list_tail)
		list_tail = NULL;
	list_size--;

	pthread_mutex_unlock(&list_mutex);

	pthread_cond_signal(&list_full);

	timeout = RX_TIMEOUT;

	return msg;
}

static void print_line(unsigned int index, const unsigned char *buf,
		       unsigned int len)
{
	unsigned int i;

	pr_info("%04x:", index);

	for (i = 0; i < len; i++)
		printf(" %02x", buf[i]);
	for (i = len; i < 16; i++)
		printf("   ");

	printf(" |");
	for (i = 0; i < len; i++)
		putchar(buf[i] >= 32 && buf[i] < 127 ? buf[i] : '.');
	puts("|");
}

static void print_buffer(const void *buf, unsigned int len)
{
	unsigned int i;

	for (i = 0; i < len; i += 16)
		print_line(i, buf + i, min(len - i, 16u));
}

static int cmp_line(unsigned int address, const unsigned char *buf1,
		    const unsigned char *buf2, unsigned int len)
{
	unsigned int i;
	unsigned char c;
	int res = 0;

	pr_info("%04x:", address);

	for (i = 0; i < len; i++)
		if (buf1[i] == buf2[i]) {
			printf(" %02x", buf1[i]);
		} else {
			printf(" " ESC_RED "%02x" ESC_RM, buf1[i]);
			res++;
		}
	for (i = len; i < 16; i++)
		printf("   ");

	printf(" |");
	for (i = 0; i < len; i++) {
		c = buf1[i] >= 32 && buf1[i] < 127 ? buf1[i] : '.';
		if (buf1[i] == buf2[i])
			putchar(c);
		else
			printf(ESC_RED "%c" ESC_RM, c);
	}
	puts("|");
	return res;
}

static void cmp_buffer(const void *buf1, const void *buf2, unsigned int len)
{
	unsigned int i;

	for (i = 0; i < len; i += 16) {
		if (!cmp_line(i, buf1 + i, buf2 + i, min(len - i, 16u)))
			continue;
		pr_info("Expected:\n");
		print_line(i, buf2 + i, min(len - i, 16u));

	}
}

static void msg_dump(const struct msg *msg)
{
	pr_info("Message with %u bytes of data\n", msg->len);
	print_buffer(msg->buf, msg->len);
}

static void *transmit_start(void *arg)
{
	int fd = (unsigned long)arg;
	struct msg *msg;
	ssize_t res;

	while (1) {
		msg = opt_master ? msg_gen(-opt_msglen) : msg_get();

		if (opt_verbose)
			msg_dump(msg);

		res = write(fd, msg->buf, msg->len);
		if (res < 0) {
			pr_error("Write error %d\n", errno);
			exit(-1);
		}

		if (res < msg->len) {
			pr_error("Short write %zd < %u\n", res, msg->len);
			exit(-1);
		}

		tx_bytes += res;

		if (opt_master && opt_chain)
			msg_add(msg);
		else
			free(msg);
	}

	return NULL;
}

static int msg_verify(struct msg *msg, unsigned char *buf)
{
	static int first = 1;
	unsigned int i;

	if (first) {
		numslaves = buf[0] - msg->buf[0];
		if (opt_master)
			pr_info("Detected %u slaves in the chain\n",
				numslaves);
		else
			pr_info("Detected slave order %u\n",
				numslaves);
		first = 0;
	}
	if (numslaves)
		for (i = 0; i < msg->len; i++)
			msg->buf[i] += numslaves;

	return !memcmp(buf, msg->buf, msg->len);
}

static void msg_inc(struct msg *msg)
{
	unsigned int i;

	for (i = 0; i < msg->len; i++)
		msg->buf[i]++;
}

static void *receive_start(void *arg)
{
	int fd = (unsigned long)arg;
	struct msg *msg;
	ssize_t res;
	unsigned int avail = 0;
	static unsigned char buf[MAX_MAX_MSG_LEN];
	unsigned long long delay;

	while (1) {
		msg = opt_master ? msg_get() : msg_gen(-opt_msglen);

		while (avail < msg->len) {
			res = read(fd, buf + avail, sizeof(buf) - avail);
			if (res < 0) {
				pr_error("Read error %d\n", errno);
				exit(-1);
			}
			avail += res;
			rx_bytes += res;
		}

		if (!msg_verify(msg, buf)) {
			pr_error("Data mismatch\n");
			cmp_buffer(buf, msg->buf, msg->len);
			print_stats();
			exit(-1);
		}

		delay = get_timestamp() - msg->stamp;
		if (!delay)
			delay = 1;
		pr_debug(ESC_GREEN "OK %u bytes in %llu us (%llu bps)\n",
			msg->len, delay, msg->len * 8000000 / delay);

		avail -= msg->len;
		if (avail > 0) {
			pr_debug("Keeping %u bytes\n", avail);
			memmove(buf, buf + msg->len, avail);
		}

		if (!opt_master && opt_chain) {
			msg_inc(msg);
			msg_add(msg);
		} else {
			free(msg);
		}
	}

	return NULL;
}

static void signal_handler(int signum)
{
	print_stats();
	if (signum == SIGQUIT)
		exit(-1);
}

static struct sigaction signal_action = {
	.sa_handler = signal_handler,
};

static void __attribute__ ((noreturn)) usage(void)
{
	fprintf(stderr,
		"\n"
		"%s: [options] <device> [<device2>]\n\n"
		"Valid options are:\n"
		"    -h, --help       Display this usage information\n"
		"    --master         Use master mode\n"
		"    --slave          Use slave mode (default)\n"
		"    --chain          Master and slave(s) are daisy chained\n"
		"    --icount         Only print icount\n"
		"    -i, --seed       Initial seed (zero is pseudorandom)\n"
		"    -l, --len        Maximum message length (default %u, must be <= %u)\n"
		"    -s, --speed      Serial speed\n"
		"    -v, --verbose    Enable verbose mode\n"
		"\n"
		"If one device is specified, it is used for bidirectional communication.\n"
		"If two devices are specified, the first device is used for output, and\n"
		"the second device is used for input.\n"
		"\n"
		"Use \"CTRL-C\" to print transfer statistics, \"CTRL-\\\" to quit.\n"
		"\n",
		getprogname(), DEFAULT_MAX_MSG_LEN, MAX_MAX_MSG_LEN);
	exit(1);
}

static int device_open(const char *pathname, int flags, int makeraw)
{
	struct termios termios;
	int fd;

	pr_debug("Trying to open %s...\n", pathname);
	fd = open(pathname, flags);
	if (fd < 0) {
		pr_error("Failed to open %s%s: %s\n", pathname,
			 flags == O_WRONLY ? " for writing" :
			 flags == O_RDONLY ? " for reading" : "",
			 strerror(errno));
		exit(-1);
	}

	if (!makeraw)
		return fd;

	if (tcgetattr(fd, &termios)) {
		if (errno == ENOTTY) {
			pr_info("%s is not a tty, skipping tty config\n",
				pathname);
			return fd;
		}
		pr_error("Failed to get terminal attributes: %s\n",
			 strerror(errno));
		exit(-1);
	}
	pr_debug("termios.c_iflag = 0%o\n", termios.c_iflag);
	pr_debug("termios.c_oflag = 0%o\n", termios.c_oflag);
	pr_debug("termios.c_cflag = 0%o\n", termios.c_cflag);
	pr_debug("termios.c_lflag = 0%o\n", termios.c_lflag);

	cfmakeraw(&termios);
	if (tcsetattr(fd, TCSANOW, &termios)) {
		pr_error("Failed to enable raw mode: %s\n", strerror(errno));
		exit(-1);
	}

	if (opt_speed) {
		int sym = get_speed_sym(opt_speed);

		if (sym == -1) {
			pr_error("Unknown serial speed %u\n", opt_speed);
			exit(-1);
		}
		if (cfsetspeed(&termios, sym)) {
			pr_error("Failed to set terminal speed: %s\n",
				 strerror(errno));
			exit(-1);
		}
		if (tcsetattr(fd, TCSANOW, &termios)) {
			pr_error("Failed to set speed attribute: %s\n",
				 strerror(errno));
			exit(-1);
		}
	} else {
		pr_debug("Serial speed is %u/%u\n",
			 get_speed_val(cfgetispeed(&termios)),
			 get_speed_val(cfgetospeed(&termios)));
	}

	if (tcflush(fd, TCIOFLUSH)) {
		pr_error("Failed to flush: %s\n", strerror(errno));
		exit(-1);
	}

	return fd;
}

static void do_icount(void)
{
	struct serial_icounter_struct icount;
	int fd;

	fd = device_open(opt_device1, O_RDONLY, 0);

	if (ioctl(fd, TIOCGICOUNT, &icount)) {
		pr_error("Failed to get icount: %s\n", strerror(errno));
		exit(-1);
	}

	printf("cts         = %d\n", icount.cts);
	printf("dsr         = %d\n", icount.dsr);
	printf("rng         = %d\n", icount.rng);
	printf("dcd         = %d\n", icount.dcd);
	printf("rx          = %d\n", icount.rx);
	printf("tx          = %d\n", icount.tx);
	printf("frame       = %d\n", icount.frame);
	printf("overrun     = %d\n", icount.overrun);
	printf("parity      = %d\n", icount.parity);
	printf("brk         = %d\n", icount.brk);
	printf("buf_overrun = %d\n", icount.buf_overrun);

	close(fd);
	exit(0);
}

int main(int argc, char *argv[])
{
	int tx_fd = -1, rx_fd = -1;

	while (argc > 1) {
		if (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help")) {
			usage();
		} else if (!strcmp(argv[1], "--master")) {
			opt_master = 1;
		} else if (!strcmp(argv[1], "--slave")) {
			opt_master = 0;
		} else if (!strcmp(argv[1], "--chain")) {
			opt_chain = 1;
		} else if (!strcmp(argv[1], "--icount")) {
			opt_icount = 1;
		} else if (!strcmp(argv[1], "-i") ||
			   !strcmp(argv[1], "--seed")) {
			if (argc <= 2)
				usage();
			opt_seed = strtoul(argv[2], NULL, 0);
			argv++;
			argc--;
		} else if (!strcmp(argv[1], "-l") ||
			   !strcmp(argv[1], "--len")) {
			if (argc <= 2)
				usage();
			opt_msglen = strtoul(argv[2], NULL, 0);
			if (!opt_msglen || opt_msglen > MAX_MAX_MSG_LEN)
				usage();
			argv++;
			argc--;
		} else if (!strcmp(argv[1], "-s") ||
			   !strcmp(argv[1], "--speed")) {
			if (argc <= 2)
				usage();
			opt_speed = strtoul(argv[2], NULL, 0);
			argv++;
			argc--;
		} else if (!strcmp(argv[1], "-v") ||
			   !strcmp(argv[1], "--verbose")) {
			opt_verbose = 1;
		} else if (!opt_device1) {
			opt_device1 = argv[1];
		} else if (!opt_device2) {
			opt_device2 = argv[1];
		} else {
			usage();
		}
		argv++;
		argc--;
	}

	if (!opt_device1 || (opt_device2 && !opt_chain))
		usage();

	if (opt_icount)
		do_icount();

	brahe_prng_init(&prng, BRAHE_PRNG_MARSENNE_TWISTER, opt_seed);

	if (opt_device2) {
		if (opt_master) {
			tx_fd = device_open(opt_device1, O_WRONLY, 1);
			rx_fd = device_open(opt_device2, O_RDONLY, 1);
		} else {
			rx_fd = device_open(opt_device2, O_RDONLY, 1);
			tx_fd = device_open(opt_device1, O_WRONLY, 1);
		}
	} else if (opt_chain) {
		tx_fd = rx_fd = device_open(opt_device1, O_RDWR, 1);
	} else if (opt_master) {
		tx_fd = device_open(opt_device1, O_WRONLY, 1);
	} else {
		rx_fd = device_open(opt_device1, O_RDONLY, 1);
	}

	start_stamp = get_timestamp();

	sigaction(SIGINT, &signal_action, NULL);
	sigaction(SIGQUIT, &signal_action, NULL);

	if (rx_fd >= 0)
		pthread_create(&rx_thread, NULL, receive_start,
			       (void *)(unsigned long)rx_fd);
	if (tx_fd >= 0)
		pthread_create(&tx_thread, NULL, transmit_start,
			       (void *)(unsigned long)tx_fd);

	if (rx_fd >= 0)
		pthread_join(rx_thread, NULL);
	if (tx_fd >= 0)
		pthread_join(tx_thread, NULL);

	if (tx_fd >= 0)
		close(tx_fd);
	if (rx_fd >= 0 && rx_fd != tx_fd)
		close(rx_fd);

	exit(0);
}

