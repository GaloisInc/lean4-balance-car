// Serial port setup/initalization based on linux serial port example here:
//   https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/#full-example-standard-baud-rates
// C library headers
#include <stdio.h>
#include <string.h>
#include <cmath>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/ioctl.h>
#include <linux/serial.h>

// Lean4 headers
#include <lean/object.h>
#include <lean/io.h>

#define DEBUG 0

#define LOG(fmt, ...) ((void)0) //do \
// { \
//   if (DEBUG) { \
//     fprintf(stderr, "[LOG] "); \
//     fprintf(stderr, fmt __VA_OPT__(,) __VA_ARGS__); \
//   } \
// } while (0)

#define CHECK_SERIAL_PORT() ((void)0) //do \
// { \
//   if (serial_port < 0) { \
//     fprintf(stderr, "ERROR: serial port is not initialized!"); \
//     fprintf(stderr, "       See `initializeSerialPort` in `BalanceCar.lean`."); \
//     exit(1); \
//   } \
// } while (0)

struct termios tty;
int serial_port = -1;
unsigned char buffer[256];

extern "C" lean_object * lean_initialize_serial_port(b_lean_obj_arg port_path, speed_t baudrate, lean_object /* w */) {
  LOG("entering lean_initialize_serial_port\n");
  LOG("opening serial port at %s\n", lean_string_cstr(port_path));
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  serial_port = open(lean_string_cstr(port_path), O_RDWR);
  LOG("serial_port = %d\n", serial_port);

  struct serial_struct serial;
  if (ioctl(serial_port, TIOCGSERIAL, &serial) == 0) {
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(serial_port, TIOCSSERIAL, &serial);
  }

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
    fprintf(stderr, "ERROR for %s: %i from tcgetattr: %s\n",
      lean_string_cstr(port_path),
      errno,
      strerror(errno));
    exit(1);
  }
  LOG("setting tty.c_flags\n");
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  LOG("setting in/out baud rate\n");
  // Set in/out baud rate
  cfsetispeed(&tty, baudrate);
  cfsetospeed(&tty, baudrate);

  LOG("saving tty settings\n");
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    fprintf(stderr, "ERROR: %i from tcsetattr: %s\n", errno, strerror(errno));
    exit(1);
  }

  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  memset(&buffer, '\0', sizeof(buffer));

  LOG("exiting lean_initialize_serial_port\n");
  return lean_io_result_mk_ok(lean_box(0));
}

extern "C" lean_object * lean_start_car(lean_object /* w */) {
  LOG("entering lean_start_car\n");
  CHECK_SERIAL_PORT();
  buffer[0] = '!';
  write(serial_port, buffer, 1);
  LOG("exiting lean_analog_pin_write\n");
  return lean_io_result_mk_ok(lean_box(0));
}

extern "C" lean_object * lean_drive_car(bool mode1, bool mode2, double val1, double val2, lean_object /* w */) {
  LOG("entering lean_drive_car\n");
  CHECK_SERIAL_PORT();
  val1 = fmin(fabs(val1), 255);
  val2 = fmin(fabs(val2), 255);
  buffer[0] = 'P';
  buffer[1] = 'D';
  buffer[2] = 'X';
  buffer[3] = mode1 ? 0b11111111 : 0b00000000;
  buffer[4] = mode2 ? 0b11111111 : 0b00000000;
  buffer[5] = (uint8_t) val1;
  buffer[6] = (uint8_t) val2;
  write(serial_port, buffer, 7);
  LOG("exiting lean_analog_pin_write\n");
  return lean_io_result_mk_ok(lean_box(0));
}

extern "C" lean_obj_res lean_rx_int16_as_int(lean_object /* w */) {
  LOG("entering lean_rx_int16_as_int\n");
  CHECK_SERIAL_PORT();
  int num_bytes = 2;
  int res;
  unsigned char *ptr = buffer;
  do {
    res = read(serial_port, ptr, num_bytes);
    if (res <= 0) {
      fprintf(stderr, "ERROR: lean_rx_int16_as_int read failure, got %d", res);
      exit(1);
    } else if (res <= num_bytes) {
      num_bytes -= res;
      ptr += res;
    } else {
      fprintf(stderr, "ERROR: lean_rx_int16_as_int read %d but expected %d", res, num_bytes);
      exit(1);
    }
  } while (num_bytes > 0);

  uint16_t val = (uint16_t)buffer[1] << 8 | buffer[0]; // Not positive on the endianness here...
  LOG("exiting lean_rx_int16_as_int\n");
  return lean_io_result_mk_ok(lean_int_to_int(val));
}

extern "C" lean_obj_res lean_rx_long_as_int(lean_object /* w */) {
  LOG("entering lean_rx_long_as_int\n");
  CHECK_SERIAL_PORT();
  int num_bytes = 4;
  int res;
  unsigned char *ptr = buffer;
  do {
    res = read(serial_port, ptr, num_bytes);
    if (res <= 0) {
      fprintf(stderr, "ERROR: lean_rx_long_as_int read failure, got %d", res);
      exit(1);
    } else if (res <= num_bytes) {
      num_bytes -= res;
      ptr += res;
    } else {
      fprintf(stderr, "ERROR: lean_rx_long_as_int read %d but expected %d", res, num_bytes);
      exit(1);
    }
  } while (num_bytes > 0);

  long val = (long)buffer[3] << 24 | (long)buffer[2] << 16 | (long)buffer[1] << 8 | buffer[0]; // Not positive on the endianness here...
  LOG("exiting lean_rx_long_as_int\n");
  return lean_io_result_mk_ok(lean_int_to_int(val));
}

// loop until bytes 'P' 'D' 'X' come over the serial port
extern "C" lean_obj_res lean_wait_for_header(lean_object /* w */) {
  LOG("entering lean_wait_for_header\n");
  CHECK_SERIAL_PORT();
  int res;
  unsigned char actual, expected;
  expected = 'P';
  while (1) {
    res = read(serial_port, &actual, 1);
    if (res <= 0) {
      fprintf(stderr, "ERROR: lean_wait_for_header read failure, got %d", res);
      exit(1);
    } else if (res == 1) {
      if (actual == expected) {
        switch (expected) {
          case 'P': expected = 'D'; break;
          case 'D': expected = 'X'; break;
          case 'X': return lean_io_result_mk_ok(lean_box(0));
          default: 
            fprintf(stderr, "ERROR: lean_wait_for_header reached impossible state waiting for header");
            exit(1);
        }
      } else {
        expected = 'P';
      }
    } else {
      fprintf(stderr, "ERROR: lean_wait_for_header read %d but expected 1 byte", res);
      exit(1);
    }
  }
}
