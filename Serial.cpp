// Linux serial port example here:
//   https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/#full-example-standard-baud-rates
// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

// Lean4 headers
#include <lean/object.h>
#include <lean/io.h>

struct termios tty;
int serial_port = -1;
unsigned char buffer[256];

#define CHECK_SERIAL_PORT() do { \
  if (serial_port < 0) initialize_serial_port(); \
} while (0)

extern "C" void initialize_serial_port() {
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  serial_port = open("/dev/ttyUSB0", O_RDWR);

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
    fprintf(stderr, "ERROR: %i from tcgetattr: %s\n", errno, strerror(errno));
    exit(1);
  }

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

  // Set in/out baud rate to be 115200
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    fprintf(stderr, "ERROR: %i from tcsetattr: %s\n", errno, strerror(errno));
    exit(1);
  }

  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  memset(&buffer, '\0', sizeof(buffer));
}

extern "C" lean_object * lean_digital_pin_write(uint8_t pin, bool value) {
  CHECK_SERIAL_PORT();
  buffer[0] = 'd';
  buffer[1] = pin;
  buffer[2] = value ? 0b11111111 : 0b00000000;
  write(serial_port, buffer, 3);
  return lean::io_result_mk_ok(lean_box(0));
}

extern "C" lean_object * lean_analog_pin_write(uint8_t pin, double value) {
  CHECK_SERIAL_PORT();
  value = value < 0 ? -value : value;
  value = value > 255 ? 255 : value;
  buffer[0] = 'a';
  buffer[1] = pin;
  buffer[2] = (uint8_t) value;
  write(serial_port, buffer, 3);
  return lean::io_result_mk_ok(lean_box(0));
}

extern "C" uint32_t lean_rx_char_as_uint32() {
  CHECK_SERIAL_PORT();
  int num_bytes = read(serial_port, &buffer, 1);
  if (num_bytes == 1) {
      return buffer[0];
  } else {
      fprintf(
        stderr,
        "ERROR: failed to read a single byte in lean_rx_char_as_uint32, instead got %d", 
        num_bytes);
      exit(1);
  }
}

extern "C" lean_obj_res lean_rx_int16_as_int() {
  CHECK_SERIAL_PORT();
  int num_bytes = read(serial_port, &buffer, 2);
  if (num_bytes == 2) {
    uint16_t val = buffer[0] | (uint16_t)buffer[1] << 8;
    return lean_int_to_int(val);
  } else {
    fprintf(
      stderr,
      "ERROR: failed to read two bytes in lean_rx_int16_as_int, instead got %d",
      num_bytes);
    exit(1);
  }
}

