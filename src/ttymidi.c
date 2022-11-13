/*
    This file is part of ttymidi.

    ttymidi is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ttymidi is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ttymidi.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <argp.h>
#include <alsa/asoundlib.h>
#include <assert.h>
// Linux-specific
#include <linux/serial.h>
#include <termios.h>
#include <sys/ioctl.h>
#include "baud.h"


#define MAX_DEV_STR_LEN               32
#define MAX_MSG_SIZE                1024
#define MAX_BUF_SIZE                1024

/* --------------------------------------------------------------------- */
// Program options

static struct argp_option options[] =
{
	{"serialdevice" , 's', "DEV" , 0, "Serial device to use. Default = /dev/ttyUSB0" },
	{"baudrate"     , 'b', "BAUD", 0, "Serial port baud rate. Default = 115200" },
	{"verbose"      , 'v', 0     , 0, "For debugging: Produce verbose output" },
	{"printonly"    , 'p', 0     , 0, "Super debugging: Print values read from serial -- and do nothing else" },
	{"quiet"        , 'q', 0     , 0, "Don't produce any output, even when the print command is sent" },
	{"name"		, 'n', "NAME", 0, "Name of the Alsa MIDI client. Default = ttymidi" },
	{ 0 }
};

typedef struct _arguments
{
	int  silent, verbose, printonly;
	char serialdevice[MAX_DEV_STR_LEN];
	int  baudrate;
	char name[MAX_DEV_STR_LEN];
} arguments_t;

static error_t parse_opt (int key, char *arg, struct argp_state *state)
{
	/* Get the input argument from argp_parse, which we
	   know is a pointer to our arguments structure. */
	arguments_t *arguments = state->input;
	int baud_temp;

	switch (key)
	{
		case 'p':
			arguments->printonly = 1;
			break;
		case 'q':
			arguments->silent = 1;
			break;
		case 'v':
			arguments->verbose = 1;
			break;
		case 's':
			if (arg == NULL) break;
			strncpy(arguments->serialdevice, arg, MAX_DEV_STR_LEN);
			break;
		case 'n':
			if (arg == NULL) break;
			strncpy(arguments->name, arg, MAX_DEV_STR_LEN);
			break;
		case 'b':
			if (arg == NULL) break;
			baud_temp = strtol(arg, NULL, 0);
			if (baud_temp != EINVAL && baud_temp != ERANGE)
                arguments->baudrate = baud_temp;

		case ARGP_KEY_ARG:
		case ARGP_KEY_END:
			break;

		default:
			return ARGP_ERR_UNKNOWN;
	}

	return 0;
}

void arg_set_defaults(arguments_t *arguments)
{
	char *serialdevice_temp = "/dev/ttyUSB0";
	arguments->printonly    = 0;
	arguments->silent       = 0;
	arguments->verbose      = 0;
	arguments->baudrate     = 31250;
	char *name_tmp		= (char *)"ttymidi";
	strncpy(arguments->serialdevice, serialdevice_temp, MAX_DEV_STR_LEN);
	strncpy(arguments->name, name_tmp, MAX_DEV_STR_LEN);
}

const char *argp_program_version     = "ttymidi 0.60";
const char *argp_program_bug_address = "tvst@hotmail.com";
static char doc[]       = "ttymidi - Connect serial port devices to ALSA MIDI programs!";
static struct argp argp = { options, parse_opt, 0, doc };
arguments_t arguments;


ssize_t serial_read(int fd, unsigned char *buf, size_t count) {
    int i;
    for(i = 0; i < count; i++) {
        int n = read(fd, &buf[i], 1);
        if(n == 0) {
            printf("EOF\n");
            abort();
        }
        if(n < 0) {
            printf("Read error: %s\n", strerror(errno));
            abort();
        }
    }
    return count;
}

/* --------------------------------------------------------------------- */
// MIDI stuff

int open_seq(snd_seq_t** seq) 
{
    int port_out_id = -1;
	if (snd_seq_open(seq, "default", SND_SEQ_OPEN_DUPLEX, 0) < 0)
	{
		fprintf(stderr, "Error opening ALSA sequencer.\n");
		exit(1);
	}

	snd_seq_set_client_name(*seq, arguments.name);

	if ((port_out_id = snd_seq_create_simple_port(*seq, "MIDI out",
					SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ,
					SND_SEQ_PORT_TYPE_APPLICATION)) < 0) 
	{
		fprintf(stderr, "Error creating sequencer port.\n");
	}

	return port_out_id;
}

void init_event(snd_seq_event_t *ev, int port_out_id) {
    snd_seq_ev_clear(ev);
    snd_seq_ev_set_direct(ev);
    snd_seq_ev_set_source(ev, port_out_id);
    snd_seq_ev_set_subs(ev);
}

int stop;

void read_midi_from_serial_port(int serial, snd_seq_t* seq, int port_out_id)
{
	unsigned char buf[MAX_BUF_SIZE];
    unsigned char status = 0;
    int sync = 0;
	int i;

	for(;;){
        // sync to first status byte
        if (sync == 0) {
            for(;;){
                serial_read(serial, &status, 1);
                if (status >> 7 != 0 && status != 0xF7) {
                    break;
                }
            }
            sync = 1;
        }
        else {
            serial_read(serial, &status, 1);
        }
        snd_seq_event_t ev;
        init_event(&ev, port_out_id);
        unsigned char channel = status & 0x0F;
        int tmp, ret;
        switch (status & 0xF0) {
            case 0x80:
                serial_read(serial, buf, 2);
                printf("%02x %02x %02x\n", status, buf[0], buf[1]);
                ret = snd_seq_ev_set_noteoff(&ev, channel, buf[0], buf[1]);
                break;

            case 0x90:
                serial_read(serial, buf, 2);
                printf("%02x %02x %02x\n", status, buf[0], buf[1]);
                snd_seq_ev_set_noteon(&ev, channel, buf[0], buf[1]);
                break;

            case 0xA0:
                serial_read(serial, buf, 2);
                printf("%02x %02x %02x\n", status, buf[0], buf[1]);
                snd_seq_ev_set_keypress(&ev, channel, buf[0], buf[1]);
                break;

            case 0xB0:
                serial_read(serial, buf, 2);
                printf("%02x %02x %02x\n", status, buf[0], buf[1]);
                snd_seq_ev_set_controller(&ev, channel, buf[0], buf[1]);
                break;

            case 0xC0:
                serial_read(serial, buf, 1);
                printf("%02x %02x\n", status, buf[0]);
                snd_seq_ev_set_pgmchange(&ev, channel, buf[0]);
                break;

            case 0xD0:
                serial_read(serial, buf, 1);
                printf("%02x %02x\n", status, buf[0]);
                snd_seq_ev_set_chanpress(&ev, channel, buf[0]);
                break;

            case 0xE0:
                serial_read(serial, buf, 2);
                printf("%02x %02x %02x\n", status, buf[0], buf[1]);
                tmp = (((int)buf[0]) & 0x7F) + ((((int)buf[1]) & 0x7F) << 7);
                snd_seq_ev_set_pitchbend(&ev, channel, tmp - 8192); // in alsa MIDI we want signed int
                break;

            case 0xF0:
                buf[0] = 0xF0;
                printf("%02x", status);
                for(i = 1; i < MAX_BUF_SIZE; i++) {
                    read(serial, &buf[i], 1);
                    printf(" %02x", buf[i]);
                    if (buf[i] == 0xF7 && i > 1) {
                        break;
                    }
                }
                printf("\n");

                snd_seq_ev_set_sysex(&ev, i , buf);
                break;
            default:
                sync = 0;
                printf("0x%x Unknown MIDI cmd\n", status );
                break;

        }
        ret = snd_seq_event_output_direct(seq, &ev);
        assert(ret >= 0);
//        ret = snd_seq_drain_output(seq);
//        assert(ret >= 0);
        if (stop) {
            break;
        }
    }
}

/* --------------------------------------------------------------------- */
// Main program

int main(int argc, char** argv)
{
	struct serial_struct ser_info;
	snd_seq_t *seq;

	arg_set_defaults(&arguments);
	argp_parse(&argp, argc, argv, 0, 0, &arguments);

	/*
	 *  Open modem device for reading and not as controlling tty because we don't
	 *  want to get killed if linenoise sends CTRL-C.
	 */

	int serial = open(arguments.serialdevice, O_RDWR);

	if (serial < 0) 
	{
		perror(arguments.serialdevice); 
		exit(-1); 
	}

    struct termios tio;
    int ret = tcgetattr(serial, &tio);
    assert(ret == 0);
    cfmakeraw(&tio);
    ret = tcsetattr(serial, TCSANOW, &tio);
    assert(ret == 0);

	// Linux-specific: enable low latency mode (FTDI "nagling off")
	ret = ioctl(serial, TIOCGSERIAL, &ser_info);
    assert(ret == 0);
	ser_info.flags |= ASYNC_LOW_LATENCY;
	ret = ioctl(serial, TIOCSSERIAL, &ser_info);
    assert(ret == 0);

    ret = setbaud(serial, arguments.baudrate);
    assert(ret == 0);

    int port_out_id = open_seq(&seq);
    assert(port_out_id >= 0);
    read_midi_from_serial_port(serial, seq, port_out_id);

	printf("\ndone!\n");
}

