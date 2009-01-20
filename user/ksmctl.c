#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "../kernel/ksm/ksm.h"

int main(int argc, char *argv[])
{
	int fd;
	int used = 0;
	int fd_start;
	struct ksm_kthread_info info;
	

	if (argc < 2) {
		fprintf(stderr, "usage: %s {start npages sleep | stop | info}\n", argv[0]);
		exit(1);
	}

	fd = open("/dev/ksm", O_RDWR | O_TRUNC, (mode_t)0600);
	if (fd == -1) {
		fprintf(stderr, "could not open /dev/ksm\n");
		exit(1);
	}

	if (!strncmp(argv[1], "start", strlen(argv[1]))) {
		used = 1;
		if (argc < 4) {
			fprintf(stderr,
		    "usage: %s start npages_to_scan sleep\n",
		    argv[0]);
			exit(1);
		}
		info.pages_to_scan = atoi(argv[2]);
		info.sleep = atoi(argv[3]);
		info.flags = ksm_control_flags_run;

		fd_start = ioctl(fd, KSM_START_STOP_KTHREAD, &info);
		if (fd_start == -1) {
			fprintf(stderr, "KSM_START_KTHREAD failed\n");
			exit(1);
		}
		printf("created scanner\n");
	}

	if (!strncmp(argv[1], "stop", strlen(argv[1]))) {
		used = 1;
		info.flags = 0;
		fd_start = ioctl(fd, KSM_START_STOP_KTHREAD, &info);
		printf("stopped scanner\n");
	}

	if (!strncmp(argv[1], "info", strlen(argv[1]))) {
		used = 1;
		ioctl(fd, KSM_GET_INFO_KTHREAD, &info);
	 printf("flags %d, pages_to_scan %d, sleep_time %d\n",
	 info.flags, info.pages_to_scan, info.sleep);
	}

	if (!used)
		fprintf(stderr, "unknown command %s\n", argv[1]);

	return 0;
}
