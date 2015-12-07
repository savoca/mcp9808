#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>

void convert_to_temp(unsigned short temp);

int main(int argc, const char **argv)
{
	int i, fd;
	unsigned char buf[2];
	unsigned short temp;

	if (argc != 2)
		return -1;

	fd = open(argv[1], O_RDONLY);
	if (fd < 0) {
		printf("failed to open %s\n", argv[1]);
		return -1;
	}

	for (;;) {
		read(fd, buf, sizeof(buf));
		temp = (buf[0] << 8) | buf[1];
		convert_to_temp(temp);
		memset(buf, 0x00, sizeof(buf));
		sleep(1);
	}

	close(fd);
	return 0;
}

void convert_to_temp(unsigned short temp)
{
	char base;
	float frac, out;

	base = ((temp & 0xf00) + (temp & 0xf0)) >> 4;
	frac = (temp & 0xf) / 16.0;

	if (temp & 0x1000)
		base = (0xff - base) + 1;

	out = base + frac;

	printf("c: %f\n", out);
	printf("f: %f\n\n", (out * 1.8) + 32);
}
