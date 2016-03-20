#include <stdio.h>
#include <signal.h>
#include <stdbool.h>
#include "u401.h"

static bool end = false;

void int_handler(int sig)
{
	end = true;
}

void gpio_isr(int id)
{
	printf("irq raised!\n");
}

int main(int argc, char **argv)
{
	int ret;
	signal(SIGINT, int_handler);
	ret = u401_attach();
	if (ret) {
		printf("failed to attach\n");
		return ret;
	}

	ret = u401_gpio_setup(0, GPIO_INPUT);
	if (ret) {
		printf("failed to setup gpio0\n");
		return ret;
	}

	ret = u401_gpio_request_irq(0, IRQ_FALLING, gpio_isr);
	if (ret) {
		printf("failed to request irq\n");
		u401_detach();
		return ret;
	}
	while(!end) {
		sleep(3);
		printf("I am alive\n");
	}
	u401_detach();
	return 0;
}
