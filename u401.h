/*
This file is part of the U401 library.

The U401 library is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

The U401 library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with The U401 library.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef U401_H
#define U401_H

#define U401_GPIOS 16

enum U401_IRQ_TRIGGER
{
	IRQ_FALLING = 0,
	IRQ_RISING,
	IRQ_RISING_FALLING,
};

enum U401_GPIO_DIR
{
	GPIO_INPUT = 0,
	GPIO_OUTPUT,
};

#ifdef __cplusplus
extern "C" {
#endif

int u401_attach(void);
void u401_detach(void);

int u401_gpio_set_value(int id, bool value);
int u401_gpio_get_value(int id, int *value);
int u401_gpio_setup(int id, bool dir);
int u401_gpio_request_irq(int id, int trigger, void (*isr)(int));

#ifdef __cplusplus
}
#endif

#endif
