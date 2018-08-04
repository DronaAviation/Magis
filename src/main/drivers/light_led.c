/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "light_led.h"

initLeds(void)
{
    struct {
        GPIO_TypeDef *gpio;
        gpio_config_t cfg;
    } gpio_setup[] = {
#ifdef LED_M
            {
                .gpio = LED0_GPIO,
                .cfg = {LED0_PIN, Mode_Out_PP, Speed_2MHz}
            },
#endif
#ifdef LED_R

            {
                .gpio = LED1_GPIO,
                .cfg = {LED1_PIN, Mode_Out_PP, Speed_2MHz}
            },
#endif
        }

    uint8_t gpio_count = sizeof(gpio_setup) / sizeof(gpio_setup[0]);

}
