/*
 * Buttons driver for Axim X50/X51(v).
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 * Copyright (C) 2007 Pierre Gaufillet
 *
 */

#include <linux/input.h>
#include <linux/input_pda.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>
#include <asm/arch/pxa27x_keyboard.h>
#include <asm/arch/loox720-gpio.h>

/****************************************************************
 * Keyboard
 ****************************************************************/

static struct pxa27x_keyboard_platform_data loox720_kbd = {
    .nr_rows = 6,
    .nr_cols = 3,
    .keycodes = {
        {    /* row 0 */
            KEY_CAMERA,     // left side button
            KEY_UP,
            KEY_OK    // Rocker press
        },
        {    /* row 1 */
            _KEY_RECORD,
            KEY_DOWN,
            KEY_SCROLLDOWN    // Rocker down
        },
        {    /* row 2 */
            _KEY_CALENDAR,
            KEY_RIGHT,
            KEY_SCROLLUP    // Rocker up
        },
        {    /* row 3 */
            _KEY_APP3,    // Easy2Connect button
            KEY_LEFT,
            -1,
        },
        {    /* row 4 */
            _KEY_CONTACTS,
            KEY_ENTER,    // joypad center button
            -1
        },
        {    /* row 5 */
            _KEY_HOMEPAGE,
            -1,
            -1
        },
    },
};

static struct platform_device loox720_pxa_keyboard = {
        .name   = "pxa27x-keyboard",
        .id     = -1,
        .dev    =  {
                .platform_data  = &loox720_kbd,
        },
};

static int __devinit loox720_buttons_probe(struct platform_device *dev)
{
        platform_device_register(&loox720_pxa_keyboard);
        return 0;
}

static struct platform_driver loox720_buttons_driver = {
        .driver                = {
            .name       = "loox720-buttons",
        },
        .probe          = loox720_buttons_probe,
};

static int __init loox720_buttons_init(void)
{
        if (!machine_is_loox720())
                return -ENODEV;

        return platform_driver_register(&loox720_buttons_driver);
}

static void __exit loox720_buttons_exit(void)
{
        platform_driver_unregister(&loox720_buttons_driver);
}

module_init(loox720_buttons_init);
module_exit(loox720_buttons_exit);

MODULE_AUTHOR ("Pierre Gaufillet");
MODULE_DESCRIPTION ("Buttons support for FSC Loox 720");
MODULE_LICENSE ("GPL");
