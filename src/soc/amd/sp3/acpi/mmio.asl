/* SPDX-License-Identifier: GPL-2.0-only */

#include <soc/gpio.h>
#include <amdblocks/acpimmio_map.h>

Device (GPIO)
{
	Name (_HID, GPIO_DEVICE_NAME)
	Name (_CID, GPIO_DEVICE_NAME)
	Name (_UID, 0)
	Name (_DDN, GPIO_DEVICE_DESC)

	Method (_CRS, 0) {
		Local0 = ResourceTemplate() {
			Interrupt (
				ResourceConsumer,
				Level,
				ActiveLow,
				Shared, , , IRQR)
			{ 0 }
			Memory32Fixed (ReadWrite, ACPIMMIO_GPIO0_BASE, 0x400)
		}
		CreateDWordField (Local0, IRQR._INT, IRQN)
		If (PICM) {
			IRQN = IGPI
		} Else {
			IRQN = PGPI
		}
		If (IRQN == 0x1f) {
			Return (ResourceTemplate() {
				Memory32Fixed (ReadWrite, ACPIMMIO_GPIO0_BASE, 0x400)
			})
		} Else {
			Return (Local0)
		}
	}

	Method (_STA, 0x0, NotSerialized)
	{
		Return (0x0F)
	}
}
