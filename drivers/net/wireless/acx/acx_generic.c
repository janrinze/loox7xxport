#include <linux/version.h>

#include <linux/vmalloc.h>

//
#include "acx.h"
#include "acx_generic.h"

inline void write_reg32(acx_device_t *adev, unsigned int offset, u32 val)
{
    adev->ops->write_reg32(adev, offset, val);
}

inline u32 read_reg32(acx_device_t *adev, unsigned int offset)
{
    return adev->ops->read_reg32(adev, offset);
}

inline void set_regbits (acx_device_t *adev, unsigned int offset, u32 bits)
{
    adev->ops->set_regbits(adev, offset, bits);
}

inline void clear_regbits (acx_device_t *adev, unsigned int offset, u32 bits)
{
    adev->ops->clear_regbits(adev, offset, bits);
}

inline void write_flush(acx_device_t *adev)
{
    adev->ops->write_flush(adev);
}

inline u16 read_reg16(acx_device_t *adev, unsigned int offset)
{
    return adev->ops->read_reg16(adev, offset);
}

inline void write_reg16(acx_device_t *adev, unsigned int offset, u16 val)
{
    return adev->ops->write_reg16(adev, offset, val);
}

#define NO_AUTO_INCREMENT	1

void
acx_l_reset_mac(acx_device_t *adev)
{
  int count;
	FN_ENTER;

	/* halt eCPU */
	set_regbits (adev, IO_ACX_ECPU_CTRL, 0x1);

	/* now do soft reset of eCPU, set bit */
	set_regbits (adev, IO_ACX_SOFT_RESET, 0x1);
	log(L_DEBUG, "%s: enable soft reset...\n", __func__);

	/* Windows driver sleeps here for a while with this sequence */
	for (count = 0; count < 200; count++) {
	  udelay (50);
	}

	/* now clear bit again: deassert eCPU reset */
	log(L_DEBUG, "%s: disable soft reset and go to init mode...\n", __func__);
	clear_regbits (adev, IO_ACX_SOFT_RESET, 0x1);

	/* now start a burst read from initial EEPROM */
	set_regbits (adev, IO_ACX_EE_START, 0x1);

	/*
	 * Windows driver sleeps here for a while with this sequence
	 */
	for (count = 0; count < 200; count++) {
	  udelay (50);
	}

	/* Windows driver writes 0x10000 to register 0x808 here */
	
	write_reg32 (adev, 0x808, 0x10000);

	FN_EXIT0;
}

/***********************************************************************
** acxpci_s_validate_fw
**
** Compare the firmware image given with
** the firmware image written into the card.
**
** Arguments:
**	adev		wlan device structure
**   fw_image  firmware image.
**
** Returns:
**	NOT_OK	firmware image corrupted or not correctly written
**	OK	success
**
** Origin: Standard csum + Read IO
*/
static int acx_s_validate_fw(acx_device_t * adev, const firmware_image_t *fw_image,
		     u32 offset)
{
	u32 sum, v32, w32;
	int len, size;
	int result = OK;
	/* we skip the first four bytes which contain the control sum */
	const u8 *p = (u8 *) fw_image + 4;

	FN_ENTER;

	/* start the image checksum by adding the image size value */
	sum = p[0] + p[1] + p[2] + p[3];
	p += 4;

	write_reg32(adev, IO_ACX_SLV_END_CTL, 0);

#if NO_AUTO_INCREMENT
	write_reg32(adev, IO_ACX_SLV_MEM_CTL, 0);	/* use basic mode */
#else
	write_reg32(adev, IO_ACX_SLV_MEM_CTL, 1);	/* use autoincrement mode */
	write_reg32(adev, IO_ACX_SLV_MEM_ADDR, offset);	/* configure start address */
#endif

	len = 0;
	size = le32_to_cpu(fw_image->size) & (~3);

	while (likely(len < size)) {
		v32 = be32_to_cpu(*(u32 *) p);
		p += 4;
		len += 4;

#if NO_AUTO_INCREMENT
		write_reg32(adev, IO_ACX_SLV_MEM_ADDR, offset + len - 4);
#endif
		w32 = read_reg32(adev, IO_ACX_SLV_MEM_DATA);

		if (unlikely(w32 != v32)) {
			printk("acx: FATAL: firmware upload: "
			       "data parts at offset %d don't match (0x%08X vs. 0x%08X)! "
			       "I/O timing issues or defective memory, with DWL-xx0+? "
			       "ACX_IO_WIDTH=16 may help. Please report\n",
			       len, v32, w32);
			result = NOT_OK;
			break;
		}

		sum +=
		    (u8) w32 + (u8) (w32 >> 8) + (u8) (w32 >> 16) +
		    (u8) (w32 >> 24);
	}

	/* sum control verification */
	if (result != NOT_OK) {
		if (sum != le32_to_cpu(fw_image->chksum)) {
			printk("acx: FATAL: firmware upload: "
			       "checksums don't match!\n");
			result = NOT_OK;
		}
	}

	FN_EXIT1(result);
	return result;
}

int acx_s_upload_fw(acx_device_t * adev)
{
	firmware_image_t *fw_image = NULL;
	int res = NOT_OK;
	int try;
	u32 file_size;
	char filename[sizeof("tiacx1NNcNN")];

	FN_ENTER;

	/* print exact chipset and radio ID to make sure people
	 * really get a clue on which files exactly they need to provide.
	 * Firmware loading is a frequent end-user PITA with these chipsets.
	 */
	printk( "acx: need firmware for acx1%02d chipset with radio ID %02X\n"
		"Please provide via firmware hotplug:\n"
		"either combined firmware (single file named 'tiacx1%02dc%02X')\n"
		"or two files (base firmware file 'tiacx1%02d' "
		"+ radio fw 'tiacx1%02dr%02X')\n",
		IS_ACX111(adev)*11, adev->radio_type,
		IS_ACX111(adev)*11, adev->radio_type,
		IS_ACX111(adev)*11,
		IS_ACX111(adev)*11, adev->radio_type
		);

	/* print exact chipset and radio ID to make sure people really get a clue on which files exactly they are supposed to provide,
	 * since firmware loading is the biggest enduser PITA with these chipsets.
	 * Not printing radio ID in 0xHEX in order to not confuse them into wrong file naming */
	printk(	"acx: need to load firmware for acx1%02d chipset with radio ID %02x, please provide via firmware hotplug:\n"
		"acx: either one file only (<c>ombined firmware image file, radio-specific) or two files (radio-less base image file *plus* separate <r>adio-specific extension file)\n",
		IS_ACX111(adev)*11, adev->radio_type);

	/* Try combined, then main image */
	adev->need_radio_fw = 0;
	snprintf(filename, sizeof(filename), "tiacx1%02dc%02X",
		 IS_ACX111(adev) * 11, adev->radio_type);

	fw_image = acx_s_read_fw(adev, filename, &file_size);
	if (!fw_image) {
		adev->need_radio_fw = 1;
		filename[sizeof("tiacx1NN") - 1] = '\0';
		fw_image =
		    acx_s_read_fw(adev, filename, &file_size);
		if (!fw_image) {
			FN_EXIT1(NOT_OK);
			return NOT_OK;
		}
	}

	for (try = 1; try <= 5; try++) {
		res = acx_s_write_fw(adev, fw_image, 0);
		log(L_DEBUG | L_INIT, "acx_write_fw (main/combined): %d\n", res);
		if (OK == res) {
			res = acx_s_validate_fw(adev, fw_image, 0);
			log(L_DEBUG | L_INIT, "acx_validate_fw "
			    		"(main/combined): %d\n", res);
		}

		if (OK == res) {
			SET_BIT(adev->dev_state_mask, ACX_STATE_FW_LOADED);
			break;
		}
		printk("acx: firmware upload attempt #%d FAILED, "
		       "retrying...\n", try);
		acx_s_mwait(1000);	/* better wait for a while... */
	}

	vfree(fw_image);

	FN_EXIT1(res);
	return res;
}

/***********************************************************************
** acxpci_s_write_fw
**
** Write the firmware image into the card.
**
** Arguments:
**	adev		wlan device structure
**	fw_image	firmware image.
**
** Returns:
**	1	firmware image corrupted
**	0	success
**
** Standard csum implementation + write to IO
*/
static int acx_s_write_fw(acx_device_t * adev, const firmware_image_t *fw_image,
		  u32 offset)
{
	int len, size;
	u32 sum, v32;
	/* we skip the first four bytes which contain the control sum */

	const u8 *p = (u8 *) fw_image + 4;

	FN_ENTER;

	/* start the image checksum by adding the image size value */
	sum = p[0] + p[1] + p[2] + p[3];
	p += 4;

	write_reg32(adev, IO_ACX_SLV_END_CTL, 0);

#if NO_AUTO_INCREMENT
	write_reg32(adev, IO_ACX_SLV_MEM_CTL, 0);	/* use basic mode */
#else
	write_reg32(adev, IO_ACX_SLV_MEM_CTL, 1);	/* use autoincrement mode */
	write_reg32(adev, IO_ACX_SLV_MEM_ADDR, offset);	/* configure start address */
	write_flush(adev);
#endif

	len = 0;
	size = le32_to_cpu(fw_image->size) & (~3);

	while (likely(len < size)) {
		v32 = be32_to_cpu(*(u32 *) p);
		sum += p[0] + p[1] + p[2] + p[3];
		p += 4;
		len += 4;

#if NO_AUTO_INCREMENT
		write_reg32(adev, IO_ACX_SLV_MEM_ADDR, offset + len - 4);
		write_flush(adev);
#endif
		write_reg32(adev, IO_ACX_SLV_MEM_DATA, v32);
	}

	log(L_DEBUG, "firmware written, size:%d sum1:%x sum2:%x\n",
	    size, sum, le32_to_cpu(fw_image->chksum));

	/* compare our checksum with the stored image checksum */
	FN_EXIT1(sum != le32_to_cpu(fw_image->chksum));
	return (sum != le32_to_cpu(fw_image->chksum));
}

/***********************************************************************
** acxpci_s_verify_init
*/
int acx_s_verify_init(acx_device_t * adev)
{
	int result = NOT_OK;
	unsigned long timeout;

	FN_ENTER;

	timeout = jiffies + 2 * HZ;
	for (;;) {
		u16 irqstat = read_reg16(adev, IO_ACX_IRQ_STATUS_NON_DES);
		if (irqstat & HOST_INT_FCS_THRESHOLD) {
			result = OK;
			write_reg16(adev, IO_ACX_IRQ_ACK,
				    HOST_INT_FCS_THRESHOLD);
			break;
		}
		if (time_after(jiffies, timeout))
			break;
		/* Init may take up to ~0.5 sec total */
		acx_s_mwait(50);
	}

	FN_EXIT1(result);
	return result;
}

