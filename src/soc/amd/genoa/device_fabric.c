/* SPDX-License-Identifier: GPL-2.0-only */

#include <console/console.h>
#include <device/pci_ops.h>
#include <device/device.h>
#include <stdlib.h>

union df_cfg_base_address {
	struct {
		uint32_t re : 1;
		uint32_t we : 1;
		uint32_t _reserved : 6;
		uint32_t segment_num : 8;
		uint32_t bus_num_base : 8;
		uint32_t _reserver2: 8;
	};
	uint32_t raw;
};

_Static_assert(sizeof(union df_cfg_base_address) == sizeof(uint32_t));

union df_cfg_limit_address {
	struct {
		uint32_t dst_fabric_id : 12;
		uint32_t _reserved : 4;
		uint32_t bus_num_limit : 8;
		uint32_t _reserver2: 8;
	};
	uint32_t raw;
};

_Static_assert(sizeof(union df_cfg_limit_address) == sizeof(uint32_t));

/* D18F0 - Fabric Configuration PCI bus registers */
#define D18F0_CFG_BASE_ADDRESS 0xc80
#define D18F0_CFG_LIMIT_ADDRESS 0xc84

#define NB_CFG_BASE(reg) (D18F0_CFG_BASE_ADDRESS + 8 * (reg))
#define NB_CFG_LIMIT(reg) (D18F0_CFG_LIMIT_ADDRESS + 8 * (reg))

/* OpenSIL has no API to output what resources have been allocated so simply read back registers */
static void genoa_domain_scan_bus(struct device *dev)
{
	uint32_t bus;
	int i;
	/* TODO check if this also works on multi CPU setups */
	for (i = 0; i < 16; i++) {
		union df_cfg_base_address base = { .raw = pci_read_config32(pcidev_on_root(0x18, 0), NB_CFG_BASE(i)) };
		union df_cfg_limit_address limit = { .raw = pci_read_config32(pcidev_on_root(0x18, 0), NB_CFG_LIMIT(i)) };
		if (limit.dst_fabric_id == dev->path.domain.domain &&
		    base.we == 1 && base.re == 1) {
			printk(BIOS_DEBUG, "DestinationID 0x%x, Segment %d, BusBase %d, BusLimit %d WE %d, RE %d\n",
		       limit.dst_fabric_id, base.segment_num, base.bus_num_base, limit.bus_num_limit,
		       base.we, base.re);
			bus = base.bus_num_base;
			break;
		}
	}
	if (i == 16) {
		printk(BIOS_ERR, "Failed to find right bus cfg pair\n");
		return;
	}

	// Set bus number
	dev->link_list->secondary = bus;
	dev->link_list->subordinate = bus;

	pci_domain_scan_bus(dev);
}

union df_x86_io_base_address {
	struct {
		uint32_t re : 1;
		uint32_t we : 1;
		uint32_t _reserved : 3;
		uint32_t ie : 1;
		uint32_t _reserved2 : 10;
		uint32_t io_base: 13;
		uint32_t _reserved3 : 3;
	};
	uint32_t raw;
};
_Static_assert(sizeof(union df_x86_io_base_address) == sizeof(uint32_t));

union df_x86_io_limit_address {
	struct {
		uint32_t dst_fabric_id : 12;
		uint32_t _reserved : 4;
		uint32_t io_limit : 13;
		uint32_t _reserved2: 3;
	};
	uint32_t raw;
};
_Static_assert(sizeof(union df_x86_io_limit_address) == sizeof(uint32_t));

union df_mmio_address_ctl {
	struct {
		uint32_t re : 1;
		uint32_t we : 1;
		uint32_t cpu_dis : 1;
		uint32_t np : 1;
		uint32_t _reserved : 12;
		uint32_t dst_fabric_id: 12;
		uint32_t _reserved2 : 4;
	};
	uint32_t raw;
};
_Static_assert(sizeof(union df_mmio_address_ctl) == sizeof(uint32_t));

union df_mmio_ext_address {
	struct {
		uint32_t base_addr_ext : 8;
		uint32_t _reserved : 8;
		uint32_t limit_addr_ext : 8;
		uint32_t _reserved2 : 8;
	};
	uint32_t raw;
};
_Static_assert(sizeof(union df_mmio_ext_address) == sizeof(uint32_t));

/* D18F0 - Fabric Configuration PCI bus registers */
#define D18F0_X86_IO_BASE_ADDRESS 0xd00
#define D18F0_X86_IO_LIMIT_ADDRESS 0xd04

#define NB_IO_BASE(reg)  (D18F0_X86_IO_BASE_ADDRESS+ 8 * (reg))
#define NB_IO_LIMIT(reg) (D18F0_X86_IO_LIMIT_ADDRESS + 8 * (reg))

#define D18F0_MMIO_BASE_ADDRESS 0xd80
#define D18F0_MMIO_LIMIT_ADDRESS 0xd84
#define D18F0_MMIO_ADDRESS_CTL 0xd88
#define D18F0_MMIO_ADDRESS_EXT 0xd8c

#define NB_MMIO_BASE(reg)  (D18F0_MMIO_BASE_ADDRESS + 16 * (reg))
#define NB_MMIO_LIMIT(reg) (D18F0_MMIO_LIMIT_ADDRESS + 16 * (reg))
#define NB_MMIO_CTL(reg)  (D18F0_MMIO_ADDRESS_CTL + 16 * (reg))
#define NB_MMIO_EXT(reg) (D18F0_MMIO_ADDRESS_EXT + 16 * (reg))


static void genoa_domain_read_resources(struct device *dev)
{
	// IO
	int io_index = 0;
	for (int i = 0; i < 16; i++) {
		union df_x86_io_base_address base_reg = { .raw = pci_read_config32(pcidev_on_root(0x18, 0), NB_IO_BASE(i)) };
		union df_x86_io_limit_address limit_reg = { .raw = pci_read_config32(pcidev_on_root(0x18, 0), NB_IO_LIMIT(i)) };
		if (limit_reg.dst_fabric_id == dev->path.domain.domain &&
		    base_reg.we == 1 && base_reg.re == 1) {
			uint32_t base = base_reg.io_base << 12;
			uint32_t limit = (limit_reg.io_limit << 12) + 0xfff;
			printk(BIOS_DEBUG, "DestinationID 0x%x, io_base 0x%x, io_limit 0x%x WE %d, RE %d\n",
			       limit_reg.dst_fabric_id, base, limit,
			       base_reg.we, base_reg.re);
			struct resource *res;
			res = new_resource(dev, IOINDEX_SUBTRACTIVE(0, io_index++));
			res->base = base;
			res->limit = limit;
			res->flags = IORESOURCE_IO | IORESOURCE_SUBTRACTIVE | IORESOURCE_ASSIGNED;
		}
	}
	// MEM
	int mmio_index = 0;
	for (int i = 0; i < 16; i++) {
		uint64_t base = (uint64_t)pci_read_config32(pcidev_on_root(0x18, 0), NB_MMIO_BASE(i)) << 16;
		uint64_t limit = ((uint64_t)pci_read_config32(pcidev_on_root(0x18, 0), NB_MMIO_LIMIT(i)) << 16) + 0xffff;
		union df_mmio_address_ctl ctl = { .raw = pci_read_config32(pcidev_on_root(0x18, 0), NB_MMIO_CTL(i)) };
		union df_mmio_ext_address ext = { .raw = pci_read_config32(pcidev_on_root(0x18, 0), NB_MMIO_EXT(i)) };
		base |= (uint64_t)ext.base_addr_ext << 48;
		limit |= (uint64_t)ext.limit_addr_ext << 48;
		if (ctl.dst_fabric_id == dev->path.domain.domain &&
		    ctl.we == 1 && ctl.re == 1) {

			printk(BIOS_DEBUG, "DestinationID 0x%x, mmio_base %llx, mmio_limit %llx WE %d, RE %d\n",
			       ctl.dst_fabric_id, base, limit,
			       ctl.we, ctl.re);
			struct resource *res;
			res = new_resource(dev, IOINDEX_SUBTRACTIVE(1, mmio_index++));
			res->base = base;
			res->limit = limit;
			res->flags = IORESOURCE_MEM | IORESOURCE_SUBTRACTIVE | IORESOURCE_ASSIGNED;
		}
	}
}


struct device_operations genoa_pci_domain_ops = {
	.read_resources	  = genoa_domain_read_resources,
	.set_resources	  = pci_domain_set_resources,
	.scan_bus	  = genoa_domain_scan_bus,
#if CONFIG(HAVE_ACPI_TABLES)
	.acpi_name	  = soc_acpi_name,
#endif
};
