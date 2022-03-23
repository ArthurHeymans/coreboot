/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpigen.h>
#include <amdblocks/data_fabric.h>
#include <amdblocks/smn.h>
#include <arch/ioapic.h>
#include <console/console.h>
#include <device/pci_def.h>
#include <device/pci_ops.h>
#include <device/device.h>
#include <stdint.h>
#include <stdlib.h>

#include <vendorcode/amd/opensil/opensil.h>

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
	uint8_t bus, limit;
	int i;
	/* TODO check if this also works on multi CPU setups */
	for (i = 0; i < 16; i++) {
		union df_cfg_base_address cfg_base = { .raw = pci_read_config32(pcidev_on_root(0x18, 0), NB_CFG_BASE(i)) };
		union df_cfg_limit_address cfg_limit = { .raw = pci_read_config32(pcidev_on_root(0x18, 0), NB_CFG_LIMIT(i)) };
		if (cfg_limit.dst_fabric_id == dev->path.domain.domain &&
		    cfg_base.we == 1 && cfg_base.re == 1) {
			printk(BIOS_DEBUG, "DestinationID 0x%x, Segment %d, BusBase %d, BusLimit %d WE %d, RE %d\n",
		       cfg_limit.dst_fabric_id, cfg_base.segment_num, cfg_base.bus_num_base, cfg_limit.bus_num_limit,
		       cfg_base.we, cfg_base.re);
			bus = cfg_base.bus_num_base;
			limit = cfg_limit.bus_num_limit;
			break;
		}
	}
	if (i == 16) {
		printk(BIOS_ERR, "Failed to find right bus cfg pair\n");
		return;
	}

	// Set bus number
	dev->link_list->secondary = bus;
	/* secondary needs to be the same as subordinate before scan_bus. */
	dev->link_list->subordinate = bus;

	pci_domain_scan_bus(dev);

	/* Scan bus will modify subordinate, so change it back for ACPI generation. */
	dev->link_list->subordinate = limit;
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

static uint32_t get_iohcmisc_smnbase(const struct device *dev)
{
	switch (dev->path.domain.domain) {
	case 0x22: return 0x13c10000;
	case 0x23: return 0x13b10000;
	case 0x21: return 0x13e10000;
	case 0x20: return 0x13d10000;
	default:
		printk(BIOS_ERR, "Invalid IOHC device 0x%x\n", dev->path.domain.domain);
		return 0;
	}
}

static uint64_t iohc_read64(const uint32_t iohc_base, const uint32_t reg)
{
	return smn_read32(iohc_base | reg) | (uint64_t)smn_read32(iohc_base | (reg + 4));
}

#define IOHC_CCP_BASE_ADDR_LO 0x2d8
#define IOHC_CCP_BASE_ADDR_HI 0x2dc

#define IOHC_PSP_BASE_ADDR_LO 0x2e0
#define IOHC_PSP_BASE_ADDR_HI 0x2e4

#define IOHC_SMU_BASE_ADDR_LO 0x2e8
#define IOHC_SMU_BASE_ADDR_HI 0x2ec

#define IOHC_IOAPIC_BASE_ADDR_LO 0x2f0
#define IOHC_IOAPIC_BASE_ADDR_HI 0x2f4

#define IOHC_DBG_BASE_ADDR_LO 0x2f8
#define IOHC_DBG_BASE_ADDR_HI 0x2fc

#define IOHC_FASTREG_BASE_ADDR_LO 0x300
#define IOHC_FASTREG_BASE_ADDR_HI 0x304

#define IOHC_FASTREGCNTL_BASE_ADDR_LO 0x308
#define IOHC_FASTREGCNTL_BASE_ADDR_HI 0x30c


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

	int idx = 0;
	// We only want to add the DRAM memory map once
	if (dev->link_list->secondary == 0) {
		idx = add_opensil_memmap(dev, idx);
	}

	// non-PCI resources
	const struct non_pci_resources {
		const char *name;
		uint32_t smn_base;
		uint32_t idx;
		uint64_t mask;
		uint64_t size;
	} non_pci_res[] =
		{ { "CCP MMIO APERTURE", IOHC_CCP_BASE_ADDR_LO, 0, ~0xfffffull, 1 * MiB },
		  { "MP0 Public Mailbox", IOHC_PSP_BASE_ADDR_LO, 0, ~0xfffffull, 1 * MiB },
		  { "MP1 Public Mailbox", IOHC_SMU_BASE_ADDR_LO, 0, ~0xfffffull, 1 * MiB },
		  { "IOAPIC MMIO Registers", IOHC_IOAPIC_BASE_ADDR_LO, IOMMU_IOAPIC_IDX, ~0xffull, 4 * KiB },
		  { "Debug/UMC Registers", IOHC_DBG_BASE_ADDR_LO, 0, ~0xfffffull, 1 * MiB },
		  { "PM_MPDMA MMIO APERTURE", IOHC_FASTREG_BASE_ADDR_LO, 0, ~0xfffffull, 1 * MiB },
		  { "SMN MMIO Control Registers", IOHC_FASTREGCNTL_BASE_ADDR_LO, 0, ~0xfffffull, 4 * KiB }, };

	const uint32_t iohc = get_iohcmisc_smnbase(dev);
	for (int i = 0; i < ARRAY_SIZE(non_pci_res); i++) {
		const uint64_t reg64 = iohc_read64(iohc, non_pci_res[i].smn_base);
		printk(BIOS_DEBUG, "%s: %s %016llx\n", __func__, non_pci_res[i].name, reg64);
		const uint64_t base = reg64 & non_pci_res[i].mask;
		if (!base)
			continue;

		mmio_range(dev, non_pci_res[i].idx == 0 ? idx++ : non_pci_res[i].idx, base,
				   non_pci_res[i].size);
	}
}

#define D18F0_VGA_EN 0xc08

union df_vga_en {
	struct {
		uint32_t ve : 1;
		uint32_t np : 1;
		uint32_t cpu_dis : 1;
		uint32_t _reserved0 : 1;
		uint32_t dst_fabric_id : 12;
		uint32_t _reserved1 : 16;
	};
	uint32_t raw;
};
_Static_assert(sizeof(union df_vga_en) == sizeof(uint32_t),
	       "union df_vga_en: incorrect struct size ");

static void genoa_domain_set_resources(struct device *dev)
{
	if (dev->link_list->bridge_ctrl & PCI_BRIDGE_CTL_VGA) {
		printk(BIOS_DEBUG, "Setting VGA decoding for domain 0x%x\n", dev->path.domain.domain);
		const union df_vga_en vga_en = { .ve = 1, .dst_fabric_id = dev->path.domain.domain, };
		pci_write_config32(pcidev_on_root(0x18, 0), D18F0_VGA_EN, vga_en.raw);
	}

	pci_domain_set_resources(dev);

	/* Enable IOAPIC memory decoding */
	struct resource *res = probe_resource(dev, IOMMU_IOAPIC_IDX);
	if (res) {
		const uint32_t iohc = get_iohcmisc_smnbase(dev);
		uint32_t ioapic_base = smn_read32(iohc | IOHC_IOAPIC_BASE_ADDR_LO);
		ioapic_base |= (1 << 0);
		smn_write32(iohc | IOHC_IOAPIC_BASE_ADDR_LO, ioapic_base);
	}
}

static const char *df_acpi_name(const struct device *dev)
{
	const struct {
		uint8_t fabric_id;
		const char *acpi_name;
	} acpi_names[] = {
		{ 0x22, "S0B0"},
		{ 0x23, "S0B1"},
		{ 0x21, "S0B2"},
		{ 0x20, "S0B3"},
	};

	for (int i = 0; ARRAY_SIZE(acpi_names); i++)
		if (acpi_names[i].fabric_id == dev->path.domain.domain)
			return acpi_names[i].acpi_name;

	return NULL;
}

static void df_fill_ssdt(const struct device *dev)
{
	const char *acpi_scope = acpi_device_path(dev);
	printk(BIOS_DEBUG, "%s ACPI scope: '%s'\n", __FUNCTION__, acpi_scope);
	acpigen_write_scope(acpi_device_path(dev));

	acpigen_write_name("_CRS");
	acpigen_write_resourcetemplate_header();

	/* PCI busses */
	printk(BIOS_DEBUG, "%s _CRS: adding busses [%x-%x)\n", __FUNCTION__,
	       dev->link_list->secondary, dev->link_list->subordinate);
	acpigen_resource_word(RSRC_TYPE_BUS, /* res_type */
			      ADDR_SPACE_GENERAL_FLAG_MAX_FIXED
			      | ADDR_SPACE_GENERAL_FLAG_MIN_FIXED
			      | ADDR_SPACE_GENERAL_FLAG_DEC_POS, /* gen_flags */
			      BUS_NUM_RANGE_RESOURCE_FLAG, /* type_flags */
			      0, /* gran */
			      dev->link_list->secondary, /* range_min */
			      dev->link_list->subordinate, /* range_max */
			      0x0, /* translation */
			      dev->link_list->subordinate - dev->link_list->secondary + 1); /* length */

	if (dev->link_list->secondary == 0)
		/* ACPI 6.4.2.5 I/O Port Descriptor */
		acpigen_write_io16(0xCF8, 0xCFF, 0x1, 0x8, 1);


	struct resource *res;
	for (res = dev->resource_list; res != NULL; res = res->next) {
		if (!(res->flags & IORESOURCE_SUBTRACTIVE))
			continue;
		if (!(res->flags & IORESOURCE_ASSIGNED))
			continue;
		switch (res->flags & IORESOURCE_TYPE_MASK) {
		case IORESOURCE_IO:
			printk(BIOS_DEBUG, "%s _CRS: adding IO range [%llx-%llx)\n", __FUNCTION__,
			       res->base, res->limit);
			acpigen_resource_word(RSRC_TYPE_IO, /* res_type */
					      ADDR_SPACE_GENERAL_FLAG_MAX_FIXED
					      | ADDR_SPACE_GENERAL_FLAG_MIN_FIXED
					      | ADDR_SPACE_GENERAL_FLAG_DEC_POS, /* gen_flags */
					      IO_RSRC_FLAG_ENTIRE_RANGE, /* type_flags */
					      0, /* gran */
					      res->base, /* range_min */
					      res->limit, /* range_max */
					      0x0, /* translation */
					      res->limit - res->base + 1); /* length */
		        break;
		case IORESOURCE_MEM:
			printk(BIOS_DEBUG, "%s _CRS: adding MEM range [%llx-%llx)\n", __FUNCTION__,
			       res->base, res->limit);
			acpigen_resource_qword(RSRC_TYPE_MEM, /* res_type */
					      ADDR_SPACE_GENERAL_FLAG_MAX_FIXED
					      | ADDR_SPACE_GENERAL_FLAG_MIN_FIXED
					      | ADDR_SPACE_GENERAL_FLAG_DEC_POS, /* gen_flags */
					      MEM_RSRC_FLAG_MEM_READ_WRITE, /* type_flags */
					      0, /* gran */
					      res->base, /* range_min */
					      res->limit, /* range_max */
					      0x0, /* translation */
					      res->limit - res->base + 1); /* length */
			break;
		default:
			break;
		}
	}

	/* TODO deal with VGA resources */

	acpigen_write_resourcetemplate_footer();
	/* Scope */
	acpigen_pop_len();
}

static void genoa_domain_init(struct device *dev)
{
	struct resource *res = probe_resource(dev, IOMMU_IOAPIC_IDX);
	if (!res)
		return;

	register_new_ioapic((void *)(uintptr_t)res->base);
}

struct device_operations genoa_pci_domain_ops = {
	.read_resources	  = genoa_domain_read_resources,
	.set_resources	  = genoa_domain_set_resources,
	.scan_bus	  = genoa_domain_scan_bus,
	.init		  = genoa_domain_init,
#if CONFIG(HAVE_ACPI_TABLES)
	.acpi_name	  = df_acpi_name,
	.acpi_fill_ssdt   = df_fill_ssdt,
#endif
};
