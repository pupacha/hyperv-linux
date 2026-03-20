// SPDX-License-Identifier: GPL-2.0

/*
 * Hyper-V ARM64 IOMMU driver for L1VH (Level 1 Virtual Host).
 *
 * Copyright (C) 2024, Microsoft, Inc.
 *
 * This driver supports L1VH host and VMM scenarios using direct device
 * attachment model. It does not support IRQ remapping or S2 device domains
 * which are x86-specific features.
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interval_tree.h>
#include <linux/dma-map-ops.h>

#include <asm/hypervisor.h>
#include <asm/mshyperv.h>
#include "dma-iommu.h"

#ifdef CONFIG_HYPERV_IOMMU_ARM



/* The IOMMU will not claim these PCI devices. */
static char *pci_devs_to_skip;
static int __init hv_iommu_setup_skip(char *str)
{
	pci_devs_to_skip = str;
	return 0;
}
struct rid_data {
	struct pci_dev *bridge;
	u32 rid;
};
/* Build device id for direct attached devices */
static u64 hv_build_devid_type_logical(struct pci_dev *pdev)
{
	hv_pci_segment segment;
	union hv_device_id hv_devid;
	union hv_pci_bdf bdf = {.as_uint16 = 0};
	struct rid_data data = {
		.bridge = NULL,
		.rid = PCI_DEVID(pdev->bus->number, pdev->devfn)
	};

	segment = pci_domain_nr(pdev->bus);
	bdf.bus = PCI_BUS_NUM(data.rid);
	bdf.device = PCI_SLOT(data.rid);
	bdf.function = PCI_FUNC(data.rid);

	hv_devid.as_uint64 = 0;
	hv_devid.device_type = HV_DEVICE_TYPE_LOGICAL;
	hv_devid.logical.id = (u64)segment << 16 | bdf.as_uint16;

	return hv_devid.as_uint64;
}
static void hv_iommu_detach_dev(struct iommu_domain *immdom,
				struct device *dev);
static size_t hv_iommu_unmap_pages(struct iommu_domain *immdom, ulong iova,
				   size_t pgsize, size_t pgcount,
				   struct iommu_iotlb_gather *gather);

u64 hv_build_devid_oftype(struct pci_dev *pdev, enum hv_device_type type)
{

		if (hv_l1vh_partition())
			return hv_pci_vmbus_device_id(pdev);
		else
			return hv_build_devid_type_logical(pdev);

	return 0;
}

/* hv_iommu_skip=(SSSS:BB:DD.F)(SSSS:BB:DD.F) */
__setup("hv_iommu_skip=", hv_iommu_setup_skip);

/* iommu device that we export to the world. HyperV supports one device only */
static struct iommu_device hv_virt_iommu;

struct hv_domain {
	struct iommu_domain iommu_dom;
	u32 domid_num;			      /* domain id number */
	u32 num_attchd;		      /* number of currently attached devices */
	bool attached_dom;		      /* is this direct attached dom */
	spinlock_t mappings_lock;	      /* protects mappings_tree */
	struct rb_root_cached mappings_tree;  /* iova to pa interval tree */
};

#define to_hv_domain(d) container_of(d, struct hv_domain, iommu_dom)

bool hv_pcidev_is_attached_dev(struct pci_dev *pdev)
{
	struct iommu_domain *iommu_domain;
	struct hv_domain *hvdom;
	struct device *dev = &pdev->dev;

	iommu_domain = iommu_get_domain_for_dev(dev);
	if (iommu_domain) {
		hvdom = to_hv_domain(iommu_domain);
		return hvdom->attached_dom;
	}

	return false;
}
EXPORT_SYMBOL_GPL(hv_pcidev_is_attached_dev);

/*
 * For L1VH on ARM64, we only support direct device attachment model.
 * The hypervisor does not support S2 device domains on ARM64 L1VH.
 * All devices use direct attachment where the guest hardware page tables
 * are used directly by the hypervisor for IOMMU translation.
 */

/*
 * Create 2 dummy domains to correspond to hypervisor behavior:
 * default identity domain and null domain (for blocking).
 */
static struct hv_domain hv_def_identity_dom, hv_null_dom;

static bool hv_special_domain(struct hv_domain *hvdom)
{
	return hvdom == &hv_def_identity_dom || hvdom == &hv_null_dom;
}

struct iommu_domain_geometry default_geometry = (struct iommu_domain_geometry) {
	.aperture_start = 0,
	.aperture_end = -1UL,
	.force_aperture = true,
};

static u32 unique_id;	      /* unique numeric id of a new domain */

/*
 * Page sizes for ARM64 - we support 4K and 2M (large page) mappings.
 * This matches the typical ARM64 page table capabilities.
 */
#define HV_IOMMU_PGSIZES (SZ_4K | SZ_2M)

struct hv_iommu_mapping {
	phys_addr_t paddr;
	struct interval_tree_node iova;
	u32 flags;
};

/*
 * If the current thread is a VMM thread, return the partition id of the vm it
 * is managing, otherwise return HV_PARTITION_ID_INVALID.
 */
u64 hv_iommu_get_curr_partid(void)
{
	u64 (*fn)(pid_t pid);
	u64 partid;

	fn = symbol_get(mshv_pid_to_partid);
	if (!fn)
		return HV_PARTITION_ID_INVALID;

	partid = fn(current->tgid);
	symbol_put(mshv_pid_to_partid);

	return partid;
}

/* If this is a VMM thread, then this domain is for a guest vm */
static bool hv_curr_thread_is_vmm(void)
{
	return hv_iommu_get_curr_partid() != HV_PARTITION_ID_INVALID;
}

/* Create a new device domain in the hypervisor */
static int hv_iommu_create_hyp_devdom(struct hv_domain *hvdom)
{
	u64 status;
	unsigned long flags;
	struct hv_input_device_domain *ddp;
	struct hv_input_create_device_domain *input;

	local_irq_save(flags);

	input = *this_cpu_ptr(hyperv_pcpu_input_arg);
	memset(input, 0, sizeof(*input));

	ddp = &input->device_domain;
	ddp->partition_id = HV_PARTITION_ID_SELF;
	ddp->domain_id.type = HV_DEVICE_DOMAIN_TYPE_S2;
	ddp->domain_id.id = hvdom->domid_num;

	input->create_device_domain_flags.forward_progress_required = 1;
	input->create_device_domain_flags.inherit_owning_vtl = 0;

	status = hv_do_hypercall(HVCALL_CREATE_DEVICE_DOMAIN, input, NULL);

	local_irq_restore(flags);

	if (!hv_result_success(status))
		pr_err("%s: hypercall failed, status 0x%llx\n", __func__,
		       status);

	return hv_result_to_errno(status);
}


static bool hv_iommu_capable(struct device *dev, enum iommu_cap cap)
{
	switch (cap) {
	case IOMMU_CAP_CACHE_COHERENCY:
		return true;
	default:
		return false;
	}
}

static struct iommu_domain *hv_iommu_domain_alloc_identity(struct device *dev)
{
	return &hv_def_identity_dom.iommu_dom;
}

static struct iommu_domain *hv_iommu_domain_alloc_paging(struct device *dev)
{
	// pr_err("Hyper-V ARM64: hv_iommu_domain_alloc: type: %u \n", type);
	// msleep(500);
	struct hv_domain *hvdom;
	int rc;

	/*
	 * L1VH on ARM64 does not support host device passthrough (DPDK, etc.)
	 * unless we're in a VMM thread managing a guest or hv_no_attdev is set.
	 */
	if (hv_l1vh_partition() && !hv_curr_thread_is_vmm()) {
		pr_err("Hyper-V ARM64: l1vh iommu does not support host devices\n");
		return NULL;
	}

	hvdom = kzalloc(sizeof(struct hv_domain), GFP_KERNEL);
	if (hvdom == NULL)
		goto out;

	spin_lock_init(&hvdom->mappings_lock);
	hvdom->mappings_tree = RB_ROOT_CACHED;

	if (++unique_id == 0)   /* avoid 0, reserved for default */
		unique_id++;

	hvdom->domid_num = unique_id;
	hvdom->iommu_dom.geometry = default_geometry;
	hvdom->iommu_dom.pgsize_bitmap = HV_IOMMU_PGSIZES;

	/*
	 * For L1VH on ARM64, we always use direct attach model.
	 * VMM threads use this for guest device passthrough.
	 */
	if (hv_curr_thread_is_vmm() && !hv_no_attdev)
		hvdom->attached_dom = true;
	else {
		rc = hv_iommu_create_hyp_devdom(hvdom);
		if (rc)
			goto out_free_id;
	}

	// hvdom->attached_dom = true; /* L1VH always uses direct attach */

	// pr_err("Hyper-V ARM64: hv_iommu_domain_alloc [DONE]: type: %u, hvdom->domid_num:%u \n", type, hvdom->domid_num);
	// msleep(500);
	return &hvdom->iommu_dom;

out_free_id:
	unique_id--;
out_free:
	kfree(hvdom);
out:
	return NULL;
}

static void hv_iommu_domain_free(struct iommu_domain *immdom)
{
	struct hv_domain *hvdom = to_hv_domain(immdom);
	u64 status;
	unsigned long flags;
	struct hv_input_delete_device_domain *input;

	if (hv_special_domain(hvdom))
		return;

	if (hvdom->num_attchd) {
		pr_err("Hyper-V ARM64: can't free busy iommu domain (%p)\n",
		       immdom);
		return;
	}

	if (!hv_curr_thread_is_vmm() || hv_no_attdev) {
		struct hv_input_device_domain *ddp;

		local_irq_save(flags);
		input = *this_cpu_ptr(hyperv_pcpu_input_arg);
		ddp = &input->device_domain;
		memset(input, 0, sizeof(*input));

		ddp->partition_id = HV_PARTITION_ID_SELF;
		ddp->domain_id.type = HV_DEVICE_DOMAIN_TYPE_S2;
		ddp->domain_id.id = hvdom->domid_num;

		status = hv_do_hypercall(HVCALL_DELETE_DEVICE_DOMAIN, input,
					 NULL);
		local_irq_restore(flags);

		if (!hv_result_success(status))
			pr_err("%s: hypercall failed, status 0x%llx\n",
			       __func__, status);
	}

	/*
	 * For L1VH ARM64, no hypervisor-side domain cleanup needed
	 * since we use direct attachment model only.
	 */

	if (immdom->type == IOMMU_DOMAIN_DMA)
		iommu_put_dma_cookie(immdom);

	kfree(hvdom);
}

/*
 * Do an attach device hypercall for direct attachment.
 * This tells the hypervisor to use the guest hardware page tables
 * directly for this device's IOMMU translation.
 */
int hv_iommu_direct_attach_device(struct pci_dev *pdev)
{
	struct hv_input_attach_device *input;
	u64 status;
	int rc;
	unsigned long flags;
	union hv_device_id host_devid;
	u64 ptid = hv_iommu_get_curr_partid();
	enum hv_device_type dev_type;

	/*
	 * On ARM64 L1VH, devices are typically identified as LOGICAL type.
	 * The hypervisor assigns a logical ID to each device.
	 */
	dev_type = HV_DEVICE_TYPE_LOGICAL;
	host_devid.as_uint64 = hv_build_devid_oftype(pdev, dev_type);
	pr_err("Hyper-V ARM64: hv_iommu_direct_attach_device: pdev->vendor=%u, pdev->device=%u\n", pdev->vendor, pdev->device);
	pr_err("Hyper-V ARM64: hv_iommu_direct_attach_device: host_devid.as_uint64 = %llu\n", host_devid.as_uint64);
	pr_err("Hyper-V ARM64: hv_iommu_direct_attach_device: ptid = %llu\n", ptid);


	do {
		// pr_err("Hyper-V ARM64: hv_iommu_direct_attach_device: calling HVCALL_ATTACH_DEVICE\n");
		// msleep(200);
		local_irq_save(flags);
		input = *this_cpu_ptr(hyperv_pcpu_input_arg);
		memset(input, 0, sizeof(*input));
		input->partition_id = ptid;
		input->device_id = host_devid;

		/*
		 * Hypervisor associates logical_id with this device.
		 * For some hypercalls like retarget interrupts, logical_id
		 * must be used instead of the BDF.
		 */
		input->attdev_flags.logical_id = 1;
		input->logical_devid =
			   hv_build_devid_oftype(pdev, HV_DEVICE_TYPE_LOGICAL);

		status = hv_do_hypercall(HVCALL_ATTACH_DEVICE, input, NULL);
		local_irq_restore(flags);

		if (hv_result(status) == HV_STATUS_INSUFFICIENT_MEMORY) {
			rc = hv_call_deposit_pages(NUMA_NO_NODE, ptid, 1);
			if (rc)
				break;
		}
	} while (hv_result(status) == HV_STATUS_INSUFFICIENT_MEMORY);

	if (!hv_result_success(status))
		pr_err("%s: hypercall failed, status 0x%llx\n", __func__,
		       status);

	return hv_result_to_errno(status);
}
EXPORT_SYMBOL_GPL(hv_iommu_direct_attach_device);

/* Attach a device to a domain (L1VH uses direct attach only) */
static int hv_iommu_attach_dev(struct iommu_domain *immdom, struct device *dev,
			       struct iommu_domain *old)
{
	struct pci_dev *pdev;
	int rc, rc1;
	struct hv_domain *hvdom_new = to_hv_domain(immdom);
	struct hv_domain *hvdom_prev = dev_iommu_priv_get(dev);

	pr_err("Hyper-V ARM64: hv_iommu_attach_dev: dev->id=%u,  hvdom_new->domid_num=%u, hv_special_domain=%d \n", dev->id, hvdom_new->domid_num, hv_special_domain(hvdom_new));

	/* Only allow PCI devices for now */
	if (!dev_is_pci(dev))
		return -EINVAL;

	pdev = to_pci_dev(dev);

	/*
	 * L1VH on ARM64 does not support host device passthru (e.g., DPDK)
	 * unless it's a special domain or an attached domain.
	 */
	if (hv_l1vh_partition() && !hv_special_domain(hvdom_new) &&
	    !hvdom_new->attached_dom)
		return -EINVAL;

	/*
	 * Check if we need to detach from previous domain first.
	 * In case of guest shutdown, the VMM thread attaches it back to
	 * the hv_def_identity_dom, and hvdom_prev will not be null.
	 */
	if (hvdom_prev) {
		pr_err("Hyper-V ARM64: hv_iommu_attach_dev: hvdom_prev = true \n");
		if (!hv_l1vh_partition() || !hv_special_domain(hvdom_prev))
			hv_iommu_detach_dev(&hvdom_prev->iommu_dom, dev);
		else
			pr_err("Hyper-V ARM64: hv_iommu_attach_dev: no hv_iommu_detach_dev\n");
	}

	/* Special domains don't need actual attachment */
	if (hv_l1vh_partition() && hv_special_domain(hvdom_new)) {
		dev_iommu_priv_set(dev, hvdom_new);
		return 0;
	}

	pr_err("Hyper-V ARM64: hv_iommu_attach_dev: calling hv_iommu_direct_attach_device\n");
	msleep(100);

	/* For L1VH ARM64, always use direct attachment */
	rc = hv_iommu_direct_attach_device(pdev);

	pr_err("Hyper-V ARM64: hv_iommu_attach_dev: ret hv_iommu_direct_attach_device = %d\n", rc);
	msleep(100);
	/* Try to restore previous state on failure */
	if (rc && hvdom_prev) {
		rc1 = hv_iommu_direct_attach_device(pdev);
		if (rc1)
			pr_err("Hyper-V ARM64: iommu could not restore orig device "
			       "state.. dev:%s\n", dev_name(dev));
	}

	if (rc == 0) {
		dev_iommu_priv_set(dev, hvdom_new);
		hvdom_new->num_attchd++;
	}

	return rc;
}

static void hv_iommu_det_dev_from_guest(struct hv_domain *hvdom,
					struct pci_dev *pdev)
{
	struct hv_input_detach_device *input;
	u64 status, log_devid;
	unsigned long flags;

	log_devid = hv_build_devid_oftype(pdev, HV_DEVICE_TYPE_LOGICAL);

	local_irq_save(flags);
	input = *this_cpu_ptr(hyperv_pcpu_input_arg);
	memset(input, 0, sizeof(*input));

	input->partition_id = hv_iommu_get_curr_partid();
	input->logical_devid = log_devid;
	status = hv_do_hypercall(HVCALL_DETACH_DEVICE, input, NULL);
	local_irq_restore(flags);

	if (!hv_result_success(status))
		pr_err("%s: hypercall failed, status 0x%llx\n", __func__,
		       status);
}

static void hv_iommu_detach_dev(struct iommu_domain *immdom, struct device *dev)
{
	struct pci_dev *pdev;
	struct hv_domain *hvdom = to_hv_domain(immdom);

	/* Only PCI devices supported */
	if (!dev_is_pci(dev))
		return;

	if (hvdom->num_attchd == 0)
		pr_err("Hyper-V ARM64: num_attchd is zero (%s)\n", dev_name(dev));

	pdev = to_pci_dev(dev);

	/* For L1VH ARM64, detach from guest (direct attach model) */
	if (hvdom->attached_dom)
		hv_iommu_det_dev_from_guest(hvdom, pdev);

	hvdom->num_attchd--;
}

static int hv_iommu_add_tree_mapping(struct hv_domain *hvdom,
				     unsigned long iova, phys_addr_t paddr,
				     size_t size, u32 flags)
{
	unsigned long irqflags;
	struct hv_iommu_mapping *mapping;

	mapping = kzalloc(sizeof(*mapping), GFP_ATOMIC);
	if (!mapping)
		return -ENOMEM;

	mapping->paddr = paddr;
	mapping->iova.start = iova;
	mapping->iova.last = iova + size - 1;
	mapping->flags = flags;

	spin_lock_irqsave(&hvdom->mappings_lock, irqflags);
	interval_tree_insert(&mapping->iova, &hvdom->mappings_tree);
	spin_unlock_irqrestore(&hvdom->mappings_lock, irqflags);

	return 0;
}

static size_t hv_iommu_del_tree_mappings(struct hv_domain *hvdom,
					unsigned long iova, size_t size)
{
	unsigned long flags;
	size_t unmapped = 0;
	unsigned long last = iova + size - 1;
	struct hv_iommu_mapping *mapping = NULL;
	struct interval_tree_node *node, *next;

	spin_lock_irqsave(&hvdom->mappings_lock, flags);
	next = interval_tree_iter_first(&hvdom->mappings_tree, iova, last);
	while (next) {
		node = next;
		mapping = container_of(node, struct hv_iommu_mapping, iova);
		next = interval_tree_iter_next(node, iova, last);

		/* Trying to split a mapping? Not supported for now. */
		if (mapping->iova.start < iova)
			break;

		unmapped += mapping->iova.last - mapping->iova.start + 1;

		interval_tree_remove(node, &hvdom->mappings_tree);
		kfree(mapping);
	}
	spin_unlock_irqrestore(&hvdom->mappings_lock, flags);

	return unmapped;
}

/* Return: must return exact status from the hypercall without changes */
static u64 hv_iommu_map_pgs(struct hv_domain *hvdom,
			    unsigned long iova, phys_addr_t paddr,
			    unsigned long npages, u32 map_flags)
{
	u64 status;
	int i;
	struct hv_input_map_device_gpa_pages *input;
	unsigned long flags, pfn = paddr >> HV_HYP_PAGE_SHIFT;

	local_irq_save(flags);
	input = *this_cpu_ptr(hyperv_pcpu_input_arg);
	memset(input, 0, sizeof(*input));

	input->device_domain.partition_id = HV_PARTITION_ID_SELF;
	input->device_domain.domain_id.type = HV_DEVICE_DOMAIN_TYPE_S2;
	input->device_domain.domain_id.id = hvdom->domid_num;
	input->map_flags = map_flags;
	input->target_device_va_base = iova;

	pfn = paddr >> HV_HYP_PAGE_SHIFT;
	for (i = 0; i < npages; i++, pfn++)
		input->gpa_page_list[i] = pfn;

	status = hv_do_rep_hypercall(HVCALL_MAP_DEVICE_GPA_PAGES, npages, 0,
				     input, NULL);

	local_irq_restore(flags);
	return status;
}

/*
 * The core VFIO code loops over memory ranges calling this function with
 * the largest size from HV_IOMMU_PGSIZES. cond_resched() is in vfio_iommu_map.
 */
static int hv_iommu_map_pages(struct iommu_domain *immdom, ulong iova,
			      phys_addr_t paddr, size_t pgsize, size_t pgcount,
			      int prot, gfp_t gfp, size_t *mapped)
{
	u32 map_flags;
	int ret;
	u64 status;
	unsigned long npages, done = 0;
	struct hv_domain *hvdom = to_hv_domain(immdom);
	size_t size = pgsize * pgcount;

	map_flags = HV_MAP_GPA_READABLE;	/* required */
	map_flags |= prot & IOMMU_WRITE ? HV_MAP_GPA_WRITABLE : 0;

	ret = hv_iommu_add_tree_mapping(hvdom, iova, paddr, size, map_flags);
	if (ret)
		return ret;

	if (hvdom->attached_dom) {
		*mapped = size;
		return 0;
	}

	npages = size >> HV_HYP_PAGE_SHIFT;
	while (done < npages) {
		ulong completed, remain = npages - done;

		status = hv_iommu_map_pgs(hvdom, iova, paddr, remain,
					  map_flags);

		completed = hv_repcomp(status);
		done = done + completed;
		iova = iova + (completed << HV_HYP_PAGE_SHIFT);
		paddr = paddr + (completed << HV_HYP_PAGE_SHIFT);

		if (hv_result(status) == HV_STATUS_INSUFFICIENT_MEMORY) {
			ret = hv_call_deposit_pages(NUMA_NO_NODE,
						    hv_current_partition_id,
						    256);
			if (ret)
				break;
		}
		if (!hv_result_success(status))
			break;
	}

	if (!hv_result_success(status)) {
		size_t done_size = done << HV_HYP_PAGE_SHIFT;

		hv_status_err(status, "pgs:%lx/%lx iova:%lx\n",
			      done, npages, iova);
		/*
		 * lookup tree has all mappings [0 - size-1]. Below unmap will
		 * only remove from [0 - done], we need to remove second chunk
		 * [done+1 - size-1].
		 */
		hv_iommu_del_tree_mappings(hvdom, iova, size - done_size);
		hv_iommu_unmap_pages(immdom, iova - done_size, pgsize,
				     done, NULL);
		if (mapped)
			*mapped = 0;
	} else
		if (mapped)
			*mapped = size;

	return hv_result_to_errno(status);
}

static size_t hv_iommu_unmap_pages(struct iommu_domain *immdom, ulong iova,
				   size_t pgsize, size_t pgcount,
				   struct iommu_iotlb_gather *gather)
{
	unsigned long flags, npages;
	struct hv_input_unmap_device_gpa_pages *input;
	u64 status;
	struct hv_domain *hvdom = to_hv_domain(immdom);
	size_t unmapped, size = pgsize * pgcount;

	unmapped = hv_iommu_del_tree_mappings(hvdom, iova, size);
	if (unmapped < size)
		pr_err("%s: could not delete all mappings (%lx:%lx/%lx)\n",
		       __func__, iova, unmapped, size);

	if (hvdom->attached_dom)
		return size;

	npages = size >> HV_HYP_PAGE_SHIFT;

	local_irq_save(flags);
	input = *this_cpu_ptr(hyperv_pcpu_input_arg);
	memset(input, 0, sizeof(*input));

	input->device_domain.partition_id = HV_PARTITION_ID_SELF;
	input->device_domain.domain_id.type = HV_DEVICE_DOMAIN_TYPE_S2;
	input->device_domain.domain_id.id = hvdom->domid_num;
	input->target_device_va_base = iova;

	status = hv_do_rep_hypercall(HVCALL_UNMAP_DEVICE_GPA_PAGES, npages,
				     0, input, NULL);
	local_irq_restore(flags);

	if (!hv_result_success(status))
		hv_status_err(status, "\n");

	return unmapped;
}

static phys_addr_t hv_iommu_iova_to_phys(struct iommu_domain *immdom,
					 dma_addr_t iova)
{
	u64 paddr = 0;
	unsigned long flags;
	struct hv_iommu_mapping *mapping;
	struct interval_tree_node *node;
	struct hv_domain *hvdom = to_hv_domain(immdom);

	spin_lock_irqsave(&hvdom->mappings_lock, flags);
	node = interval_tree_iter_first(&hvdom->mappings_tree, iova, iova);
	if (node) {
		mapping = container_of(node, struct hv_iommu_mapping, iova);
		paddr = mapping->paddr + (iova - mapping->iova.start);
	}
	spin_unlock_irqrestore(&hvdom->mappings_lock, flags);

	return paddr;
}

static struct iommu_device *hv_iommu_probe_device(struct device *dev)
{
	pr_err("HYPER-V ARM64: hv_iommu_probe_device: dev->id=%u \n", dev->id);
	msleep(100);
	if (!dev_is_pci(dev))
		return ERR_PTR(-ENODEV);

	/*
	 * Skip the PCI device specified in `pci_devs_to_skip`. This is a
	 * temporary solution until we figure out a way to extract information
	 * from the hypervisor what devices it is already using.
	 */
	if (pci_devs_to_skip && *pci_devs_to_skip) {
		int pos = 0;
		int parsed;
		int segment, bus, slot, func;
		struct pci_dev *pdev = to_pci_dev(dev);

		do {
			parsed = 0;

			sscanf(pci_devs_to_skip + pos, " (%x:%x:%x.%x) %n",
				&segment, &bus, &slot, &func, &parsed);

			if (parsed <= 0)
				break;

			if (pci_domain_nr(pdev->bus) == segment &&
			    pdev->bus->number == bus &&
			    PCI_SLOT(pdev->devfn) == slot &&
			    PCI_FUNC(pdev->devfn) == func) {

				dev_info(dev, "skipped by Hyper-V ARM64 IOMMU\n");
				return ERR_PTR(-ENODEV);
			}

			pos += parsed;

		} while (pci_devs_to_skip[pos]);
	}

	/*
	 * Device will be explicitly attached to the default domain,
	 * so no need to dev_iommu_priv_set() here.
	 */
	return &hv_virt_iommu;
}

static void hv_iommu_probe_finalize(struct device *dev)
{
	// pr_err("Hyper-V ARM64: hv_iommu_probe_finalize: dev->id=%u \n", dev->id);
	// msleep(100);
	struct iommu_domain *immdom = iommu_get_domain_for_dev(dev);

	if (immdom && immdom->type == IOMMU_DOMAIN_DMA)
		iommu_setup_dma_ops(dev);
	else
		set_dma_ops(dev, NULL);
}

static void hv_iommu_release_device(struct device *dev)
{
	struct hv_domain *hvdom = dev_iommu_priv_get(dev);

	/* Need to detach device from device domain if necessary. */
	if (hvdom)
		hv_iommu_detach_dev(&hvdom->iommu_dom, dev);

	dev_iommu_priv_set(dev, NULL);
	set_dma_ops(dev, NULL);
}

static struct iommu_group *hv_iommu_device_group(struct device *dev)
{
	if (dev_is_pci(dev))
		return pci_device_group(dev);
	else
		return generic_device_group(dev);
}

static void hv_iommu_get_resv_regions(struct device *dev,
				      struct list_head *head)
{
	/*
	 * On ARM64 L1VH, there are no BIOS/firmware reserved regions
	 * like RMRR on Intel x86. Reserved regions would come from
	 * device tree or ACPI IORT tables if needed.
	 */
	return;
}

static int hv_iommu_def_domain_type(struct device *dev)
{
	/*
	 * Hypervisor creates identity domain by default during boot.
	 * All devices start in identity-mapped mode.
	 */
	return IOMMU_DOMAIN_IDENTITY;
}

static struct iommu_ops hv_iommu_ops = {
	.capable	    = hv_iommu_capable,
	.domain_alloc_identity	    = hv_iommu_domain_alloc_identity,
	.domain_alloc_paging	= hv_iommu_domain_alloc_paging,
	.probe_device	    = hv_iommu_probe_device,
	.probe_finalize     = hv_iommu_probe_finalize,
	.release_device     = hv_iommu_release_device,
	.def_domain_type    = hv_iommu_def_domain_type,
	.device_group	    = hv_iommu_device_group,
	.get_resv_regions   = hv_iommu_get_resv_regions,
	.default_domain_ops = &(const struct iommu_domain_ops) {
		.attach_dev   = hv_iommu_attach_dev,
		.map_pages    = hv_iommu_map_pages,
		.unmap_pages  = hv_iommu_unmap_pages,
		.iova_to_phys = hv_iommu_iova_to_phys,
		.free	      = hv_iommu_domain_free,
	},
	.owner		    = THIS_MODULE,
};

static void __init hv_initialize_special_domains(void)
{
	hv_def_identity_dom.iommu_dom.geometry = default_geometry;
	hv_def_identity_dom.domid_num = 0;  /* Default identity domain */

	hv_null_dom.iommu_dom.geometry = default_geometry;
	hv_null_dom.domid_num = 0xFFFFFFFF;  /* Blocked domain */
}

static int __init hv_iommu_init(void)
{
	int ret;
	struct iommu_device *iommup = &hv_virt_iommu;

	if (!hv_is_hyperv_initialized())
		return -ENODEV;

	ret = iommu_device_sysfs_add(iommup, NULL, NULL, "%s",
				     "hyperv-iommu-arm");
	if (ret) {
		pr_err("Hyper-V ARM64: iommu_device_sysfs_add failed: %d\n", ret);
		return ret;
	}

	/*
	 * Initialize special domains before registering.
	 * iommu_device_register() will call into our hooks.
	 */
	hv_initialize_special_domains();

	ret = iommu_device_register(iommup, &hv_iommu_ops, NULL);
	if (ret) {
		pr_err("Hyper-V ARM64: iommu_device_register failed: %d\n", ret);
		goto err_sysfs_remove;
	}

	pr_err("Hyper-V ARM64 IOMMU initialized (L1VH direct attach mode)\n");

	return 0;

err_sysfs_remove:
	iommu_device_sysfs_remove(iommup);
	return ret;
}

static int __init hv_iommu_arm_init(void)
{

	if (hv_l1vh_partition())
		return hv_iommu_init();

	return -ENODEV;
}

static void __exit hv_iommu_arm_exit(void)
{
	/* Cleanup if needed */
}

module_init(hv_iommu_arm_init);
module_exit(hv_iommu_arm_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Hyper-V ARM64 IOMMU driver for L1VH");
MODULE_AUTHOR("Microsoft Corporation");

#endif /* CONFIG_HYPERV_IOMMU_ARM */
