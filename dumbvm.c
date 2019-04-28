/*
 * Copyright (c) 2000, 2001, 2002, 2003, 2004, 2005, 2008, 2009
 *	The President and Fellows of Harvard College.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <types.h>
#include <kern/errno.h>
#include <lib.h>
#include <spl.h>
#include <spinlock.h>
#include <proc.h>
#include <current.h>
#include <mips/tlb.h>
#include <addrspace.h>
#include <vm.h>
#include "opt-A3.h"

/*
 * Dumb MIPS-only "VM system" that is intended to only be just barely
 * enough to struggle off the ground.
 */

/* under dumbvm, always have 48k of user stack */
#define DUMBVM_STACKPAGES    12

/*
 * Wrap rma_stealmem in a spinlock.
 */
static struct spinlock stealmem_lock = SPINLOCK_INITIALIZER;

/* Spinlock for synchronizing the coremap access */ 
static struct spinlock coremap_lock = SPINLOCK_INITIALIZER;

/* Coremap pointer */
static struct coremapEntry *coreMap = NULL;

/* Variable to check if coremap has been setup */
static bool coremapReady = false;

/* coreMap start index and number of pages */
static int cmStartIndex;
static int cmTotalPages;

void
vm_bootstrap(void)
{
#if OPT_A3
	paddr_t startAddr = 0;
    paddr_t endAddr = 0;
    int coreMapSize;
    int coreMapStorePages;

    ram_getsize(&startAddr, &endAddr);

    /* Pages available in the ram */
	cmTotalPages = (endAddr - startAddr) / PAGE_SIZE;

	/* Size of entries to be stored in the coremap */
	coreMapSize = sizeof(struct coremapEntry) * cmTotalPages;

	/* Pages required to store the entire coremap entries */
	coreMapStorePages = DIVROUNDUP(coreMapSize, PAGE_SIZE);

	/* Store the coremap */
	coreMap = (struct coremapEntry *)PADDR_TO_KVADDR(startAddr);

	/* Initialize all the coreMap entries */
	spinlock_acquire(&coremap_lock);

	for(int i = 0; i < cmTotalPages; i++)
	{
		coreMap[i].paddr = startAddr + (i * PAGE_SIZE);
		coreMap[i].isUsed = false;
		coreMap[i].blockLen = 0;
	}

	/* Initialize the entries used to store the coremap as occupied */
	for (int i = 0; i < coreMapStorePages; i++)
	{
		coreMap[i].isUsed = true;
		coreMap[i].blockLen = coreMapStorePages - i;
	}

	cmStartIndex = coreMapStorePages;

	spinlock_release(&coremap_lock);

	/* Set coreMap as ready */
	coremapReady = true;
#endif
}

static
paddr_t
getppages(unsigned long npages)
{
	paddr_t addr;

#if OPT_A3
	if (coremapReady)
	{
		spinlock_acquire(&coremap_lock);

		int i;
		
		for (i = cmStartIndex; i < cmTotalPages; i++)
		{
			if (!coreMap[i].isUsed)
			{
				/* Check if a whole block equal to npages is available */
				int j;
				for (j = 0; (unsigned)j < npages; j++)
				{
					if (i + j >= cmTotalPages)
					{
						kprintf("Could not allocate required page block: Insufficient memory\n");
						spinlock_release(&coremap_lock);
						return 0;
					}

					/* Complete block is not available */
					if (coreMap[i + j].isUsed)
					{
						break;
					}
				}

				/* In case the block is not available, look again for the next empty space */
				if ((unsigned)j < npages)
				{
					continue;
				}
				
				break;	
			}
		}

		if (i == cmTotalPages)
		{
			kprintf("Out of memory: All pages already in use\n");
			spinlock_release(&coremap_lock);
			return 0;
		}

		addr = coreMap[i].paddr;
		
		for (int j = 0; (unsigned)j < npages; j++)
		{
			coreMap[i + j].isUsed = true;
			coreMap[i + j].blockLen = npages - j;
		}

		spinlock_release(&coremap_lock);
		return addr;
	}
	/* Case when the coremap has not yet been setup */
	else
#endif
	{
		spinlock_acquire(&stealmem_lock);

		addr = ram_stealmem(npages);
	
		spinlock_release(&stealmem_lock);
		return addr;
	}
}

/* Allocate/free some kernel-space virtual pages */
vaddr_t 
alloc_kpages(int npages)
{
	paddr_t pa;
	pa = getppages(npages);
	if (pa==0) {
		return 0;
	}
	return PADDR_TO_KVADDR(pa);
}

void 
free_kpages(vaddr_t addr)
{
	paddr_t p_addr = KVADDR_TO_PADDR(addr);
	
#if OPT_A3

	spinlock_acquire(&coremap_lock);

	int i;
	for (i = cmStartIndex; i < cmTotalPages; i++)
	{
		if (coreMap[i].paddr == p_addr)
		{
			break;
		}
	}

	if (i >= cmTotalPages)
	{
		panic("Invalid address provided that needs to be freed\n");
	}

	int block_len = coreMap[i].blockLen;

	for (int j = 0; j < block_len; j++)
	{
		coreMap[i + j].isUsed = false;
		coreMap[i + j].blockLen = 0;
	}

	spinlock_release(&coremap_lock);
#endif
}

void
vm_tlbshootdown_all(void)
{
	panic("dumbvm tried to do tlb shootdown?!\n");
}

void
vm_tlbshootdown(const struct tlbshootdown *ts)
{
	(void)ts;
	panic("dumbvm tried to do tlb shootdown?!\n");
}

int
vm_fault(int faulttype, vaddr_t faultaddress)
{
	vaddr_t vbase1, vtop1, vbase2, vtop2, stackbase, stacktop;
	paddr_t paddr;
	int i;
	uint32_t ehi, elo;
	struct addrspace *as;
	int spl;
	bool write_allowed;
	int pageIdx;

	faultaddress &= PAGE_FRAME;

	DEBUG(DB_VM, "dumbvm: fault: 0x%x\n", faultaddress);

	switch (faulttype) {
	    case VM_FAULT_READONLY:
		/* Need to kill current process */
		return EFAULT;
	    case VM_FAULT_READ:
	    case VM_FAULT_WRITE:
		break;
	    default:
		return EINVAL;
	}

	if (curproc == NULL) {
		/*
		 * No process. This is probably a kernel fault early
		 * in boot. Return EFAULT so as to panic instead of
		 * getting into an infinite faulting loop.
		 */
		return EFAULT;
	}

	as = curproc_getas();
	if (as == NULL) {
		/*
		 * No address space set up. This is probably also a
		 * kernel fault early in boot.
		 */
		return EFAULT;
	}

	/* Assert that the address space has been set up properly. */
	KASSERT(as->as_vbase1 != 0);
	KASSERT(as->as_npages1 != 0);
	KASSERT(as->as_vbase2 != 0);
	KASSERT(as->as_npages2 != 0);
	KASSERT((as->as_vbase1 & PAGE_FRAME) == as->as_vbase1);
	KASSERT((as->as_vbase2 & PAGE_FRAME) == as->as_vbase2);


	vbase1 = as->as_vbase1;
	vtop1 = vbase1 + as->as_npages1 * PAGE_SIZE;
	vbase2 = as->as_vbase2;
	vtop2 = vbase2 + as->as_npages2 * PAGE_SIZE;
	stackbase = USERSTACK - DUMBVM_STACKPAGES * PAGE_SIZE;
	stacktop = USERSTACK;

	if (faultaddress >= vbase1 && faultaddress < vtop1) {
		/* Text segment of memory is read only */
		write_allowed = false;
		pageIdx = (faultaddress - vbase1)/ PAGE_SIZE;
		paddr = as->as_pgTable1[pageIdx].frameAddr;
	}
	else if (faultaddress >= vbase2 && faultaddress < vtop2) {
		write_allowed = true;
		pageIdx = (faultaddress - vbase2)/ PAGE_SIZE;
		paddr = as->as_pgTable2[pageIdx].frameAddr;
	}
	else if (faultaddress >= stackbase && faultaddress < stacktop) {
		write_allowed = true;
		pageIdx = (faultaddress - stackbase)/ PAGE_SIZE;
		paddr = as->as_pgStackTable[pageIdx].frameAddr;
	}
	else {
		return EFAULT;
	}

	if (as->as_load_complete == false)
	{
		write_allowed = true;
	}

	/* make sure it's page-aligned */
	KASSERT((paddr & PAGE_FRAME) == paddr);

	/* Disable interrupts on this CPU while frobbing the TLB. */
	spl = splhigh();

	for (i=0; i<NUM_TLB; i++) {
		tlb_read(&ehi, &elo, i);
		if (elo & TLBLO_VALID) {
			continue;
		}
		ehi = faultaddress;
		elo = paddr | (write_allowed == true ? TLBLO_DIRTY : 0) | TLBLO_VALID;
		DEBUG(DB_VM, "dumbvm: 0x%x -> 0x%x\n", faultaddress, paddr);
		tlb_write(ehi, elo, i);
		splx(spl);
		return 0;
	}

#if OPT_A3
	ehi = faultaddress;
    elo = paddr | (write_allowed == true ? TLBLO_DIRTY : 0) | TLBLO_VALID;
	tlb_random(ehi, elo);
	DEBUG(DB_VM, "dumbvm: TLB Replacement occurred at 0x%x\n", faultaddress);
	splx(spl);
	return 0;
#else
	kprintf("dumbvm: Ran out of TLB entries - cannot handle page fault\n");
	splx(spl);
	return EFAULT;
#endif

}

struct addrspace *
as_create(void)
{
	struct addrspace *as = kmalloc(sizeof(struct addrspace));
	if (as==NULL) {
		return NULL;
	}

	as->as_vbase1 = 0;
	as->as_npages1 = 0;
	as->as_vbase2 = 0;
	as->as_npages2 = 0;
	

#if OPT_A3
	as->as_load_complete = false;
	as->as_pgTable1 = NULL;
	as->as_pgTable2 = NULL;
	as->as_pgStackTable = NULL;
#endif

	return as;
}

void
as_destroy(struct addrspace *as)
{

#if OPT_A3
	for (int i = 0; (unsigned)i < as->as_npages1; i++)
	{
		free_kpages(PADDR_TO_KVADDR(as->as_pgTable1[i].frameAddr));
	}

	for (int i = 0; (unsigned)i < as->as_npages2; i++)
	{
		free_kpages(PADDR_TO_KVADDR(as->as_pgTable2[i].frameAddr));
	}

	for (int i = 0; (unsigned)i < DUMBVM_STACKPAGES; i++)
	{
		free_kpages(PADDR_TO_KVADDR(as->as_pgStackTable[i].frameAddr));	
	}

	kfree(as->as_pgTable1);
	kfree(as->as_pgTable2);
	kfree(as->as_pgStackTable);

#endif

	kfree(as);
}

void
as_activate(void)
{
	int i, spl;
	struct addrspace *as;

	as = curproc_getas();
#ifdef UW
        /* Kernel threads don't have an address spaces to activate */
#endif
	if (as == NULL) {
		return;
	}

	/* Disable interrupts on this CPU while frobbing the TLB. */
	spl = splhigh();

	for (i=0; i<NUM_TLB; i++) {
		tlb_write(TLBHI_INVALID(i), TLBLO_INVALID(), i);
	}

	splx(spl);
}

void
as_deactivate(void)
{
	/* nothing */
}

int
as_define_region(struct addrspace *as, vaddr_t vaddr, size_t sz,
		 int readable, int writeable, int executable)
{
	size_t npages; 

	/* Align the region. First, the base... */
	sz += vaddr & ~(vaddr_t)PAGE_FRAME;
	vaddr &= PAGE_FRAME;

	/* ...and now the length. */
	sz = (sz + PAGE_SIZE - 1) & PAGE_FRAME;

	npages = sz / PAGE_SIZE;

	/* We don't use these - all pages are read-write */
	(void)readable;
	(void)writeable;
	(void)executable;

	if (as->as_vbase1 == 0) {
		as->as_vbase1 = vaddr;
		as->as_npages1 = npages;
		return 0;
	}

	if (as->as_vbase2 == 0) {
		as->as_vbase2 = vaddr;
		as->as_npages2 = npages;
		return 0;
	}

	/*
	 * Support for more than two regions is not available.
	 */
	kprintf("dumbvm: Warning: too many regions\n");
	return EUNIMP;
}

static
void
as_zero_region(paddr_t paddr, unsigned npages)
{
	bzero((void *)PADDR_TO_KVADDR(paddr), npages * PAGE_SIZE);
}

int
as_prepare_load(struct addrspace *as)
{
#if OPT_A3
	KASSERT(as->as_pgTable1 == NULL/*0*/);
	KASSERT(as->as_pgTable2 == NULL/*0*/);
	KASSERT(as->as_pgStackTable == NULL/*0*/);

	as->as_pgTable1 = kmalloc(sizeof(struct pageTableEntry)*as->as_npages1);
	if(as->as_pgTable1 == NULL)
	{
		return ENOMEM;
	}

	as->as_pgTable2 = kmalloc(sizeof(struct pageTableEntry)*as->as_npages2);
	if(as->as_pgTable1 == NULL)
	{
		return ENOMEM;
	}

	as->as_pgStackTable = kmalloc(sizeof(struct pageTableEntry)*DUMBVM_STACKPAGES);
	if(as->as_pgTable1 == NULL)
	{
		return ENOMEM;
	}

	for (int i = 0; (unsigned)i < as->as_npages1; i++)
	{
		as->as_pgTable1[i].frameAddr = getppages(1);
		as_zero_region(as->as_pgTable1[i].frameAddr, 1);
	}

	for (int i = 0; (unsigned)i < as->as_npages2; i++)
	{
		as->as_pgTable2[i].frameAddr = getppages(1);
		as_zero_region(as->as_pgTable2[i].frameAddr, 1);
	}

	for (int i = 0; (unsigned)i < DUMBVM_STACKPAGES; i++)
	{
		as->as_pgStackTable[i].frameAddr = getppages(1);
		as_zero_region(as->as_pgStackTable[i].frameAddr, 1);
	}
#endif
	return 0;
}

int
as_complete_load(struct addrspace *as)
{
	as->as_load_complete = true;
	return 0;
}

int
as_define_stack(struct addrspace *as, vaddr_t *stackptr)
{
	KASSERT(as->as_pgStackTable != NULL);

	*stackptr = USERSTACK;
	return 0;
}

int
as_copy(struct addrspace *old, struct addrspace **ret)
{
	struct addrspace *new;

	new = as_create();
	if (new==NULL) {
		return ENOMEM;
	}

	new->as_vbase1 = old->as_vbase1;
	new->as_npages1 = old->as_npages1;
	new->as_vbase2 = old->as_vbase2;
	new->as_npages2 = old->as_npages2;

	/* (Mis)use as_prepare_load to allocate some physical memory. */
	if (as_prepare_load(new)) {
		as_destroy(new);
		return ENOMEM;
	}
#if OPT_A3

	KASSERT(new->as_pgTable1 != NULL);
	KASSERT(new->as_pgTable2 != NULL);
	KASSERT(new->as_pgStackTable != NULL);

	for (int i = 0; (unsigned)i < new->as_npages1; i++)
	{
		memmove((void *)PADDR_TO_KVADDR(new->as_pgTable1[i].frameAddr),
			(const void *)PADDR_TO_KVADDR(old->as_pgTable1[i].frameAddr),
			PAGE_SIZE);
	}

	for (int i = 0; (unsigned)i < new->as_npages2; i++)
	{
		memmove((void *)PADDR_TO_KVADDR(new->as_pgTable2[i].frameAddr),
			(const void *)PADDR_TO_KVADDR(old->as_pgTable2[i].frameAddr),
			PAGE_SIZE);
	}

	for (int i = 0; (unsigned)i < DUMBVM_STACKPAGES; i++)
	{
		memmove((void *)PADDR_TO_KVADDR(new->as_pgStackTable[i].frameAddr),
			(const void *)PADDR_TO_KVADDR(old->as_pgStackTable[i].frameAddr),
			PAGE_SIZE);
	}
#endif
	*ret = new;
	return 0;
}
