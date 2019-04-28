# Virtual Memory Implementation

OS/161 has a very simple virtual memory system, called dumbvm.  The aim of this project is to replace dumbvm with a new virtual memory system that upgrades its functionality. In particular:

* It should be possible for the pages in process’ address spaces to be placed into any free frame of physical memory. That is, the kernel should no longer require that address space segments be stored contiguously in physical memory.

* When a process terminates, the physical frames that were used to hold its pages should be freed, and should become available for use by other processes.
