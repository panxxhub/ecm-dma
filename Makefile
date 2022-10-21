KDIR = ../linux-xlnx

obj-m += ecm_dma.o
# specify flags for the module compilation

default:
	make -C $(KDIR) SUBDIRS=$(PWD) M=$(PWD)  modules 
clean:
	make -C $(KDIR) SUBDIRS=$(PWD) clean

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
