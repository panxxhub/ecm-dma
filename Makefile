KDIR = ../linux-xlnx

obj-m += ecm_dma.o
# specify flags for the module compilation

all:
	make -C $(KDIR) SUBDIRS=$(PWD) M=$(PWD)  modules 
clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
