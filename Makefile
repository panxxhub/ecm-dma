KDIR = ../linux-xlnx

obj-m += ecm_dma.o
# specify flags for the module compilation

all:
	make -C $(KDIR) SUBDIRS=$(PWD) M=$(PWD)  modules 
clean:
	make -C $(KDIR) SUBDIRS=$(PWD) M=$(PWD)  clean
